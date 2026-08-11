#include <linux/hrtimer.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <media/v4l2-device.h>
#include <media/v4l2-dev.h>
#include <linux/mm.h>
#include <linux/vmalloc.h>
#include <media/videobuf2-core.h>
#include <media/videobuf2-vmalloc.h>
#include <media/videobuf2-v4l2.h>
#include <linux/list.h>
#include <linux/spinlock.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/ktime.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-ioctl.h>
#include <linux/sysfs.h>
#include <linux/firmware.h>
#include <media/media-device.h>
#include <media/media-entity.h>
#include <linux/vmalloc.h>
#include <media/v4l2-subdev.h>
#include "vcam_custom_image.h"
#define VCAM_BPP        2
#define VCAM_FRAME_SIZE (VCAM_WIDTH * VCAM_HEIGHT * VCAM_BPP)
#define VCAM_WIDTH   640
#define VCAM_HEIGHT  480
#define VCAM_FRAME_PERIOD_NS (33 * 1000 * 1000)
#define VCAM_FIRMWARE_NAME "vcam_custom.yuyv"

/* Custom control IDs - base + private class offset */
#define V4L2_CID_VCAM_BASE      (V4L2_CID_USER_BASE | 0xf000)

#define V4L2_CID_VCAM_EDGE_EN   (V4L2_CID_VCAM_BASE + 0)
#define VCAM_TIMING_SAMPLES 30
#define V4L2_CID_VCAM_MODE      (V4L2_CID_VCAM_BASE + 2)
#define V4L2_CID_VCAM_SOURCE (V4L2_CID_VCAM_BASE + 3)
#define V4L2_CID_VCAM_EDGE_THR  (V4L2_CID_VCAM_BASE + 1)
#define VCAM_PIXFMT  V4L2_PIX_FMT_YUYV
#define VCAM_Y_PLANE_SIZE (VCAM_WIDTH * VCAM_HEIGHT)


MODULE_LICENSE("GPL");
MODULE_AUTHOR("Ruthra");
MODULE_DESCRIPTION("Virtual Camera Driver");

struct virtual_camera {
    struct v4l2_device v4l2_dev;
    struct video_device vdev;
    struct mutex lock;
    struct vb2_queue queue; 
    struct mutex queue_lock;
    spinlock_t buf_lock;
    struct list_head buf_list;
    struct task_struct *kthread;
    unsigned int sequence;
    bool streaming;
    struct v4l2_ctrl_handler ctrl_handler;
    u32 pattern_speed;
    u32 block_size;
    bool edge_enable;
    u32 edge_threshold;
/* --- Week 5 Day 4: Media Controller topology --- */
    struct media_device mdev;
    struct v4l2_subdev sensor_sd;
    struct v4l2_subdev isp_sd;
    struct media_pad sensor_pad;
    struct media_pad isp_pads[2];   /* 0 = sink, 1 = source */
    struct media_pad vdev_pad;      /* video device's sink pad */
    u8 *y_scratch;      /* extracted Y-plane, width*height bytes */
    u8 *y_edges;        /* Sobel output Y-plane, width*height bytes */
    size_t y_plane_size;
/* --- Day 4: motion detection --- *
    u8 *y_previous;     /* previous frame's Y-plane, for differencing */
    u32 source_mode;
    bool have_previous;  /* false until first frame captured */
    u32 proc_mode;       /* 0 = raw, 1 = Sobel edges, 2 = motion diff */
    u8 *y_edges2;
/* --- Week 4: custom image via firmware API --- */
    u8 *custom_image_buf;   /* VCAM_FRAME_SIZE bytes, YUYV */
/* --- Week 5 Day 1: performance instrumentation --- */
    u64 frame_time_ns[30];   /* rolling window of last 30 frame times */
    u32 frame_time_idx;
    u64 extract_time_ns[30];
    u64 compute_time_ns[30];
    u64 writeback_time_ns[30];
    spinlock_t timing_lock;  /* protects the above arrays */
    struct hrtimer frame_timer;
    ktime_t frame_period;
    bool custom_image_loaded;
};


struct vcam_buffer
{
    struct vb2_v4l2_buffer vb;
    struct list_head list;
};

static struct virtual_camera vcam;
/* These subdevs exist for topology representation; actual processing
 * still happens in virtual_fill_frame() via direct function calls.
 * See NOTES.md for the rationale on this staged approach.
 */

static const struct v4l2_subdev_ops vcam_sensor_subdev_ops = {
};

static const struct v4l2_subdev_ops vcam_isp_subdev_ops = {
};

static int vcam_load_custom_image(struct virtual_camera *vcam,
                                  struct device *dev);static int vcam_load_custom_image(struct virtual_camera *vcam,
                                  struct device *dev);
static int vcam_queue_setup(struct vb2_queue *vq,
                            unsigned int *nbuffers,
                            unsigned int *nplanes,
                            unsigned int sizes[],
                            struct device *alloc_devs[])
{
    if (*nbuffers < 2)
        *nbuffers = 2;
    if (*nplanes)
        return sizes[0] < VCAM_FRAME_SIZE ? -EINVAL : 0;

    *nplanes = 1;
    sizes[0] = VCAM_FRAME_SIZE;
    return 0;
}

/* Pull the Y (luma) byte out of each YUYV pixel pair into a
 * flat 8-bit plane for convolution. YUYV layout: Y0 U Y1 V per 2 pixels.
 */

static void vcam_extract_y_plane(struct virtual_camera *vcam,
                                  const u8 *yuyv_buf)
{
    int i, pixel_count = VCAM_WIDTH * VCAM_HEIGHT;

    for (i = 0; i < pixel_count; i++)
        vcam->y_scratch[i] = yuyv_buf[i * 2];  /* every Y byte, skip U/V */
}
/* Fixed-point Sobel: Gx/Gy kernels, magnitude = |Gx| + |Gy| (cheap,
 * avoids sqrt in kernel space). Border pixels (row/col 0 and max)
 * are left as 0 — no full 3x3 neighborhood available there.
 */

static void vcam_sobel_compute(struct virtual_camera *vcam, u32 threshold)
{
    int x, y, gx, gy, mag;
    int w = VCAM_WIDTH, h = VCAM_HEIGHT;
    const u8 *src = vcam->y_scratch;
    u8 *dst = vcam->y_edges;

    memset(dst, 0, vcam->y_plane_size);

    for (y = 1; y < h - 1; y++) {
        for (x = 1; x < w - 1; x++) {
            int tl = src[(y - 1) * w + (x - 1)];
            int tc = src[(y - 1) * w + x];
            int tr = src[(y - 1) * w + (x + 1)];
            int ml = src[y * w + (x - 1)];
            int mr = src[y * w + (x + 1)];
            int bl = src[(y + 1) * w + (x - 1)];
            int bc = src[(y + 1) * w + x];
            int br = src[(y + 1) * w + (x + 1)];

            /* Gx kernel: [-1 0 1; -2 0 2; -1 0 1] */
            gx = (tr + 2 * mr + br) - (tl + 2 * ml + bl);

            /* Gy kernel: [-1 -2 -1; 0 0 0; 1 2 1] */
            gy = (bl + 2 * bc + br) - (tl + 2 * tc + tr);

            mag = abs(gx) + abs(gy);

            dst[y * w + x] = (mag > threshold) ? 255 : 0;
        }
    }
}

static enum hrtimer_restart vcam_frame_timer_fn(struct hrtimer *timer)
{
    struct virtual_camera *vcam =
        container_of(timer, struct virtual_camera, frame_timer);

    if (vcam->kthread)
        wake_up_process(vcam->kthread);

    hrtimer_forward_now(timer, vcam->frame_period);
    return HRTIMER_RESTART;
}
/* Write edge-detected Y plane back into YUYV output, forcing U/V
 * to neutral (128) for a clean grayscale-edges appearance.
 */
static void vcam_write_y_to_yuyv(struct virtual_camera *vcam, u8 *yuyv_buf)
{
    int i, pixel_count = VCAM_WIDTH * VCAM_HEIGHT;

    for (i = 0; i < pixel_count; i++) {
        yuyv_buf[i * 2] = vcam->y_edges[i];
        if (i % 2 == 0)
            yuyv_buf[i * 2 + 1] = 128;  /* U or V byte, neutral */
    }
}
static int vcam_buf_prepare(struct vb2_buffer *vb) {
    struct vb2_v4l2_buffer *vbuf =
        to_vb2_v4l2_buffer(vb);

    vb2_set_plane_payload(vb,
                          0,
                          VCAM_FRAME_SIZE);

    vbuf->field = V4L2_FIELD_NONE;

    return 0;
}

static void vcam_record_timing(u64 *array, u32 idx, u64 value_ns)
{
    array[idx % VCAM_TIMING_SAMPLES] = value_ns;
}

static u64 vcam_avg_timing(u64 *array)
{
    u64 sum = 0;
    int i, count = 0;

    for (i = 0; i < VCAM_TIMING_SAMPLES; i++) {
        if (array[i] != 0) {
            sum += array[i];
            count++;
        }
    }
    return count ? sum / count : 0;
}
static ssize_t reload_image_store(struct device *dev,
                                   struct device_attribute *attr,
                                   const char *buf, size_t count)
{
    int ret;

    ret = vcam_load_custom_image(&vcam, dev);
    if (ret)
        return ret;

    return count;
}
static DEVICE_ATTR_WO(reload_image);

static void vcam_buf_queue(struct vb2_buffer *vb)
{
    struct virtual_camera *vcam =
        vb2_get_drv_priv(vb->vb2_queue);

    struct vb2_v4l2_buffer *vbuf =
        to_vb2_v4l2_buffer(vb);

    struct vcam_buffer *buf =
        container_of(vbuf,
                     struct vcam_buffer,
                     vb);

    unsigned long flags;
    pr_info("vcam: QBUF received\n");
    spin_lock_irqsave(&vcam->buf_lock,
                      flags);

    list_add_tail(&buf->list,
                  &vcam->buf_list);

    spin_unlock_irqrestore(&vcam->buf_lock,
                           flags);
}
/* Frame-differencing motion detection: compares current Y-plane
 * against the previous frame's Y-plane, highlights pixels that
 * changed beyond threshold. Requires y_scratch already populated
 * by vcam_extract_y_plane() before calling this.
 */
static void vcam_motion_compute(struct virtual_camera *vcam, u32 threshold)
{
    int i, diff;
    const u8 *cur = vcam->y_scratch;
    u8 *prev = vcam->y_previous;
    u8 *dst = vcam->y_edges;
    int pixel_count = VCAM_WIDTH * VCAM_HEIGHT;

    if (!vcam->have_previous) {
        /* first frame — nothing to diff against yet, output black */
        memset(dst, 0, vcam->y_plane_size);
        memcpy(prev, cur, vcam->y_plane_size);
        vcam->have_previous = true;
        return;
    }

    for (i = 0; i < pixel_count; i++) {
        diff = abs((int)cur[i] - (int)prev[i]);
        dst[i] = (diff > threshold) ? 255 : 0;
    }

    /* current frame becomes "previous" for the next call */
    memcpy(prev, cur, vcam->y_plane_size);
}


static void virtual_fill_frame(struct virtual_camera *vcam,
                               struct vb2_v4l2_buffer *vbuf,
                               int frame_num);
static void vcam_combined_compute(struct virtual_camera *vcam, u32 threshold);



static int vcam_thread_fn(void *data)
{
    struct virtual_camera *vcam = data;
    struct vcam_buffer *buf;
    unsigned long flags;

    while (!kthread_should_stop()) {

        /* Sleep until the hrtimer wakes us, or a stop is requested */
        set_current_state(TASK_INTERRUPTIBLE);
        if (kthread_should_stop()) {
            __set_current_state(TASK_RUNNING);
            break;
        }
        schedule();
        __set_current_state(TASK_RUNNING);

        if (kthread_should_stop())
            break;

        spin_lock_irqsave(&vcam->buf_lock, flags);

        if (!list_empty(&vcam->buf_list)) {
            buf = list_first_entry(&vcam->buf_list,
                                    struct vcam_buffer, list);
            list_del(&buf->list);
            spin_unlock_irqrestore(&vcam->buf_lock, flags);

            virtual_fill_frame(vcam, &buf->vb, vcam->sequence);

            buf->vb.vb2_buf.timestamp = ktime_get_ns();
            buf->vb.sequence = vcam->sequence++;
            buf->vb.field = V4L2_FIELD_NONE;

            vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_DONE);
        } else {
            spin_unlock_irqrestore(&vcam->buf_lock, flags);
        }
    }

    return 0;
}

static int vcam_start_streaming(struct vb2_queue *vq,
                                unsigned int count)
{
    struct virtual_camera *vcam;

    vcam = vb2_get_drv_priv(vq);
    pr_info("vcam: STREAMON\n");
    vcam->sequence = 0;
    vcam->streaming = true;

    vcam->kthread =
        kthread_run(vcam_thread_fn,
                    vcam,
                    "vcam_thread");

    if (IS_ERR(vcam->kthread))
        return PTR_ERR(vcam->kthread);

    /* --- Day 2: start hrtimer-driven frame pacing --- */
    vcam->frame_period = ktime_set(0, VCAM_FRAME_PERIOD_NS);
    hrtimer_init(&vcam->frame_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
    vcam->frame_timer.function = vcam_frame_timer_fn;
    hrtimer_start(&vcam->frame_timer, vcam->frame_period, HRTIMER_MODE_REL);
    /* ------------------------------------------------ */

    return 0;
}

static void vcam_stop_streaming(struct vb2_queue *vq)
{
    struct virtual_camera *vcam;
    struct vcam_buffer *buf;
    struct vcam_buffer *tmp;
    unsigned long flags;

    vcam = vb2_get_drv_priv(vq);
    pr_info("vcam: STREAMOFF\n");
    vcam->streaming = false;

    /* --- Day 2: stop hrtimer before stopping kthread --- */
    hrtimer_cancel(&vcam->frame_timer);
    /* ------------------------------------------------ */

    if (vcam->kthread) {
        kthread_stop(vcam->kthread);
        vcam->kthread = NULL;
    }

    spin_lock_irqsave(&vcam->buf_lock, flags);
    list_for_each_entry_safe(buf, tmp, &vcam->buf_list, list) {
        list_del(&buf->list);
        vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_ERROR);
    }
    spin_unlock_irqrestore(&vcam->buf_lock, flags);
}
static int vcam_load_custom_image(struct virtual_camera *vcam, struct device *dev)
{
    const struct firmware *fw;
    int ret;

    /* Intended production path: runtime-loadable firmware, allows
     * swapping the image without recompiling. Known issue: fails
     * with -ENOENT on this WSL2 dev kernel despite correct file
     * placement/permissions — root cause not resolved after ruling
     * out permissions, filesystem type, mount namespace, lockdown,
     * and AppArmor. Falls back to a compiled-in image below.
     */
    ret = request_firmware(&fw, VCAM_FIRMWARE_NAME, dev);
    if (!ret) {
        if (fw->size != VCAM_FRAME_SIZE) {
            pr_err("vcam: firmware size mismatch: got %zu, expected %d\n",
                   fw->size, VCAM_FRAME_SIZE);
            release_firmware(fw);
        } else {
            memcpy(vcam->custom_image_buf, fw->data, VCAM_FRAME_SIZE);
            release_firmware(fw);
            vcam->custom_image_loaded = true;
            pr_info("vcam: custom image loaded via request_firmware (%d bytes)\n",
                    VCAM_FRAME_SIZE);
            return 0;
        }
    } else {
        pr_warn("vcam: request_firmware failed (%d), falling back to embedded image\n",
                ret);
    }

    /* Fallback: compiled-in image */
    if (vcam_embedded_image_len != VCAM_FRAME_SIZE) {
        pr_err("vcam: embedded image size mismatch: got %u, expected %d\n",
               vcam_embedded_image_len, VCAM_FRAME_SIZE);
        vcam->custom_image_loaded = false;
        return -EINVAL;
    }

    memcpy(vcam->custom_image_buf, vcam_embedded_image, VCAM_FRAME_SIZE);
    vcam->custom_image_loaded = true;
    pr_info("vcam: embedded custom image loaded (%d bytes)\n", VCAM_FRAME_SIZE);
    return 0;
}
static int vcam_s_ctrl(struct v4l2_ctrl *ctrl)
{
    struct virtual_camera *vcam;

    vcam = container_of(ctrl->handler,
                        struct virtual_camera,
                        ctrl_handler);

    switch (ctrl->id) {

    case V4L2_CID_BRIGHTNESS:

        vcam->pattern_speed = ctrl->val;

        pr_info("Pattern speed = %u\n",
                vcam->pattern_speed);

        return 0;
case V4L2_CID_SHARPNESS:

    vcam->block_size = ctrl->val;
pr_info("Block size = %u\n",
        vcam->block_size);

    return 0;
case V4L2_CID_VCAM_EDGE_EN:
        vcam->edge_enable = ctrl->val;
        pr_info("Edge detection %s\n",
                vcam->edge_enable ? "enabled" : "disabled");
        return 0;

    case V4L2_CID_VCAM_EDGE_THR:
        vcam->edge_threshold = ctrl->val;
        pr_info("Edge threshold = %u\n", vcam->edge_threshold);
        return 0;
case V4L2_CID_VCAM_MODE:
        vcam->proc_mode = ctrl->val;
        pr_info("Processing mode = %u (%s)\n", vcam->proc_mode,
                vcam->proc_mode == 0 ? "raw" :
                vcam->proc_mode == 1 ? "sobel" :
                vcam->proc_mode == 2 ? "motion" :
                vcam->proc_mode == 3 ? "custom_image" : "combined");
        return 0;

case V4L2_CID_VCAM_SOURCE:

    vcam->source_mode = ctrl->val;

    pr_info("Image source = %s\n",

            vcam->source_mode ?
            "custom image" :
            "checkerboard");

    return 0;


}




    return -EINVAL;
}
static const struct v4l2_ctrl_ops vcam_ctrl_ops =
{
    .s_ctrl = vcam_s_ctrl,
};
static const struct vb2_ops vcam_qops =
{
    .queue_setup     = vcam_queue_setup,
    .buf_prepare     = vcam_buf_prepare,
    .buf_queue       = vcam_buf_queue,
    .start_streaming = vcam_start_streaming,
    .stop_streaming  = vcam_stop_streaming,
};




static void virtual_fill_frame(struct virtual_camera *vcam,
                               struct vb2_v4l2_buffer *vbuf,
                               int frame_num)
{
    u8 *buf;
    unsigned int speed, block;
    int x, y;
    ktime_t t0, t1, t2, t3;
    unsigned long flags;

    buf = vb2_plane_vaddr(&vbuf->vb2_buf, 0);
    if (!buf)
        return;

    t0 = ktime_get();
/* --- Source selection: now driven by source_mode, independent of proc_mode --- */
    if (vcam->source_mode == 1 && vcam->custom_image_loaded) {
        memcpy(buf, vcam->custom_image_buf, VCAM_FRAME_SIZE);
    } else {
        speed = max(1U, vcam->pattern_speed);
        block = max(8U, vcam->block_size);

        for (y = 0; y < VCAM_HEIGHT; y++) {
            for (x = 0; x < VCAM_WIDTH; x += 2) {
                bool white =
                    (((x / block) + (y / block) +
                      frame_num / speed) % 2);
                u8 yval = white ? 235 : 16;
                int idx = (y * VCAM_WIDTH + x) * 2;

                buf[idx + 0] = yval;
                buf[idx + 1] = 128;
                buf[idx + 2] = yval;
                buf[idx + 3] = 128;
            }
        }
    }

    t1 = ktime_get();

    /* --- Processing pass: proc_mode now purely selects algorithm --- */
    switch (vcam->proc_mode) {
    case 1: /* Sobel edges */
        vcam_extract_y_plane(vcam, buf);
        t2 = ktime_get();
        vcam_sobel_compute(vcam, vcam->edge_threshold);
        t3 = ktime_get();
        vcam_write_y_to_yuyv(vcam, buf);
        break;

    case 2: /* motion diff */
        vcam_extract_y_plane(vcam, buf);
        t2 = ktime_get();
        vcam_motion_compute(vcam, vcam->edge_threshold);
        t3 = ktime_get();
        vcam_write_y_to_yuyv(vcam, buf);
        break;

    case 4: /* combined */
        vcam_extract_y_plane(vcam, buf);
        t2 = ktime_get();
        vcam_combined_compute(vcam, vcam->edge_threshold);
        t3 = ktime_get();
        vcam_write_y_to_yuyv(vcam, buf);
        break;

    case 0: /* raw passthrough */
    default:
        t2 = t1;
        t3 = t1;
        break;
    }
    /* --- Source selection --- */
    //if (vcam->proc_mode == 3 && vcam->custom_image_loaded) {
      //  memcpy(buf, vcam->custom_image_buf, VCAM_FRAME_SIZE);
    //} else {
      //  speed = max(1U, vcam->pattern_speed);
        //block = max(8U, vcam->block_size);

        //for (y = 0; y < VCAM_HEIGHT; y++) {
          //  for (x = 0; x < VCAM_WIDTH; x += 2) {
            //    bool white =
              //      (((x / block) + (y / block) +
                //      frame_num / speed) % 2);
                //u8 yval = white ? 235 : 16;
                //int idx = (y * VCAM_WIDTH + x) * 2;

                //buf[idx + 0] = yval;
                //buf[idx + 1] = 128;
                //buf[idx + 2] = yval;
                //buf[idx + 3] = 128;
      //      }
    //    }
  //  }
//
 /*   t1 = ktime_get();  /* end of source generation */

    /* --- Processing pass --- */
    /*switch (vcam->proc_mode) {
    case 1: /* Sobel edges */
        //vcam_extract_y_plane(vcam, buf);
        //t2 = ktime_get();
        //vcam_sobel_compute(vcam, vcam->edge_threshold);
        //t3 = ktime_get();
      //  vcam_write_y_to_yuyv(vcam, buf);
    //    break;

  //  case 2: /* motion diff */
        //vcam_extract_y_plane(vcam, buf);
        //t2 = ktime_get();
        //vcam_motion_compute(vcam, vcam->edge_threshold);
       // t3 = ktime_get();
        //vcam_write_y_to_yuyv(vcam, buf);
      //  break;

    //case 0: /* raw checkerboard */
   // case 3: /* custom image */
 //case 4: /* combined: Sobel + motion, demonstrates headroom */
        //vcam_extract_y_plane(vcam, buf);
        //t2 = ktime_get();
        //vcam_combined_compute(vcam, vcam->edge_threshold);
        //t3 = ktime_get();
        //vcam_write_y_to_yuyv(vcam, buf);
      //  break;
    //default:
        //t2 = t1;
        //t3 = t1;
      //  break;
    //}

    vb2_set_plane_payload(&vbuf->vb2_buf, 0, VCAM_FRAME_SIZE);

    /* --- Record timing --- */
    spin_lock_irqsave(&vcam->timing_lock, flags);
    vcam_record_timing(vcam->frame_time_ns, vcam->frame_time_idx,
                        ktime_to_ns(ktime_sub(ktime_get(), t0)));
    vcam_record_timing(vcam->extract_time_ns, vcam->frame_time_idx,
                        ktime_to_ns(ktime_sub(t2, t1)));
    vcam_record_timing(vcam->compute_time_ns, vcam->frame_time_idx,
                        ktime_to_ns(ktime_sub(t3, t2)));
    vcam_record_timing(vcam->writeback_time_ns, vcam->frame_time_idx,
                        ktime_to_ns(ktime_sub(ktime_get(), t3)));
    vcam->frame_time_idx++;
    spin_unlock_irqrestore(&vcam->timing_lock, flags);
}

static int virtual_querycap(struct file *file, void *priv,
                             struct v4l2_capability *cap)
{
    strscpy(cap->driver, "virtual_camera", sizeof(cap->driver));
    strscpy(cap->card, "Virtual Camera", sizeof(cap->card));
    strscpy(cap->bus_info, "platform:virtual_camera", sizeof(cap->bus_info));
    cap->device_caps = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING;
    cap->capabilities = cap->device_caps | V4L2_CAP_DEVICE_CAPS;
    return 0;
}

#define VCAM_WIDTH   640
#define VCAM_HEIGHT  480
#define VCAM_PIXFMT  V4L2_PIX_FMT_YUYV

static int virtual_enum_input(struct file *file, void *priv,
                               struct v4l2_input *inp)
{
    if (inp->index != 0)
        return -EINVAL;

    strscpy(inp->name, "Virtual Input", sizeof(inp->name));
    inp->type = V4L2_INPUT_TYPE_CAMERA;
    inp->std = 0;
    return 0;
}

static int virtual_g_input(struct file *file, void *priv, unsigned int *i)
{
    *i = 0;
    return 0;
}

static int virtual_s_input(struct file *file, void *priv, unsigned int i)
{
    if (i != 0)
        return -EINVAL;
    return 0;
}

static int virtual_enum_fmt_vid_cap(struct file *file, void *priv,
                                     struct v4l2_fmtdesc *f)
{
    if (f->index != 0)
        return -EINVAL;

    f->pixelformat = VCAM_PIXFMT;
    strscpy(f->description, "YUYV 4:2:2", sizeof(f->description));
    return 0;
}

static int virtual_g_fmt_vid_cap(struct file *file, void *priv,
                                  struct v4l2_format *f)
{
    f->fmt.pix.width        = VCAM_WIDTH;
    f->fmt.pix.height       = VCAM_HEIGHT;
    f->fmt.pix.pixelformat  = VCAM_PIXFMT;
    f->fmt.pix.field        = V4L2_FIELD_NONE;
    f->fmt.pix.bytesperline = VCAM_WIDTH * 2; /* YUYV = 2 bytes/pixel */
    f->fmt.pix.sizeimage    = VCAM_FRAME_SIZE;
    f->fmt.pix.colorspace   = V4L2_COLORSPACE_SRGB;
    return 0;
}

static int virtual_s_fmt_vid_cap(struct file *file, void *priv,
                                  struct v4l2_format *f)
{
    /* Fixed format driver for now — just echo back the supported format,
     * same as g_fmt. Week 3 revisit if variable formats are added. */
    return virtual_g_fmt_vid_cap(file, priv, f);
}

static int virtual_try_fmt_vid_cap(struct file *file, void *priv,
                                    struct v4l2_format *f)
{
    return virtual_g_fmt_vid_cap(file, priv, f);
}

static const struct v4l2_ioctl_ops virtual_ioctl_ops = {
    .vidioc_querycap = virtual_querycap,
     .vidioc_enum_input       = virtual_enum_input,
    .vidioc_g_input          = virtual_g_input,
    .vidioc_s_input          = virtual_s_input,
    .vidioc_enum_fmt_vid_cap = virtual_enum_fmt_vid_cap,
    .vidioc_g_fmt_vid_cap    = virtual_g_fmt_vid_cap,
    .vidioc_s_fmt_vid_cap    = virtual_s_fmt_vid_cap,
    .vidioc_try_fmt_vid_cap  = virtual_try_fmt_vid_cap,
    .vidioc_reqbufs      = vb2_ioctl_reqbufs,
    .vidioc_querybuf     = vb2_ioctl_querybuf,
    .vidioc_qbuf         = vb2_ioctl_qbuf,
    .vidioc_dqbuf        = vb2_ioctl_dqbuf,
    .vidioc_expbuf       = vb2_ioctl_expbuf,
    .vidioc_streamon     = vb2_ioctl_streamon,
    .vidioc_streamoff    = vb2_ioctl_streamoff,
    .vidioc_create_bufs  = vb2_ioctl_create_bufs,


};

static const struct v4l2_file_operations virtual_fops = {

    .owner = THIS_MODULE,

    .unlocked_ioctl = video_ioctl2,
    .open = v4l2_fh_open,
    .mmap    = vb2_fop_mmap,
    .poll    = vb2_fop_poll,
    .release = vb2_fop_release,

};
static void vcam_sobel_selftest(struct virtual_camera *vcam)
{
    pr_info("=== Sobel scratch buffer self-test ===\n");
    pr_info("y_scratch addr: %px, size: %zu\n",
            vcam->y_scratch, vcam->y_plane_size);
    pr_info("y_edges  addr: %px, size: %zu\n",
            vcam->y_edges, vcam->y_plane_size);

    /* write a known pattern, read it back, confirm no corruption */
    memset(vcam->y_scratch, 0xAA, vcam->y_plane_size);
    memset(vcam->y_edges, 0x55, vcam->y_plane_size);

    if (vcam->y_scratch[0] != 0xAA ||
        vcam->y_scratch[vcam->y_plane_size - 1] != 0xAA)
        pr_err("y_scratch buffer corruption detected!\n");
    else
        pr_info("y_scratch buffer read/write OK\n");

    if (vcam->y_edges[0] != 0x55 ||
        vcam->y_edges[vcam->y_plane_size - 1] != 0x55)
        pr_err("y_edges buffer corruption detected!\n");
    else
        pr_info("y_edges buffer read/write OK\n");

    /* reset to zero for actual use later */
    memset(vcam->y_scratch, 0, vcam->y_plane_size);
    memset(vcam->y_edges, 0, vcam->y_plane_size);

    pr_info("=== Self-test complete ===\n");
}

/* Combined mode: runs Sobel and motion-diff sequentially on the same
 * source frame, then blends results (OR'd together — a pixel lights up
 * if it's either an edge or in motion). Demonstrates compute headroom:
 * roughly 2x the work of either algorithm alone.
 */
static void vcam_combined_compute(struct virtual_camera *vcam, u32 threshold)
{
    int i, pixel_count = VCAM_WIDTH * VCAM_HEIGHT;

    /* Sobel pass -> y_edges */
    vcam_sobel_compute(vcam, threshold);

    /* Motion pass -> y_edges2 (needs its own temp since motion also
     * writes to y_edges normally — redirect by computing manually here) */
    {
        const u8 *cur = vcam->y_scratch;
        u8 *prev = vcam->y_previous;
        u8 *dst2 = vcam->y_edges2;
        int diff;

        if (!vcam->have_previous) {
            memset(dst2, 0, vcam->y_plane_size);
            memcpy(prev, cur, vcam->y_plane_size);
            vcam->have_previous = true;
        } else {
            for (i = 0; i < pixel_count; i++) {
                diff = abs((int)cur[i] - (int)prev[i]);
                dst2[i] = (diff > threshold) ? 255 : 0;
            }
            memcpy(prev, cur, vcam->y_plane_size);
        }
    }

    /* Blend: pixel is "hot" if either edge OR motion fired */
    for (i = 0; i < pixel_count; i++)
        vcam->y_edges[i] = (vcam->y_edges[i] || vcam->y_edges2[i]) ? 255 : 0;
}


static const struct v4l2_ctrl_config vcam_ctrl_edge_enable = {
    .ops  = &vcam_ctrl_ops,
    .id   = V4L2_CID_VCAM_EDGE_EN,
    .name = "Edge Detection Enable",
    .type = V4L2_CTRL_TYPE_BOOLEAN,
    .min  = 0,
    .max  = 1,
    .step = 1,
    .def  = 0,
};

static const struct v4l2_ctrl_config vcam_ctrl_edge_threshold = {
    .ops  = &vcam_ctrl_ops,
    .id   = V4L2_CID_VCAM_EDGE_THR,
    .name = "Edge Detection Threshold",
    .type = V4L2_CTRL_TYPE_INTEGER,
    .min  = 0,
    .max  = 255,
    .step = 1,
    .def  = 60,
};
static const struct v4l2_ctrl_config vcam_ctrl_mode = {
    .ops  = &vcam_ctrl_ops,
    .id   = V4L2_CID_VCAM_MODE,
    .name = "Processing Mode",
    .type = V4L2_CTRL_TYPE_INTEGER,
    .min  = 0,
    .max  = 4,
    .step = 1,
    .def  = 0,
};

static const struct v4l2_ctrl_config vcam_ctrl_source = {

    .ops  = &vcam_ctrl_ops,
    .id   = V4L2_CID_VCAM_SOURCE,
    .name = "Image Source",
    .type = V4L2_CTRL_TYPE_INTEGER,
    .min  = 0,
    .max  = 1,
    .step = 1,
    .def  = 0,
};
static ssize_t perf_stats_show(struct device *dev,
                                struct device_attribute *attr,
                                char *buf)
{
    unsigned long flags;
    u64 avg_total, avg_extract, avg_compute, avg_writeback;
    spin_lock_irqsave(&vcam.timing_lock, flags);
    avg_total     = vcam_avg_timing(vcam.frame_time_ns);
    avg_extract   = vcam_avg_timing(vcam.extract_time_ns);
    avg_compute   = vcam_avg_timing(vcam.compute_time_ns);
    avg_writeback = vcam_avg_timing(vcam.writeback_time_ns);
    spin_unlock_irqrestore(&vcam.timing_lock, flags);

    return sysfs_emit(buf,
        "mode=%u total_us=%llu extract_us=%llu compute_us=%llu writeback_us=%llu\n",
        vcam.proc_mode,
        avg_total / 1000,
        avg_extract / 1000,
        avg_compute / 1000,
        avg_writeback / 1000);
}
static DEVICE_ATTR_RO(perf_stats);
static int virtual_probe(struct platform_device *pdev)
{
    int ret;
    pr_info("Virtual Camera Probe\n");
    mutex_init(&vcam.lock);
spin_lock_init(&vcam.timing_lock);

ret = v4l2_device_register(&pdev->dev, &vcam.v4l2_dev);
    if (ret) {
        pr_err("v4l2_device_register failed: %d\n", ret);
        return ret;
    }
vcam.pattern_speed = 5;
vcam.block_size = 32;
vcam.edge_enable = false;
vcam.edge_threshold = 60;
vcam.proc_mode  = 0;
vcam.source_mode = 0;

vcam.proc_mode = 0;

v4l2_ctrl_handler_init(
        &vcam.ctrl_handler,
        6);

v4l2_ctrl_new_std(
        &vcam.ctrl_handler,
        &vcam_ctrl_ops,
        V4L2_CID_BRIGHTNESS,
        1,
        15,
        1,
        5);
v4l2_ctrl_new_std(
        &vcam.ctrl_handler,
        &vcam_ctrl_ops,
        V4L2_CID_SHARPNESS,
        8,
        128,
        8,
        32);
 v4l2_ctrl_new_custom(&vcam.ctrl_handler,
                         &vcam_ctrl_edge_enable,
                         NULL);


v4l2_ctrl_new_custom(&vcam.ctrl_handler,
                     &vcam_ctrl_source,
                     NULL);
    v4l2_ctrl_new_custom(&vcam.ctrl_handler,
                         &vcam_ctrl_edge_threshold,
                         NULL);

v4l2_ctrl_new_custom(&vcam.ctrl_handler,
                         &vcam_ctrl_mode,
                         NULL);

pr_info("ctrl_handler.error = %d\n",
        vcam.ctrl_handler.error);
if (vcam.ctrl_handler.error) {

    ret = vcam.ctrl_handler.error;

    v4l2_ctrl_handler_free(
        &vcam.ctrl_handler);

    return ret;
}

vcam.v4l2_dev.ctrl_handler =
        &vcam.ctrl_handler;
    memset(&vcam.vdev, 0, sizeof(vcam.vdev));
    strscpy(vcam.vdev.name, "Virtual Camera", sizeof(vcam.vdev.name));

    vcam.vdev.v4l2_dev   = &vcam.v4l2_dev;
    vcam.vdev.fops       = &virtual_fops;
    vcam.vdev.ioctl_ops  = &virtual_ioctl_ops;
    vcam.vdev.release    = video_device_release_empty;
    vcam.vdev.lock       = &vcam.lock;

    /* --- These two are what were missing and caused -EINVAL --- */
    vcam.vdev.vfl_dir     = VFL_DIR_RX;
    vcam.vdev.device_caps = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING;
    /* ------------------------------------------------------------ */

    video_set_drvdata(&vcam.vdev, &vcam);
     mutex_init(&vcam.queue_lock);
/* --- Week 5 Day 4: Media Controller registration --- */
    vcam.mdev.dev = &pdev->dev;
    strscpy(vcam.mdev.model, "Virtual Camera MC", sizeof(vcam.mdev.model));
    strscpy(vcam.mdev.bus_info, "platform:virtual_camera",
            sizeof(vcam.mdev.bus_info));
    media_device_init(&vcam.mdev);

    ret = media_device_register(&vcam.mdev);
    if (ret) {
        pr_err("media_device_register failed: %d\n", ret);
        v4l2_device_unregister(&vcam.v4l2_dev);
        return ret;
    }

    /* Link the v4l2_device to the media_device so video_register_device
     * later can associate the /dev/videoN node with this topology. */
    vcam.v4l2_dev.mdev = &vcam.mdev;

    /* --- Sensor subdev: represents pattern/image generation --- */
    v4l2_subdev_init(&vcam.sensor_sd, &vcam_sensor_subdev_ops);
    strscpy(vcam.sensor_sd.name, "vcam-sensor", sizeof(vcam.sensor_sd.name));
    vcam.sensor_sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;

    vcam.sensor_pad.flags = MEDIA_PAD_FL_SOURCE;
    ret = media_entity_pads_init(&vcam.sensor_sd.entity, 1, &vcam.sensor_pad);
    if (ret) {
        pr_err("sensor entity pad init failed: %d\n", ret);
        goto err_media;
    }

    ret = v4l2_device_register_subdev(&vcam.v4l2_dev, &vcam.sensor_sd);
    if (ret) {
        pr_err("sensor subdev register failed: %d\n", ret);
        goto err_media;
    }

    /* --- ISP subdev: represents Sobel/motion processing --- */
    v4l2_subdev_init(&vcam.isp_sd, &vcam_isp_subdev_ops);
    strscpy(vcam.isp_sd.name, "vcam-isp", sizeof(vcam.isp_sd.name));
    vcam.isp_sd.entity.function = MEDIA_ENT_F_PROC_VIDEO_PIXEL_FORMATTER;

    vcam.isp_pads[0].flags = MEDIA_PAD_FL_SINK;
    vcam.isp_pads[1].flags = MEDIA_PAD_FL_SOURCE;
    ret = media_entity_pads_init(&vcam.isp_sd.entity, 2, vcam.isp_pads);
    if (ret) {
        pr_err("isp entity pad init failed: %d\n", ret);
        goto err_media;
    }

    ret = v4l2_device_register_subdev(&vcam.v4l2_dev, &vcam.isp_sd);
    if (ret) {
        pr_err("isp subdev register failed: %d\n", ret);
        goto err_media;
    }

    /* --- Link sensor -> ISP --- */
    ret = media_create_pad_link(&vcam.sensor_sd.entity, 0,
                                 &vcam.isp_sd.entity, 0,
                                 MEDIA_LNK_FL_ENABLED | MEDIA_LNK_FL_IMMUTABLE);
    if (ret) {
        pr_err("sensor->isp link failed: %d\n", ret);
        goto err_media;
    }

 /* ------------------------------------------------------------ */
goto media_ok;
err_media:
    media_device_unregister(&vcam.mdev);
    media_device_cleanup(&vcam.mdev);
    v4l2_device_unregister(&vcam.v4l2_dev);
    return ret;

media_ok:
vcam.queue.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
vcam.queue.io_modes = VB2_MMAP | VB2_READ | VB2_DMABUF;
vcam.queue.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
vcam.queue.io_modes = VB2_MMAP | VB2_READ | VB2_DMABUF;
vcam.queue.dev = &pdev->dev;
vcam.queue.drv_priv = &vcam;
vcam.queue.buf_struct_size = sizeof(struct vcam_buffer);
vcam.queue.ops = &vcam_qops;
vcam.queue.mem_ops = &vb2_vmalloc_memops;
vcam.queue.timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
vcam.queue.min_buffers_needed = 2;
vcam.queue.lock = &vcam.queue_lock;
ret = vb2_queue_init(&vcam.queue);
if (ret)
    {
    pr_err("vb2_queue_init failed: %d\n", ret);
    v4l2_device_unregister(&vcam.v4l2_dev);

return ret;
}

vcam.vdev.queue = &vcam.queue;
vcam.vdev.vfl_dir = VFL_DIR_RX;

INIT_LIST_HEAD(&vcam.buf_list);
spin_lock_init(&vcam.buf_lock);
/* --- Week 3: allocate Sobel scratch buffers --- */
    vcam.y_plane_size = VCAM_Y_PLANE_SIZE;

    vcam.y_scratch = vzalloc(vcam.y_plane_size);
    if (!vcam.y_scratch) {
        pr_err("Failed to allocate y_scratch buffer\n");
        ret = -ENOMEM;
        goto err_vb2_queue;
    }

    vcam.y_edges = vzalloc(vcam.y_plane_size);
    if (!vcam.y_edges) {
        pr_err("Failed to allocate y_edges buffer\n");
        vfree(vcam.y_scratch);
        ret = -ENOMEM;
        goto err_vb2_queue;
    }
vcam.y_edges2 = vzalloc(vcam.y_plane_size);
    if (!vcam.y_edges2) {
        pr_err("Failed to allocate y_edges2 buffer\n");
        vfree(vcam.y_edges);
        vfree(vcam.y_scratch);
        ret = -ENOMEM;
        goto err_vb2_queue;
    }

 /* --- Day 4: motion-detection previous-frame buffer --- */
    vcam.y_previous = vzalloc(vcam.y_plane_size);
    if (!vcam.y_previous) {
        pr_err("Failed to allocate y_previous buffer\n");
        vfree(vcam.y_edges);
        vfree(vcam.y_scratch);
        ret = -ENOMEM;
        goto err_vb2_queue;
    }
    vcam.have_previous = false;
    /* ------------------------------------------------------ */
/* --- Week 4: custom image buffer + firmware load --- */
    vcam.custom_image_buf = vzalloc(VCAM_FRAME_SIZE);
    if (!vcam.custom_image_buf) {
        pr_err("Failed to allocate custom_image_buf\n");
        vfree(vcam.y_previous);
        vfree(vcam.y_edges);
        vfree(vcam.y_scratch);
        ret = -ENOMEM;
        goto err_vb2_queue;
    }

    ret = vcam_load_custom_image(&vcam, &pdev->dev);
    if (ret) {
        pr_err("vcam_load_custom_image failed: %d\n", ret);
        vfree(vcam.custom_image_buf);
        vfree(vcam.y_previous);
        vfree(vcam.y_edges);
        vfree(vcam.y_scratch);
        goto err_vb2_queue;
    }
    /* ------------------------------------------------------ */

    pr_info("Sobel scratch buffers allocated: %zu bytes each\n",
            vcam.y_plane_size);
vcam_sobel_selftest(&vcam);
    ret = video_register_device(&vcam.vdev, VFL_TYPE_VIDEO, -1);
    if (ret) {
        pr_err("video_register_device failed: %d\n", ret);

vfree(vcam.custom_image_buf);
vfree(vcam.y_previous);
        vfree(vcam.y_edges);
        vfree(vcam.y_scratch);
        v4l2_ctrl_handler_free(&vcam.ctrl_handler);

        v4l2_device_unregister(&vcam.v4l2_dev);
        return ret;
    }
vcam.vdev_pad.flags = MEDIA_PAD_FL_SINK;
    ret = media_entity_pads_init(&vcam.vdev.entity, 1, &vcam.vdev_pad);
    if (ret) {
        pr_err("video device pad init failed: %d\n", ret);
        /* fall through to existing cleanup below, add media cleanup too */
    }

    ret = media_create_pad_link(&vcam.isp_sd.entity, 1,
                                 &vcam.vdev.entity, 0,
                                 MEDIA_LNK_FL_ENABLED | MEDIA_LNK_FL_IMMUTABLE);
    if (ret)
        pr_err("isp->capture link failed: %d\n", ret);

ret = device_create_file(&pdev->dev,
                         &dev_attr_reload_image);

if (ret) {
    pr_err("Failed to create reload_image sysfs file\n");
     device_remove_file(&pdev->dev, &dev_attr_reload_image);
    video_unregister_device(&vcam.vdev);

    vfree(vcam.custom_image_buf);
    vfree(vcam.y_previous);
    vfree(vcam.y_edges);
    vfree(vcam.y_scratch);

    v4l2_ctrl_handler_free(&vcam.ctrl_handler);
    v4l2_device_unregister(&vcam.v4l2_dev);

    return ret;
}
ret = device_create_file(&pdev->dev,
                         &dev_attr_perf_stats);
if (ret) {
    pr_err("Failed to create perf_stats sysfs file\n");
    device_remove_file(&pdev->dev, &dev_attr_reload_image);
    device_remove_file(&pdev->dev, &dev_attr_perf_stats);
    video_unregister_device(&vcam.vdev);
    vfree(vcam.custom_image_buf);
    vfree(vcam.y_previous);
    vfree(vcam.y_edges);
    vfree(vcam.y_scratch);
    v4l2_ctrl_handler_free(&vcam.ctrl_handler);
    v4l2_device_unregister(&vcam.v4l2_dev);
    return ret;
}

    pr_info("Registered as /dev/video%d\n", vcam.vdev.num);
    return 0;
err_vb2_queue:
    v4l2_ctrl_handler_free(&vcam.ctrl_handler);
    v4l2_device_unregister(&vcam.v4l2_dev);
    return ret;

}
static int virtual_remove(struct platform_device *pdev)
{

device_remove_file(&pdev->dev,
                   &dev_attr_reload_image);

device_remove_file(&pdev->dev, &dev_attr_perf_stats);
 if (vcam.kthread)
        kthread_stop(vcam.kthread);

 v4l2_device_unregister_subdev(&vcam.sensor_sd);
    v4l2_device_unregister_subdev(&vcam.isp_sd);
    media_device_unregister(&vcam.mdev);
    media_device_cleanup(&vcam.mdev);
    vfree(vcam.custom_image_buf);
    vfree(vcam.y_previous);
    vfree(vcam.y_edges);
    vfree(vcam.y_scratch);
    vfree(vcam.y_edges2);
    v4l2_ctrl_handler_free(&vcam.ctrl_handler);
    video_unregister_device(&vcam.vdev);
    v4l2_device_unregister(&vcam.v4l2_dev);
    pr_info("Driver removed, Sobel buffers freed\n");
    return 0;
}
static struct platform_driver virtual_driver = {
    .probe  = virtual_probe,
    .remove = virtual_remove,
    .driver = {
    .name = "virtual_camera",
    },
};

static struct platform_device *virtual_device;

static int __init virtual_init(void)
{
    int ret;

    ret = platform_driver_register(&virtual_driver);
    if (ret)
        return ret;

    virtual_device = platform_device_register_simple("virtual_camera", -1, NULL, 0);
    if (IS_ERR(virtual_device)) {
        platform_driver_unregister(&virtual_driver);
        return PTR_ERR(virtual_device);
    }

    pr_info("Virtual Camera Loaded\n");
    return 0;
}

static void __exit virtual_exit(void)
{
    platform_device_unregister(virtual_device);
    platform_driver_unregister(&virtual_driver);
    pr_info("Driver Removed\n");
}

module_init(virtual_init);
module_exit(virtual_exit);