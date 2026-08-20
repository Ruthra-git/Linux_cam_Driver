
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <linux/videodev2.h>

#define FRAME_SIZE (640 * 480 * 2)
#define NUM_BUFS 2

int main(void)
{
    int fd, dmabuf_fd;
    struct v4l2_requestbuffers req = {0};
    struct v4l2_buffer buf = {0};
    struct v4l2_exportbuffer expbuf = {0};
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    uint8_t *mapped;
    int i;

    fd = open("/dev/video0", O_RDWR);
    if (fd < 0) {
        perror("open");
        return 1;
    }

    /* Request buffers in MMAP mode first — vb2 requires this
     * even when the intent is to export via dma-buf.
     */
    req.count = NUM_BUFS;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;

    if (ioctl(fd, VIDIOC_REQBUFS, &req) < 0) {
        perror("VIDIOC_REQBUFS");
        close(fd);
        return 1;
    }
    printf("Allocated %d buffers\n", req.count);

    /* Export buffer 0 as a dma-buf fd */
    expbuf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    expbuf.index = 0;
    expbuf.flags = O_RDONLY;

    if (ioctl(fd, VIDIOC_EXPBUF, &expbuf) < 0) {
        perror("VIDIOC_EXPBUF");
        close(fd);
        return 1;
    }
    dmabuf_fd = expbuf.fd;
    printf("Exported dma-buf fd = %d\n", dmabuf_fd);

    /* Queue all buffers */
    for (i = 0; i < NUM_BUFS; i++) {
        memset(&buf, 0, sizeof(buf));
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;
        if (ioctl(fd, VIDIOC_QBUF, &buf) < 0) {
            perror("VIDIOC_QBUF");
            close(fd);
            return 1;
        }
    }

    if (ioctl(fd, VIDIOC_STREAMON, &type) < 0) {
        perror("VIDIOC_STREAMON");
        close(fd);
        return 1;
    }
    printf("Streaming started, waiting for a frame...\n");

    sleep(1);  /* let the kthread fill at least one buffer */

    /* Map the exported dma-buf fd directly — this is the point:
     * we're reading frame data via the dma-buf fd, NOT via the
     * V4L2 device's own mmap path.
     */
    mapped = mmap(NULL, FRAME_SIZE, PROT_READ, MAP_SHARED, dmabuf_fd, 0);
    if (mapped == MAP_FAILED) {
        perror("mmap(dmabuf_fd)");
        close(dmabuf_fd);
        close(fd);
        return 1;
    }

    printf("First 32 bytes via dma-buf mmap:\n");
    for (i = 0; i < 32; i++)
        printf("%02X ", mapped[i]);
    printf("\n");

    FILE *fp = fopen("dmabuf_frame.raw", "wb");
    fwrite(mapped, 1, FRAME_SIZE, fp);
    fclose(fp);
    printf("Wrote dmabuf_frame.raw\n");

    munmap(mapped, FRAME_SIZE);

    ioctl(fd, VIDIOC_STREAMOFF, &type);
    close(dmabuf_fd);
    close(fd);
    return 0;
}