# Virtual V4L2 Camera Driver

## 1. Overview

This document describes the internal architecture of the driver: how the major subsystems fit together, why each was chosen, and how data actually moves from frame generation to a userspace consumer.

It is meant to be read alongside `src/virtual_camera.c`, not as a substitute for it.

The driver is a **platform driver** with no backing hardware. It registers a synthetic platform device at module load time using `platform_device_register_simple`, purely so it has a `struct device` to hang V4L2, videobuf2, and Media Controller registration off of.

This follows the same structural pattern a real sensor/ISP driver would use, minus the actual hardware-bus probing.



## 2. High-Level Data Flow

flowchart TD
    A["hrtimer\n33 ms"] -->|"wake_up_process()"| B["Capture kthread"]
    B --> C["Pull next queued vb2 buffer"]
    C --> D["virtual_fill_frame()"]
    D --> E["Source selection"]
    E --> F["Processing selection"]
    F --> G["Timing instrumentation"]
    G --> H["vb2_buffer_done(DONE)"]
    H --> I["Userspace consumer"]


### Sequential Flow

1. The `hrtimer` runs at a 33 ms frame period.
2. The timer uses `CLOCK_MONOTONIC`.
3. The timer forwards itself using `hrtimer_forward_now()`.
4. The timer calls `wake_up_process()`.
5. The capture kthread `vcam_thread_fn` wakes up.
6. The kthread pulls the next queued `vb2` buffer.
7. The kthread calls `virtual_fill_frame()`.
8. `source_mode` selects the source image.
9. `source_mode = 0` selects the checkerboard source.
10. `source_mode = 1` selects the custom image source.
11. `proc_mode` selects the processing algorithm.
12. `proc_mode = 0` selects raw processing.
13. `proc_mode = 1` selects Sobel edge detection.
14. `proc_mode = 2` selects motion detection.
15. `proc_mode = 3` selects custom processing.
16. `proc_mode = 4` selects combined processing.
17. `ktime_get()` records timing information for each processing stage.
18. The completed buffer is returned using `vb2_buffer_done(DONE)`.
19. The userspace consumer receives the completed frame.
20. The consumer can access the frame using `mmap`, such as through `ffplay` or `v4l2-ctl`.
21. The consumer can alternatively use DMA-BUF, such as through `test_dmabuf` or GStreamer with `io-mode=dmabuf`.

Source generation and processing are deliberately **decoupled**.

`source_mode` determines what the frame is, while `proc_mode` determines what algorithm operates on that frame.



## 3. Why Videobuf2 Instead of a Hand-Rolled Buffer

The Week 1 implementation used a single `vmalloc_user()` buffer and a manual `.mmap` file operation.

This was replaced in Week 2 with **videobuf2**.

### Sequential Reasoning

1. `vb2` provides standard implementations for `REQBUFS`, `QBUF`, `DQBUF`, `STREAMON/OFF`, and `EXPBUF`.
2. This avoids manually reimplementing complex V4L2 buffer-management behaviour.
3. `vb2_fop_mmap` handles userspace memory mapping.
4. `vb2_fop_poll` handles polling.
5. `vb2_fop_release` handles release operations and per-open state.
6. These facilities support multi-buffer queuing correctly.
7. DMA-BUF export through `VIDIOC_EXPBUF` becomes available through the configured `vb2` queue.
8. The queue uses software-only memory through `vb2_vmalloc_memops`.
9. The queue is configured with:
10. No DMA-capable hardware exists behind this driver because the camera is completely virtual.


## 4. Why a kthread + hrtimer

Frame generation needs a context that can sleep and be woken while performing controlled per-frame processing.

### Sequential Operation

1. The capture kthread is implemented by `vcam_thread_fn`.
2. The kthread blocks in `TASK_INTERRUPTIBLE`.
3. The kthread uses `schedule()` rather than continuously polling.
4. The high-resolution timer is implemented by `vcam_frame_timer_fn`.
5. The timer is started by `vcam_start_streaming`.
6. The timer is cancelled by `vcam_stop_streaming`.
7. The timer fires every `VCAM_FRAME_PERIOD_NS`.
8. The configured frame period corresponds to 33 ms.
9. The timer calls `wake_up_process()` on the capture kthread.
10. `hrtimer_forward_now()` schedules the next timer event relative to the actual timer firing.
11. This prevents accumulated frame-to-frame timing drift.

This architecture replaced an earlier `msleep_interruptible(33)` loop.

The earlier loop was affected by `CONFIG_HZ=250` jiffy rounding.

The resulting design treats **frame pacing as a first-class design concern**, rather than treating sleep timing as an incidental implementation detail.



## 5. Processing Pipeline Internals

### 5.1 Buffer Roles

#### `y_scratch`

1. Stores the extracted luma (Y) plane from the current YUYV frame.
2. Acts as read-only input for convolution and differencing.

#### `y_edges`

1. Stores the primary processing output.
2. Contains the Sobel result in Sobel mode.
3. Contains the motion result in motion mode.
4. Contains the blended result in combined mode.

#### `y_edges2`

1. Provides a secondary output plane.
2. Is used by combined mode.
3. Prevents motion processing from overwriting the Sobel result before blending.

#### `y_previous`

1. Stores the previous frame's luma plane.
2. Retains state across frames.
3. Provides the previous-frame input required for motion differencing.

#### `custom_image_buf`

1. Stores the complete custom YUYV frame.
2. The image is loaded once at probe time.
3. Loading can use firmware or the embedded fallback.

Separate scratch buffers are required because convolution and differencing read existing input while writing new output.

In-place processing would corrupt input data during computation.


### 5.2 Sobel Edge Detection

1. Sobel processing uses a fixed-point 3×3 convolution.
2. The horizontal gradient `Gx` is calculated.
3. The vertical gradient `Gy` is calculated.
4. The gradient magnitude is calculated as:



5. The design avoids `sqrt(Gx² + Gy²)`.
6. This avoids floating-point or integer square-root processing in the kernel path.
7. Border pixels are left at zero.
8. The reason is that a complete 3×3 neighbourhood is unavailable at the image boundary.
9. Handling clamped or wrapped border indexing was considered unnecessary for the 640×480 frame.



### 5.3 Motion Detection

1. The current luma value is compared with the corresponding previous-frame luma value.
2. The difference is calculated as:
3. The difference is compared against the configured threshold.
4. The thresholded result produces a binary motion output.
5. The first frame after `STREAMON` has no previous frame.
6. Therefore, the first frame is explicitly defined to output black rather than undefined data.


### 5.4 Combined Mode

1. Sobel processing writes its result into `y_edges`.
2. Motion processing writes its result into `y_edges2`.
3. Inline differencing logic is used for the motion calculation.
4. This avoids directly calling `vcam_motion_compute()` because that function always targets `y_edges`.
5. The Sobel and motion outputs are combined.
6. The combination uses a logical OR for each pixel.
7. The mode demonstrates available compute headroom under the hrtimer-paced pipeline.



## 6. Custom Image Loading


flowchart TD
    A["vcam_load_custom_image()"] --> B["request_firmware()"]
    B --> C{"Firmware available?"}
    C -->|"Yes"| D["Use firmware image"]
    C -->|"No"| E["Use embedded byte array"]
    E --> F["vcam_embedded_image"]
    D --> G["Custom image loaded"]
    F --> G


### Sequential Flow

1. `vcam_load_custom_image()` begins the custom-image loading process.
2. The driver first calls `request_firmware()`.
3. The intended firmware path is:


4. If firmware is available, the driver uses the firmware image.
5. If firmware loading fails, the driver uses the compiled-in fallback.
6. The fallback is stored in `vcam_embedded_image`.
7. The embedded image was generated using `xxd -i`.
8. The source image was converted to raw YUYV before embedding.
9. The driver logs which loading path succeeded.

This is an explicit architectural compromise rather than a hidden fallback.



## 7. Runtime Controls

All controls are implemented using the standard `v4l2_ctrl` framework.

### Control Sequence

1. The driver creates a `v4l2_ctrl_handler`.
2. Standard controls are created using `v4l2_ctrl_new_std()`.
3. Driver-specific controls are created using `v4l2_ctrl_new_custom()`.
4. Driver-specific controls use the private base `V4L2_CID_VCAM_BASE`.
5. The `brightness` reused CID maps to `pattern_speed`.
6. `pattern_speed` controls checkerboard animation speed.
7. The `sharpness` reused CID maps to `block_size`.
8. `block_size` controls checkerboard block size.
9. `Edge Detection Threshold` maps to `edge_threshold`.
10. `edge_threshold` provides the shared threshold for Sobel and motion processing.
11. `Processing Mode` maps to `proc_mode`.
12. `proc_mode = 0` selects raw mode.
13. `proc_mode = 1` selects Sobel mode.
14. `proc_mode = 2` selects motion mode.
15. `proc_mode = 3` selects custom mode.
16. `proc_mode = 4` selects combined mode.
17. `Image Source` maps to `source_mode`.
18. `source_mode = 0` selects checkerboard.
19. `source_mode = 1` selects custom image.
20. All controls are live-adjustable while streaming.
21. A control change from a second process must not disturb an active stream.



## 8. Media Controller Topology


flowchart LR
    A["vcam-sensor"] --> B["vcam-isp"]
    B --> C["Virtual Camera\n/dev/video0"]


### Sequential Topology

1. The driver creates the `vcam-sensor` entity.
2. The sensor exposes a source pad.
3. The source pad is linked to `vcam-isp`.
4. `vcam-isp` represents the image-processing entity.
5. The ISP provides the next source connection.
6. The ISP is linked to the Virtual Camera entity.
7. The Virtual Camera exposes `/dev/video0`.
8. The topology is registered through `media_device`, `v4l2_subdev`, and `media_create_pad_link`.
9. The topology can be verified with:

```bash
media-ctl -d /dev/media0 -p
```

10. Structurally, this mirrors the sensor → ISP → capture-node separation used by real ISP-based camera stacks.

### Important Scope Boundary

1. The Media Controller topology is real and registered.
2. The topology is not yet the actual frame-data path.
3. Actual frame data still flows through the direct internal function `virtual_fill_frame()`.
4. Full subdevice-to-subdevice streaming operations are not yet used.
5. Full `v4l2_subdev_video_ops` / pad-level streaming has not yet been implemented.
6. This was a deliberate scope decision for the current project stage.


## 9. Concurrency and Locking

### `vcam.lock`

1. `vcam.lock` is a mutex.
2. It belongs to the `video_device.lock` serialization domain.
3. Its purpose is to serialize ioctl calls.

### `vcam.queue_lock`

1. `vcam.queue_lock` is a mutex.
2. It belongs to the `vb2_queue.lock` serialization domain.
3. Its purpose is to serialize vb2 queue operations.

### `vcam.buf_lock`

1. `vcam.buf_lock` is a spinlock.
2. It protects `buf_list`.
3. `buf_list` contains pending buffers.
4. The list is shared between `vcam_buf_queue()` and the capture kthread.
5. The list is accessed using `spin_lock_irqsave()` for consistency.

### `vcam.timing_lock`

1. `vcam.timing_lock` is a spinlock.
2. It protects the rolling timing-sample arrays.
3. The kthread writes timing information during frame processing.
4. `perf_stats_show()` can read the timing information from another context.
5. Therefore, synchronization is required around the timing data.

### Concurrency Flow


flowchart TD
    A["Userspace ioctl / buffer queue"] --> B["video_device.lock"]
    B --> C["vb2_queue.lock"]
    C --> D["Pending buffer list"]
    D --> E["vcam.buf_lock"]
    E --> F["Capture kthread"]
    F --> G["Frame generation"]
    F --> H["Timing samples"]
    H --> I["vcam.timing_lock"]
    I --> J["perf_stats_show()"]


### Why Two Mutexes?

1. `video_device.lock` and `vb2_queue.lock` serve different serialization domains.
2. The V4L2 ioctl domain is separate from the vb2 queue-management domain.
3. Combining both domains into one lock can create unnecessary contention.
4. Therefore, the two mutexes are intentionally retained.



## 10. Known Architectural Boundaries

### Boundary 1 — Fixed Format

The driver supports only: 640 × 480
V4L2_PIX_FMT_YUYV
There is no `S_FMT` negotiation.

### Boundary 2 — Media Controller

The Media Controller topology is registered and verified, but it is not yet the actual frame-data path.

### Boundary 3 — Firmware Loading

`request_firmware()` is the documented intended path for custom images.

### Boundary 4 — WSL2 Limitation

`request_firmware()` is known not to succeed in the specific WSL2 development environment described by the project.

### Boundary 5 — Embedded Fallback

The embedded fallback keeps the custom-image feature functional in that environment.

### Boundary 6 — Single Driver Instance

The implementation uses:

```c
static struct virtual_camera vcam;
```

### Boundary 7 — Multiple Instances

Multiple simultaneous driver instances are not supported.

### Boundary 8 — Current Simplification

A single global instance is acceptable for the current synthetic platform device.

### Boundary 9 — Future Hardware Binding

A real multi-instance hardware driver would require per-device allocation.

A typical approach would use:

```c
platform_get_drvdata()
```

to retrieve the per-device driver state.
