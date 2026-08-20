# V4L2 Virtual Camera Linux Kernel Driver

A software-only **V4L2 virtual camera driver** implemented as an out-of-tree Linux kernel module. The driver creates a virtual video capture device, generates frames in kernel space, performs configurable image processing, and exposes the resulting stream through the Linux V4L2 framework.

## Environment

* **Host:** Windows 11
* **Development environment:** WSL2
* **Distribution:** Ubuntu 24.04 LTS
* **Kernel:** 6.6.87.2-microsoft-standard-WSL2+
* **Language:** C
* **Frameworks:** V4L2, videobuf2, Media Controller
* **Resolution:** 640 × 480
* **Pixel format:** YUYV 4:2:2
* **Target frame rate:** ~30 FPS

## Features

* V4L2 capture device exposed as `/dev/video0`
* `videobuf2` based buffer management
* Kernel-thread based frame generation
* Runtime-selectable processing modes:

  * Raw/passthrough
  * Sobel edge detection
  * Frame-difference motion detection
  * Custom image
  * Sobel + motion
* Animated checkerboard and custom-image sources
* Custom V4L2 controls
* Custom image loading through `request_firmware()`
* Compile-time embedded-image fallback for the tested WSL2 environment
* DMA-BUF buffer export
* Kernel-side performance instrumentation
* `perf_stats` sysfs interface
* High-resolution timer based frame pacing
* Media Controller topology
* Automated regression testing

## Image Processing

Sobel processing operates on the extracted luminance plane using the approximation:

```text
|Gx| + |Gy|
```

instead of calculating the square-root gradient magnitude.

Motion detection compares the current luminance data against the previous frame.

Separate scratch buffers are used for intermediate processing to avoid overwriting data required by subsequent operations.

## Performance

The initial frame-generation loop used `msleep_interruptible(33)` and produced approximately **22.7 FPS**.

Timing measurements showed that this limit was largely independent of processing complexity. The frame-pacing mechanism was therefore changed to an `hrtimer` using `CLOCK_MONOTONIC` and `hrtimer_forward_now()`.

The measured frame rate increased to approximately **30.3 FPS**.

| Mode           | Approx. compute time |
| -------------- | -------------------: |
| Raw            |                 0 µs |
| Sobel          |              1400 µs |
| Motion         |               400 µs |
| Custom image   |                 0 µs |
| Sobel + motion |              2200 µs |

## DMA-BUF

The driver supports buffer export through `VIDIOC_EXPBUF`.

A separate userspace C test program was used to export a V4L2 buffer, map the resulting DMA-BUF file descriptor, and verify the buffer contents independently of the driver's normal V4L2 `mmap()` path.

## Media Controller

The driver registers a three-entity Media Controller topology consisting of:

* Virtual sensor subdevice
* Image-signal-processor subdevice
* Video capture node

The topology can be inspected with:

```bash
media-ctl -p
```

The current implementation registers the Media Controller architecture, while frame data still uses direct internal processing calls rather than a complete subdevice-to-subdevice streaming path.

## Testing

Basic device inspection:

```bash
v4l2-ctl --all -d /dev/video0
```

List supported formats:

```bash
v4l2-ctl --list-formats-ext -d /dev/video0
```

List controls:

```bash
v4l2-ctl --list-ctrls -d /dev/video0
```

Test streaming:

```bash
v4l2-ctl --stream-mmap --stream-count=300 -d /dev/video0
```

The driver can also be tested with:

```bash
ffplay /dev/video0
```

Kernel messages can be inspected using:

```bash
dmesg
```

An automated regression script covers module loading/unloading, V4L2 registration, Media Controller topology, processing modes, image sources, streaming, and DMA-BUF export.

## Build

The driver is built as an out-of-tree kernel module against the prepared WSL2 kernel source.

```bash
make
```

Load the module:

```bash
sudo insmod <module>.ko
```

Verify the device:

```bash
ls -l /dev/video0
```

Unload:

```bash
sudo rmmod <module>
```

The exact module name depends on the repository Makefile.

## Current Limitations

* Resolution and pixel format are fixed at 640 × 480 YUYV.
* Dynamic format negotiation is not implemented.
* The Media Controller topology does not yet provide a complete subdevice-based data path.
* `request_firmware()` does not currently succeed in the tested WSL2 environment; the embedded image is used as a fallback.
* A minor `perf_stats` reporting inconsistency can occur during an immediate processing-mode transition.

## Project Status

The implemented driver has been verified for V4L2 registration, streaming, all five processing modes, both image sources, DMA-BUF export, performance instrumentation, Media Controller registration, and automated regression testing.

The final measured streaming performance is approximately **30.3 FPS** on the documented WSL2 environment.

For the detailed development history, debugging process, root causes, and corrections, refer to the accompanying technical documentation.
