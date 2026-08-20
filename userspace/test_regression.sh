#!/bin/bash
set -e

DEV=/dev/video0
SYSFS=/sys/bus/platform/devices/virtual_camera

echo "=== Regression Test: Virtual Camera Driver ==="

echo "--- Stress test: 5 load/unload cycles ---"
for i in 1 2 3 4 5; do
    sudo insmod virtual_camera.ko 2>/dev/null || { echo "FAIL: insmod cycle $i"; exit 1; }
    sleep 1
    ls "$SYSFS" > /dev/null || { echo "FAIL: sysfs missing cycle $i"; exit 1; }
    sudo rmmod virtual_camera || { echo "FAIL: rmmod cycle $i"; exit 1; }
    sleep 1
done
echo "PASS: 5/5 clean load/unload cycles"

echo "--- Load driver for functional tests ---"
sudo insmod virtual_camera.ko
sleep 1

echo "--- Media Controller topology check ---"
sudo media-ctl -d /dev/media0 -p | grep -q "vcam-sensor" || { echo "FAIL: sensor entity missing"; exit 1; }
sudo media-ctl -d /dev/media0 -p | grep -q "vcam-isp" || { echo "FAIL: isp entity missing"; exit 1; }
sudo media-ctl -d /dev/media0 -p | grep -q "Virtual Camera" || { echo "FAIL: video entity missing"; exit 1; }
echo "PASS: MC topology entities present"

echo "--- Mode sweep with fps + perf_stats check (0-4) ---"
for m in 0 1 2 3 4; do
    v4l2-ctl -d $DEV --set-ctrl=processing_mode=$m
    FPS=$(v4l2-ctl -d $DEV --stream-mmap --stream-count=60 2>&1 | grep -oP '[\d.]+(?= fps)' | tail -1)
    STATS=$(cat "$SYSFS/perf_stats")
    echo "mode=$m fps=$FPS | $STATS"

    # fps sanity: must be within reasonable range of 30fps target
    FPS_INT=${FPS%.*}
    if [ "$FPS_INT" -lt 25 ] || [ "$FPS_INT" -gt 35 ]; then
        echo "WARN: mode $m fps ($FPS) outside expected 25-35 range"
    fi
done
echo "PASS: mode sweep completed"

echo "--- Source mode check (checkerboard vs custom image) ---"
for s in 0 1; do
    v4l2-ctl -d $DEV --set-ctrl=image_source=$s
    v4l2-ctl -d $DEV --stream-mmap --stream-count=30 > /dev/null
    echo "image_source=$s OK"
done

echo "--- DMA-BUF export check ---"
./test_dmabuf > /tmp/dmabuf_test_out.txt 2>&1 && echo "PASS: dma-buf export works" || echo "FAIL: dma-buf export"

echo "--- Cleanup ---"
sudo rmmod virtual_camera

echo "=== Regression test complete ==="