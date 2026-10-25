# RealSense D405: startup and compressed images

Use a terminal on Mocha's local desktop (**Ctrl+Alt+T**). The terminal loads
`~/robotics_setup/env.sh` through `.bashrc`. Connect the D405 to a USB 3 port.

## Launch

```bash
ros2 launch realsense2_camera rs_launch.py \
  enable_color:=true \
  enable_depth:=true \
  depth_module.depth_profile:=640x480x30 \
  depth_module.color_profile:=640x480x30
```

This requests color and depth at 640 × 480, 30 frames per second. D405 color
uses `depth_module.color_profile`, not `rgb_camera.color_profile`.
Leave the launch terminal running.

## Compressed topics

The image-transport compression plugins are already installed:
`compressed_image_transport`, `compressed_depth_image_transport`, and
`image_transport_plugins`. No separate compression launch argument is needed.

| Stream | Topic | Format |
| --- | --- | --- |
| Color | `/camera/camera/color/image_raw/compressed` | JPEG |
| Depth | `/camera/camera/depth/image_rect_raw/compressedDepth` | Lossless compressed 16-bit depth |

Both topics use `sensor_msgs/msg/CompressedImage`. Subscribe to these topics,
or choose the corresponding `compressed` / `compressedDepth` transport in an
image-transport client. Encoding is performed when compressed subscribers are
present. Raw image topics remain available to subscribers that need them.

Use `compressedDepth` for depth values; it has its own transport header and
should be decoded with the compressed-depth transport, rather than treated
as an ordinary JPEG color image.

From a second local terminal, inspect the available topics:

```bash
ros2 topic list | grep -E 'compressed|compressedDepth'
```

## Stop

Press **Ctrl+C** in the camera launch terminal and wait for shutdown before
opening another program that needs the camera, such as RealSense Viewer.

## Verified installation

RealSense SDK 2.59.1 and ROS driver 4.59.1. A hardware check received and decoded
30 compressed color frames and 30 compressed depth frames at 640 × 480;
decoded depth retained `uint16` values. Evidence:
`~/robotics_setup/validation/realsense-compressed.json`.

Keep this guide local and uncommitted.
