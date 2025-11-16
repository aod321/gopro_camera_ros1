# GoPro Camera ROS1 Node

This package provides a ROS Noetic Python node that converts the GoPro preview stream into `sensor_msgs/Image` messages on `/gopro_cam/color/image_raw`, publishes synchronized camera info, and optionally broadcasts the calibrated static transform between the camera and the body frame.

## Dependencies
- Go Pro Preview Stream: https://github.com/aod321/gopro_as_webcam_on_linux 
- ROS Noetic with a catkin workspace
- `python3-numpy`, `python3-yaml`
- `ros-noetic-cv-bridge`, `ros-noetic-image-transport`, `ros-noetic-rospy`
- `ros-noetic-geometry-msgs`, `ros-noetic-tf2-ros`, `ros-noetic-tf`

Install the ROS dependencies with apt:

```bash
sudo apt install python3-numpy python3-yaml ros-noetic-cv-bridge ros-noetic-image-transport \
    ros-noetic-geometry-msgs ros-noetic-tf2-ros ros-noetic-tf
```

PyAV is most reliably installed via pip. First install the FFmpeg development headers, then install PyAV:

```bash
sudo apt install libavformat-dev libavdevice-dev libavfilter-dev libswresample-dev libswscale-dev libavcodec-dev libavutil-dev pkg-config python3-dev
pip3 install av
```

(Optional) If you prefer the Ubuntu package, enable the `universe` repository and install `python3-av` with apt, though availability depends on your base image:

```bash
sudo add-apt-repository universe && sudo apt update
sudo apt install python3-av
```

## Build

Place this repository under `catkin_ws/src` and build:

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

## Run

Start the GoPro stream, then launch the node to publish `/gopro_cam/color/image_raw` and `/gopro_cam/color/camera_info`:

```bash
rosrun gopro_camera_ros1 gopro_camera_node.py \
  _stream_url:=udp://127.0.0.1:8554 \
  _frame_id:=gopro_color_optical_frame \
  _body_frame_id:=gopro_body \
  _camera_config:=$(rospack find gopro_camera_ros1)/config/gopro_kannalabrandt8.yaml \
  _max_publish_rate:=60
```

- `stream_url`: Optional, defaults to `udp://127.0.0.1:8554`
- `frame_id`: Frame ID stored in the published image header
- `max_publish_rate`: Sleep rate (Hz) while waiting for frames
- `body_frame_id`: Parent frame for the static transform derived from `Tbc`
- `camera_config`: YAML file containing intrinsics/distortion/extrinsics
- `publish_camera_info` / `publish_static_tf`: Enable/disable auxiliary publishers

Use tools such as `rostopic echo /gopro_cam/color/image_raw`, `rviz`, or `rqt_image_view` to inspect the published images.

## Calibration & Static Transform

- The default calibration lives in `config/gopro_kannalabrandt8.yaml` and contains the Kannala-Brandt intrinsics, distortion coefficients, and the 4×4 `Tbc` matrix (body → camera). Override `~camera_config` with your own YAML if you re-calibrate.
- The node copies these values into the `/gopro_cam/color/camera_info` message for downstream rectification pipelines.
- If `~publish_static_tf` is true, the extrinsic matrix is converted into a static transform from `body_frame_id` to `frame_id` using `tf2_ros::StaticTransformBroadcaster`.
- GoPro cameras cannot stream IMU data live; only the static transform is published. For time-synchronized IMU information you must extract it offline from recorded files.
