#!/usr/bin/env python3

import copy
import os
import threading
import time

import av
import rospkg
import rospy
import tf2_ros
import yaml
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import CameraInfo, Image
from tf.transformations import quaternion_from_matrix


class GoProCameraNode:
    def __init__(self):
        self.stream_url = rospy.get_param("~stream_url", "udp://127.0.0.1:8554")
        self.frame_id = rospy.get_param("~frame_id", "gopro_color_optical_frame")
        self.max_publish_rate = rospy.get_param("~max_publish_rate", 60.0)
        self.body_frame_id = rospy.get_param("~body_frame_id", "gopro_body")
        self.camera_config_path = rospy.get_param("~camera_config", self._default_camera_config_path())
        self.publish_camera_info = rospy.get_param("~publish_camera_info", True)
        self.publish_static_tf = rospy.get_param("~publish_static_tf", True)

        self.publisher = rospy.Publisher("/gopro_cam/color/image_raw", Image, queue_size=10)
        self.camera_info_pub = (
            rospy.Publisher("/gopro_cam/color/camera_info", CameraInfo, queue_size=10)
            if self.publish_camera_info
            else None
        )
        self.bridge = CvBridge()
        self.static_broadcaster = tf2_ros.StaticTransformBroadcaster() if self.publish_static_tf else None
        self.camera_info_template = None

        self._frame_lock = threading.Lock()
        self._latest_frame = None
        self._running = True

        self._load_camera_calibration()

        self._decode_thread = threading.Thread(target=self._decode_loop, daemon=True)
        self._decode_thread.start()

        rospy.on_shutdown(self.shutdown)
        rospy.loginfo("GoPro camera node streaming from %s", self.stream_url)

    def _decode_loop(self):
        options = {
            "fflags": "nobuffer",
            "flags": "low_delay",
            "framedrop": "",
            "probesize": "32",
            "analyzeduration": "0",
        }

        while self._running and not rospy.is_shutdown():
            container = None
            try:
                container = av.open(self.stream_url, options=options)
                for frame in container.decode(video=0):
                    if not self._running or rospy.is_shutdown():
                        break

                    image = frame.to_ndarray(format="bgr24")
                    with self._frame_lock:
                        self._latest_frame = image
            except av.AVError as exc:
                if not rospy.is_shutdown():
                    rospy.logwarn("GoPro decode error: %s", exc)
                    time.sleep(1.0)
            except Exception as exc:
                if not rospy.is_shutdown():
                    rospy.logerr("Unexpected decode error: %s", exc)
                    time.sleep(1.0)
            finally:
                if container is not None:
                    container.close()

        rospy.loginfo("Decode thread stopped")

    def publish_frames(self):
        rate = rospy.Rate(self.max_publish_rate) if self.max_publish_rate > 0 else None
        while not rospy.is_shutdown():
            frame = None
            with self._frame_lock:
                if self._latest_frame is not None:
                    frame = self._latest_frame.copy()
                    self._latest_frame = None

            if frame is not None:
                msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                msg.header.stamp = rospy.Time.now()
                msg.header.frame_id = self.frame_id
                self.publisher.publish(msg)
                self._publish_camera_info(msg)

            if rate is not None:
                rate.sleep()
            else:
                rospy.sleep(0.001)

    def shutdown(self):
        if not self._running:
            return

        self._running = False
        if self._decode_thread.is_alive():
            self._decode_thread.join(timeout=2.0)

    def _default_camera_config_path(self):
        try:
            package_path = rospkg.RosPack().get_path("gopro_camera_ros1")
            return os.path.join(package_path, "config", "gopro_kannalabrandt8.yaml")
        except rospkg.ResourceNotFound:
            local_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
            return os.path.join(local_dir, "config", "gopro_kannalabrandt8.yaml")

    def _load_camera_calibration(self):
        if not self.camera_config_path:
            rospy.logwarn("No camera calibration file provided; skipping camera_info and static tf")
            return

        if not os.path.exists(self.camera_config_path):
            rospy.logwarn("Camera calibration file %s not found", self.camera_config_path)
            return

        try:
            with open(self.camera_config_path, "r") as config_file:
                raw_data = yaml.safe_load(config_file)
        except Exception as exc:
            rospy.logwarn("Failed to load camera calibration from %s: %s", self.camera_config_path, exc)
            return

        camera_cfg = raw_data.get("camera", raw_data)
        if not isinstance(camera_cfg, dict):
            rospy.logwarn("Camera calibration format invalid in %s", self.camera_config_path)
            return

        if self.publish_camera_info:
            self.camera_info_template = self._build_camera_info(camera_cfg)
            if self.camera_info_template is not None:
                rospy.loginfo("Loaded camera intrinsics from %s", self.camera_config_path)

        if self.publish_static_tf and self.static_broadcaster is not None:
            transform = self._build_static_transform(camera_cfg.get("Tbc"))
            if transform is not None:
                self.static_broadcaster.sendTransform(transform)
                rospy.loginfo(
                    "Published static transform from %s to %s",
                    transform.header.frame_id,
                    transform.child_frame_id,
                )

    def _build_camera_info(self, config):
        intrinsics = config.get("intrinsics", {})
        fx = intrinsics.get("fx")
        fy = intrinsics.get("fy")
        cx = intrinsics.get("cx")
        cy = intrinsics.get("cy")
        required = [fx, fy, cx, cy]
        if any(value is None for value in required):
            rospy.logwarn("Incomplete intrinsics in camera config; camera_info disabled")
            return None

        camera_info = CameraInfo()
        camera_info.width = int(config.get("width", 0))
        camera_info.height = int(config.get("height", 0))
        camera_info.distortion_model = config.get("distortion_model", "plumb_bob")

        distortion_coeffs = config.get("distortion_coeffs", [])
        camera_info.D = list(distortion_coeffs)

        camera_info.K = [
            float(fx),
            0.0,
            float(cx),
            0.0,
            float(fy),
            float(cy),
            0.0,
            0.0,
            1.0,
        ]

        camera_info.P = [
            float(fx),
            0.0,
            float(cx),
            0.0,
            0.0,
            float(fy),
            float(cy),
            0.0,
            0.0,
            0.0,
            1.0,
            0.0,
        ]

        camera_info.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        camera_info.header.frame_id = self.frame_id
        return camera_info

    def _build_static_transform(self, tbc_config):
        if tbc_config is None:
            return None

        rows = tbc_config.get("rows")
        cols = tbc_config.get("cols")
        data = tbc_config.get("data", [])
        if rows != 4 or cols != 4 or len(data) != 16:
            rospy.logwarn("Tbc configuration must describe a 4x4 matrix")
            return None

        matrix = [data[i * cols : (i + 1) * cols] for i in range(rows)]
        transform = TransformStamped()
        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = self.body_frame_id
        transform.child_frame_id = self.frame_id
        transform.transform.translation.x = float(matrix[0][3])
        transform.transform.translation.y = float(matrix[1][3])
        transform.transform.translation.z = float(matrix[2][3])

        quaternion = quaternion_from_matrix(matrix)
        transform.transform.rotation.x = float(quaternion[0])
        transform.transform.rotation.y = float(quaternion[1])
        transform.transform.rotation.z = float(quaternion[2])
        transform.transform.rotation.w = float(quaternion[3])
        return transform

    def _publish_camera_info(self, image_msg):
        if self.camera_info_template is None or self.camera_info_pub is None:
            return

        camera_info = copy.deepcopy(self.camera_info_template)
        camera_info.header.stamp = image_msg.header.stamp
        camera_info.header.frame_id = image_msg.header.frame_id
        self.camera_info_pub.publish(camera_info)


def main():
    rospy.init_node("gopro_camera_node", anonymous=False)
    node = GoProCameraNode()
    try:
        node.publish_frames()
    except rospy.ROSInterruptException:
        pass
    finally:
        node.shutdown()


if __name__ == "__main__":
    main()
