#!/usr/bin/env python3
"""Subscribe to a sensor_msgs/Image topic and write every frame to an MP4
file via OpenCV -- the actual "save the simulation's camera feed to a
file" half of visual:=true (camera_trajectory_node.py only moves the
camera; this is what turns its image topic into a video on disk).

output_path has no default on purpose (see gazebo.launch.py's own
video_path launch argument): a recording silently landing at some
made-up default path is easy to lose track of, so this node refuses to
start without an explicit destination instead.
"""
import os

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


class VideoRecorderNode(Node):
    def __init__(self):
        super().__init__("video_recorder_node")
        self.declare_parameter("image_topic", "trajectory_camera/image")
        self.declare_parameter("output_path", "")
        self.declare_parameter("fps", 30.0)

        output_path = self.get_parameter("output_path").value
        if not output_path:
            self.get_logger().fatal(
                "video_recorder_node requires 'output_path' -- pass "
                "video_path:=/some/file.mp4 to sim_bringup.launch.py."
            )
            raise SystemExit(1)
        self.output_path = output_path
        out_dir = os.path.dirname(os.path.abspath(output_path))
        os.makedirs(out_dir, exist_ok=True)

        self.fps = self.get_parameter("fps").value
        self.bridge = CvBridge()
        self.writer = None
        self.frame_count = 0

        self.create_subscription(
            Image, self.get_parameter("image_topic").value, self._on_image, 10
        )
        self.get_logger().info(f"Recording to {self.output_path}")

    def _on_image(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        if self.writer is None:
            height, width = frame.shape[:2]
            fourcc = cv2.VideoWriter_fourcc(*"mp4v")
            self.writer = cv2.VideoWriter(
                self.output_path, fourcc, self.fps, (width, height)
            )
            if not self.writer.isOpened():
                self.get_logger().fatal(
                    f"Could not open a video writer for {self.output_path}"
                )
                raise SystemExit(1)
        self.writer.write(frame)
        self.frame_count += 1

    def destroy_node(self):
        if self.writer is not None:
            self.writer.release()
            self.get_logger().info(
                f"Wrote {self.frame_count} frames to {self.output_path}"
            )
        super().destroy_node()


def main():
    rclpy.init()
    node = VideoRecorderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
