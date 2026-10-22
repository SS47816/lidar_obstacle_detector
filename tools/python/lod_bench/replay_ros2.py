"""Replay a benchmark-format sequence into ROS 2 (PointCloud2 + odom->lidar TF).

Useful to try the ROS 2 node on AV2 / nuScenes data without converting bags:

    python replay_ros2.py --seq ~/datasets/lod_bench/nuscenes/scene-0061 --rate 1.0
    ros2 launch lidar_obstacle_detector obstacle_detector.launch.py points:=/points \
        tracking_frame:=odom

Messages are stamped with the current time; the TF is published alongside each cloud.
"""

import argparse
import json
import os
import time

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from tf2_ros import TransformBroadcaster


def rot_to_quat(r):
    w = np.sqrt(max(0.0, 1.0 + r[0, 0] + r[1, 1] + r[2, 2])) / 2.0
    x = np.copysign(np.sqrt(max(0.0, 1.0 + r[0, 0] - r[1, 1] - r[2, 2])) / 2.0, r[2, 1] - r[1, 2])
    y = np.copysign(np.sqrt(max(0.0, 1.0 - r[0, 0] + r[1, 1] - r[2, 2])) / 2.0, r[0, 2] - r[2, 0])
    z = np.copysign(np.sqrt(max(0.0, 1.0 - r[0, 0] - r[1, 1] + r[2, 2])) / 2.0, r[1, 0] - r[0, 1])
    return x, y, z, w


class Replayer(Node):
    def __init__(self, args):
        super().__init__("sequence_replay")
        self.args = args
        with open(os.path.join(args.seq, "frames.jsonl")) as f:
            self.frames = [json.loads(line) for line in f]
        self.pub = self.create_publisher(PointCloud2, args.topic, 5)
        self.tf = TransformBroadcaster(self)
        self.origin = np.asarray(self.frames[0]["pose"]).reshape(4, 4)[:3, 3].copy()

    def cloud_msg(self, idx, stamp):
        pts = np.fromfile(os.path.join(self.args.seq, "points", f"{idx:06d}.bin"), np.float32)
        msg = PointCloud2()
        msg.header.stamp = stamp
        msg.header.frame_id = self.args.frame
        msg.height, msg.width = 1, len(pts) // 4
        msg.fields = [
            PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
            for i, n in enumerate(("x", "y", "z", "intensity"))
        ]
        msg.is_bigendian, msg.point_step = False, 16
        msg.row_step, msg.is_dense = 16 * msg.width, True
        msg.data = pts.tobytes()
        return msg

    def tf_msg(self, pose, stamp):
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = "odom"
        t.child_frame_id = self.args.frame
        tr = pose[:3, 3] - self.origin
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = map(float, tr)
        q = rot_to_quat(pose[:3, :3])
        t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = q
        return t

    def run(self):
        t0_data, t0_wall = self.frames[0]["t"], time.monotonic()
        for k, fr in enumerate(self.frames):
            if self.args.max_frames and k >= self.args.max_frames:
                break
            delay = (fr["t"] - t0_data) / self.args.rate - (time.monotonic() - t0_wall)
            if delay > 0:
                time.sleep(delay)
            stamp = self.get_clock().now().to_msg()
            self.tf.sendTransform(self.tf_msg(np.asarray(fr["pose"]).reshape(4, 4), stamp))
            self.pub.publish(self.cloud_msg(fr["idx"], stamp))
            rclpy.spin_once(self, timeout_sec=0.0)
        self.get_logger().info(f"replayed {k + 1} frames")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seq", required=True)
    ap.add_argument("--topic", default="/points")
    ap.add_argument("--frame", default="lidar")
    ap.add_argument("--rate", type=float, default=1.0)
    ap.add_argument("--max-frames", type=int, default=0)
    args = ap.parse_args()
    args.seq = os.path.expanduser(args.seq)
    rclpy.init()
    node = Replayer(args)
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
