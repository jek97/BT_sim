#!/usr/bin/env python3
"""Render the behaviour tree's own live status stream (bt_runner's
BTVisualizationLogger, amiga_ros2_behavior_tree) into a video, AND log
every raw event to a JSON-lines file regardless of whether rendering
keeps up -- visual:=true's "record the BT execution too" feature (see
gazebo.launch.py's own video_path docstring for the camera half of that
flag).

Deliberately a separate process/node from bt_runner, subscribing to a
plain ROS2 topic rather than rendering inline: the actual work here
(shelling out to Graphviz, encoding video frames) is much heavier than
one status-change callback should ever cost inside a 50 Hz tick loop.
Splitting it out means this node falling behind, stalling, or even
crashing can never slow down or block the tree's own execution -- at
worst this misses frames.

Two outputs, from the SAME event stream, satisfying both halves of what
was asked for:
  * `video_path` (an .mp4) -- rendered periodically (`render_rate`) from
    whatever the current known tree structure + per-node status is, via
    a hand-built Graphviz DOT graph.
  * `bag_path` (a .jsonl) -- every structure/event message, one JSON
    object per line, appended immediately as it arrives, independent of
    the render timer. This is the "save something similar to a bag file"
    half: replay_bt_bag() below rebuilds the exact same video from this
    file alone, with no simulation, no bt_runner, and no ROS graph
    running at all -- see its own docstring for how, and for why a video
    produced this way is frame-for-frame identical to one recorded live.
"""
import json
import os
import subprocess
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String

# BT.CPP NodeStatus -> (fill color, text color), matching Groot2's own
# convention closely enough to read at a glance: green/red/orange for the
# three outcomes ticking actually produces, gray for "not part of this
# tick's active path" (IDLE) or "gated off by a precondition" (SKIPPED).
STATUS_COLORS = {
    "IDLE": ("#e0e0e0", "#333333"),
    "RUNNING": ("#ffb703", "#000000"),
    "SUCCESS": ("#4caf50", "#ffffff"),
    "FAILURE": ("#e53935", "#ffffff"),
    "SKIPPED": ("#bdbdbd", "#333333"),
}


def _dot_escape(text: str) -> str:
    return text.replace("\\", "\\\\").replace('"', '\\"')


def render_tree_png(nodes: dict, statuses: dict, width: int, height: int) -> np.ndarray:
    """nodes: uid -> {"name", "registration_name", "node_type", "children"}.
    statuses: uid -> BT.CPP status name (defaults to "IDLE" if absent).

    Returns a BGR image of exactly (height, width), letterboxed if
    Graphviz's own natural layout size doesn't match the target aspect
    ratio -- the video's frame size must never change between calls.
    """
    lines = ["digraph bt {", 'rankdir="TB";', 'node [shape=box, style=filled, fontname="Helvetica"];']
    for uid, info in nodes.items():
        # Escape each piece BEFORE joining with the literal "\n" -- DOT's
        # own line-break escape, not a real newline character. Escaping
        # the joined string afterwards would double the backslash in
        # that "\n" too, printing the literal two characters "\n" in the
        # label instead of breaking the line (caught by actually looking
        # at a rendered frame, not just reading the DOT text).
        label = _dot_escape(info["name"])
        if info["name"] != info["registration_name"]:
            label += f"\\n({_dot_escape(info['registration_name'])})"
        status = statuses.get(uid, "IDLE")
        fill, text_color = STATUS_COLORS.get(status, STATUS_COLORS["IDLE"])
        lines.append(
            f'  n{uid} [label="{label}", fillcolor="{fill}", '
            f'fontcolor="{text_color}"];'
        )
        for child_uid in info["children"]:
            lines.append(f"  n{uid} -> n{child_uid};")
    lines.append("}")
    dot_text = "\n".join(lines)

    result = subprocess.run(
        ["dot", "-Tpng"], input=dot_text.encode(), capture_output=True, check=False
    )
    canvas = np.full((height, width, 3), 32, dtype=np.uint8)
    if result.returncode != 0 or not result.stdout:
        return canvas

    png = cv2.imdecode(np.frombuffer(result.stdout, np.uint8), cv2.IMREAD_COLOR)
    if png is None:
        return canvas

    # Letterbox: scale to fit inside (width, height) keeping aspect ratio,
    # center on the fixed canvas -- Graphviz's own output size varies with
    # tree shape, but a VideoWriter's frame size cannot vary between calls.
    scale = min(width / png.shape[1], height / png.shape[0])
    new_w, new_h = max(1, int(png.shape[1] * scale)), max(1, int(png.shape[0] * scale))
    resized = cv2.resize(png, (new_w, new_h))
    x_off, y_off = (width - new_w) // 2, (height - new_h) // 2
    canvas[y_off : y_off + new_h, x_off : x_off + new_w] = resized
    return canvas


class BTVisualizerNode(Node):
    def __init__(self):
        super().__init__("bt_visualizer_node")
        self.declare_parameter("viz_topic", "/bt/visualization")
        self.declare_parameter("video_path", "")
        self.declare_parameter("bag_path", "")
        self.declare_parameter("render_rate", 2.0)
        self.declare_parameter("image_width", 1280)
        self.declare_parameter("image_height", 720)

        video_path = self.get_parameter("video_path").value
        bag_path = self.get_parameter("bag_path").value
        if not video_path and not bag_path:
            self.get_logger().fatal(
                "bt_visualizer_node needs at least one of 'video_path' or "
                "'bag_path' set -- nothing to do otherwise."
            )
            raise SystemExit(1)

        self.video_path = video_path
        self.width = self.get_parameter("image_width").value
        self.height = self.get_parameter("image_height").value
        self.writer = None

        self.bag_file = None
        if bag_path:
            os.makedirs(os.path.dirname(os.path.abspath(bag_path)), exist_ok=True)
            self.bag_file = open(bag_path, "a")
            self.get_logger().info(f"Logging BT events to {bag_path}")
        if video_path:
            os.makedirs(os.path.dirname(os.path.abspath(video_path)), exist_ok=True)
            self.get_logger().info(f"Recording BT visualization to {video_path}")

        self.nodes = {}
        self.statuses = {}

        qos = QoSProfile(depth=200, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(
            String, self.get_parameter("viz_topic").value, self._on_message, qos
        )

        if video_path:
            rate = self.get_parameter("render_rate").value
            self.create_timer(1.0 / rate, self._render_frame)

    def _on_message(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn(f"Ignoring malformed BT visualization message: {msg.data!r}")
            return

        if self.bag_file is not None:
            record = dict(payload)
            record["_received_wall_time"] = time.time()
            self.bag_file.write(json.dumps(record) + "\n")
            self.bag_file.flush()

        self._apply(payload)

    def _apply(self, payload: dict):
        """Shared with replay_bt_bag() below -- one event/structure
        message updates (self.nodes, self.statuses) the same way whether
        it arrived live over ROS2 or is being replayed from a bag line.
        """
        kind = payload.get("kind")
        if kind == "structure":
            # A fresh mission's tree replaces whatever the previous one
            # left on screen -- same one-video-per-launch-session
            # lifecycle as video_recorder_node.py's own camera feed, just
            # with the displayed graph cut over cleanly at each new
            # mission's first structure message instead of starting a new
            # file.
            self.nodes = {n["uid"]: n for n in payload["nodes"]}
            self.statuses = {uid: "IDLE" for uid in self.nodes}
        elif kind == "event":
            self.statuses[payload["uid"]] = payload["status"]

    def _render_frame(self):
        if not self.nodes:
            return
        frame = render_tree_png(self.nodes, self.statuses, self.width, self.height)
        if self.writer is None:
            fourcc = cv2.VideoWriter_fourcc(*"mp4v")
            rate = self.get_parameter("render_rate").value
            self.writer = cv2.VideoWriter(self.video_path, fourcc, rate, (self.width, self.height))
            if not self.writer.isOpened():
                self.get_logger().fatal(f"Could not open a video writer for {self.video_path}")
                raise SystemExit(1)
        self.writer.write(frame)

    def destroy_node(self):
        if self.writer is not None:
            self.writer.release()
        if self.bag_file is not None:
            self.bag_file.close()
        super().destroy_node()


def replay_bt_bag(bag_path: str, video_path: str, fps: float = 2.0,
                  width: int = 1280, height: int = 720):
    """Rebuild a BT visualization video from a .jsonl file produced by
    BTVisualizerNode, with no ROS graph, no bt_runner, and no simulation
    running at all -- the "obtain the visualization later" half of
    visual:=true's BT recording feature.

    One rendered frame per bag line (not wall-clock-paced replay): the
    bag already IS the full-resolution sequence of every status change
    in order, so a video built one frame per line is exactly as detailed
    as a live recording would have been at an infinite render_rate --
    strictly more faithful than re-deriving timing from
    _received_wall_time, which only ever reflects how fast this node
    happened to process ROS callbacks, not the tree's own tick timing.

    Usage: python3 bt_visualizer_node.py --replay events.jsonl output.mp4
    """
    nodes, statuses = {}, {}
    writer = None
    with open(bag_path) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            payload = json.loads(line)
            kind = payload.get("kind")
            if kind == "structure":
                nodes = {n["uid"]: n for n in payload["nodes"]}
                statuses = {uid: "IDLE" for uid in nodes}
            elif kind == "event":
                statuses[payload["uid"]] = payload["status"]
            else:
                continue
            if not nodes:
                continue
            frame = render_tree_png(nodes, statuses, width, height)
            if writer is None:
                os.makedirs(os.path.dirname(os.path.abspath(video_path)), exist_ok=True)
                fourcc = cv2.VideoWriter_fourcc(*"mp4v")
                writer = cv2.VideoWriter(video_path, fourcc, fps, (width, height))
            writer.write(frame)
    if writer is not None:
        writer.release()
        print(f"Wrote {video_path} from {bag_path}")
    else:
        print(f"{bag_path} had no structure message -- nothing to render")


def main():
    import sys

    if len(sys.argv) >= 2 and sys.argv[1] == "--replay":
        if len(sys.argv) != 4:
            print("usage: bt_visualizer_node.py --replay <bag.jsonl> <output.mp4>")
            raise SystemExit(1)
        replay_bt_bag(sys.argv[2], sys.argv[3])
        return

    rclpy.init()
    node = BTVisualizerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
