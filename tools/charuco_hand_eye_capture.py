#!/usr/bin/env python3
"""Save ChArUco images paired with timestamped robot poses on keypress."""

import argparse
import json
import sys
import threading
from pathlib import Path

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformException, TransformListener

from charuco_calibrate_ros import detect_charuco, make_board


class CaptureNode(Node):
    def __init__(self, image_topic, info_topic):
        super().__init__("charuco_hand_eye_capture")
        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.frame = None
        self.camera_info = None
        self.create_subscription(Image, image_topic, self.on_image, 1)
        self.create_subscription(CameraInfo, info_topic, self.on_info, 1)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def on_image(self, message):
        try:
            image = self.bridge.imgmsg_to_cv2(message, desired_encoding="mono8")
        except Exception as error:
            self.get_logger().warning(f"Could not convert image: {error}")
            return
        with self.lock:
            self.frame = image.copy(), message.header

    def on_info(self, message):
        with self.lock:
            self.camera_info = message

    def latest(self):
        with self.lock:
            if self.frame is None:
                return None, None, None
            image, header = self.frame
            return image.copy(), header, self.camera_info


def next_index(directory):
    index = 1
    while any((directory / f"sample_{index:04d}{suffix}").exists()
              for suffix in (".png", ".json")):
        index += 1
    return index


def spin(executor):
    try:
        executor.spin()
    except ExternalShutdownException:
        pass


def save_sample(directory, index, image, header, camera_info, transform, dictionary):
    image_path = directory / f"sample_{index:04d}.png"
    metadata_path = directory / f"sample_{index:04d}.json"
    if not cv2.imwrite(str(image_path), image):
        raise OSError(f"Could not save {image_path}")
    data = {
        "image": image_path.name,
        "stamp": {"sec": header.stamp.sec, "nanosec": header.stamp.nanosec},
        "camera_frame": header.frame_id,
        "dictionary": dictionary,
        "board": {"columns": 24, "rows": 17, "square_size_m": 0.015,
                  "marker_size_m": 0.011, "legacy": True},
        "camera_info": {
            "width": camera_info.width, "height": camera_info.height,
            "distortion_model": camera_info.distortion_model,
            "k": list(camera_info.k), "d": list(camera_info.d),
        },
        "base_to_flange": {
            "parent": transform.header.frame_id,
            "child": transform.child_frame_id,
            "translation_m": [transform.transform.translation.x,
                              transform.transform.translation.y,
                              transform.transform.translation.z],
            "rotation_xyzw": [transform.transform.rotation.x,
                              transform.transform.rotation.y,
                              transform.transform.rotation.z,
                              transform.transform.rotation.w],
        },
    }
    try:
        with metadata_path.open("w", encoding="utf-8") as output:
            json.dump(data, output, indent=2)
            output.write("\n")
    except OSError:
        image_path.unlink()
        raise


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--image-topic", default="/my_camera/pylon_ros2_camera_node/image_raw")
    parser.add_argument("--info-topic", default="/my_camera/pylon_ros2_camera_node/camera_info")
    parser.add_argument("--base-frame", default="base_link")
    parser.add_argument("--tool-frame", default="flange")
    parser.add_argument("--output-dir", default="/data/hand_eye")
    args = parser.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])
    if not hasattr(cv2, "aruco"):
        raise RuntimeError("OpenCV contrib with ChArUco support is required")
    dictionaries = [
        (name, *make_board(24, 17, 0.015, 0.011, True, dictionary_id))
        for name, dictionary_id in (
            ("DICT_5X5_250", cv2.aruco.DICT_5X5_250),
            ("DICT_5X5_1000", cv2.aruco.DICT_5X5_1000),
        )
    ]
    directory = Path(args.output_dir)
    directory.mkdir(parents=True, exist_ok=True)
    index = next_index(directory)

    rclpy.init(args=None)
    node = CaptureNode(args.image_topic, args.info_topic)
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=spin, args=(executor,), daemon=True)
    spin_thread.start()
    last_stamp = None
    print(f"Listening on {args.image_topic}; saving to {directory}", flush=True)
    print("Stop the robot: s = save image and pose, q = quit", flush=True)
    try:
        cv2.namedWindow("Hand-eye capture", cv2.WINDOW_NORMAL)
        while rclpy.ok():
            image, header, camera_info = node.latest()
            if image is None:
                if cv2.waitKey(30) & 0xFF in (ord("q"), 27):
                    break
                continue
            detections = []
            for name, dictionary, board in dictionaries:
                corners, ids, marker_corners, marker_ids = detect_charuco(
                    image, dictionary, board
                )
                detections.append((0 if ids is None else len(ids), name, corners,
                                   ids, marker_corners, marker_ids))
            count, name, corners, ids, marker_corners, marker_ids = max(detections)
            preview = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
            if marker_ids is not None:
                cv2.aruco.drawDetectedMarkers(preview, marker_corners, marker_ids)
            if ids is not None:
                cv2.aruco.drawDetectedCornersCharuco(preview, corners, ids)
            cv2.putText(preview, f"{name}: {count} corners | next: {index:04d}",
                        (15, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 220, 0), 2)
            cv2.putText(preview, "s: save    q: quit", (15, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 220, 0), 2)
            cv2.imshow("Hand-eye capture", preview)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
            if key != ord("s"):
                continue
            stamp = (header.stamp.sec, header.stamp.nanosec)
            if count < 6 or camera_info is None or not camera_info.k[0]:
                print("Need >=6 board corners and calibrated camera_info; not saved.", flush=True)
                continue
            if (camera_info.width, camera_info.height) != (image.shape[1], image.shape[0]):
                print("camera_info resolution does not match raw image; not saved.", flush=True)
                continue
            if stamp == (0, 0) or stamp == last_stamp:
                print("No new timestamped image; not saved.", flush=True)
                continue
            age = (node.get_clock().now() - Time.from_msg(header.stamp)).nanoseconds
            if not 0 <= age <= 1_000_000_000:
                print("Image is older than 1 s or clocks disagree; not saved.", flush=True)
                continue
            try:
                transform = node.tf_buffer.lookup_transform(
                    args.base_frame, args.tool_frame, Time.from_msg(header.stamp),
                    timeout=Duration(seconds=0.5),
                )
            except TransformException as error:
                print(f"No robot TF at image time: {error}; not saved.", flush=True)
                continue
            save_sample(directory, index, image, header, camera_info, transform, name)
            last_stamp = stamp
            print(f"Saved sample_{index:04d}.png + .json", flush=True)
            index = next_index(directory)
    finally:
        cv2.destroyAllWindows()
        executor.shutdown()
        spin_thread.join(timeout=2.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()