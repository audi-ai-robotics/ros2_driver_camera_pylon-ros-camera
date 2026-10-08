#!/usr/bin/env python3
"""Calibrate a ROS 2 camera from a calib.io ChArUco board."""

import argparse
import json
import sys
import threading
from pathlib import Path

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


class ImageSubscriber(Node):
    def __init__(self, topic):
        super().__init__("charuco_calibration")
        self.bridge = CvBridge()
        self.image = None
        self.lock = threading.Lock()
        self.create_subscription(Image, topic, self._image_callback, 1)

    def _image_callback(self, message):
        try:
            image = self.bridge.imgmsg_to_cv2(message, desired_encoding="mono8")
        except Exception as error:
            self.get_logger().warning(f"Could not convert image: {error}")
            return
        with self.lock:
            self.image = image

    def latest_image(self):
        with self.lock:
            return None if self.image is None else self.image.copy()


def make_board(columns, rows, square_size, marker_size, legacy, dictionary_id):
    aruco = cv2.aruco
    dictionary = aruco.getPredefinedDictionary(dictionary_id)
    if hasattr(aruco, "CharucoBoard_create"):
        board = aruco.CharucoBoard_create(
            columns, rows, square_size, marker_size, dictionary
        )
        if not legacy and not hasattr(board, "setLegacyPattern"):
            raise RuntimeError(
                "This OpenCV version only supports legacy ChArUco boards. "
                "Use --legacy or install OpenCV 4.7 or newer."
            )
    elif hasattr(aruco, "CharucoBoard"):
        board = aruco.CharucoBoard(
            (columns, rows), square_size, marker_size, dictionary
        )
        setter = getattr(board, "setLegacyPattern", None)
        if setter is not None:
            setter(legacy)
        elif legacy:
            raise RuntimeError(
                "This OpenCV version cannot set the ChArUco legacy pattern. "
                "Install OpenCV 4.7 or newer."
            )
    else:
        raise RuntimeError("This OpenCV version does not provide CharucoBoard.")
    return dictionary, board


def detect_charuco(image, dictionary, board):
    aruco = cv2.aruco
    if hasattr(aruco, "ArucoDetector"):
        parameters = aruco.DetectorParameters()
        detector = aruco.ArucoDetector(dictionary, parameters)
        marker_corners, marker_ids, _ = detector.detectMarkers(image)
    else:
        parameters_factory = getattr(aruco, "DetectorParameters_create", None)
        parameters = (
            parameters_factory()
            if parameters_factory is not None
            else aruco.DetectorParameters()
        )
        marker_corners, marker_ids, _ = aruco.detectMarkers(
            image, dictionary, parameters=parameters
        )
    if marker_ids is None or len(marker_ids) < 2:
        return None, None, marker_corners, marker_ids
    _, corners, ids = aruco.interpolateCornersCharuco(
        marker_corners, marker_ids, image, board
    )
    return corners, ids, marker_corners, marker_ids


def board_corners(board):
    getter = getattr(board, "getChessboardCorners", None)
    return getter() if getter else board.chessboardCorners


def save_camera_info(path, camera_name, image_size, camera_matrix, distortion):
    width, height = image_size
    k = camera_matrix.astype(float).reshape(-1).tolist()
    d = distortion.astype(float).reshape(-1).tolist()
    data = {
        "image_width": int(width),
        "image_height": int(height),
        "camera_name": camera_name,
        "camera_matrix": {"rows": 3, "cols": 3, "data": k},
        "distortion_model": "plumb_bob",
        "distortion_coefficients": {
            "rows": 1,
            "cols": len(d),
            "data": d,
        },
        "rectification_matrix": {
            "rows": 3,
            "cols": 3,
            "data": [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
        },
        "projection_matrix": {
            "rows": 3,
            "cols": 4,
            "data": [
                k[0], k[1], k[2], 0.0,
                k[3], k[4], k[5], 0.0,
                k[6], k[7], k[8], 0.0,
            ],
        },
    }
    with open(path, "w", encoding="utf-8") as output:
        json.dump(data, output, indent=2)
        output.write("\n")


def show_comparison(image, camera_matrix, distortion):
    image_size = (image.shape[1], image.shape[0])
    new_camera_matrix, _ = cv2.getOptimalNewCameraMatrix(
        camera_matrix, distortion, image_size, 1.0, image_size
    )
    corrected = cv2.undistort(image, camera_matrix, distortion, None, new_camera_matrix)
    raw_view = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    corrected_view = cv2.cvtColor(corrected, cv2.COLOR_GRAY2BGR)
    cv2.putText(
        raw_view, "Before: raw", (20, 35), cv2.FONT_HERSHEY_SIMPLEX, 1,
        (0, 255, 255), 2
    )
    cv2.putText(
        corrected_view, "After: undistorted", (20, 35),
        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2
    )
    comparison = cv2.hconcat([raw_view, corrected_view])
    scale = min(1.0, 1600.0 / comparison.shape[1], 900.0 / comparison.shape[0])
    if scale < 1.0:
        comparison = cv2.resize(
            comparison, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA
        )
    cv2.namedWindow("Calibration: before | after", cv2.WINDOW_NORMAL)
    cv2.imshow("Calibration: before | after", comparison)
    print("Showing raw and undistorted image side by side; press any key to close.")
    cv2.waitKey(0)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--topic", default="/my_camera/pylon_ros2_camera_node/image_raw"
    )
    parser.add_argument("--output", default="/tmp/camera_calibration.yaml")
    parser.add_argument("--camera-name", default="pylon_camera")
    parser.add_argument("--columns", type=int, default=24)
    parser.add_argument("--rows", type=int, default=17)
    parser.add_argument("--square-size", type=float, default=0.015)
    parser.add_argument("--marker-size", type=float, default=0.011)
    parser.add_argument("--min-samples", type=int, default=12)
    parser.add_argument(
        "--legacy",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Match calib.io's checked 'ChArUco Legacy' option (default: on).",
    )
    args = parser.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])

    if not hasattr(cv2, "aruco"):
        raise RuntimeError(
            "cv2.aruco is missing; install an OpenCV contrib build in this "
            "Python environment."
        )

    dictionary_options = [
        ("DICT_5X5_250", cv2.aruco.DICT_5X5_250),
        ("DICT_5X5_1000", cv2.aruco.DICT_5X5_1000),
    ]
    candidates = []
    for name, dictionary_id in dictionary_options:
        dictionary, board = make_board(
            args.columns,
            args.rows,
            args.square_size,
            args.marker_size,
            args.legacy,
            dictionary_id,
        )
        candidates.append((name, dictionary, board, []))

    rclpy.init(args=None)
    node = ImageSubscriber(args.topic)
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    image_size = None
    samples = 0
    print(f"Listening on {args.topic}")
    print("Show the whole board, vary its position, tilt and distance.")
    print("Live view: s = capture a view, q = calibrate and compare.")
    cv2.namedWindow("ChArUco calibration", cv2.WINDOW_NORMAL)

    try:
        while rclpy.ok():
            image = node.latest_image()
            if image is None:
                cv2.waitKey(30)
                continue
            current_size = (image.shape[1], image.shape[0])
            if image_size is not None and current_size != image_size:
                raise RuntimeError("Image resolution changed during calibration.")
            image_size = current_size

            detections = []
            for name, dictionary, board, _ in candidates:
                corners, ids, marker_corners, marker_ids = detect_charuco(
                    image, dictionary, board
                )
                count = 0 if ids is None else len(ids)
                detections.append(
                    (count, name, corners, ids, marker_corners, marker_ids)
                )
            best = max(detections, key=lambda result: result[0])
            counts = ", ".join(
                f"{detection[1]}: {detection[0]} corners"
                for detection in detections
            )
            preview = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
            if best[5] is not None:
                cv2.aruco.drawDetectedMarkers(preview, best[4], best[5])
            if best[2] is not None:
                cv2.aruco.drawDetectedCornersCharuco(
                    preview, best[2], best[3], (0, 220, 0)
                )
            cv2.putText(
                preview,
                f"{best[1]} | corners: {best[0]} | saved: {samples}",
                (15, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 220, 0),
                2,
            )
            cv2.putText(
                preview,
                "s: capture    q: calibrate and compare",
                (15, 60),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 220, 0),
                2,
            )
            cv2.imshow("ChArUco calibration", preview)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q") or key == 27:
                if samples < args.min_samples:
                    print(
                        f"Captured {samples}/{args.min_samples} views; "
                        "capture more before quitting."
                    )
                    continue
                break
            if key != ord("s"):
                continue
            if best[0] < 6:
                print("Not enough ChArUco corners detected; adjust the board and retry.")
                continue
            print(counts)
            for detection, candidate in zip(detections, candidates):
                _, _, corners, ids, _, _ = detection
                if ids is not None:
                    candidate[3].append((corners.copy(), ids.copy()))
                else:
                    candidate[3].append((None, None))
            samples += 1
            print(f"Captured view {samples}.")

        if samples < args.min_samples:
            raise RuntimeError(
                f"Captured {samples} views; need at least {args.min_samples}. "
                "Capture more varied views and press q again."
            )

        results = []
        for name, _, board, observations in candidates:
            object_points = board_corners(board)
            object_views = []
            image_views = []
            total_corners = 0
            for corners, ids in observations:
                if ids is None or len(ids) < 6:
                    continue
                indices = ids.reshape(-1).astype(int)
                object_views.append(object_points[indices].astype(np.float32))
                image_views.append(corners.astype(np.float32))
                total_corners += len(indices)
            if len(object_views) < args.min_samples:
                continue
            rms, camera_matrix, distortion, _, _ = cv2.calibrateCamera(
                object_views, image_views, image_size, None, None
            )
            results.append(
                (total_corners, name, rms, camera_matrix, distortion)
            )

        if not results:
            raise RuntimeError("Could not calibrate; board detections were insufficient.")
        total_corners, name, rms, camera_matrix, distortion = max(
            results, key=lambda result: result[0]
        )
        save_camera_info(
            args.output,
            args.camera_name,
            image_size,
            camera_matrix,
            distortion,
        )
        print(f"Selected {name} using {total_corners} ChArUco corners.")
        print(f"Reprojection RMS: {rms:.4f} px")
        print(f"Wrote {args.output}")
        print(f"Use camera_info_url: {Path(args.output).resolve().as_uri()}")
        latest_image = node.latest_image()
        if latest_image is not None:
            show_comparison(latest_image, camera_matrix, distortion)
    finally:
        cv2.destroyAllWindows()
        executor.shutdown()
        spin_thread.join(timeout=2.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()