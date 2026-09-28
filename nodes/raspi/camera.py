"""
Raspberry Pi Camera Node
Captures frames from a camera (OpenCV VideoCapture or Picamera2).
Falls back to a synthetic animated test pattern if no physical camera is attached.
Publishes:
  - image: Compressed JPEG byte stream
"""

import time
import os
import cv2
import numpy as np
import pyarrow as pa
from dora import Node


def open_camera():
    """Attempt to open hardware camera, returns None if unavailable."""
    # Test camera index 0
    cap = cv2.VideoCapture(0)
    if cap.isOpened():
        ret, frame = cap.read()
        if ret and frame is not None:
            print("[Camera] Physical camera initialized successfully.")
            return cap
        cap.release()
    print("[Camera] Physical camera not detected. Running in synthetic mock camera mode.")
    return None


def generate_synthetic_frame(width=320, height=240, frame_count=0):
    """Generate dynamic synthetic HUD camera frame for testing."""
    img = np.zeros((height, width, 3), dtype=np.uint8)

    # Artificial horizon / ground grid
    horizon_y = int(height * 0.5 + 20 * np.sin(frame_count * 0.05))
    cv2.rectangle(img, (0, 0), (width, horizon_y), (40, 30, 20), -1)      # Dark sky
    cv2.rectangle(img, (0, horizon_y), (width, height), (30, 60, 30), -1) # Green ground

    # Animated horizon line
    cv2.line(img, (0, horizon_y), (width, horizon_y), (0, 255, 0), 2)

    # Bouncing ball (simulating visual object)
    ball_x = int(width * 0.5 + (width * 0.35) * np.sin(frame_count * 0.08))
    ball_y = int(height * 0.5 + (height * 0.25) * np.cos(frame_count * 0.06))
    cv2.circle(img, (ball_x, ball_y), 14, (0, 165, 255), -1)
    cv2.circle(img, (ball_x, ball_y), 14, (255, 255, 255), 1)

    # Robot HUD Overlay
    cv2.putText(
        img,
        f"QuadKen Camera (RasPi)",
        (10, 20),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.45,
        (255, 255, 255),
        1,
    )
    cv2.putText(
        img,
        f"Time: {time.strftime('%H:%M:%S')} #{frame_count}",
        (10, height - 12),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.4,
        (200, 200, 200),
        1,
    )

    return img


def main():
    node = Node()
    cap = open_camera()
    frame_count = 0

    for event in node:
        event_type = event["type"]
        if event_type == "STOP":
            print("[Camera] Received STOP event. Exiting.")
            sys.exit(0)

        if event_type == "INPUT":
            frame_count += 1
            if cap is not None:
                ret, frame = cap.read()
                if not ret or frame is None:
                    frame = generate_synthetic_frame(frame_count=frame_count)
            else:
                frame = generate_synthetic_frame(frame_count=frame_count)

            # Compress to JPEG to minimize bandwidth across network
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 75]
            success, encoded_img = cv2.imencode(".jpg", frame, encode_param)

            if success:
                jpeg_bytes = encoded_img.tobytes()
                node.send_output("image", pa.array([jpeg_bytes]))


if __name__ == "__main__":
    main()
