"""
Raspberry Pi Camera Node (QuadKen Underwater AUV)
Captures frames from a camera (OpenCV VideoCapture or Picamera2).
Falls back to a synthetic underwater camera feed if no physical camera is attached.
Publishes:
  - image: Compressed JPEG byte stream
"""

import sys
import time
import os
import cv2
import numpy as np
import pyarrow as pa
from dora import Node


def open_camera():
    """Attempt to open hardware camera, returns None if unavailable."""
    cap = cv2.VideoCapture(0)
    if cap.isOpened():
        ret, frame = cap.read()
        if ret and frame is not None:
            print("[Camera] Physical underwater camera initialized successfully.")
            return cap
        cap.release()
    print("[Camera] Physical camera not detected. Running in synthetic underwater HUD mode.")
    return None


def generate_underwater_frame(width=320, height=240, frame_count=0):
    """Generate dynamic synthetic underwater camera frame with bubbles and HUD."""
    # Deep blue/cyan underwater gradient
    img = np.zeros((height, width, 3), dtype=np.uint8)
    gradient = np.linspace(25, 65, height)[:, np.newaxis]
    img[:, :, 0] = np.clip(gradient + 50, 0, 255)  # Blue
    img[:, :, 1] = np.clip(gradient + 20, 0, 255)  # Green
    img[:, :, 2] = np.clip(gradient - 10, 0, 255)  # Red (absorbed in water)

    # Simulated light rays / caustics wave
    for i in range(3):
        ray_x = int(width * 0.25 * (i + 1) + 20 * np.sin(frame_count * 0.04 + i))
        cv2.line(img, (ray_x - 15, 0), (ray_x + 15, height), (120, 100, 40), 3)

    # Rising underwater bubbles
    for i in range(4):
        bx = int((width * 0.2 * (i + 1) + 15 * np.cos(frame_count * 0.08 + i)) % width)
        by = int((height - ((frame_count * 2 + i * 50) % height)))
        cv2.circle(img, (bx, by), 4 + i % 3, (200, 230, 255), 1)

    # Simulated seabed or underwater object
    t = frame_count * 0.03
    obj_x = int(width * 0.5 + 40 * np.cos(t))
    obj_y = int(height * 0.6 + 15 * np.sin(t))
    cv2.circle(img, (obj_x, obj_y), 16, (40, 90, 60), -1)
    cv2.circle(img, (obj_x, obj_y), 18, (60, 140, 90), 2)

    # Target HUD Crosshair
    cx, cy = width // 2, height // 2
    cv2.line(img, (cx - 15, cy), (cx + 15, cy), (0, 220, 220), 1)
    cv2.line(img, (cx, cy - 15), (cx, cy + 15), (0, 220, 220), 1)
    cv2.circle(img, (cx, cy), 10, (0, 220, 220), 1)

    # Telemetry HUD Overlay
    cv2.putText(
        img,
        "QUADKEN UNDERWATER CAM",
        (10, 20),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.42,
        (0, 255, 255),
        1,
    )
    depth = 1.5 + 0.3 * np.sin(frame_count * 0.02)
    cv2.putText(
        img,
        f"DEPTH: {depth:.2f}m | HEAD WATER-INTAKE: READY",
        (10, height - 12),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.35,
        (200, 255, 200),
        1,
    )

    return img


def main():
    node = Node()
    cap = open_camera()

    frame_count = 0
    encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 75]

    try:
        for event in node:
            event_type = event["type"]
            if event_type == "STOP":
                print("[Camera] Received STOP event. Exiting.")
                break

            if event_type == "INPUT":
                frame_count += 1

                if cap is not None:
                    ret, frame = cap.read()
                    if not ret or frame is None:
                        frame = generate_underwater_frame(320, 240, frame_count)
                    else:
                        frame = cv2.resize(frame, (320, 240))
                else:
                    frame = generate_underwater_frame(320, 240, frame_count)

                # Compress to JPEG byte array
                success, encoded_img = cv2.imencode(".jpg", frame, encode_param)
                if success:
                    jpeg_bytes = encoded_img.tobytes()
                    node.send_output("image", pa.array([jpeg_bytes]))
    except KeyboardInterrupt:
        pass
    finally:
        if cap is not None:
            cap.release()
        sys.exit(0)


if __name__ == "__main__":
    main()
