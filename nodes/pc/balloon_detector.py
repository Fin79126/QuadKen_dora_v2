"""
PC Perception Node for QuadKen Underwater AUV: Balloon Detector.
Subscribes to forward underwater camera feed, detects target balloons via OpenCV,
estimates geometric relative azimuth/elevation and distance, and publishes
target_relative_info conforming to the QuadKen GNC guidance interface.

Inputs:
  - image: Raw JPEG bytes from forward camera (e.g. mujoco_sim/image)
  - bno_data (optional): Virtual/Real BNO055 telemetry (for orientation & depth)

Outputs:
  - target_relative_info: JSON payload containing:
      * target_found: bool
      * azimuth_deg: float (-180 to +180 deg, Left: negative, Right: positive)
      * elevation_deg: float (-90 to +90 deg, Down: negative, Up: positive)
      * distance_m: float (meters from nose)
      * mode: str ("LOCKED", "APPROACH", "STRIKE_HOLD", "SEARCHING")
  - image_annotated: JPEG bytes with visual HUD overlays for Rerun visualizer
"""

import os
import sys
import time
import json
import math
import cv2
import numpy as np
import pyarrow as pa
from dora import Node

# Ensure stdout uses UTF-8 on Windows
if sys.platform == "win32":
    try:
        sys.stdout.reconfigure(encoding="utf-8")
        sys.stderr.reconfigure(encoding="utf-8")
    except Exception:
        pass


class BalloonDetector:
    """
    OpenCV-based underwater balloon detector and geometric angle/distance estimator.
    Conforms to QuadKen target_relative_info data interface.
    """

    def __init__(self, fov_y_deg: float = 75.0, width: int = 320, height: int = 240, balloon_radius_m: float = 0.25):
        self.width = width
        self.height = height
        self.cx = width / 2.0
        self.cy = height / 2.0
        self.balloon_radius_m = balloon_radius_m

        # Pinhole camera focal length
        self.focal_y = (height / 2.0) / math.tan(math.radians(fov_y_deg / 2.0))
        self.focal_x = self.focal_y  # Square pixel assumption

        # HSV Color ranges for underwater balloons
        # Red, Orange, Yellow, Lime, Pink/Purple
        self.color_ranges = [
            # Red (two intervals in HSV wrap-around)
            (np.array([0, 100, 60]), np.array([12, 255, 255])),
            (np.array([165, 100, 60]), np.array([180, 255, 255])),
            # Orange / Gold / Yellow
            (np.array([13, 110, 70]), np.array([35, 255, 255])),
            # Lime / Green
            (np.array([36, 90, 60]), np.array([85, 255, 255])),
            # Pink / Purple / Magenta
            (np.array([135, 70, 60]), np.array([164, 255, 255])),
        ]

        # Morphological structuring elements
        self.open_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        self.close_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 9))
        self.encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 80]

        # Tracking state
        self.last_target = None
        self.last_detect_time = None
        self.strike_hold_frames = 0
        self.last_pop_count = 0

    def detect(self, bgr_image: np.ndarray, bno_data=None):
        """
        Detect target balloon in BGR image.
        Returns:
            relative_info: dict matching target_relative_info schema
            annotated_image: BGR image with bounding box, reticle, and telemetry text
        """
        h, w = bgr_image.shape[:2]
        hsv = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)

        # 1. Multi-color mask creation
        full_mask = np.zeros((h, w), dtype=np.uint8)
        for lower, upper in self.color_ranges:
            mask = cv2.inRange(hsv, lower, upper)
            full_mask = cv2.bitwise_or(full_mask, mask)

        # 2. Morphological cleanup & hole filling (handles near-clipping hollows and reflections)
        clean_mask = cv2.morphologyEx(full_mask, cv2.MORPH_OPEN, self.open_kernel)
        clean_mask = cv2.morphologyEx(clean_mask, cv2.MORPH_CLOSE, self.close_kernel)

        # Fill internal holes (e.g. Near-clipping hollow in middle of balloon)
        cnts_fill, _ = cv2.findContours(clean_mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(clean_mask, cnts_fill, -1, 255, thickness=cv2.FILLED)

        # 3. Find clean external contours
        contours, _ = cv2.findContours(clean_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        candidates = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < 30:  # Filter out tiny noise specks
                continue

            x, y, bw, bh = cv2.boundingRect(cnt)
            touches_border = (x <= 2 or y <= 2 or (x + bw) >= (w - 2) or (y + bh) >= (h - 2))

            # Use convex hull for stable geometry against knot/tether notches
            hull = cv2.convexHull(cnt)
            hull_area = cv2.contourArea(hull)
            perimeter = cv2.arcLength(hull, True)
            if perimeter <= 0:
                continue

            circularity = 4.0 * math.pi * hull_area / (perimeter * perimeter)
            min_circ = 0.25 if (touches_border and hull_area > 500) else 0.45
            if circularity < min_circ:
                continue

            (u, v), radius = cv2.minEnclosingCircle(hull)
            if radius < 3.0:
                continue

            # Calculate azimuth/elevation for this candidate
            c_az = math.degrees(math.atan2(u - self.cx, self.focal_x))
            c_el = math.degrees(math.atan2(-(v - self.cy), self.focal_y))
            c_dist = (self.focal_y * self.balloon_radius_m) / max(1.0, radius)

            candidates.append({
                "contour": cnt,
                "hull": hull,
                "center": (float(u), float(v)),
                "radius": float(radius),
                "area": float(hull_area),
                "circularity": float(circularity),
                "touches_border": touches_border,
                "azimuth": c_az,
                "elevation": c_el,
                "distance": c_dist,
            })

        annotated_image = bgr_image.copy()

        # Draw central reticle (crosshair)
        cx_i, cy_i = int(self.cx), int(self.cy)
        cv2.line(annotated_image, (cx_i - 12, cy_i), (cx_i + 12, cy_i), (0, 240, 240), 1)
        cv2.line(annotated_image, (cx_i, cy_i - 12), (cx_i, cy_i + 12), (0, 240, 240), 1)
        cv2.circle(annotated_image, (cx_i, cy_i), 8, (0, 240, 240), 1)

        now = cv2.getTickCount() / cv2.getTickFrequency()
        dt = (now - self.last_detect_time) if self.last_detect_time is not None else 0.02
        self.last_detect_time = now

        # Check for balloon destruction event from telemetry
        just_popped = False
        pop_count = self.last_pop_count
        if bno_data is not None:
            pop_count = int(bno_data.get("pop_count", self.last_pop_count))
            just_popped = bool(bno_data.get("just_popped", False))
            if pop_count > self.last_pop_count or just_popped:
                self.last_pop_count = max(self.last_pop_count, pop_count)
                # Target was popped! Reset strike hold immediately
                self.last_target = None
                self.strike_hold_frames = 0

        # Check STRIKE blind-zone hold (when approaching within 1.15m and camera enters balloon sphere)
        is_in_strike_range = (
            hasattr(self, "last_target") and self.last_target is not None and self.last_target.get("distance", 999.0) < 1.15
        )

        if not candidates:
            if is_in_strike_range and getattr(self, "strike_hold_frames", 0) < 20:
                self.strike_hold_frames = getattr(self, "strike_hold_frames", 0) + 1
                hold_dist = max(0.20, self.last_target["distance"] - 0.7 * dt)
                self.last_target["distance"] = hold_dist
                cv2.putText(
                    annotated_image,
                    f"PERCEPTION: STRIKE HOLD (Dist:{hold_dist:4.2f}m)",
                    (8, 20),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.38,
                    (0, 255, 255),
                    1,
                    cv2.LINE_AA,
                )
                return {
                    "target_found": True,
                    "azimuth_deg": round(float(self.last_target["azimuth"]), 2),
                    "elevation_deg": round(float(self.last_target["elevation"]), 2),
                    "distance_m": round(float(hold_dist), 2),
                    "mode": "STRIKE_HOLD",
                    "pop_count": pop_count,
                    "just_popped": just_popped,
                }, annotated_image

            self.last_target = None
            self.strike_hold_frames = 0
            if just_popped:
                cv2.putText(
                    annotated_image,
                    f"PERCEPTION: TARGET POPPED! (Pops: {pop_count})",
                    (8, 20),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.38,
                    (0, 255, 100),
                    1,
                    cv2.LINE_AA,
                )
            else:
                cv2.putText(
                    annotated_image,
                    "PERCEPTION: SEARCHING (No Target)",
                    (8, 20),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.38,
                    (0, 165, 255),
                    1,
                    cv2.LINE_AA,
                )
            return {
                "target_found": False,
                "azimuth_deg": 0.0,
                "elevation_deg": 0.0,
                "distance_m": 0.0,
                "mode": "SEARCHING",
                "pop_count": pop_count,
                "just_popped": just_popped,
            }, annotated_image

        # Candidate Association: Maintain lock-on continuity
        best = None
        if hasattr(self, "last_target") and self.last_target is not None:
            last_u, last_v = self.last_target["center"]
            last_az = self.last_target["azimuth"]

            associated = []
            for c in candidates:
                d_pos = math.hypot(c["center"][0] - last_u, c["center"][1] - last_v)
                d_az = abs(c["azimuth"] - last_az)

                # Reject sudden jumps to far-away balloons when already in strike range
                if is_in_strike_range and c["distance"] > 1.8:
                    continue

                if d_pos < 100.0 or d_az < 18.0:
                    associated.append((d_pos, c))

            if associated:
                associated.sort(key=lambda item: item[0])
                best = associated[0][1]
            elif is_in_strike_range and getattr(self, "strike_hold_frames", 0) < 50:
                # All candidates far away -> Keep holding previous strike target
                self.strike_hold_frames = getattr(self, "strike_hold_frames", 0) + 1
                hold_dist = max(0.20, self.last_target["distance"] - 0.7 * dt)
                self.last_target["distance"] = hold_dist
                cv2.putText(
                    annotated_image,
                    f"PERCEPTION: STRIKE HOLD (Dist:{hold_dist:4.2f}m)",
                    (8, 20),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.38,
                    (0, 255, 255),
                    1,
                    cv2.LINE_AA,
                )
                return {
                    "target_found": True,
                    "azimuth_deg": round(float(self.last_target["azimuth"]), 2),
                    "elevation_deg": round(float(self.last_target["elevation"]), 2),
                    "distance_m": round(float(hold_dist), 2),
                    "mode": "STRIKE_HOLD",
                }, annotated_image

        # Fallback to largest area if no continuity match
        if best is None:
            best = max(candidates, key=lambda c: c["area"])
            self.strike_hold_frames = 0
        else:
            self.strike_hold_frames = 0

        # In strike range, reject drastic angular jumps caused by clipped edge fragments
        if is_in_strike_range and hasattr(self, "last_target") and self.last_target is not None:
            last_el = self.last_target["elevation"]
            last_az = self.last_target["azimuth"]
            if abs(best["elevation"] - last_el) > 5.0 or abs(best["azimuth"] - last_az) > 6.0:
                best = dict(self.last_target)
                hold_dist = max(0.20, self.last_target["distance"] - 0.7 * dt)
                best["distance"] = hold_dist

        # Apply realistic angular rate limiter (max 150 deg/s -> ~3.0 deg/frame at 50Hz)
        if hasattr(self, "last_target") and self.last_target is not None:
            max_d_ang = 2.5
            d_az = max(-max_d_ang, min(max_d_ang, best["azimuth"] - self.last_target["azimuth"]))
            d_el = max(-max_d_ang, min(max_d_ang, best["elevation"] - self.last_target["elevation"]))
            azimuth_deg = self.last_target["azimuth"] + d_az
            elevation_deg = self.last_target["elevation"] + d_el
        else:
            azimuth_deg = best["azimuth"]
            elevation_deg = best["elevation"]

        u, v = best["center"]
        radius = best["radius"]
        distance_m = best["distance"]

        # Store smoothed state
        best["azimuth"] = azimuth_deg
        best["elevation"] = elevation_deg
        best["distance"] = distance_m
        self.last_target = best

        # Draw detection visualization
        u_i, v_i = int(round(u)), int(round(v))
        r_i = int(round(radius))

        # Green circle around detected balloon
        cv2.circle(annotated_image, (u_i, v_i), r_i, (0, 255, 0), 2)
        # Center point
        cv2.circle(annotated_image, (u_i, v_i), 3, (0, 0, 255), -1)

        # Line from screen center to target
        cv2.line(annotated_image, (cx_i, cy_i), (u_i, v_i), (0, 255, 255), 1, cv2.LINE_AA)

        # Target annotation box
        label_text = f"Dist:{distance_m:.2f}m [Az:{azimuth_deg:+4.1f} El:{elevation_deg:+4.1f}]"
        cv2.putText(
            annotated_image,
            label_text,
            (max(4, u_i - 60), max(18, v_i - r_i - 6)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.35,
            (0, 255, 0),
            1,
            cv2.LINE_AA,
        )

        # Header status
        header_text = f"PERCEPTION: LOCKED [Az:{azimuth_deg:+4.1f} El:{elevation_deg:+4.1f} Dist:{distance_m:4.2f}m]"
        cv2.putText(
            annotated_image,
            header_text,
            (8, 20),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.38,
            (0, 255, 0),
            1,
            cv2.LINE_AA,
        )

        relative_info = {
            "target_found": True,
            "azimuth_deg": round(float(azimuth_deg), 2),
            "elevation_deg": round(float(elevation_deg), 2),
            "distance_m": round(float(distance_m), 2),
            "mode": "LOCKED",
            "pop_count": pop_count,
            "just_popped": just_popped,
        }

        return relative_info, annotated_image


def main():
    print("[Balloon Detector Node] Starting Perception node for QuadKen AUV...")
    node = Node()
    detector = BalloonDetector()
    last_bno = {}

    encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 80]

    for event in node:
        event_type = event["type"]
        if event_type == "STOP":
            print("[Balloon Detector Node] Received STOP event. Exiting.")
            break

        if event_type == "INPUT":
            input_id = event["id"]
            raw_value = event["value"]

            if input_id == "bno_data":
                try:
                    raw_bytes = raw_value.to_pylist()[0]
                    last_bno = json.loads(
                        raw_bytes if isinstance(raw_bytes, str) else raw_bytes.decode("utf-8")
                    )
                except Exception:
                    pass

            elif input_id == "image":
                try:
                    raw_bytes = raw_value.to_pylist()[0]
                    if not isinstance(raw_bytes, bytes):
                        continue

                    # Decode JPEG image
                    nparr = np.frombuffer(raw_bytes, np.uint8)
                    frame_bgr = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
                    if frame_bgr is None:
                        continue

                    # Run OpenCV perception
                    rel_info, annotated_bgr = detector.detect(frame_bgr, last_bno)

                    # 1. Publish target_relative_info (JSON)
                    node.send_output(
                        "target_relative_info",
                        pa.array([json.dumps(rel_info).encode("utf-8")]),
                    )

                    # 2. Publish image_annotated (JPEG)
                    success, encoded_jpg = cv2.imencode(".jpg", annotated_bgr, encode_param)
                    if success:
                        node.send_output(
                            "image_annotated",
                            pa.array([encoded_jpg.tobytes()]),
                        )

                except Exception as e:
                    print(f"[Balloon Detector Node] Detection error: {e}")


if __name__ == "__main__":
    main()
