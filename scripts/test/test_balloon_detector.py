"""
Test script for QuadKen Underwater Balloon Detector (Step 3 Perception).
Captures forward camera images from QuadKenMuJoCoSim, performs OpenCV-based
balloon detection, estimates azimuth, elevation, and distance, and benchmarks
the estimated values against MuJoCo Ground Truth (sim.get_target_relative_info).
"""

import os
import sys
import math
import cv2
import numpy as np

# Ensure workspace root is in path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))

from nodes.simulation.mujoco_node import QuadKenMuJoCoSim


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
        # Balloon 0 is Ruby Red (rgba: 1.0, 0.12, 0.38)
        # Background pool is blue/cyan (H ~ 90-135)
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

        # Morphological structuring element
        self.morph_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))

    def detect(self, bgr_image: np.ndarray):
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
        clean_mask = cv2.morphologyEx(full_mask, cv2.MORPH_OPEN, self.morph_kernel)
        # Connect fragmented parts with 9x9 ellipse
        close_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 9))
        clean_mask = cv2.morphologyEx(clean_mask, cv2.MORPH_CLOSE, close_kernel)

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
        cv2.line(annotated_image, (cx_i - 12, cy_i), (cx_i + 12, cy_i), (80, 80, 80), 1)
        cv2.line(annotated_image, (cx_i, cy_i - 12), (cx_i, cy_i + 12), (80, 80, 80), 1)

        now = cv2.getTickCount() / cv2.getTickFrequency()
        dt = (now - self.last_detect_time) if getattr(self, "last_detect_time", None) is not None else 0.02
        self.last_detect_time = now

        # Check STRIKE blind-zone hold (when approaching within 1.1m and camera enters balloon sphere)
        is_in_strike_range = (
            hasattr(self, "last_target") and self.last_target is not None and self.last_target.get("distance", 999.0) < 1.15
        )

        if not candidates:
            if is_in_strike_range and getattr(self, "strike_hold_frames", 0) < 50:
                # Hold last aim during blind strike
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
                }, annotated_image

            self.last_target = None
            self.strike_hold_frames = 0
            cv2.putText(
                annotated_image,
                "PERCEPTION: SEARCHING (No Target)",
                (8, 20),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.40,
                (0, 165, 255),
                1,
                cv2.LINE_AA,
            )
            return {
                "target_found": False,
                "azimuth_deg": 0.0,
                "elevation_deg": 0.0,
                "distance_m": 0.0,
            }, annotated_image

        # Candidate Association: Maintain lock-on continuity
        best = None
        if hasattr(self, "last_target") and self.last_target is not None:
            last_u, last_v = self.last_target["center"]
            last_az = self.last_target["azimuth"]
            last_dist = self.last_target["distance"]

            # Score candidates based on spatial continuity to last target
            associated = []
            for c in candidates:
                d_pos = math.hypot(c["center"][0] - last_u, c["center"][1] - last_v)
                d_az = abs(c["azimuth"] - last_az)

                # Reject sudden jumps to far-away balloons when we are already in strike range
                if is_in_strike_range and c["distance"] > 1.8:
                    continue

                if d_pos < 100.0 or d_az < 18.0:
                    associated.append((d_pos, c))

            if associated:
                associated.sort(key=lambda item: item[0])
                best = associated[0][1]
            elif is_in_strike_range and getattr(self, "strike_hold_frames", 0) < 50:
                # All detected candidates are far away (other balloons in background)
                # Keep holding previous strike target
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
        }

        return relative_info, annotated_image


def test_balloon_detector_single_frame():
    print("=" * 65)
    print(" [TEST] Single Frame Underwater Balloon Perception Test")
    print("=" * 65)

    sim = QuadKenMuJoCoSim(random_spawn=False)
    detector = BalloonDetector()

    # Step physics slightly so initial positions stabilize
    for _ in range(10):
        sim.step()

    # 1. Grab raw BGR camera frame
    raw_bgr = sim.render_front_camera(return_raw_bgr=True)
    assert raw_bgr is not None, "Failed to render front camera image!"

    # 2. Get Ground Truth from simulation
    gt = sim.get_target_relative_info()
    print(f"Ground Truth Telemetry:")
    print(f"  Target Found : {gt['target_found']}")
    print(f"  Azimuth      : {gt['azimuth_deg']:+6.2f} deg")
    print(f"  Elevation    : {gt['elevation_deg']:+6.2f} deg")
    print(f"  Distance     : {gt['distance_m']:6.2f} m")
    print(f"  Target Pos   : {gt['target_pos_world']}")

    # 3. Run Balloon Detector
    est, annotated_img = detector.detect(raw_bgr)
    print("\nPerception Estimated Values:")
    print(f"  Target Found : {est['target_found']}")
    print(f"  Azimuth      : {est['azimuth_deg']:+6.2f} deg")
    print(f"  Elevation    : {est['elevation_deg']:+6.2f} deg")
    print(f"  Distance     : {est['distance_m']:6.2f} m")

    # 4. Compare Errors
    if est["target_found"] and gt["target_found"]:
        err_az = abs(est["azimuth_deg"] - gt["azimuth_deg"])
        err_el = abs(est["elevation_deg"] - gt["elevation_deg"])
        err_dist = abs(est["distance_m"] - gt["distance_m"])

        print("\nEstimation Errors:")
        print(f"  Delta Azimuth   : {err_az:5.2f} deg")
        print(f"  Delta Elevation : {err_el:5.2f} deg")
        print(f"  Delta Distance  : {err_dist:5.2f} m ({err_dist / gt['distance_m'] * 100:.1f}%)")

        os.makedirs("out", exist_ok=True)
        cv2.imwrite("out/test_balloon_raw.jpg", raw_bgr)
        cv2.imwrite("out/test_balloon_annotated.jpg", annotated_img)
        print("\nSaved debug images:")
        print("  - out/test_balloon_raw.jpg")
        print("  - out/test_balloon_annotated.jpg")

        assert err_az < 5.0, f"Azimuth error {err_az} deg is too large!"
        assert err_el < 5.0, f"Elevation error {err_el} deg is too large!"
        assert err_dist < 1.0, f"Distance error {err_dist} m is too large!"
        print("\n>>> [PASSED] Single-frame perception test passed with high accuracy! <<<")
    else:
        print("\n>>> [FAILED] Balloon was not detected in frame! <<<")
        assert False, "Balloon not detected!"


def test_balloon_detector_dynamic_tracking():
    print("\n" + "=" * 65)
    print(" [TEST] Dynamic Approach & Pop Perception Tracking Test")
    print("=" * 65)

    from nodes.pc.compute import UnderwaterDynamics
    from nodes.pc.ai_guidance import AIGuidanceController

    sim = QuadKenMuJoCoSim(random_spawn=False)
    dynamics = UnderwaterDynamics()
    guidance = AIGuidanceController()
    detector = BalloonDetector()

    dt = 0.02
    max_steps = 400  # 8.0 seconds

    detected_count = 0
    total_steps = 0
    az_errors = []
    el_errors = []
    dist_errors = []
    popped = False

    for step in range(max_steps):
        current_time = step * dt

        # 1. Telemetry
        gt_info = sim.get_target_relative_info()
        bno_data = sim.get_bno_payload(step)

        if gt_info.get("pop_count", 0) > 0:
            popped = True
            print(f"\n>>> [HIT] Balloon #1 Popped at t={current_time:.2f}s! <<<")
            break

        # 2. Camera perception
        raw_bgr = sim.render_front_camera(return_raw_bgr=True)
        if abs(current_time - 5.50) < 0.01:
            cv2.imwrite("out/frame_5_50.jpg", raw_bgr)
        if abs(current_time - 5.52) < 0.01:
            cv2.imwrite("out/frame_5_52.jpg", raw_bgr)
        est_info, annotated = detector.detect(raw_bgr)
        total_steps += 1

        if est_info["target_found"]:
            detected_count += 1
            err_az = abs(est_info["azimuth_deg"] - gt_info["azimuth_deg"])
            err_el = abs(est_info["elevation_deg"] - gt_info["elevation_deg"])
            err_dist = abs(est_info["distance_m"] - gt_info["distance_m"])
            az_errors.append(err_az)
            el_errors.append(err_el)
            dist_errors.append(err_dist)

        # 3. Control step (using ground truth for approach)
        cmd, status = guidance.update(gt_info, bno_data, current_time)
        leg_angles, bldc_pwm, ballast_servos = dynamics.compute_actuators(
            dt=dt,
            throttle=cmd["throttle"],
            steer_yaw=cmd["steer_yaw"],
            steer_pitch=cmd["steer_pitch"],
            steer_roll=cmd["steer_roll"],
            ballast_cmd=cmd["ballast"],
            brake=cmd["brake"],
            roll_deg=bno_data["roll"],
            pitch_deg=bno_data["pitch"],
            stick_right_x=cmd["stick_right_x"],
            stick_right_y=cmd["stick_right_y"],
        )
        sim.set_actuator_commands({
            "esp1": {"motors": bldc_pwm, "servos": leg_angles},
            "esp2": {"servos": ballast_servos},
        })
        sim.step()

        if step % 25 == 0 or (current_time >= 5.5 and current_time <= 6.5):
            det_str = f"LOCKED (Dist:{est_info['distance_m']:4.2f}m Az:{est_info['azimuth_deg']:+5.1f} El:{est_info['elevation_deg']:+5.1f})" if est_info["target_found"] else "LOST"
            print(
                f"[t={current_time:4.2f}s] GT: Dist={gt_info['distance_m']:4.2f}m Az={gt_info['azimuth_deg']:+5.1f} | "
                f"Vision: {det_str} (candidates: {len(detector.last_candidates if hasattr(detector, 'last_candidates') else [])})"
            )

    det_rate = (detected_count / total_steps) * 100.0 if total_steps > 0 else 0
    mean_az = np.mean(az_errors) if az_errors else 999.0
    mean_el = np.mean(el_errors) if el_errors else 999.0
    mean_dist = np.mean(dist_errors) if dist_errors else 999.0

    print("-" * 65)
    print(f"Tracking Statistics over {total_steps} frames:")
    print(f"  Detection Rate     : {det_rate:.1f}% ({detected_count}/{total_steps})")
    print(f"  Mean Azimuth Err   : {mean_az:.2f} deg (max: {np.max(az_errors):.2f} deg)")
    print(f"  Mean Elevation Err : {mean_el:.2f} deg (max: {np.max(el_errors):.2f} deg)")
    print(f"  Mean Distance Err  : {mean_dist:.2f} m (max: {np.max(dist_errors):.2f} m)")
    print("-" * 65)

    assert det_rate >= 90.0, f"Detection rate {det_rate:.1f}% too low!"
    assert mean_az < 2.0, f"Mean azimuth error {mean_az:.2f} too large!"
    assert mean_el < 2.0, f"Mean elevation error {mean_el:.2f} too large!"
    print("\n>>> [PASSED] Dynamic perception tracking test passed with flying colors! <<<")


if __name__ == "__main__":
    test_balloon_detector_single_frame()
    test_balloon_detector_dynamic_tracking()

