"""
Verification of BNO Roll De-roll Transformation in AI Guidance
"""
import math
import numpy as np

def test_deroll(roll_deg: float, true_world_dx: float, true_world_dy: float, true_world_dz: float):
    roll_rad = math.radians(roll_deg)
    # AUV forward at X, rolled by roll_deg around X
    rot_roll = np.array([
        [1.0, 0.0, 0.0],
        [0.0, math.cos(roll_rad), -math.sin(roll_rad)],
        [0.0, math.sin(roll_rad), math.cos(roll_rad)]
    ])

    v_world = np.array([true_world_dx, true_world_dy, true_world_dz])
    v_body = rot_roll.T @ v_world

    xb, yb, zb = v_body
    azimuth_body = math.degrees(math.atan2(-yb, xb))
    elevation_body = math.degrees(math.atan2(zb, math.hypot(xb, yb)))

    # Apply de-roll
    az_rad = math.radians(azimuth_body)
    el_rad = math.radians(elevation_body)
    xb_est = math.cos(el_rad) * math.cos(az_rad)
    yb_est = -math.cos(el_rad) * math.sin(az_rad)
    zb_est = math.sin(el_rad)

    y_level = math.cos(roll_rad) * yb_est - math.sin(roll_rad) * zb_est
    z_level = math.sin(roll_rad) * yb_est + math.cos(roll_rad) * zb_est

    azimuth_level = math.degrees(math.atan2(-y_level, max(1e-4, xb_est)))
    elevation_level = math.degrees(math.atan2(z_level, math.hypot(xb_est, y_level)))

    true_azimuth_world = math.degrees(math.atan2(-true_world_dy, true_world_dx))
    true_elevation_world = math.degrees(math.atan2(true_world_dz, math.hypot(true_world_dx, true_world_dy)))

    print(f"Roll: {roll_deg:+5.1f} deg | True World: [Az={true_azimuth_world:+5.1f}°, El={true_elevation_world:+5.1f}°]")
    print(f"               | Body (Cam) : [Az={azimuth_body:+5.1f}°, El={elevation_body:+5.1f}°]")
    print(f"               | De-Rolled  : [Az={azimuth_level:+5.1f}°, El={elevation_level:+5.1f}°]")
    assert abs(azimuth_level - true_azimuth_world) < 0.1, f"Azimuth mismatch: {azimuth_level} vs {true_azimuth_world}"
    assert abs(elevation_level - true_elevation_world) < 0.1, f"Elevation mismatch: {elevation_level} vs {true_elevation_world}"
    print("               --> MATCH SUCCESS!\n")

if __name__ == "__main__":
    print("=== Testing Pure Right Target (dy=-2.0) under various roll angles ===")
    test_deroll(0.0, 5.0, -2.0, 0.0)
    test_deroll(30.0, 5.0, -2.0, 0.0)
    test_deroll(60.0, 5.0, -2.0, 0.0)
    test_deroll(90.0, 5.0, -2.0, 0.0)
    test_deroll(-45.0, 5.0, -2.0, 0.0)

    print("=== Testing Pure UP Target (dz=+1.5) under various roll angles ===")
    test_deroll(0.0, 5.0, 0.0, 1.5)
    test_deroll(45.0, 5.0, 0.0, 1.5)
    test_deroll(90.0, 5.0, 0.0, 1.5)
    test_deroll(-60.0, 5.0, 0.0, 1.5)
