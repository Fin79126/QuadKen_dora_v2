"""
Test script for verifying balloon target tracking, relative angle/distance calculation,
and collision / pop mechanics in QuadKenMuJoCoSim.
"""
import os
import sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
import numpy as np
from nodes.simulation.mujoco_node import QuadKenMuJoCoSim

def test_balloon_mechanics():
    print("[TEST] Initializing QuadKenMuJoCoSim...")
    sim = QuadKenMuJoCoSim()

    print(f"[TEST] Balloon mocap ID: {sim.balloon_mocap_id}")
    assert sim.balloon_mocap_id != -1, "Balloon mocap body not found!"

    # 1. Test Initial Relative Position
    t_info = sim.get_target_relative_info()
    print(f"[TEST] Initial target info: {t_info}")
    assert t_info["target_found"] is True
    assert t_info["distance_m"] > 0
    print(f"[TEST] Initial Distance: {t_info['distance_m']}m, Azimuth: {t_info['azimuth_deg']}deg, Elevation: {t_info['elevation_deg']}deg")

    # 2. Simulate moving balloon right onto the robot's nose to trigger POP
    # Nose is at X=0.72 from robot origin (robot is at [0, 0, -1.5])
    sim.data.mocap_pos[sim.balloon_mocap_id] = [0.80, 0.0, -1.5]
    
    # Step physics
    sim.step()

    print(f"[TEST] After collision step -> Pop count: {sim.balloon_pop_count}")
    assert sim.balloon_pop_count == 1, f"Expected pop_count=1, got {sim.balloon_pop_count}"

    # 3. Verify Respawn at next waypoint
    new_target_info = sim.get_target_relative_info()
    print(f"[TEST] After pop -> Next target info: {new_target_info}")
    assert new_target_info["pop_count"] == 1
    assert new_target_info["just_popped"] is True
    print("[TEST] SUCCESS! Balloon collision, pop counting, and respawning all verified perfectly.")

if __name__ == "__main__":
    test_balloon_mechanics()
