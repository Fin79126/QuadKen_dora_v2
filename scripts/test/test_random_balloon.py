import sys
import os
import numpy as np

sys.path.insert(0, r"c:\Users\tacky\MyApp\QuadKen\QuadKen_dora_v2")

from nodes.simulation.mujoco_node import QuadKenMuJoCoSim

def test_random_spawn():
    print("\n--- Test 1: Random Initial Spawn Check ---")
    sim1 = QuadKenMuJoCoSim(random_spawn=True)
    positions1 = [sim1.data.mocap_pos[b["mocap_id"]].copy() for b in sim1.balloons]

    sim2 = QuadKenMuJoCoSim(random_spawn=True)
    positions2 = [sim2.data.mocap_pos[b["mocap_id"]].copy() for b in sim2.balloons]

    # Verify positions are different between sim1 and sim2
    diffs = [np.linalg.norm(positions1[i] - positions2[i]) for i in range(10)]
    print(f"Max coordinate difference between two random runs: {max(diffs):.2f}m")
    assert max(diffs) > 1.0, "Balloons did not randomize between runs!"

    print("Sample positions from Sim 1:")
    for i, p in enumerate(positions1[:4]):
        print(f"  Balloon {i}: X={p[0]:.2f}, Y={p[1]:.2f}, Z={p[2]:.2f}")
        assert 3.5 <= p[0] <= 26.0, f"X out of safe bounds: {p[0]}"
        assert -7.5 <= p[1] <= 7.5, f"Y out of safe bounds: {p[1]}"
        assert -2.8 <= p[2] <= -1.0, f"Z out of safe bounds: {p[2]}"

    print("\n--- Test 2: Instant Respawn Mode Check ---")
    sim_instant = QuadKenMuJoCoSim(random_spawn=True, respawn_mode="instant")
    target_idx = sim_instant.current_waypoint_idx
    old_pos = sim_instant.data.mocap_pos[sim_instant.balloons[target_idx]["mocap_id"]].copy()

    # Move balloon onto nose to simulate pop
    p_nose = [0.80, 0.0, -1.5]
    sim_instant.data.mocap_pos[sim_instant.balloons[target_idx]["mocap_id"]] = p_nose
    sim_instant.step()

    new_pos = sim_instant.data.mocap_pos[sim_instant.balloons[target_idx]["mocap_id"]].copy()
    print(f"Old pos: {old_pos.round(2)}, Popped at nose, New instant respawn pos: {new_pos.round(2)}")
    assert np.linalg.norm(new_pos - p_nose) > 2.0, "Respawned too close to nose!"
    assert sim_instant.balloon_pop_count == 1
    assert not sim_instant.balloons[target_idx]["popped"], "Balloon should be active again in instant mode!"

    print("\n--- Test 3: Batch Respawn Mode (10 Popped -> All Random Respawn) ---")
    sim_batch = QuadKenMuJoCoSim(random_spawn=True, respawn_mode="batch")
    for step in range(10):
        t_b = sim_batch.balloons[sim_batch.current_waypoint_idx]
        sim_batch.data.mocap_pos[t_b["mocap_id"]] = [0.80, 0.0, -1.5]
        sim_batch.last_sim_pop_time = -999.0 # allow immediate next pop
        sim_batch.step()

    print(f"Total pops: {sim_batch.balloon_pop_count}")
    assert sim_batch.balloon_pop_count == 10
    active_cnt = sum(1 for b in sim_batch.balloons if not b["popped"])
    print(f"Active balloons after all 10 destroyed: {active_cnt}/10")
    assert active_cnt == 10, "All 10 should have respawned at new random coordinates!"

    print("\n[ALL BALLOON RANDOMIZATION TESTS PASSED!]")

if __name__ == "__main__":
    test_random_spawn()
