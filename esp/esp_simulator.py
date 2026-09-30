"""
ESP 1 & ESP 2 Microcontroller Simulator (QuadKen Underwater AUV)
Simulates the two physical ESPs connected to Raspberry Pi:
  - ESP1 (Thrust & Drag-Steering): TCP 5001, UDP 6001
      * 4 Servos: Membrane leg deployment angles [Top, Right, Bottom, Left]
      * 2 Motors: BLDC forward thrusters
  - ESP2 (Ballast & Buoyancy): TCP 5002, UDP 6002
      * 4 Servos: Head water-intake servos
      * 0 Motors
Handles:
  - TCP Server: Responds to heartbeat PING with PONG and battery health.
  - UDP Server: Receives actuator commands, simulates servo movement, and streams sensor telemetry.
"""

import time
import json
import socket
import select
import threading
import sys
import os

# Include project root in sys.path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
from config.robot_config import ESP1_CONFIG, ESP2_CONFIG


class SingleESPSimulator(threading.Thread):
    def __init__(
        self,
        esp_id: str,
        name: str,
        tcp_port: int,
        udp_port: int,
        num_servos: int = 4,
        num_motors: int = 2,
        raspi_udp_port: int = 6000,
    ):
        super().__init__(daemon=True)
        self.esp_id = esp_id
        self.name = name
        self.tcp_port = tcp_port
        self.udp_port = udp_port
        self.num_servos = num_servos
        self.num_motors = num_motors
        self.raspi_udp_port = raspi_udp_port

        # State
        self.running = True
        self.current_servos = [0.0] * self.num_servos
        self.target_servos = [0.0] * self.num_servos
        self.motors = [0] * self.num_motors
        self.battery_voltage = 12.6
        self.packet_count = 0
        self.simulated_depth = 1.8

    def run(self):
        # 1. Setup TCP Server
        tcp_server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        tcp_server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        tcp_server.bind(("0.0.0.0", self.tcp_port))
        tcp_server.listen(1)
        tcp_server.setblocking(False)

        # 2. Setup UDP Server
        udp_server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        udp_server.bind(("0.0.0.0", self.udp_port))
        udp_server.setblocking(False)

        print(f"[{self.name}] Listening on TCP port {self.tcp_port} (Status) and UDP port {self.udp_port} (Actuators)")

        client_tcp_sock = None
        last_telemetry_time = time.time()
        raspi_addr = ("127.0.0.1", self.raspi_udp_port)

        while self.running:
            # A. Check for new TCP client connection
            try:
                readable, _, _ = select.select([tcp_server], [], [], 0.0)
                if readable:
                    conn, addr = tcp_server.accept()
                    conn.setblocking(False)
                    client_tcp_sock = conn
                    print(f"[{self.name}] RasPi connected over TCP: {addr}")
            except Exception:
                pass

            # B. Read TCP messages (Heartbeat PING)
            if client_tcp_sock:
                try:
                    readable, _, _ = select.select([client_tcp_sock], [], [], 0.0)
                    if readable:
                        data = client_tcp_sock.recv(2048)
                        if not data:
                            print(f"[{self.name}] RasPi disconnected from TCP.")
                            client_tcp_sock.close()
                            client_tcp_sock = None
                        else:
                            lines = data.decode("utf-8", errors="ignore").strip().split("\n")
                            for line in lines:
                                if not line:
                                    continue
                                msg = json.loads(line)
                                if msg.get("type") == "PING":
                                    # Respond with PONG
                                    pong_resp = {
                                        "type": "PONG",
                                        "state": "READY",
                                        "timestamp": msg.get("timestamp"),
                                        "battery": round(self.battery_voltage, 2),
                                    }
                                    resp_bytes = (json.dumps(pong_resp) + "\n").encode("utf-8")
                                    client_tcp_sock.sendall(resp_bytes)
                except Exception:
                    if client_tcp_sock:
                        client_tcp_sock.close()
                        client_tcp_sock = None

            # C. Read UDP actuator commands
            try:
                readable, _, _ = select.select([udp_server], [], [], 0.0)
                if readable:
                    data, addr = udp_server.recvfrom(2048)
                    raspi_addr = addr
                    cmd = json.loads(data.decode("utf-8"))
                    self.target_servos = cmd.get("servos", self.target_servos)
                    self.motors = cmd.get("motors", self.motors)
            except Exception:
                pass

            # D. Smooth servo motion simulation
            for i in range(len(self.current_servos)):
                if i < len(self.target_servos):
                    self.current_servos[i] += 0.3 * (self.target_servos[i] - self.current_servos[i])

            # E. Send UDP Telemetry back to RasPi at 50Hz
            now = time.time()
            if now - last_telemetry_time >= 0.02:
                last_telemetry_time = now
                self.packet_count += 1

                # Calculate power consumption
                motor_load = sum(abs(m) for m in self.motors) * 0.05
                servo_load = sum(abs(s) for s in self.current_servos) * 0.01
                current_draw = 0.4 + motor_load + servo_load

                telemetry = {
                    "esp_id": self.esp_id,
                    "seq": self.packet_count,
                    "timestamp": now,
                    "voltage": round(self.battery_voltage - 0.005 * current_draw, 2),
                    "current": round(current_draw, 2),
                    "actual_servos": [round(float(s), 1) for s in self.current_servos],
                    "actual_motors": self.motors,
                    "leak_detected": False,
                }

                # ESP2 includes water depth and ballast intake percentage
                if self.esp_id == "esp2":
                    ballast_opening = (
                        sum(self.current_servos) / (len(self.current_servos) * 90.0)
                        if self.current_servos
                        else 0.0
                    )
                    self.simulated_depth = max(0.2, min(10.0, self.simulated_depth + (ballast_opening - 0.45) * 0.01))
                    telemetry["water_depth_m"] = round(float(self.simulated_depth), 2)
                    telemetry["ballast_intake_pct"] = round(float(ballast_opening * 100.0), 1)

                try:
                    udp_server.sendto(json.dumps(telemetry).encode("utf-8"), raspi_addr)
                except Exception:
                    pass

            time.sleep(0.005)


def main():
    print("==================================================")
    print("  Starting QuadKen AUV Dual-ESP Simulator        ")
    print("  ESP1: 2 BLDC Thrusters + 4 Membrane Servos     ")
    print("  ESP2: 4 Head Water-Intake Ballast Servos       ")
    print("==================================================")

    sim_esp1 = SingleESPSimulator(
        ESP1_CONFIG.esp_id,
        ESP1_CONFIG.name,
        ESP1_CONFIG.tcp_port,
        ESP1_CONFIG.udp_port,
        num_servos=ESP1_CONFIG.num_servos,
        num_motors=ESP1_CONFIG.num_motors,
    )
    sim_esp2 = SingleESPSimulator(
        ESP2_CONFIG.esp_id,
        ESP2_CONFIG.name,
        ESP2_CONFIG.tcp_port,
        ESP2_CONFIG.udp_port,
        num_servos=ESP2_CONFIG.num_servos,
        num_motors=ESP2_CONFIG.num_motors,
    )

    sim_esp1.start()
    sim_esp2.start()

    print("[Simulator] ESP1 and ESP2 are running in background. Press Ctrl+C to terminate.")
    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        print("\n[Simulator] Shutting down.")


if __name__ == "__main__":
    main()
