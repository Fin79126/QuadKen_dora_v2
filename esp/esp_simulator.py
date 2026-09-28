"""
ESP 1 & ESP 2 Microcontroller Simulator
Simulates the two physical ESPs connected to Raspberry Pi:
  - ESP1 (Front Legs): TCP 5001, UDP 6001
  - ESP2 (Rear Legs):  TCP 5002, UDP 6002
Handles:
  - TCP Server: Accepts connections from RasPi, responds to heartbeat PING with PONG.
  - UDP Server: Receives actuator commands, simulates servo movement, and sends telemetry.
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
    def __init__(self, esp_id: str, name: str, tcp_port: int, udp_port: int, raspi_udp_port: int = 6000):
        super().__init__(daemon=True)
        self.esp_id = esp_id
        self.name = name
        self.tcp_port = tcp_port
        self.udp_port = udp_port
        self.raspi_udp_port = raspi_udp_port

        # State
        self.running = True
        self.current_servos = [0.0] * 6
        self.target_servos = [0.0] * 6
        self.motors = [0, 0]
        self.battery_voltage = 12.4
        self.packet_count = 0

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
                except Exception as e:
                    if client_tcp_sock:
                        client_tcp_sock.close()
                        client_tcp_sock = None

            # C. Read UDP actuator commands
            try:
                readable, _, _ = select.select([udp_server], [], [], 0.0)
                if readable:
                    data, addr = udp_server.recvfrom(2048)
                    raspi_addr = addr  # Auto-learn RasPi UDP address
                    cmd = json.loads(data.decode("utf-8"))
                    self.target_servos = cmd.get("servos", self.target_servos)
                    self.motors = cmd.get("motors", self.motors)
            except Exception:
                pass

            # D. Smooth servo motion simulation
            for i in range(len(self.current_servos)):
                if i < len(self.target_servos):
                    # Simple low-pass filter
                    self.current_servos[i] += 0.3 * (self.target_servos[i] - self.current_servos[i])

            # E. Send UDP Telemetry back to RasPi at 50Hz
            now = time.time()
            if now - last_telemetry_time >= 0.02:
                last_telemetry_time = now
                self.packet_count += 1
                # Simulate dynamic current draw based on motor/servo load
                current_draw = 0.3 + 0.05 * sum(abs(s) for s in self.current_servos) / 90.0

                telemetry = {
                    "esp_id": self.esp_id,
                    "seq": self.packet_count,
                    "timestamp": now,
                    "voltage": round(self.battery_voltage - 0.01 * current_draw, 2),
                    "current": round(current_draw, 2),
                    "actual_servos": [round(float(s), 1) for s in self.current_servos],
                    "actual_motors": self.motors,
                    "foot_contacts": [1 if abs(self.current_servos[1]) < 10 else 0, 1],
                }
                try:
                    udp_server.sendto(json.dumps(telemetry).encode("utf-8"), raspi_addr)
                except Exception:
                    pass

            time.sleep(0.005)


def main():
    print("==================================================")
    print("      Starting QuadKen Dual-ESP Simulator         ")
    print("==================================================")

    sim_esp1 = SingleESPSimulator(
        ESP1_CONFIG.esp_id, ESP1_CONFIG.name, ESP1_CONFIG.tcp_port, ESP1_CONFIG.udp_port
    )
    sim_esp2 = SingleESPSimulator(
        ESP2_CONFIG.esp_id, ESP2_CONFIG.name, ESP2_CONFIG.tcp_port, ESP2_CONFIG.udp_port
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
