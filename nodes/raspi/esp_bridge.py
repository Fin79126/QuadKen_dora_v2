"""
Raspberry Pi ESP Bridge Node
Manages network communication with 2 ESP microcontrollers:
  - TCP Socket: Monitors connection status, health, and latency via periodic heartbeats.
  - UDP Socket: Sends high-speed actuator commands and receives telemetry without ACK delays.
Subscribes to:
  - actuator_cmd: Actuator targets from pc/compute
Publishes:
  - esp_status: TCP connection states, latencies, and health for ESP1 & ESP2
  - esp_telemetry: Real-time sensor readings received via UDP from ESP1 & ESP2
"""

import time
import json
import socket
import select
import sys
import os
import threading
from typing import Dict, Any, Optional
import pyarrow as pa
from dora import Node

# Ensure config directory is in path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "../..")))
from config.robot_config import ESP1_CONFIG, ESP2_CONFIG, DEFAULT_RASPI_IP


class ESPConnectionHandler:
    """Manages TCP health monitoring and UDP streaming for a single ESP."""

    def __init__(self, esp_id: str, host: str, tcp_port: int, udp_port: int):
        self.esp_id = esp_id
        self.host = host
        self.tcp_port = tcp_port
        self.udp_port = udp_port

        # State
        self.connected = False
        self.latency_ms = 0.0
        self.state_string = "DISCONNECTED"
        self.last_pong_time = 0.0

        # TCP Socket
        self.tcp_sock: Optional[socket.socket] = None
        self._lock = threading.Lock()

    def connect_tcp(self) -> bool:
        """Attempt non-blocking/short-timeout TCP connection to ESP."""
        try:
            if self.tcp_sock:
                try:
                    self.tcp_sock.close()
                except Exception:
                    pass

            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(0.3)
            s.connect((self.host, self.tcp_port))
            s.setblocking(False)
            self.tcp_sock = s
            self.connected = True
            self.state_string = "CONNECTED"
            self.last_pong_time = time.time()
            return True
        except Exception:
            self.connected = False
            self.state_string = "DISCONNECTED"
            self.tcp_sock = None
            return False

    def send_tcp_ping(self):
        """Send heartbeat ping over TCP."""
        if not self.tcp_sock:
            self.connect_tcp()
            return

        try:
            ping_msg = json.dumps({"type": "PING", "timestamp": time.time()}) + "\n"
            self.tcp_sock.sendall(ping_msg.encode("utf-8"))
        except Exception:
            self.connected = False
            self.state_string = "DISCONNECTED"
            if self.tcp_sock:
                try:
                    self.tcp_sock.close()
                except Exception:
                    pass
                self.tcp_sock = None

    def check_tcp_response(self):
        """Read pending TCP pong/status data from ESP."""
        if not self.tcp_sock:
            return

        try:
            readable, _, _ = select.select([self.tcp_sock], [], [], 0.0)
            if readable:
                data = self.tcp_sock.recv(4096)
                if not data:
                    # Remote disconnected
                    self.connected = False
                    self.state_string = "DISCONNECTED"
                    self.tcp_sock.close()
                    self.tcp_sock = None
                    return

                lines = data.decode("utf-8", errors="ignore").strip().split("\n")
                now = time.time()
                for line in lines:
                    if not line:
                        continue
                    try:
                        msg = json.loads(line)
                        if msg.get("type") == "PONG":
                            sent_time = msg.get("timestamp", now)
                            self.latency_ms = round(max(0.1, (now - sent_time) * 1000.0), 2)
                            self.state_string = msg.get("state", "RUNNING")
                            self.connected = True
                            self.last_pong_time = now
                    except Exception:
                        pass
        except Exception:
            self.connected = False
            self.state_string = "ERROR"

        # Check timeout
        if self.connected and (time.time() - self.last_pong_time > 1.5):
            self.connected = False
            self.state_string = "HEARTBEAT_TIMEOUT"

    def get_status_dict(self) -> Dict[str, Any]:
        return {
            "connected": self.connected,
            "latency_ms": self.latency_ms,
            "state": self.state_string,
            "host": f"{self.host}:{self.tcp_port}",
        }

    def close(self):
        if self.tcp_sock:
            try:
                self.tcp_sock.close()
            except Exception:
                pass
            self.tcp_sock = None


def main():
    node = Node()

    # Allow overriding ESP hosts from environment (useful when running distributed)
    esp1_host = os.environ.get("ESP1_HOST", DEFAULT_RASPI_IP)
    esp2_host = os.environ.get("ESP2_HOST", DEFAULT_RASPI_IP)

    esp1 = ESPConnectionHandler(
        ESP1_CONFIG.esp_id, esp1_host, ESP1_CONFIG.tcp_port, ESP1_CONFIG.udp_port
    )
    esp2 = ESPConnectionHandler(
        ESP2_CONFIG.esp_id, esp2_host, ESP2_CONFIG.tcp_port, ESP2_CONFIG.udp_port
    )

    # UDP socket for sending actuator commands & receiving telemetry
    udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp_sock.setblocking(False)
    # Bind to listen for incoming telemetry from ESPs
    try:
        udp_sock.bind(("0.0.0.0", 6000))
    except Exception as e:
        print(f"[ESP Bridge] UDP bind notice: {e}")

    last_heartbeat_time = 0.0
    last_status_pub_time = 0.0

    print(f"[ESP Bridge] Bridge initialized. Targeting ESP1 ({esp1_host}:{ESP1_CONFIG.tcp_port}) and ESP2 ({esp2_host}:{ESP2_CONFIG.tcp_port})")

    try:
        for event in node:
            event_type = event["type"]
            if event_type == "STOP":
                print("[ESP Bridge] Received STOP event. Exiting.")
                break

            now = time.time()

            # 1. Periodic TCP Heartbeat check (every 500ms)
            if now - last_heartbeat_time > 0.5:
                last_heartbeat_time = now
                esp1.send_tcp_ping()
                esp2.send_tcp_ping()

            esp1.check_tcp_response()
            esp2.check_tcp_response()

            # 2. Check incoming UDP telemetry from ESPs
            telemetry_batch = {}
            while True:
                try:
                    readable, _, _ = select.select([udp_sock], [], [], 0.0)
                    if not readable:
                        break
                    data, addr = udp_sock.recvfrom(4096)
                    telemetry = json.loads(data.decode("utf-8"))
                    esp_id = telemetry.get("esp_id")
                    if esp_id:
                        telemetry_batch[esp_id] = telemetry
                except Exception:
                    break

            if telemetry_batch:
                node.send_output(
                    "esp_telemetry", pa.array([json.dumps(telemetry_batch).encode("utf-8")])
                )

            # 3. Handle Dora Inputs (Actuator commands from PC)
            if event_type == "INPUT":
                input_id = event["id"]
                if input_id == "actuator_cmd":
                    try:
                        raw_val = event["value"].to_pylist()[0]
                        cmd_dict = json.loads(raw_val if isinstance(raw_val, str) else raw_val.decode("utf-8"))

                        # Route actuator commands to ESP1 via UDP
                        if "esp1" in cmd_dict:
                            esp1_packet = {
                                "seq": cmd_dict.get("seq", 0),
                                "robot_state": cmd_dict.get("robot_state", "STAND"),
                                "servos": cmd_dict["esp1"].get("servos", []),
                                "motors": cmd_dict["esp1"].get("motors", []),
                            }
                            udp_sock.sendto(
                                json.dumps(esp1_packet).encode("utf-8"),
                                (esp1.host, esp1.udp_port),
                            )

                        # Route actuator commands to ESP2 via UDP
                        if "esp2" in cmd_dict:
                            esp2_packet = {
                                "seq": cmd_dict.get("seq", 0),
                                "robot_state": cmd_dict.get("robot_state", "STAND"),
                                "servos": cmd_dict["esp2"].get("servos", []),
                                "motors": cmd_dict["esp2"].get("motors", []),
                            }
                            udp_sock.sendto(
                                json.dumps(esp2_packet).encode("utf-8"),
                                (esp2.host, esp2.udp_port),
                            )
                    except Exception as e:
                        pass

            # 4. Publish ESP connection status to dora (throttled to 10 Hz)
            if now - last_status_pub_time >= 0.1:
                last_status_pub_time = now
                status_payload = {
                    "timestamp": now,
                    "esp1": esp1.get_status_dict(),
                    "esp2": esp2.get_status_dict(),
                }
                node.send_output("esp_status", pa.array([json.dumps(status_payload).encode("utf-8")]))
    except KeyboardInterrupt:
        pass
    finally:
        esp1.close()
        esp2.close()
        try:
            udp_sock.close()
        except Exception:
            pass
        sys.exit(0)


if __name__ == "__main__":
    main()
