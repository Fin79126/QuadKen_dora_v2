"""
Shared configuration for QuadKen Dora v2.
Defines IP addresses, ports, protocol structures, and robot parameters.
"""

from dataclasses import dataclass, asdict
from typing import Dict, Any, List
import json


@dataclass
class ESPConfig:
    esp_id: str          # "esp1" or "esp2"
    name: str            # e.g. "Front Actuator Unit", "Rear Actuator Unit"
    tcp_port: int        # TCP port for connection status & heartbeat
    udp_port: int        # UDP port for high-speed actuator commands & sensor telemetry
    num_servos: int = 6  # Number of servos connected to this ESP
    num_motors: int = 2  # Number of motors connected to this ESP


# Network configuration
DEFAULT_RASPI_IP = "127.0.0.1"  # Replace with RasPi IP in distributed mode (e.g., 192.168.1.100)
DEFAULT_PC_IP = "127.0.0.1"     # Replace with PC IP in distributed mode (e.g., 192.168.1.50)

# ESP Network settings (Default: 127.0.0.1 for local testing / simulator)
ESP1_CONFIG = ESPConfig(
    esp_id="esp1",
    name="ESP1_Front",
    tcp_port=5001,
    udp_port=6001,
    num_servos=6,
    num_motors=2,
)

ESP2_CONFIG = ESPConfig(
    esp_id="esp2",
    name="ESP2_Rear",
    tcp_port=5002,
    udp_port=6002,
    num_servos=6,
    num_motors=2,
)

# Frequency & Timing
CONTROL_LOOP_HZ = 50       # 50 Hz control loop (20ms interval)
HEARTBEAT_INTERVAL_SEC = 0.5  # TCP ping-pong every 500ms
FAILSAFE_TIMEOUT_SEC = 1.0     # Trigger failsafe if no packet for 1.0s

# Telemetry and Command Helpers
def serialize_json(data: Any) -> bytes:
    """Serialize Python dictionary or dataclass to JSON bytes."""
    if hasattr(data, "__dict__"):
        data = asdict(data)
    return json.dumps(data).encode("utf-8")


def deserialize_json(raw_bytes: bytes) -> Dict[str, Any]:
    """Deserialize JSON bytes to Python dictionary."""
    try:
        return json.loads(raw_bytes.decode("utf-8"))
    except Exception:
        return {}
