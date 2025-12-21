"""Multi-robot device simulator built on top of local XBee adapters.

This script discovers connected XBee radios, matches them against the
robots declared in ``robots.json``, and simulates the behaviour of each
robot concurrently. Incoming request frames are decoded using
``availableMessages.json`` so that both request and response handling
stay aligned with the control system contract.
"""

from __future__ import annotations

import argparse
import json
import logging
import random
import signal
import threading
import time
from contextlib import suppress
from datetime import datetime, timezone
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple

import serial.tools.list_ports  # type: ignore[import]
from digi.xbee.devices import RemoteXBeeDevice, XBeeDevice  # type: ignore[import]
from digi.xbee.exception import TimeoutException  # type: ignore[import]
from digi.xbee.models.address import XBee64BitAddress  # type: ignore[import]

from solar_simulator import SolarConfig, SolarIrradianceSimulator

GREEN = "\033[92m"
PINK = "\033[95m"
BLUE = "\033[94m"
PURPLE = "\033[35m"
TURQUOISE = "\033[96m"
RESET = "\033[0m"
BAUD_RATE = 9600
SYNC_TIMEOUT = 1.0
DEFAULT_RESPONSE_DELAY = 3.0
DEFAULT_SEND_TIMEOUT = 1.0
WRITE_ACTION_KEY = "write-ctrl-mac-address"
ACK_RESPONSE_KEY = "mac-succesfully-written"
ACK_KEY = ACK_RESPONSE_KEY
TYPE_CODE_BY_TYPE = {
    "SMD": 0x01,
    "SST": 0x02,
}
DISPLACEMENT_ACTIONS = {
    "play",
    "play-reverse",
    "to-start",
    "stop",
    "forward-no-brush",
    "back",
    "back-reverse",
    "backward-no-brush",
    "move-cw-no-brush",
    "move-ccw-no-brush",
    "move-to-(45)",
    "move-to-(-45)",
    "move-to-(90)",
    "move-to-(-90)",
    "jog-dir1-brush-on",
    "jog-dir2-brush-on",
    "jog-dir1-brush-off",
    "jog-dir2-brush-off",
    "jog-stop",
}
INFORMATION_REQUESTS = {
    "battery",
    "total-current",
    "encoder",
    "gearbox-current",
    "brush1-current",
    "brush2-current",
    "robot-staus",
    "robot-cycles",
    "robot-status-and-cycles",
    "robot-pyr-readings",
    "request-all-data",
}
CYCLE_INCREMENT_EXCLUDED_ACTIONS = {
    "to-start",
    "back",
    "back-reverse",
    "move-to-(45)",
    "move-to-(-45)",
    "move-to-(90)",
    "move-to-(-90)",
}
BROADCAST_ADDRESS = "000000000000FFFF"
remote_write_registry: Dict[str, Dict[str, Any]] = {}
ROBOTS_FILE_PATH: Optional[Path] = None
ROBOTS_FILE_MTIME: Optional[float] = None
ROBOTS_CACHE_BY_MAC: Dict[str, Dict[str, Any]] = {}
LOW_BATTERY_REGISTRY: Dict[str, bool] = {}


class MessageLoggingFilter(logging.Filter):
    """Filter to silence XBee-related logs when disabled."""

    def __init__(self) -> None:
        super().__init__()
        self._enabled = True

    def set_enabled(self, enabled: bool) -> None:
        self._enabled = enabled

    def filter(self, record: logging.LogRecord) -> bool:
        if self._enabled:
            return True

        if record.name.startswith("digi.xbee"):
            return False

        message = record.getMessage()
        if "XBee" in message or "SST" in message or "SMD" in message or "RX " in message or "TX " in message:
            return False

        return True


MESSAGE_LOG_FILTER = MessageLoggingFilter()
SEND_TIMEOUT = DEFAULT_SEND_TIMEOUT


@dataclass
class Robot:
    """Robot metadata sourced from robots.json."""

    id: str
    mac: str
    type: str
    description: str
    scada_id: str
    low_battery: bool = False
    simulated_cycles: int = 0

    @classmethod
    def from_dict(cls, payload: Dict[str, str]) -> "Robot":
        return cls(
            id=payload["id"],
            mac=payload["mac"].upper(),
            type=payload["type"].upper(),
            description=payload.get("description", ""),
            scada_id=payload.get("scadaId", ""),
            low_battery=_coerce_bool(payload.get("low-battery") or payload.get("lowBattery"), False),
            simulated_cycles=_parse_simulated_cycles(
                payload.get("simulated-robot-cycles")
                or payload.get("simulatedRobotCycles")
                or payload.get("cycles"),
            ),
        )


@dataclass
class Match:
    """Represents a connected XBee module associated to a robot."""

    robot: Robot
    port: str
    mac: str
    node_id: str


def resolve_existing_path(candidate_paths: Iterable[Path]) -> Path:
    for path in candidate_paths:
        if path.is_file():
            return path
    raise FileNotFoundError("None of the candidate paths exist: {}".format(", ".join(map(str, candidate_paths))))


def _coerce_bool(value: Any, default: bool) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        lowered = value.strip().lower()
        if lowered in {"true", "1", "yes", "on"}:
            return True
        if lowered in {"false", "0", "no", "off"}:
            return False
        return default
    if isinstance(value, (int, float)):
        return value != 0
    return default


def _parse_simulated_cycles(value: Any) -> int:
    if value is None:
        return 0
    if isinstance(value, str):
        stripped = value.strip()
        if stripped == "":
            return 0
        try:
            numeric = float(stripped)
        except ValueError:
            return 0
    else:
        try:
            numeric = float(value)
        except (TypeError, ValueError):
            return 0
    try:
        return max(0, int(round(numeric)))
    except (TypeError, ValueError):
        return 0


def load_config(project_root: Path) -> Dict[str, Any]:
    config_path = project_root / "config" / "device_simulator.json"
    if not config_path.is_file():
        logging.debug("Config file %s not found; using defaults", config_path)
        return {}
    with config_path.open("r", encoding="utf-8") as fh:
        return json.load(fh)


def load_available_messages(project_root: Path, config: Dict[str, Any]) -> Dict[str, Dict[str, Dict[str, str]]]:
    path_override = config.get("availableMessagesPath")
    if isinstance(path_override, str) and path_override.strip():
        override_path = Path(path_override)
        if not override_path.is_absolute():
            override_path = (project_root / override_path).resolve()
        if override_path.is_file():
            logging.info("Using availableMessages.json from config override: %s", override_path)
            with override_path.open("r", encoding="utf-8") as fh:
                return json.load(fh)
        else:
            logging.warning("Configured availableMessages path %s not found; falling back to defaults", override_path)

    workspace_root = project_root.parent
    candidates = [
        workspace_root / "it-gateway" / "config" / "availableMessages.json",
        project_root / "config" / "availableMessages.json",
    ]
    path = resolve_existing_path(candidates)
    with path.open("r", encoding="utf-8") as fh:
        return json.load(fh)


latest_displacement_status: Dict[str, Dict[str, Any]] = {}
simulated_cycles_registry: Dict[str, int] = {}
previous_displacement_action: Dict[str, str] = {}


def _gauss(mean: float, std_dev: float, minimum: Optional[float] = None, maximum: Optional[float] = None) -> float:
    value = random.gauss(mean, std_dev)
    if minimum is not None:
        value = max(minimum, value)
    if maximum is not None:
        value = min(maximum, value)
    return value


def _clamp_uint16(value: int) -> int:
    return max(0, min(0xFFFF, value))


def _clamp_int16(value: int) -> int:
    return max(-0x8000, min(0x7FFF, value))


def _clamp_uint32(value: int) -> int:
    return max(0, min(0xFFFFFFFF, value))


def _split_uint16(value: int) -> List[int]:
    safe_value = value & 0xFFFF
    # Big-endian ordering to match on-wire encoding
    return [(safe_value >> 8) & 0xFF, safe_value & 0xFF]


def _split_uint32(value: int) -> List[int]:
    safe_value = value & 0xFFFFFFFF
    return [
        (safe_value >> 24) & 0xFF,
        (safe_value >> 16) & 0xFF,
        (safe_value >> 8) & 0xFF,
        safe_value & 0xFF,
    ]


def _generate_status_telemetry(
    mac: str,
    status_record: Dict[str, Any],
    solar_simulator: Optional[SolarIrradianceSimulator],
) -> Tuple[List[int], Dict[str, Any]]:
    status_key = str(status_record.get('key') or '').lower()
    is_working = status_key == 'working'

    angle_deg = _gauss(45.0, 5.0, minimum=0.0, maximum=180.0)
    angle_word = _clamp_int16(int(round(angle_deg * 100))) & 0xFFFF

    voltage_v = _gauss(24.0, 0.25, minimum=0.0)
    voltage_word = _clamp_int16(int(round(voltage_v * 100))) & 0xFFFF

    if is_working:
        total_current = _gauss(2.20, 0.55, minimum=0.0)
        gearbox_current = _gauss(1.5, 0.55, minimum=0.0)
        brush_mean = 0.35
        brush_std = 0.55
    else:
        total_current = _gauss(0.42, 0.01, minimum=0.0)
        gearbox_current = _gauss(0.0, 0.15, minimum=0.0)
        brush_mean = 0.0
        brush_std = 0.15

    brush1_current = _gauss(brush_mean, brush_std, minimum=0.0)
    brush2_current = _gauss(brush_mean, brush_std, minimum=0.0)

    total_current_word = _clamp_int16(int(round(total_current * 100))) & 0xFFFF
    gearbox_word = _clamp_int16(int(round(gearbox_current * 100))) & 0xFFFF
    brush1_word = _clamp_int16(int(round(brush1_current * 100))) & 0xFFFF
    brush2_word = _clamp_int16(int(round(brush2_current * 100))) & 0xFFFF

    cycles_value = get_simulated_cycles(mac)
    cycles_word = _clamp_uint32(cycles_value)

    irradiance_reading = None
    if solar_simulator is not None:
        try:
            irradiance_reading = solar_simulator.compute()
        except Exception:
            irradiance_reading = solar_simulator.last_reading

    irradiance_w_m2 = max(0.0, getattr(irradiance_reading, 'irradiance_w_m2', 0.0))
    # Add small stochastic variation so values don't stick at zero when light is low
    irradiance_w_m2 = max(0.0, irradiance_w_m2 + _gauss(0.0, 5.0))
    irradiance_word = _clamp_uint32(int(round(irradiance_w_m2 * 100)))

    # Pyranometer 1 data
    pyr_temperature_c = _gauss(25.0, 5.0)
    pyr_temperature_word = _clamp_int16(int(round(pyr_temperature_c * 100))) & 0xFFFF

    pyr_voltage_v = _gauss(5.0, 0.25, minimum=0.0)
    pyr_voltage_word = _clamp_int16(int(round(pyr_voltage_v * 100))) & 0xFFFF

    # Pyranometer 2 data
    irradiance2_reading = None
    if solar_simulator is not None:
        try:
            irradiance2_reading = solar_simulator.compute()
        except Exception:
            irradiance2_reading = solar_simulator.last_reading
    irradiance2_w_m2 = max(0.0, getattr(irradiance2_reading, 'irradiance_w_m2', 0.0))
    # Add slight variation for second pyranometer
    irradiance2_w_m2 = max(0.0, irradiance2_w_m2 + _gauss(0.0, 5.0))
    irradiance2_word = _clamp_uint32(int(round(irradiance2_w_m2 * 100)))

    pyr2_temperature_c = _gauss(25.0, 5.0)
    pyr2_temperature_word = _clamp_int16(int(round(pyr2_temperature_c * 100))) & 0xFFFF

    pyr2_voltage_v = _gauss(5.0, 0.25, minimum=0.0)
    pyr2_voltage_word = _clamp_int16(int(round(pyr2_voltage_v * 100))) & 0xFFFF

    telemetry_bytes: List[int] = []
    telemetry_bytes.extend(_split_uint16(angle_word))
    telemetry_bytes.extend(_split_uint16(voltage_word))
    telemetry_bytes.extend(_split_uint16(total_current_word))
    telemetry_bytes.extend(_split_uint16(gearbox_word))
    telemetry_bytes.extend(_split_uint16(brush1_word))
    telemetry_bytes.extend(_split_uint16(brush2_word))
    telemetry_bytes.extend(_split_uint32(cycles_word))
    # Pyranometer 1 data
    telemetry_bytes.extend(_split_uint32(irradiance_word))
    telemetry_bytes.extend(_split_uint16(pyr_temperature_word))
    telemetry_bytes.extend(_split_uint16(pyr_voltage_word))
    # Pyranometer 2 data
    telemetry_bytes.extend(_split_uint32(irradiance2_word))
    telemetry_bytes.extend(_split_uint16(pyr2_temperature_word))
    telemetry_bytes.extend(_split_uint16(pyr2_voltage_word))

    telemetry_info = {
        "angle_deg": round(angle_deg, 2),
        "voltage_v": round(voltage_v, 2),
        "total_current_a": round(total_current, 2),
        "gearbox_current_a": round(gearbox_current, 2),
        "brush1_current_a": round(brush1_current, 2),
        "brush2_current_a": round(brush2_current, 2),
        "cycles": cycles_value,
        "irradiance_w_m2": round(irradiance_w_m2, 2),
        "pyr_temperature_c": round(pyr_temperature_c, 2),
        "pyr_voltage_v": round(pyr_voltage_v, 2),
        "irradiance2_w_m2": round(irradiance2_w_m2, 2),
        "pyr2_temperature_c": round(pyr2_temperature_c, 2),
        "pyr2_voltage_v": round(pyr2_voltage_v, 2),
    }

    return telemetry_bytes, telemetry_info


def _generate_smd_status_telemetry(
    mac: str,
    status_record: Dict[str, Any],
) -> Tuple[List[int], Dict[str, Any]]:
    status_key = str(status_record.get('key') or '').lower()
    is_working = status_key == 'working'

    voltage_v = _gauss(24.0, 0.25, minimum=0.0)
    charging_voltage_v = max(voltage_v, _gauss(26.5, 0.35, minimum=0.0))

    if is_working:
        motor_high_mean = 2.2
        motor_low_mean = 1.8
        temp_base = 35.0
    else:
        motor_high_mean = 0.35
        motor_low_mean = 0.25
        temp_base = 27.5

    motor22_current = _gauss(motor_high_mean, 0.35, minimum=0.0)
    motor21_current = _gauss(motor_low_mean, 0.30, minimum=0.0)
    motor12_current = _gauss(motor_high_mean, 0.35, minimum=0.0)
    motor11_current = _gauss(motor_low_mean, 0.30, minimum=0.0)

    motor2_drivers_temp_c = _gauss(temp_base, 3.0)
    motor1_drivers_temp_c = _gauss(temp_base - 0.5, 3.0)

    cycles_value = get_simulated_cycles(mac)
    cycles_word = _clamp_uint32(cycles_value)

    telemetry_bytes: List[int] = []
    telemetry_bytes.extend(_split_uint16(_clamp_int16(int(round(voltage_v * 100))) & 0xFFFF))
    telemetry_bytes.extend(_split_uint16(_clamp_int16(int(round(charging_voltage_v * 100))) & 0xFFFF))
    telemetry_bytes.extend(_split_uint16(_clamp_int16(int(round(motor22_current * 100))) & 0xFFFF))
    telemetry_bytes.extend(_split_uint16(_clamp_int16(int(round(motor21_current * 100))) & 0xFFFF))
    telemetry_bytes.extend(_split_uint16(_clamp_int16(int(round(motor12_current * 100))) & 0xFFFF))
    telemetry_bytes.extend(_split_uint16(_clamp_int16(int(round(motor11_current * 100))) & 0xFFFF))
    telemetry_bytes.extend(_split_uint16(_clamp_int16(int(round(motor2_drivers_temp_c * 100))) & 0xFFFF))
    telemetry_bytes.extend(_split_uint16(_clamp_int16(int(round(motor1_drivers_temp_c * 100))) & 0xFFFF))
    telemetry_bytes.extend(_split_uint32(cycles_word))

    telemetry_info = {
        "voltage_v": round(voltage_v, 2),
        "charging_voltage_v": round(charging_voltage_v, 2),
        "motor22_current_a": round(motor22_current, 2),
        "motor21_current_a": round(motor21_current, 2),
        "motor12_current_a": round(motor12_current, 2),
        "motor11_current_a": round(motor11_current, 2),
        "motor2_drivers_temp_c": round(motor2_drivers_temp_c, 2),
        "motor1_drivers_temp_c": round(motor1_drivers_temp_c, 2),
        "cycles": cycles_value,
    }

    return telemetry_bytes, telemetry_info


def _format_telemetry_summary(telemetry: Optional[Dict[str, Any]]) -> str:
    if not isinstance(telemetry, dict) or not telemetry:
        return ""
    labels = [
        ("angle_deg", "angle"),
        ("voltage_v", "voltage"),
        ("total_current_a", "total"),
        ("gearbox_current_a", "gearbox"),
        ("brush1_current_a", "brush1"),
        ("brush2_current_a", "brush2"),
        ("cycles", "cycles"),
        ("irradiance_w_m2", "irradiance"),
        ("pyr_temperature_c", "pyr_temp"),
        ("pyr_voltage_v", "pyr_voltage"),
        ("charging_voltage_v", "charging_voltage"),
        ("motor22_current_a", "motor22"),
        ("motor21_current_a", "motor21"),
        ("motor12_current_a", "motor12"),
        ("motor11_current_a", "motor11"),
        ("motor2_drivers_temp_c", "drv2_temp"),
        ("motor1_drivers_temp_c", "drv1_temp"),
    ]
    parts: List[str] = []
    for key, label in labels:
        value = telemetry.get(key)
        if value is None:
            continue
        if isinstance(value, float):
            parts.append(f"{label}={value:.2f}")
        else:
            parts.append(f"{label}={value}")
    return ", ".join(parts)


def update_displacement_status(mac: str, status_key: Optional[str], status_value: Optional[int], action_key: Optional[str] = None) -> None:
    if status_value is None:
        return
    latest_displacement_status[mac] = {
        "key": status_key,
        "value": status_value,
        "action": action_key,
    }


def get_displacement_status(mac: str) -> Optional[Dict[str, Any]]:
    return latest_displacement_status.get(mac)


def get_standby_status_value(robot_type: str, available_messages: Dict[str, Any]) -> Optional[int]:
    _, _, statuses_section, data_section = get_rep_sections(available_messages, robot_type)
    _, status_entries = parse_status_section(statuses_section, data_section)
    stand_by_entry = status_entries.get("stand-by") if isinstance(status_entries, dict) else None
    if stand_by_entry and "value" in stand_by_entry:
        return stand_by_entry["value"]
    return None


def initialise_status_for_robot(robot: Robot, available_messages: Dict[str, Any]) -> None:
    _, actions_section, statuses_section, data_section = get_rep_sections(available_messages, robot.type)
    _, status_entries = parse_status_section(statuses_section, data_section)
    _, action_entries = parse_action_section(actions_section, data_section)

    low_battery_active = getattr(robot, "low_battery", False)
    target_status_key = "error" if low_battery_active else "stand-by"
    status_entry = status_entries.get(target_status_key) or next(iter(status_entries.values()), None)

    target_action_key = "low-battery" if low_battery_active else "none"
    action_entry = action_entries.get(target_action_key) if isinstance(action_entries, dict) else None
    action_key = target_action_key if action_entry is None else action_entry.get("key", target_action_key)

    if status_entry and "value" in status_entry:
        update_displacement_status(robot.mac, status_entry.get("key"), status_entry.get("value"), action_key)


def initialise_robot_statuses(robots: Iterable[Robot], available_messages: Dict[str, Any]) -> None:
    latest_displacement_status.clear()
    for robot in robots:
        try:
            initialise_status_for_robot(robot, available_messages)
        except Exception:
            continue


def load_response_sequences(project_root: Path, config: Dict[str, Any]) -> Dict[str, Any]:
    path_override = config.get("responseSequencesPath")
    if isinstance(path_override, str) and path_override.strip():
        override_path = Path(path_override)
        if not override_path.is_absolute():
            override_path = (project_root / override_path).resolve()
        if override_path.is_file():
            logging.info("Using response sequences from config override: %s", override_path)
            with override_path.open("r", encoding="utf-8") as fh:
                return json.load(fh)
        logging.warning(
            "Configured response sequences path %s not found; falling back to defaults",
            override_path,
        )

    candidates = [
        project_root / "config" / "response_sequences.json",
    ]
    path = resolve_existing_path(candidates)
    with path.open("r", encoding="utf-8") as fh:
        return json.load(fh)


def get_rep_sections(available_messages: Dict[str, Any], robot_type: str) -> tuple[Dict[str, Any], Dict[str, Any], Dict[str, Any], Dict[str, Any]]:
    rep_root = get_response_root(available_messages, robot_type) or {}
    data_section = rep_root.get("data", {}) or {}
    sub_id_section = data_section.get("sub-id") if isinstance(data_section, dict) else {}
    actions_section = {}
    statuses_section = {}
    if isinstance(sub_id_section, dict):
        actions_section = sub_id_section.get("actions") or {}
        statuses_section = sub_id_section.get("statuses") or {}
    if not isinstance(actions_section, dict):
        actions_section = rep_root.get("actions", {}) or {}
    if not isinstance(statuses_section, dict):
        statuses_section = rep_root.get("statuses", {}) or {}
    return rep_root, actions_section, statuses_section, data_section


def parse_status_section(
    statuses_section: Dict[str, Any],
    data_section: Optional[Dict[str, Any]] = None,
) -> tuple[Optional[int], Dict[str, Dict[str, Any]]]:
    header_hex: Optional[str] = None
    if isinstance(data_section, dict):
        id_section = data_section.get("id")
        if isinstance(id_section, dict):
            header_hex = extract_rep_value(id_section.get("all-data"))
    if not header_hex:
        header_hex = extract_rep_value(statuses_section.get("value"))
    header = int(header_hex, 16) if header_hex else None
    entries: Dict[str, Dict[str, Any]] = {}
    for key, descriptor in statuses_section.items():
        if key == "value":
            continue
        value_hex = extract_rep_value(descriptor)
        if not value_hex:
            continue
        entries[key] = {
            "key": key,
            "value": int(value_hex, 16),
            "descriptor": descriptor,
            "color": descriptor.get("color") if isinstance(descriptor, dict) else None,
            "blinking": bool(descriptor.get("blinking")) if isinstance(descriptor, dict) else False,
        }
    return header, entries


def parse_action_section(
    actions_section: Dict[str, Any],
    data_section: Optional[Dict[str, Any]] = None,
) -> tuple[Optional[int], Dict[str, Dict[str, Any]]]:
    header_hex: Optional[str] = None
    if isinstance(data_section, dict):
        id_section = data_section.get("id")
        if isinstance(id_section, dict):
            header_hex = extract_rep_value(id_section.get("action-response"))
    if not header_hex:
        header_hex = extract_rep_value(actions_section.get("value"))
    header = int(header_hex, 16) if header_hex else None
    entries: Dict[str, Dict[str, Any]] = {}
    for key, descriptor in actions_section.items():
        if key == "value":
            continue
        value_hex = extract_rep_value(descriptor)
        if not value_hex:
            continue
        entries[key] = {
            "key": key,
            "value": int(value_hex, 16),
            "descriptor": descriptor,
            "color": descriptor.get("color") if isinstance(descriptor, dict) else None,
            "blinking": bool(descriptor.get("blinking")) if isinstance(descriptor, dict) else False,
        }
    return header, entries


def format_label(key: Optional[str]) -> str:
    if not key:
        return ''
    return str(key).replace('-', ' ').replace('_', ' ').title()


def get_rep_entry(rep_section: Dict[str, Any], key: str) -> Optional[Any]:
    if not isinstance(rep_section, dict):
        return None
    for group_name in ("actions", "statuses", "communication", "communication:", "data"):
        group = rep_section.get(group_name)
        if group_name == "data":
            if not isinstance(group, dict):
                continue
            id_group = group.get("id")
            if isinstance(id_group, dict) and key in id_group:
                return id_group[key]
            sub_id_group = group.get("sub-id")
            if isinstance(sub_id_group, dict):
                for nested in sub_id_group.values():
                    if isinstance(nested, dict) and key in nested:
                        return nested[key]
            continue
        if isinstance(group, dict) and key in group:
            return group[key]
    return rep_section.get(key)


def iter_rep_entries(rep_section: Dict[str, Any]):
    if not isinstance(rep_section, dict):
        return
    for group_name in ("actions", "statuses", "communication", "communication:", "data"):
        group = rep_section.get(group_name)
        if group_name == "data":
            if not isinstance(group, dict):
                continue
            id_group = group.get("id") if isinstance(group.get("id"), dict) else {}
            for key, value in id_group.items():
                if key == "value":
                    continue
                yield key, value
            sub_id_group = group.get("sub-id")
            if isinstance(sub_id_group, dict):
                for nested in sub_id_group.values():
                    if not isinstance(nested, dict):
                        continue
                    for key, value in nested.items():
                        if key == "value":
                            continue
                        yield key, value
            continue
        if not isinstance(group, dict):
            continue
        for key, value in group.items():
            if key == "value":
                continue
            yield key, value
    for key, value in rep_section.items():
        if key in {"actions", "statuses", "communication", "data"}:
            continue
        yield key, value


def load_robots(project_root: Path) -> List[Robot]:
    candidates = [project_root / "data" / "robots.json"]
    path = resolve_existing_path(candidates)
    global ROBOTS_FILE_PATH
    ROBOTS_FILE_PATH = path
    try:
        global ROBOTS_FILE_MTIME
        ROBOTS_FILE_MTIME = path.stat().st_mtime
    except OSError:
        ROBOTS_FILE_MTIME = None
    with path.open("r", encoding="utf-8") as fh:
        contents = json.load(fh)
    robots_payload = contents.get("robots", [])
    robots = []
    for payload in robots_payload:
        try:
            robot = Robot.from_dict(payload)
            robots.append(robot)
            simulated_cycles_registry[robot.mac] = robot.simulated_cycles
            LOW_BATTERY_REGISTRY[robot.mac.upper()] = bool(robot.low_battery)
            ROBOTS_CACHE_BY_MAC[robot.mac.upper()] = payload
        except KeyError as exc:
            logging.warning("Skipping robot entry missing key %s: %s", exc, payload)
    return robots


def persist_control_mac(robot_mac: str, controller_mac: str, messages_enabled: bool) -> None:
    if not ROBOTS_FILE_PATH:
        if messages_enabled:
            logging.debug("No robots.json path known; skipping persistence for %s", robot_mac)
        return

    try:
        with ROBOTS_FILE_PATH.open("r", encoding="utf-8") as fh:
            payload = json.load(fh)
    except (OSError, json.JSONDecodeError) as error:
        logging.warning("Failed to read robots file for MAC persistence: %s", error)
        return

    robots = payload.get("robots")
    if not isinstance(robots, list):
        logging.warning("Unexpected robots payload while persisting MAC; skipping")
        return

    target_mac = robot_mac.upper()
    updated = False
    for entry in robots:
        mac = entry.get("mac")
        if isinstance(mac, str) and mac.upper() == target_mac:
            if entry.get("control-mac") != controller_mac:
                entry["control-mac"] = controller_mac
                updated = True
            break
    else:
        logging.debug("Robot MAC %s not found in robots.json; skipping control MAC persistence", robot_mac)

    if not updated:
        return

    payload["updatedAt"] = datetime.now(timezone.utc).isoformat()
    try:
        with ROBOTS_FILE_PATH.open("w", encoding="utf-8") as fh:
            json.dump(payload, fh, indent=2)
            fh.write("\n")
    except OSError as error:
        logging.warning("Failed to write robots file for MAC persistence: %s", error)


def get_simulated_cycles(mac: str) -> int:
    return simulated_cycles_registry.get(mac.upper(), 0)


def _set_robot_status_action(
    robot: Robot,
    available_messages: Dict[str, Any],
    status_key: str,
    action_key: str,
) -> None:
    _, actions_section, statuses_section, data_section = get_rep_sections(available_messages, robot.type)
    _, status_entries = parse_status_section(statuses_section, data_section)
    _, action_entries = parse_action_section(actions_section, data_section)
    status_entry = status_entries.get(status_key)
    action_entry = action_entries.get(action_key) if isinstance(action_entries, dict) else None
    if status_entry and "value" in status_entry and action_entry and "value" in action_entry:
        update_displacement_status(robot.mac, status_entry.get("key"), status_entry.get("value"), action_entry.get("key"))


def _refresh_robot_flags_from_file(
    available_messages: Dict[str, Any],
    robot: Robot,
) -> None:
    """
    Reload the robots.json file when it changes on disk so toggling flags like
    low-battery is picked up without restarting the simulator.
    """
    if not ROBOTS_FILE_PATH or not ROBOTS_FILE_PATH.is_file():
        return
    global ROBOTS_FILE_MTIME, ROBOTS_CACHE_BY_MAC
    try:
        mtime = ROBOTS_FILE_PATH.stat().st_mtime
    except OSError:
        return
    cache_stale = ROBOTS_FILE_MTIME is None or mtime > ROBOTS_FILE_MTIME
    if cache_stale:
        try:
            with ROBOTS_FILE_PATH.open("r", encoding="utf-8") as fh:
                payload = json.load(fh)
        except Exception:
            return
        robots_payload = payload.get("robots")
        if not isinstance(robots_payload, list):
            return
        ROBOTS_CACHE_BY_MAC = {}
        for entry in robots_payload:
            mac = entry.get("mac")
            if isinstance(mac, str):
                ROBOTS_CACHE_BY_MAC[mac.upper()] = entry
        ROBOTS_FILE_MTIME = mtime

    mac_upper = robot.mac.upper()
    entry = ROBOTS_CACHE_BY_MAC.get(mac_upper)
    if not entry:
        return
    new_low_battery = _coerce_bool(entry.get("low-battery") or entry.get("lowBattery"), False)
    prev_low_battery = LOW_BATTERY_REGISTRY.get(mac_upper)
    if prev_low_battery is not None and prev_low_battery == new_low_battery:
        return

    robot.low_battery = new_low_battery
    LOW_BATTERY_REGISTRY[mac_upper] = new_low_battery

    # Adjust in-memory status to match toggled flag for immediate effect
    if new_low_battery:
        _set_robot_status_action(robot, available_messages, "error", "low-battery")
    else:
        _set_robot_status_action(robot, available_messages, "stand-by", "none")


def persist_simulated_cycles(robot_mac: str, cycles: int, messages_enabled: bool) -> None:
    if not ROBOTS_FILE_PATH:
        if messages_enabled:
            logging.debug("No robots.json path known; skipping cycles persistence for %s", robot_mac)
        return

    try:
        with ROBOTS_FILE_PATH.open("r", encoding="utf-8") as fh:
            payload = json.load(fh)
    except (OSError, json.JSONDecodeError) as error:
        logging.warning("Failed to read robots file for cycles persistence: %s", error)
        return

    robots = payload.get("robots")
    if not isinstance(robots, list):
        logging.warning("Unexpected robots payload while persisting cycles; skipping")
        return

    target_mac = robot_mac.upper()
    updated = False
    for entry in robots:
        mac = entry.get("mac")
        if isinstance(mac, str) and mac.upper() == target_mac:
            if entry.get("simulated-robot-cycles") != cycles:
                entry["simulated-robot-cycles"] = cycles
                updated = True
            break
    else:
        if messages_enabled:
            logging.debug("Robot MAC %s not found in robots.json; skipping cycles persistence", robot_mac)

    if not updated:
        return

    payload["updatedAt"] = datetime.now(timezone.utc).isoformat()
    try:
        with ROBOTS_FILE_PATH.open("w", encoding="utf-8") as fh:
            json.dump(payload, fh, indent=2)
            fh.write("\n")
    except OSError as error:
        logging.warning("Failed to write robots file for cycles persistence: %s", error)


def increment_simulated_cycles(robot: Robot, messages_enabled: bool) -> int:
    current = get_simulated_cycles(robot.mac)
    next_value = current + 1
    simulated_cycles_registry[robot.mac.upper()] = next_value
    robot.simulated_cycles = next_value
    persist_simulated_cycles(robot.mac, next_value, messages_enabled)
    if messages_enabled:
        logging.info("[Cycles] %s simulated cycles updated to %d", robot.mac, next_value)
    return next_value


def load_solar_config(config: Dict[str, Any]) -> SolarConfig:
    solar_defaults = SolarConfig()
    solar_config = config.get("solar") or {}

    def _get_float(key: str, fallback: float) -> float:
        value = solar_config.get(key)
        try:
            return float(value)
        except (TypeError, ValueError):
            return fallback

    return SolarConfig(
        latitude_degrees=_get_float("latitudeDegrees", solar_defaults.latitude_degrees),
        longitude_degrees=_get_float("longitudeDegrees", solar_defaults.longitude_degrees),
        clearness_factor=_get_float("clearnessFactor", solar_defaults.clearness_factor),
        noise_std_dev=_get_float("noiseStdDev", solar_defaults.noise_std_dev),
    )


def resolve_solar_interval(config: Dict[str, Any], cli_value: Optional[float]) -> float:
    if cli_value is not None:
        return max(1.0, cli_value)

    solar_config = config.get("solar") or {}
    value = solar_config.get("logIntervalSeconds")
    try:
        numeric = float(value)
    except (TypeError, ValueError):
        numeric = 10.0
    return max(1.0, numeric)


def resolve_solar_logging_enabled(config: Dict[str, Any]) -> bool:
    solar_config = config.get("solar") or {}
    return _coerce_bool(solar_config.get("enabled"), True)


def resolve_message_logging_enabled(config: Dict[str, Any]) -> bool:
    logging_config = config.get("messageLogging") or {}
    return _coerce_bool(logging_config.get("enabled"), True)


def resolve_send_timeout(config: Dict[str, Any], cli_value: Optional[float]) -> float:
    if cli_value is not None:
        return max(0.0, cli_value)
    transmit_cfg = config.get("transmit") or {}
    value = transmit_cfg.get("timeout")
    try:
        numeric = float(value)
    except (TypeError, ValueError):
        numeric = DEFAULT_SEND_TIMEOUT
    return max(0.0, numeric)


def resolve_displacement_simulation(config: Dict[str, Any]) -> Dict[str, Any]:
    simulation_cfg = config.get("displacementSimulation") or {}
    enabled = _coerce_bool(simulation_cfg.get("enabled"), False)
    interval_raw = (
        simulation_cfg.get("timeIntervalMs")
        if "timeIntervalMs" in simulation_cfg
        else simulation_cfg.get("timeIntervalMS")
    )
    try:
        interval_ms = float(interval_raw)
    except (TypeError, ValueError):
        interval_ms = 0.0
    interval_ms = max(0.0, interval_ms)
    return {
        "enabled": enabled,
        "interval_ms": interval_ms,
        "interval_seconds": interval_ms / 1000.0,
    }


def start_solar_logger(
    simulator: SolarIrradianceSimulator,
    stop_event: threading.Event,
    interval_seconds: float,
    log_enabled: bool,
) -> threading.Thread:
    interval = max(1.0, interval_seconds)

    if not log_enabled:
        logging.info("Solar irradiance logging disabled; sampling will continue without console output.")

    def _run() -> None:
        while not stop_event.is_set():
            reading = simulator.compute()
            if log_enabled:
                message = (
                    f"Solar irradiance: {reading.irradiance_w_m2:.2f} W/m² "
                    f"(day={reading.day_of_year}, solar_time={reading.solar_time_hours:.2f} h, k_t={reading.clearness_factor:.2f})"
                )
                logging.info("%s%s%s", GREEN, message, RESET)
            if stop_event.wait(interval):
                break

    thread = threading.Thread(target=_run, name="SolarLogger", daemon=True)
    thread.start()
    return thread


def extract_req_value(entry: Any) -> Optional[str]:
    if isinstance(entry, dict):
        value = entry.get("value")
        if isinstance(value, str) and value.strip():
            return value.strip().upper()
        return None
    if isinstance(entry, str) and entry.strip():
        return entry.strip().upper()
    return None


def get_request_entries(available_messages: Dict[str, Any], robot_type: str) -> Dict[str, Any]:
    req_section = available_messages.get("REQ", {})
    type_section = req_section.get(robot_type, {})
    if not isinstance(type_section, dict) and isinstance(robot_type, str):
        type_section = req_section.get(robot_type.upper(), {})
    entries: Dict[str, Any] = {}
    if not isinstance(type_section, dict):
        return entries
    for group_key in ("actions", "data"):
        group = type_section.get(group_key)
        if isinstance(group, dict):
            entries.update(group)
    for key, value in type_section.items():
        if key in ("actions", "data"):
            continue
        if isinstance(value, (dict, str)):
            entries[key] = value
    return entries


def build_request_lookup(available_messages: Dict[str, Dict[str, Dict[str, str]]]) -> Dict[str, Dict[str, List[str]]]:
    lookup: Dict[str, Dict[str, List[str]]] = {}
    req_section = available_messages.get("REQ", {})
    for robot_type in req_section.keys():
        norm_type = robot_type.upper()
        per_type: Dict[str, List[str]] = {}
        entries = get_request_entries(available_messages, robot_type)
        for name, descriptor in entries.items():
            code = extract_req_value(descriptor)
            if not code:
                continue
            per_type.setdefault(code, []).append(name)
        lookup[norm_type] = per_type
    return lookup


def extract_rep_value(entry: Any) -> Optional[str]:
    if isinstance(entry, dict):
        value = entry.get("value")
        if isinstance(value, str) and value.strip():
            return value.strip().upper()
        return None
    if isinstance(entry, str) and entry.strip():
        return entry.strip().upper()
    return None


def get_response_containers(available_messages: Dict[str, Any]) -> Dict[str, Dict[str, Any]]:
    containers: Dict[str, Dict[str, Any]] = {}
    if not isinstance(available_messages, dict):
        return containers
    for top_key in ("RES", "REP"):
        section = available_messages.get(top_key)
        if not isinstance(section, dict):
            continue
        for robot_key, content in section.items():
            robot_upper = robot_key.upper() if isinstance(robot_key, str) else robot_key
            if robot_upper not in containers or not containers[robot_upper]:
                containers[robot_upper] = content or {}
    return containers


def get_response_root(available_messages: Dict[str, Any], robot_type: str) -> Dict[str, Any]:
    if not robot_type:
        return {}
    containers = get_response_containers(available_messages)
    return containers.get(robot_type.upper(), {}) or {}


def build_response_sequences(
    available_messages: Dict[str, Dict[str, Dict[str, str]]],
    sequences_payload: Optional[Dict[str, Any]] = None,
) -> Dict[str, Dict[str, Sequence[str]]]:
    req_section = available_messages.get("REQ", {})
    response_map = get_response_containers(available_messages)
    sequences_section: Dict[str, Dict[str, Sequence[str]]]

    sequences_candidates = [
        sequences_payload,
        sequences_payload.get("response_sequences") if isinstance(sequences_payload, dict) else None,
        sequences_payload.get("RESPONSE_SEQUENCES") if isinstance(sequences_payload, dict) else None,
        available_messages.get("response_sequences"),
        available_messages.get("RESPONSE_SEQUENCES"),
        available_messages.get("REP_SEQUENCES"),
    ]
    for candidate in sequences_candidates:
        if isinstance(candidate, dict) and candidate:
            sequences_section = candidate
            break
    else:
        sequences_section = {}

    sequences: Dict[str, Dict[str, Sequence[str]]] = {}

    for robot_type, per_robot in sequences_section.items():
        robot_type_upper = robot_type.upper()
        robot_req = get_request_entries(available_messages, robot_type)
        robot_rep = response_map.get(robot_type_upper, {})
        if not robot_req or not robot_rep:
            logging.warning(
                "Skipping response sequences for %s; REQ/REP definitions missing in availableMessages",
                robot_type,
            )
            continue

        result: Dict[str, Sequence[str]] = {}
        for req_key, rep_keys in per_robot.items():
            descriptor = robot_req.get(req_key)
            if descriptor is None:
                logging.warning(
                    "Request key '%s' for type %s not present in availableMessages REQ section",
                    req_key,
                    robot_type,
                )
                continue

            if isinstance(rep_keys, str):
                rep_keys_iterable = [rep_keys]
            else:
                rep_keys_iterable = list(rep_keys)

            valid_rep_keys = []
            missing_rep_keys = []
            for rep_key in rep_keys_iterable:
                descriptor_entry = get_rep_entry(robot_rep, rep_key)
                rep_value = extract_rep_value(descriptor_entry)
                if rep_value:
                    valid_rep_keys.append(rep_key)
                else:
                    missing_rep_keys.append(rep_key)

            if missing_rep_keys:
                logging.warning(
                    "REP keys %s for %s/%s missing in availableMessages REP section",
                    ", ".join(missing_rep_keys),
                    robot_type,
                    req_key,
                )

            if not valid_rep_keys:
                continue

            code_value = extract_req_value(descriptor)
            if not code_value:
                logging.warning(
                    "Request key '%s' for type %s lacks a valid value in availableMessages",
                    req_key,
                    robot_type,
                )
                continue
            result[code_value] = tuple(valid_rep_keys)

        sequences[robot_type_upper] = result

    return sequences


def build_displacement_frames(
    robot_type: str,
    action_keys: Optional[Sequence[str]],
    available_messages: Dict[str, Dict[str, Dict[str, str]]],
) -> List[Dict[str, Any]]:
    """
    Build displacement frames based ONLY on the sequence defined in response_sequences.json.
    No hardcoded behavior - the sequence file is the single source of truth.

    Special handling:
    - If action_key is "stand-by" → uses Stand By status + None action
    - All other action_keys → uses Working status + the specified action
    """
    _, actions_section, statuses_section, data_section = get_rep_sections(available_messages, robot_type)
    actions_header_hex, action_entries = parse_action_section(actions_section, data_section)
    statuses_header_hex, status_entries = parse_status_section(statuses_section, data_section)

    if actions_header_hex is None or not action_entries:
        return []
    if not status_entries:
        return []

    actions_header = actions_header_hex
    working_entry = status_entries.get('working')
    stand_by_entry = status_entries.get('stand-by')
    jog_entry = None
    for key, entry in status_entries.items():
        if key == 'value':
            continue
        if key.lower() == 'jog':
            jog_entry = entry
            break
    none_entry = action_entries.get('none')

    frames: List[Dict[str, Any]] = []
    sequence_iterable = list(action_keys or [])

    for step in sequence_iterable:
        # Special case: "stand-by" is not an action, it's a status
        # Use Stand By status + None action
        if step.lower() == 'stand-by':
            if not stand_by_entry or not none_entry:
                continue
            frames.append({
                "bytes": [actions_header, stand_by_entry['value'], none_entry['value']],
                "labels": [
                    f"actions:{format_label('none')}",
                    f"status:{format_label('stand-by')}",
                ],
                "status_info": {"key": 'stand-by', "value": stand_by_entry['value']},
                "action_info": {"key": 'none', "value": none_entry['value']},
                "action_label": format_label('stand-by'),
            })
        else:
            # Regular action: use Working status by default
            action_entry = action_entries.get(step)
            if not action_entry:
                continue

            action_key_lower = step.lower()
            status_entry = None
            if action_key_lower.startswith('jog') and jog_entry:
                status_entry = jog_entry
            else:
                status_entry = working_entry

            if not status_entry:
                continue

            frames.append({
                "bytes": [actions_header, status_entry['value'], action_entry['value']],
                "labels": [
                    f"actions:{format_label(step)}",
                    f"status:{format_label(status_entry['key'])}",
                ],
                "status_info": {"key": status_entry['key'], "value": status_entry['value']},
                "action_info": {"key": step, "value": action_entry['value']},
                "action_label": format_label(step),
            })

    return frames


def build_low_battery_response(
    robot: Robot,
    available_messages: Dict[str, Dict[str, Dict[str, str]]],
    solar_simulator: Optional[SolarIrradianceSimulator],
    prefer_action_header: bool = False,
) -> Optional[Dict[str, Any]]:
    """
    Build a single response frame indicating the robot is in a low-battery error state.

    If action headers are requested but unavailable, this falls back to the standard
    data response identifier (typically 0x3A) to ensure the controller is informed.
    """
    _, actions_section, statuses_section, data_section = get_rep_sections(available_messages, robot.type)
    status_header_hex, status_entries = parse_status_section(statuses_section, data_section)
    action_header_hex, action_entries = parse_action_section(actions_section, data_section)

    error_entry = status_entries.get("error")
    low_battery_entry = action_entries.get("low-battery") if isinstance(action_entries, dict) else None

    header = action_header_hex if prefer_action_header and action_header_hex is not None else status_header_hex

    if header is None or not error_entry or not low_battery_entry:
        return None

    payload_bytes = [header, error_entry["value"], low_battery_entry["value"]]
    action_label = format_label(low_battery_entry.get("key"))

    telemetry_info: Dict[str, Any] = {}
    if status_header_hex is not None and header == status_header_hex and robot.type.upper() == "SST":
        try:
            telemetry_bytes, telemetry_info = _generate_status_telemetry(
                robot.mac,
                {"key": error_entry.get("key"), "value": error_entry.get("value"), "action": low_battery_entry.get("key")},
                solar_simulator,
            )
            payload_bytes.extend(telemetry_bytes)
        except Exception:
            telemetry_info = {}

    update_displacement_status(robot.mac, error_entry.get("key"), error_entry.get("value"), low_battery_entry.get("key"))

    return {
        "bytes": payload_bytes,
        "labels": [
            f"status:{format_label(error_entry.get('key'))}",
            f"action:{action_label}",
        ],
        "status_info": {"key": error_entry.get("key"), "value": error_entry.get("value")},
        "action_info": {
            "key": low_battery_entry.get("key"),
            "value": low_battery_entry.get("value"),
        },
        "telemetry_info": telemetry_info,
        "action_label": action_label,
    }


def build_status_response(
    robot_type: str,
    mac: str,
    available_messages: Dict[str, Dict[str, Dict[str, str]]],
    solar_simulator: Optional[SolarIrradianceSimulator] = None,
) -> Optional[Dict[str, Any]]:
    _, actions_section, statuses_section, data_section = get_rep_sections(available_messages, robot_type)
    status_header_hex, status_entries = parse_status_section(statuses_section, data_section)
    _, action_entries = parse_action_section(actions_section, data_section)
    if status_header_hex is None or not status_entries:
        return None

    record = get_displacement_status(mac)
    if not record:
        default_entry = status_entries.get('stand-by')
        if not default_entry:
            return None
        update_displacement_status(mac, 'stand-by', default_entry['value'], 'none')
        record = {"key": 'stand-by', "value": default_entry['value'], "action": 'none'}

    key = record.get('key')
    value = record.get('value')
    if value is None:
        return None
    matching_entry = status_entries.get(key) or next((entry for entry in status_entries.values() if entry['value'] == value), None)
    label_key = matching_entry['key'] if matching_entry else key or f"0x{value:02X}"

    action_key = record.get('action') or 'none'
    action_entry = action_entries.get(action_key) if isinstance(action_entries, dict) else None
    if action_entry is None and isinstance(action_entries, dict):
        action_entry = action_entries.get('none')
    action_value = action_entry['value'] if action_entry and 'value' in action_entry else 0
    normalised_action_key = action_entry['key'] if action_entry and 'key' in action_entry else action_key or 'none'
    action_label = format_label(normalised_action_key)

    payload_bytes = [status_header_hex, value, action_value]

    telemetry_info: Dict[str, Any] = {}
    try:
        robot_type_upper = robot_type.upper() if isinstance(robot_type, str) else ""
        if robot_type_upper == "SST":
            telemetry_bytes, telemetry_info = _generate_status_telemetry(mac, record, solar_simulator)
            payload_bytes.extend(telemetry_bytes)
        elif robot_type_upper == "SMD":
            telemetry_bytes, telemetry_info = _generate_smd_status_telemetry(mac, record)
            payload_bytes.extend(telemetry_bytes)
    except Exception:
        telemetry_info = {}
    return {
        "bytes": payload_bytes,
        "labels": [
            f"status:{format_label(label_key)}",
            f"action:{action_label}",
        ],
        "status_info": {"key": label_key, "value": value},
        "action_info": {
            "key": normalised_action_key,
            "value": action_value,
        },
        "telemetry_info": telemetry_info,
        "action_label": action_label,
    }


def build_cycle_report(
    robot_type: str,
    mac: str,
    available_messages: Dict[str, Dict[str, Dict[str, str]]],
    messages_enabled: bool,
) -> Optional[Dict[str, Any]]:
    _, actions_section, statuses_section, data_section = get_rep_sections(available_messages, robot_type)
    id_section = data_section.get("id") if isinstance(data_section, dict) else {}
    cycles_entry = id_section.get("cycles") if isinstance(id_section, dict) else None
    cycles_header_hex = extract_rep_value(cycles_entry)
    if not cycles_header_hex:
        return None
    try:
        cycles_header = int(cycles_header_hex, 16)
    except ValueError:
        return None

    _, status_entries = parse_status_section(statuses_section, data_section)
    status_record = get_displacement_status(mac)
    if not status_record:
        fallback = status_entries.get('stand-by') or next(iter(status_entries.values()), None)
        if fallback:
            status_record = {"key": fallback['key'], "value": fallback['value'], "action": 'none'}
        else:
            status_record = {"key": 'stand-by', "value": 0, "action": 'none'}

    cycles_value = get_simulated_cycles(mac)
    status_value = int(status_record.get('value', 0)) & 0xFF
    action_key = status_record.get('action') or 'none'
    action_entries = actions_section
    action_entry = None
    if isinstance(action_entries, dict):
        action_entry = action_entries.get(action_key) or action_entries.get('none')
    action_value = 0
    if action_entry and isinstance(action_entry, dict):
        raw_action_value = extract_rep_value(action_entry)
        if raw_action_value:
            try:
                action_value = int(raw_action_value, 16) & 0xFF
            except ValueError:
                action_value = 0

    # Convert cycles to 4 bytes (32-bit unsigned integer, little-endian)
    # This allows cycles up to 4,294,967,295 instead of being limited to 255
    cycle_byte0 = (cycles_value >> 0) & 0xFF   # LSB (least significant byte)
    cycle_byte1 = (cycles_value >> 8) & 0xFF
    cycle_byte2 = (cycles_value >> 16) & 0xFF
    cycle_byte3 = (cycles_value >> 24) & 0xFF  # MSB (most significant byte)

    labels = [
        f"status:{format_label(status_record.get('key'))}",
        f"action:{format_label(action_key)}",
        f"cycles:{cycles_value}",
    ]

    return {
        "bytes": [cycles_header, status_value, action_value, cycle_byte0, cycle_byte1, cycle_byte2, cycle_byte3],
        "labels": labels,
        "status_info": status_record,
        "action_info": {
            "key": action_key,
            "value": action_value,
        },
        "cycle_info": {
            "value": cycles_value,
            "action_key": action_key,
            "action_value": action_value,
        },
    }


def discover_matches(robots: Iterable[Robot], messages_enabled: bool) -> List[Match]:
    robots_by_mac = {robot.mac: robot for robot in robots}
    matches: List[Match] = []
    ports = serial.tools.list_ports.comports()

    if robots_by_mac and messages_enabled:
        logging.debug(
            "robots.json MAC whitelist: %s",
            ", ".join(sorted(robots_by_mac.keys())),
        )

    for port in ports:
        if "/dev/ttyUSB" not in port.device and "/dev/ttyACM" not in port.device:
            continue

        device: Optional[XBeeDevice] = None
        try:
            device = XBeeDevice(port.device, BAUD_RATE)
            device.open(force_settings=True)
            device.set_sync_ops_timeout(SYNC_TIMEOUT)

            node_id_raw = device.get_node_id()
            node_id = node_id_raw or "(no NI)"

            if node_id_raw and node_id_raw.upper() == "ITCTRL":
                if messages_enabled:
                    logging.info("Skipping controller radio on %s (NI=ITCTRL)", port.device)
                continue

            mac = str(device.get_64bit_addr()).upper()

            if messages_enabled:
                logging.debug(
                    "Discovered XBee NI=%s MAC=%s on %s",
                    node_id,
                    mac,
                    port.device,
                )

            if mac in robots_by_mac:
                robot = robots_by_mac[mac]
                matches.append(Match(robot=robot, port=port.device, mac=mac, node_id=node_id))
                if messages_enabled:
                    logging.info(
                        "Matched robot %s (%s) on %s [NI=%s]", robot.id, robot.type, port.device, node_id
                    )
            else:
                if messages_enabled:
                    logging.info(
                        "Ignoring XBee %s on %s (NI=%s); not found in robots.json", mac, port.device, node_id
                    )
        except Exception as exc:  # Broad catch: hardware access is noisy.
            if messages_enabled:
                logging.warning("Failed to inspect port %s: %s", port.device, exc)
        finally:
            with suppress(Exception):
                if device and device.is_open():
                    device.close()

    return matches


def translate_request(
    code: str, robot_type: str, lookup: Dict[str, Dict[str, List[str]]]
) -> Optional[str]:
    per_type = lookup.get(robot_type.upper(), {})
    names = per_type.get(code.upper())
    return names[0] if names else None


def translate_response(
    code: str, robot_type: str, available_messages: Dict[str, Dict[str, Dict[str, str]]]
) -> Optional[str]:
    rep_section = get_response_root(available_messages, robot_type)
    for name, entry in iter_rep_entries(rep_section):
        rep_value = extract_rep_value(entry)
        if rep_value and rep_value.upper() == code.upper():
            return name
    return None


def translate_response_entry(
    code: int, robot_type: str, available_messages: Dict[str, Dict[str, Dict[str, str]]]
) -> Optional[Dict[str, Any]]:
    rep_section = get_response_root(available_messages, robot_type)
    for name, entry in iter_rep_entries(rep_section):
        rep_value = extract_rep_value(entry)
        if rep_value and int(rep_value, 16) == code:
            if isinstance(entry, dict) and entry is not None:
                return {
                    "status": name,
                    "descriptor": entry,
                }
            return {
                "status": name,
                "descriptor": {"value": rep_value},
            }
    return None


def prepare_response_payloads(
    robot_type: str,
    request_code: str,
    sequences: Dict[str, Dict[str, Sequence[str]]],
    available_messages: Dict[str, Dict[str, Dict[str, str]]],
    messages_enabled: bool,
    sequence_keys: Optional[Sequence[str]] = None,
) -> List[int]:
    robot_type_upper = robot_type.upper()
    rep_section = get_response_root(available_messages, robot_type)
    sequence = sequence_keys if sequence_keys is not None else sequences.get(robot_type_upper, {}).get(request_code.upper())

    if not sequence:
        if messages_enabled:
            logging.debug(
                "[%s] No response sequence configured for request %s",
                robot_type_upper,
                request_code,
            )
        return []

    payload: List[int] = []
    for rep_key in sequence:
        descriptor = get_rep_entry(rep_section, rep_key)
        rep_code = extract_rep_value(descriptor)
        if rep_code is None:
            if messages_enabled:
                logging.debug("REP key %s missing for %s during payload build", rep_key, robot_type)
            continue
        payload.append(int(rep_code, 16))

    return payload


def build_mac_write_ack(
    match: Match,
    available_messages: Dict[str, Dict[str, Dict[str, str]]],
    messages_enabled: bool,
    controller_mac: Optional[str],
) -> Optional[List[Dict[str, Any]]]:
    robot_type = match.robot.type.upper()
    rep_section = get_response_root(available_messages, robot_type)
    ack_entry = get_rep_entry(rep_section, ACK_KEY)
    ack_hex = extract_rep_value(ack_entry)
    if not ack_hex:
        if messages_enabled:
            logging.debug("[%s] No REP code found for %s", match.robot.id, ACK_RESPONSE_KEY)
        return None

    try:
        ack_byte = int(ack_hex, 16)
    except ValueError:
        if messages_enabled:
            logging.debug(
                "[%s] Invalid REP hex '%s' for key %s",
                match.robot.id,
                ack_hex,
                ACK_RESPONSE_KEY,
            )
        return None

    type_code = TYPE_CODE_BY_TYPE.get(robot_type, 0x00)
    remote_write_registry[match.robot.mac] = {
        "type": robot_type,
        "timestamp": time.time(),
        "controller_mac": controller_mac,
    }
    try:
        setattr(match.robot, 'remote_write_timestamp', remote_write_registry[match.robot.mac]['timestamp'])
    except Exception:  # dataclass instances allow dynamic attrs, but guard to avoid issues
        pass

    if controller_mac:
        persist_control_mac(match.robot.mac, controller_mac, messages_enabled)

    if messages_enabled:
        logging.debug(
            "[%s] Prepared MAC write acknowledgement payload: %s",
            match.robot.id,
            [ack_byte, type_code],
        )

    return [
        {
            "bytes": [ack_byte, type_code],
            "labels": ["mac-succesfully-written", f"type:{robot_type}"],
        }
    ]


def listen_on_match(
    match: Match,
    stop_event: threading.Event,
    response_delay: float,
    request_lookup: Dict[str, Dict[str, List[str]]],
    sequences: Dict[str, Dict[str, Sequence[str]]],
    available_messages: Dict[str, Dict[str, Dict[str, str]]],
    messages_enabled: bool,
    solar_simulator: Optional[SolarIrradianceSimulator],
    displacement_simulation: Optional[Dict[str, Any]],
) -> None:
    device = XBeeDevice(match.port, BAUD_RATE)
    sim_cfg = displacement_simulation or {}
    displacement_enabled = bool(sim_cfg.get("enabled"))
    displacement_interval = float(sim_cfg.get("interval_seconds") or 0.0)

    def handler(msg) -> None:
        if stop_event.is_set():
            return

        raw = msg.data
        if not raw:
            if messages_enabled:
                logging.info("[%s] Received empty frame", match.robot.id)
            return

        frame_hex = " ".join(f"{byte:02X}" for byte in raw)

        remote_address = None
        remote_obj = getattr(msg, "remote_device", None)
        if remote_obj is not None:
            try:
                remote_address = "".join(
                    c for c in str(remote_obj.get_64bit_addr()).upper() if c.isalnum()
                )
            except Exception:
                remote_address = None
        if not remote_address:
            # Attempt to fallback on known attributes
            for attr in ("remote64", "remote64_hex", "remote64addr", "remote64Low"):
                candidate = getattr(msg, attr, None)
                if candidate:
                    remote_address = "".join(str(candidate).upper().split(":"))
                    break

        request_code = format(raw[0], "02X")
        request_name = translate_request(request_code, match.robot.type, request_lookup)
        label = request_name or f"unknown:{request_code}"
        _refresh_robot_flags_from_file(available_messages, match.robot)
        low_battery_active = getattr(match.robot, "low_battery", False)
        if request_name == WRITE_ACTION_KEY:
            print(
                f"{PINK}[Remote Find] RX {frame_hex}"
                f" from {remote_address or 'unknown'}{RESET}"
            )
        elif request_name in DISPLACEMENT_ACTIONS:
            is_jog_request = request_name.lower().startswith("jog-")
            color = TURQUOISE if is_jog_request else BLUE
            print(
                f"{color}[Displacement] RX {frame_hex}"
                f" from {remote_address or 'unknown'} → {request_name}{RESET}"
            )

        highlight_remote = request_name == WRITE_ACTION_KEY
        if messages_enabled:
            prefix = PINK if highlight_remote else ""
            suffix = RESET if highlight_remote else ""
            logging.debug(
                "%s[%s] From %s → %s (%s)%s",
                prefix,
                match.robot.id,
                msg.remote_device.get_64bit_addr(),
                request_code,
                label,
                suffix,
            )

        payload_sequences: Optional[List[Dict[str, Any]]] = None
        sequence_labels: Optional[List[str]] = None
        sequence_keys = sequences.get(match.robot.type.upper(), {}).get(request_code.upper())

        if low_battery_active and request_name != WRITE_ACTION_KEY:
            prefers_action_header = request_name in DISPLACEMENT_ACTIONS
            low_battery_frame = build_low_battery_response(
                match.robot,
                available_messages,
                solar_simulator,
                prefer_action_header=prefers_action_header,
            )
            if low_battery_frame and (
                request_name in DISPLACEMENT_ACTIONS
                or request_name in INFORMATION_REQUESTS
                or request_name is None
            ):
                payload_sequences = [low_battery_frame]
                sequence_labels = ["low-battery"]

        if request_name == WRITE_ACTION_KEY:
            payload_sequences = build_mac_write_ack(
                match,
                available_messages,
                messages_enabled,
                controller_mac=remote_address,
            )

        # Handle request-all-data: respond with status frame containing all data (0x3A)
        if payload_sequences is None and request_name == 'request-all-data':
            status_frame = build_status_response(match.robot.type, match.robot.mac, available_messages, solar_simulator)
            payload_sequences = []
            sequence_labels = []
            if status_frame:
                payload_sequences.append(status_frame)
                sequence_labels.append('all-data')

        # Handle robot-cycles and robot-status-and-cycles: respond with cycle frame (0x37)
        if payload_sequences is None and request_name in {'robot-cycles', 'robot-status-and-cycles'}:
            cycle_frame = build_cycle_report(match.robot.type, match.robot.mac, available_messages, messages_enabled)
            status_frame = None
            if request_name == 'robot-status-and-cycles':
                status_frame = build_status_response(match.robot.type, match.robot.mac, available_messages, solar_simulator)
            payload_sequences = []
            sequence_labels = []
            if cycle_frame:
                payload_sequences.append(cycle_frame)
                sequence_labels.append('cycle-report')
            if status_frame:
                payload_sequences.append(status_frame)
                sequence_labels.append('status')
            if not payload_sequences:
                payload_sequences = []

        if highlight_remote and response_delay > 0:
            if stop_event.wait(response_delay):
                return

        step_delay = response_delay
        if payload_sequences is None:
            if request_name in DISPLACEMENT_ACTIONS:
                payload_sequences = build_displacement_frames(
                    match.robot.type,
                    sequence_keys,
                    available_messages,
                )
                if not payload_sequences:
                    return
                sequence_labels = [
                    entry.get("action_label")
                    for entry in payload_sequences
                    if entry.get("action_label") and entry.get("action_label").lower() != "none"
                ]
                if displacement_enabled and sequence_labels:
                    step_delay = max(displacement_interval, 0.0)
                    label_is_jog = any(
                        isinstance(label, str) and label.lower().startswith("jog")
                        for label in sequence_labels
                    )
                    color = TURQUOISE if label_is_jog else BLUE
                    print(
                        f"{color}[Displacement] TX sequence {sequence_labels}"
                        f" to {remote_address or 'unknown'} (interval {step_delay:.2f}s){RESET}"
                    )
                elif not displacement_enabled:
                    payload_sequences = [payload_sequences[0]]
                    sequence_labels = sequence_labels[:1] if sequence_labels else None
                    step_delay = 0.0
                    if sequence_labels:
                        label_is_jog = any(
                            isinstance(label, str) and label.lower().startswith("jog")
                            for label in sequence_labels
                        )
                        color = TURQUOISE if label_is_jog else BLUE
                        print(
                            f"{color}[Displacement] TX single response {sequence_labels}"
                            f" to {remote_address or 'unknown'} (simulation disabled){RESET}"
                        )
            else:
                base_payload = prepare_response_payloads(
                    match.robot.type,
                    request_code,
                    sequences,
                    available_messages,
                    messages_enabled,
                    sequence_keys,
                )
                payload_sequences = []
                sequence_labels = []
                if not base_payload:
                    if messages_enabled and request_name != 'request-all-data':
                        logging.info(
                            "%s[%s] RX %s | no configured response for %s%s",
                            PINK if highlight_remote else "",
                            match.robot.id,
                            frame_hex,
                            request_code,
                            RESET if highlight_remote else "",
                        )
                    if request_name != 'request-all-data':
                        return
                    for value in base_payload:
                        response_code = format(value, "02X")
                        response_name = translate_response(response_code, match.robot.type, available_messages)
                        if response_name:
                            sequence_labels.append(response_name)
                        else:
                            sequence_labels.append(f"0x{response_code}")
                        payload_sequences.append({"bytes": [value], "labels": [response_name or f"0x{response_code}"]})

        if (
            low_battery_active
            and request_name != WRITE_ACTION_KEY
            and payload_sequences
            and (
                request_name in DISPLACEMENT_ACTIONS
                or request_name in INFORMATION_REQUESTS
                or request_name is None
            )
        ):
            def _is_low_battery(entry: Any) -> bool:
                if not isinstance(entry, dict):
                    return False
                status_info = entry.get("status_info") if isinstance(entry.get("status_info"), dict) else {}
                action_info = entry.get("action_info") if isinstance(entry.get("action_info"), dict) else {}
                status_key = str(status_info.get("key") or "").lower()
                action_key = str(action_info.get("key") or "").lower()
                return status_key == "error" and action_key == "low-battery"

            if not any(_is_low_battery(entry) for entry in payload_sequences):
                fallback_frame = build_low_battery_response(
                    match.robot,
                    available_messages,
                    solar_simulator,
                    prefer_action_header=request_name in DISPLACEMENT_ACTIONS,
                )
                if fallback_frame:
                    payload_sequences = [fallback_frame]
                    sequence_labels = ["low-battery"]

        mac_upper = match.robot.mac.upper()
        if request_name in DISPLACEMENT_ACTIONS:
            previous_displacement_action[mac_upper] = request_name.lower()

        is_displacement_response = request_name in DISPLACEMENT_ACTIONS

        if not payload_sequences:
            if messages_enabled:
                logging.debug("[%s] Remote write ack produced empty payload", match.robot.id)
            return

        if msg.remote_device is not None:
            target_remote = msg.remote_device
        elif remote_address:
            try:
                target_remote = RemoteXBeeDevice(device, XBee64BitAddress.from_hex_string(remote_address))
            except Exception:
                target_remote = None
        else:
            target_remote = None

        tx_records: List[str] = []
        for sequence_entry in payload_sequences:
            sequence = sequence_entry.get("bytes") if isinstance(sequence_entry, dict) else sequence_entry
            labels_list = []
            if isinstance(sequence_entry, dict):
                labels = sequence_entry.get("labels")
                if isinstance(labels, list):
                    labels_list = [str(label) for label in labels]
            status_info = None
            action_info = None
            if isinstance(sequence_entry, dict):
                status_info = sequence_entry.get("status_info")
                action_info = sequence_entry.get("action_info")
            try:
                if not target_remote:
                    raise RuntimeError("No remote device available for response")
                if SEND_TIMEOUT and SEND_TIMEOUT > 0:
                    device.set_sync_ops_timeout(SEND_TIMEOUT)
                telemetry_summary = _format_telemetry_summary(
                    sequence_entry.get("telemetry_info") if isinstance(sequence_entry, dict) else None
                )
                if telemetry_summary and messages_enabled:
                    logging.info("[%s] Telemetry → %s", match.robot.id, telemetry_summary)
                device.send_data(target_remote, bytes(sequence))
                codes = [format(value, "02X") for value in sequence]
                if not labels_list:
                    for value in sequence:
                        response_code = format(value, "02X")
                        response_name = translate_response(response_code, match.robot.type, available_messages)
                        labels_list.append(response_name or f"0x{response_code}")
                tx_records.append(f"{' '.join(codes)} ({', '.join(labels_list)})")
                action_key = (action_info.get("key") if isinstance(action_info, dict) else None) or ""
                is_jog_action = isinstance(action_key, str) and action_key.lower().startswith("jog")
                color = TURQUOISE if is_jog_action else PURPLE
                print(
                    f"{color}[TX] frame {' '.join(codes)} → {remote_address or 'unknown'} ({', '.join(labels_list)}){RESET}"
                )
                if status_info and isinstance(status_info, dict):
                    status_key = (status_info.get("key") or "").lower()
                    if status_key != "jog":
                        update_displacement_status(
                            match.robot.mac,
                            status_info.get("key"),
                            status_info.get("value"),
                            sequence_entry.get("action_info", {}).get("key") if isinstance(sequence_entry, dict) else None,
                        )
                    if (
                        is_displacement_response
                        and action_info
                        and isinstance(action_info, dict)
                    ):
                        action_key = str(action_info.get("key") or "").lower()
                        if action_key == "cycle-end":
                            last_action = previous_displacement_action.get(mac_upper)
                            if last_action in CYCLE_INCREMENT_EXCLUDED_ACTIONS:
                                if messages_enabled:
                                    logging.debug(
                                        "[%s] Skipping cycle increment due to previous action %s",
                                        match.robot.id,
                                        last_action,
                                    )
                            else:
                                increment_simulated_cycles(match.robot, messages_enabled)
                            stand_by_value = get_standby_status_value(match.robot.type, available_messages)
                            if stand_by_value is not None:
                                update_displacement_status(
                                    match.robot.mac,
                                    "stand-by",
                                    stand_by_value,
                                    "none",
                                )
            except TimeoutException:
                if messages_enabled:
                    logging.warning("[%s] Timeout sending %s", match.robot.id, sequence)
                codes = [format(value, "02X") for value in sequence]
                tx_records.append(f"{' '.join(codes)} (timeout)")
                if step_delay > 0 and not highlight_remote and stop_event.wait(step_delay):
                    break
            except Exception as exc:
                if messages_enabled:
                    logging.warning(
                        "[%s] Error sending to %s (%s); attempting broadcast fallback",
                        match.robot.id,
                        remote_address,
                        exc,
                    )
                try:
                    if SEND_TIMEOUT and SEND_TIMEOUT > 0:
                        device.set_sync_ops_timeout(SEND_TIMEOUT)
                    telemetry_summary = _format_telemetry_summary(
                        sequence_entry.get("telemetry_info") if isinstance(sequence_entry, dict) else None
                    )
                    if telemetry_summary and messages_enabled:
                        logging.info("[%s] Telemetry → %s", match.robot.id, telemetry_summary)
                    device.send_data_broadcast(bytes(sequence))
                    codes = [format(value, "02X") for value in sequence]
                    if not labels_list:
                        for value in sequence:
                            response_code = format(value, "02X")
                            response_name = translate_response(response_code, match.robot.type, available_messages)
                            labels_list.append(response_name or f"0x{response_code}")
                    tx_records.append(f"broadcast {' '.join(codes)} ({', '.join(labels_list)})")
                    action_key = (action_info.get("key") if isinstance(action_info, dict) else None) or ""
                    is_jog_action = isinstance(action_key, str) and action_key.lower().startswith("jog")
                    color = TURQUOISE if is_jog_action else PURPLE
                    print(
                        f"{color}[TX] broadcast {' '.join(codes)} → {remote_address or 'unknown'} ({', '.join(labels_list)}){RESET}"
                    )
                    if status_info and isinstance(status_info, dict):
                        status_key = (status_info.get("key") or "").lower()
                        if status_key != "jog":
                            update_displacement_status(
                                match.robot.mac,
                                status_info.get("key"),
                                status_info.get("value"),
                                sequence_entry.get("action_info", {}).get("key") if isinstance(sequence_entry, dict) else None,
                            )
                    if (
                        is_displacement_response
                        and action_info
                        and isinstance(action_info, dict)
                    ):
                        action_key = str(action_info.get("key") or "").lower()
                        if action_key == "cycle-end":
                            last_action = previous_displacement_action.get(mac_upper)
                            if last_action in CYCLE_INCREMENT_EXCLUDED_ACTIONS:
                                if messages_enabled:
                                    logging.debug(
                                        "[%s] Skipping cycle increment due to previous action %s",
                                        match.robot.id,
                                        last_action,
                                    )
                            else:
                                increment_simulated_cycles(match.robot, messages_enabled)
                            stand_by_value = get_standby_status_value(match.robot.type, available_messages)
                            if stand_by_value is not None:
                                update_displacement_status(
                                    match.robot.mac,
                                    "stand-by",
                                    stand_by_value,
                                    "none",
                                )
                except Exception as broadcast_exc:
                    if messages_enabled:
                        logging.error("[%s] Broadcast fallback failed: %s", match.robot.id, broadcast_exc)
                    continue
            else:
                if step_delay > 0 and not highlight_remote:
                    if stop_event.wait(step_delay):
                        break

        if messages_enabled and tx_records:
            logging.info(
                "%s[%s] RX %s | TX %s%s",
                PINK if highlight_remote else "",
                match.robot.id,
                frame_hex,
                ", ".join(tx_records),
                RESET if highlight_remote else "",
            )

        if request_name == WRITE_ACTION_KEY:
            ack_str = ", ".join(tx_records) if tx_records else "(no payload)"
            print(
                f"{PINK}[Remote Find] SUCCESS TX {ack_str} to {remote_address or 'unknown'}{RESET}"
            )

    try:
        device.open()
        device.set_sync_ops_timeout(SYNC_TIMEOUT)
        device.add_data_received_callback(handler)
        if messages_enabled:
            logging.info("Listening on %s (%s) for robot %s", match.port, match.node_id, match.robot.id)

        while not stop_event.is_set():
            time.sleep(0.2)
    except Exception as exc:
        if messages_enabled:
            logging.error("Listener for %s failed: %s", match.robot.id, exc)
    finally:
        with suppress(Exception):
            device.del_data_received_callback(handler)
        with suppress(Exception):
            if device.is_open():
                device.close()
        if messages_enabled:
            logging.info("Stopped listener for %s", match.robot.id)


def configure_logging(verbose: bool) -> None:
    level = logging.DEBUG if verbose else logging.INFO
    logging.basicConfig(
        level=level,
        format="%(asctime)s [%(levelname)s] %(message)s",
        datefmt="%H:%M:%S",
    )
    root_logger = logging.getLogger()
    if MESSAGE_LOG_FILTER not in root_logger.filters:
        root_logger.addFilter(MESSAGE_LOG_FILTER)


def configure_message_logging_state(enabled: bool, verbose: bool) -> None:
    MESSAGE_LOG_FILTER.set_enabled(enabled)

    target_names = [
        "digi",
        "digi.xbee",
        "digi.xbee.devices",
        "digi.xbee.reader",
        "digi.xbee.serial",
        "digi.xbee.serial.xbee_serialport",
        "XBeeSerialPort",
    ]

    active_level = logging.DEBUG if verbose else logging.INFO
    muted_level = logging.WARNING

    for name in target_names:
        logger = logging.getLogger(name)
        if not enabled:
            logger.setLevel(muted_level)
            logger.propagate = False
            logger.disabled = False
        else:
            logger.setLevel(active_level)
            logger.propagate = True
            logger.disabled = False


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Simulate INTI robot devices over XBee.")
    parser.add_argument(
        "--response-delay",
        type=float,
        default=DEFAULT_RESPONSE_DELAY,
        help=f"Delay (seconds) between REP frames in a sequence (default: {DEFAULT_RESPONSE_DELAY}).",
    )
    parser.add_argument(
        "--scan-only",
        action="store_true",
        help="Discover and report matching robots without opening listeners.",
    )
    parser.add_argument(
        "--verbose",
        action="store_true",
        help="Enable debug logging.",
    )
    parser.add_argument(
        "--solar-interval",
        type=float,
        default=None,
        help="Seconds between solar irradiance log messages (overrides config).",
    )
    parser.add_argument(
        "--send-timeout",
        type=float,
        default=None,
        help="Timeout in seconds to wait for transmit status before considering it expired (default: config/1.0).",
    )
    return parser


def main() -> None:
    parser = build_arg_parser()
    args = parser.parse_args()

    project_root = Path(__file__).resolve().parent.parent
    configure_logging(args.verbose)

    try:
        config = load_config(project_root)
        available_messages = load_available_messages(project_root, config)
        response_sequences = load_response_sequences(project_root, config)
        robots = load_robots(project_root)
        initialise_robot_statuses(robots, available_messages)
    except FileNotFoundError as exc:
        logging.error("%s", exc)
        return

    solar_config = load_solar_config(config)
    solar_simulator = SolarIrradianceSimulator(solar_config)
    solar_interval = resolve_solar_interval(config, args.solar_interval)
    solar_logging_enabled = resolve_solar_logging_enabled(config)
    messages_enabled = resolve_message_logging_enabled(config)
    global SEND_TIMEOUT
    SEND_TIMEOUT = resolve_send_timeout(config, args.send_timeout)
    displacement_simulation = resolve_displacement_simulation(config)

    configure_message_logging_state(messages_enabled, args.verbose)

    if not robots:
        logging.error("No robots defined in robots.json")
        return

    request_lookup = build_request_lookup(available_messages)
    sequences = build_response_sequences(available_messages, response_sequences)

    stop_event = threading.Event()
    solar_thread = start_solar_logger(solar_simulator, stop_event, solar_interval, solar_logging_enabled)

    matches = discover_matches(robots, messages_enabled)

    if not matches and messages_enabled:
        logging.warning("No connected XBee devices matched robots.json entries")

    if args.scan_only:
        stop_event.set()
        solar_thread.join()
        return

    if not matches:
        stop_event.set()
        solar_thread.join()
        return

    def handle_signal(signum: int, _: object) -> None:
        logging.info("Received signal %s; stopping listeners", signum)
        stop_event.set()

    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, handle_signal)

    threads: List[threading.Thread] = []
    for match in matches:
        thread = threading.Thread(
            target=listen_on_match,
            args=(
                match,
                stop_event,
                args.response_delay,
                request_lookup,
                sequences,
                available_messages,
                messages_enabled,
                solar_simulator,
                displacement_simulation,
            ),
            daemon=True,
        )
        thread.start()
        threads.append(thread)

    try:
        while not stop_event.is_set():
            time.sleep(0.5)
    except KeyboardInterrupt:
        logging.info("Keyboard interrupt; shutting down")
        stop_event.set()
    finally:
        stop_event.set()
        for thread in threads:
            thread.join()
        solar_thread.join()


if __name__ == "__main__":
    main()
