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
RESET = "\033[0m"
BAUD_RATE = 9600
SYNC_TIMEOUT = 1.0
DEFAULT_RESPONSE_DELAY = 3.0
DEFAULT_SEND_TIMEOUT = 1.0
WRITE_ACTION_KEY = "write-ctrl-mac-address"
ACK_RESPONSE_KEY = "mac-succesfully-written"
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
}
BROADCAST_ADDRESS = "000000000000FFFF"
remote_write_registry: Dict[str, Dict[str, Any]] = {}
ROBOTS_FILE_PATH: Optional[Path] = None


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

    @classmethod
    def from_dict(cls, payload: Dict[str, str]) -> "Robot":
        return cls(
            id=payload["id"],
            mac=payload["mac"].upper(),
            type=payload["type"].upper(),
            description=payload.get("description", ""),
            scada_id=payload.get("scadaId", ""),
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


def load_robots(project_root: Path) -> List[Robot]:
    workspace_root = project_root.parent
    candidates = [
        project_root / "data" / "robots.json",
        workspace_root / "it-gateway" / "robot-data" / "robots.json",
    ]
    path = resolve_existing_path(candidates)
    global ROBOTS_FILE_PATH
    ROBOTS_FILE_PATH = path
    with path.open("r", encoding="utf-8") as fh:
        contents = json.load(fh)
    robots_payload = contents.get("robots", [])
    robots = []
    for payload in robots_payload:
        try:
            robots.append(Robot.from_dict(payload))
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


def build_request_lookup(available_messages: Dict[str, Dict[str, Dict[str, str]]]) -> Dict[str, Dict[str, List[str]]]:
    req_section = available_messages.get("REQ", {})
    lookup: Dict[str, Dict[str, List[str]]] = {}
    for robot_type, mapping in req_section.items():
        norm_type = robot_type.upper()
        per_type: Dict[str, List[str]] = {}
        for name, code in mapping.items():
            hex_code = code.upper()
            per_type.setdefault(hex_code, []).append(name)
        lookup[norm_type] = per_type
    return lookup


def build_response_sequences(
    available_messages: Dict[str, Dict[str, Dict[str, str]]]
) -> Dict[str, Dict[str, Sequence[str]]]:
    req_section = available_messages.get("REQ", {})
    rep_section = available_messages.get("REP", {})
    sequences_section = (
        available_messages.get("RESPONSE_SEQUENCES")
        or available_messages.get("REP_SEQUENCES")
        or {}
    )

    sequences: Dict[str, Dict[str, Sequence[str]]] = {}

    for robot_type, per_robot in sequences_section.items():
        robot_type_upper = robot_type.upper()
        robot_req = req_section.get(robot_type, {})
        robot_rep = rep_section.get(robot_type, {})
        if not robot_req or not robot_rep:
            logging.warning(
                "Skipping response sequences for %s; REQ/REP definitions missing in availableMessages",
                robot_type,
            )
            continue

        result: Dict[str, Sequence[str]] = {}
        for req_key, rep_keys in per_robot.items():
            if req_key not in robot_req:
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
                if rep_key in robot_rep:
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

            code = robot_req[req_key].upper()
            result[code] = tuple(valid_rep_keys)

        sequences[robot_type_upper] = result

    return sequences


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
    rep_section = available_messages.get("REP", {}).get(robot_type.upper(), {})
    for name, hex_value in rep_section.items():
        if hex_value.upper() == code.upper():
            return name
    return None


def prepare_response_payloads(
    robot_type: str,
    request_code: str,
    sequences: Dict[str, Dict[str, Sequence[str]]],
    available_messages: Dict[str, Dict[str, Dict[str, str]]],
    messages_enabled: bool,
) -> List[int]:
    robot_type_upper = robot_type.upper()
    rep_section = available_messages.get("REP", {}).get(robot_type_upper, {})
    sequence = sequences.get(robot_type_upper, {}).get(request_code.upper())

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
        rep_code = rep_section.get(rep_key)
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
    rep_section = available_messages.get("REP", {}).get(robot_type, {})
    ack_hex = rep_section.get(ACK_RESPONSE_KEY)
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
        if request_name == WRITE_ACTION_KEY:
            print(
                f"{PINK}[Remote Find] RX {frame_hex}"
                f" from {remote_address or 'unknown'}{RESET}"
            )
        elif request_name in DISPLACEMENT_ACTIONS:
            print(
                f"{BLUE}[Displacement] RX {frame_hex}"
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
        if request_name == WRITE_ACTION_KEY:
            payload_sequences = build_mac_write_ack(
                match,
                available_messages,
                messages_enabled,
                controller_mac=remote_address,
            )

        if highlight_remote and response_delay > 0:
            if stop_event.wait(response_delay):
                return

        sequence_labels: Optional[List[str]] = None
        step_delay = response_delay
        if payload_sequences is None:
            base_payload = prepare_response_payloads(
                match.robot.type,
                request_code,
                sequences,
                available_messages,
                messages_enabled,
            )
            if not base_payload:
                if messages_enabled:
                    logging.info(
                        "%s[%s] RX %s | no configured response for %s%s",
                        PINK if highlight_remote else "",
                        match.robot.id,
                        frame_hex,
                        request_code,
                        RESET if highlight_remote else "",
                    )
                return
            payload_sequences = []
            sequence_labels = []
            for value in base_payload:
                response_code = format(value, "02X")
                response_name = translate_response(response_code, match.robot.type, available_messages)
                if response_name:
                    sequence_labels.append(response_name)
                else:
                    sequence_labels.append(f"0x{response_code}")
                payload_sequences.append(
                    {
                        "bytes": [value],
                        "labels": [response_name or f"0x{response_code}"],
                    }
                )
            if request_name in DISPLACEMENT_ACTIONS and payload_sequences:
                if displacement_enabled:
                    step_delay = max(displacement_interval, 0.0)
                    print(
                        f"{BLUE}[Displacement] TX sequence {sequence_labels}"
                        f" to {remote_address or 'unknown'} (interval {step_delay:.2f}s){RESET}"
                    )
                else:
                    payload_sequences = [payload_sequences[0]]
                    sequence_labels = [sequence_labels[0]]
                    step_delay = 0.0
                    print(
                        f"{BLUE}[Displacement] TX single response {sequence_labels}"
                        f" to {remote_address or 'unknown'} (simulation disabled){RESET}"
                    )
        elif len(payload_sequences) == 0:
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
            try:
                if not target_remote:
                    raise RuntimeError("No remote device available for response")
                if SEND_TIMEOUT and SEND_TIMEOUT > 0:
                    device.set_sync_ops_timeout(SEND_TIMEOUT)
                device.send_data(target_remote, bytes(sequence))
                codes = [format(value, "02X") for value in sequence]
                if not labels_list:
                    for value in sequence:
                        response_code = format(value, "02X")
                        response_name = translate_response(response_code, match.robot.type, available_messages)
                        labels_list.append(response_name or f"0x{response_code}")
                tx_records.append(f"{' '.join(codes)} ({', '.join(labels_list)})")
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
                    device.send_data_broadcast(bytes(sequence))
                    codes = [format(value, "02X") for value in sequence]
                    if not labels_list:
                        for value in sequence:
                            response_code = format(value, "02X")
                            response_name = translate_response(response_code, match.robot.type, available_messages)
                            labels_list.append(response_name or f"0x{response_code}")
                    tx_records.append(f"broadcast {' '.join(codes)} ({', '.join(labels_list)})")
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
        robots = load_robots(project_root)
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
    sequences = build_response_sequences(available_messages)

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
