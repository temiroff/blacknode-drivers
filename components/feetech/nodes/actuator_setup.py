"""Bounded STS3215 commissioning; every operation owns one serial connection."""
from __future__ import annotations

from contextlib import contextmanager
import time

from blacknode.node import Bool, Dict, Text, node
from . import bus
from .raw_monitor import _hardware_match_score

# STS/SMS protocol-0 control table. Writes are restricted to STS3215 (777).
# https://github.com/huggingface/lerobot/blob/main/src/lerobot/motors/feetech/tables.py
MODEL = 3
SERVO_ID = 5
EEPROM_LOCK = 55
SUPPORTED_MODEL = 777
BAUDRATES = (4800, 9600, 14400, 19200, 38400, 57600, 115200,
             128000, 250000, 500000, 1000000)


@contextmanager
def connection(config, sdk=None):
    sdk = sdk or bus.load_sdk()
    baudrate = int(config.get("baudrate", 1000000))
    if baudrate not in BAUDRATES:
        raise ValueError("Choose a supported servo baud rate")
    port = bus.open_port(sdk, str(config.get("port") or ""), baudrate)
    try:
        yield sdk, sdk.PacketHandler(0), port
    finally:
        port.closePort()


def read(sdk, packet, port, servo_id, address, width=1):
    value, result, flags = getattr(packet, f"read{width}ByteTxRx")(
        port, servo_id, address)
    if result != sdk.COMM_SUCCESS:
        raise RuntimeError(f"ID {servo_id}: register {address} did not respond")
    return int(value), int(flags)


def inspect_bus(sdk, packet, port, *, strict=True):
    rows = []
    for servo_id in range(254):
        model, result, flags = packet.ping(port, servo_id)
        if result == getattr(sdk, "COMM_RX_TIMEOUT", -6):
            continue
        if result != sdk.COMM_SUCCESS:
            message = (f"ID {servo_id}: reply could not be decoded (communication code {result}). "
                       "Possible shared ID, wiring, power or baud-rate issue. "
                       "Power off, connect one actuator at a time, then scan to identify and set its ID.")
            if strict:
                raise RuntimeError(message)
            # Retain unresolved addresses while continuing discovery. A damaged
            # packet is not evidence of one uniquely identified physical servo.
            rows.append({"servo_id": servo_id, "discovery_status": "unreadable",
                         "model": "Unresolved address", "model_number": None,
                         "assignment_supported": False, "hardware_error_flags": 0,
                         "errors": [message], "communication_code": int(result)})
            continue
        row = {"servo_id": servo_id, "model_number": int(model),
               "discovery_status": "responding",
               "assignment_supported": int(model) == SUPPORTED_MODEL,
               "model": "STS3215" if int(model) == SUPPORTED_MODEL else "Unknown model",
               "hardware_error_flags": int(flags), "errors": []}
        for name, address, width in (
            ("reported_id", SERVO_ID, 1), ("torque_enabled", 40, 1),
            ("raw_position", 56, 2), ("voltage_raw", 62, 1),
            ("temperature_c", 63, 1), ("servo_status", 65, 1),
        ):
            try:
                value, warning = read(sdk, packet, port, servo_id, address, width)
                row[name] = bool(value) if name == "torque_enabled" else value
                row["hardware_error_flags"] |= warning
            except RuntimeError as exc:
                row["errors"].append(str(exc))
        row["hardware_error_flags"] |= row.get("servo_status", 0)
        row["settings"] = {}
        row["settings_errors"] = []
        if row["assignment_supported"]:
            for name, address, width in (("baud_rate_code", 6, 1), ("min_position_ticks", 9, 2),
                                         ("max_position_ticks", 11, 2), ("max_torque", 16, 2),
                                         ("operating_mode", 33, 1), ("eeprom_locked", 55, 1)):
                try:
                    value, warning = read(sdk, packet, port, servo_id, address, width)
                    row["settings"][name] = value
                    row["hardware_error_flags"] |= warning
                except Exception as exc:
                    row["settings_errors"].append(str(exc))
        row["hardware_errors"] = bus.decode_hardware_errors(row["hardware_error_flags"])
        row["voltage_v"] = row.pop("voltage_raw", 0) / 10 or None
        rows.append(row)
    return rows


def scan(config, sdk=None):
    with connection(config, sdk) as (sdk, packet, port):
        return {"actuators": inspect_bus(sdk, packet, port, strict=False)}


def read_position(config, servo_id, sdk=None):
    """Read one known actuator and close the bus; never scan or write registers."""
    if isinstance(servo_id, bool) or not isinstance(servo_id, int) or not 0 <= servo_id <= 253:
        raise ValueError("Select a valid servo ID")
    with connection(config, sdk) as (sdk, packet, port):
        row = {"servo_id": servo_id, "errors": [], "hardware_error_flags": 0}
        for name, address, width in (("model_number", MODEL, 2), ("reported_id", SERVO_ID, 1),
                                     ("torque_enabled", 40, 1), ("raw_position", 56, 2),
                                     ("servo_status", 65, 1)):
            value, flags = read(sdk, packet, port, servo_id, address, width)
            row[name] = bool(value) if name == "torque_enabled" else value
            row["hardware_error_flags"] |= flags
        row["hardware_error_flags"] |= row["servo_status"]
        row["assignment_supported"] = row["model_number"] == SUPPORTED_MODEL
        row["hardware_errors"] = bus.decode_hardware_errors(row["hardware_error_flags"])
        row["sampled_at"] = time.time()
        row["position_range"] = {"min": 0, "max": 4095}
        return row


def release(config, expected, sdk=None):
    with connection(config, sdk) as (sdk, packet, port):
        rows = inspect_bus(sdk, packet, port)
        if len(rows) != 1 or rows[0]["model_number"] != SUPPORTED_MODEL:
            raise ValueError("Torque release requires one isolated STS3215")
        row = rows[0]
        if (row["servo_id"], row["model_number"]) != (expected["servo_id"], expected["model_number"]):
            raise ValueError("Actuator changed since discovery; scan again")
        # Accept hardware warnings only for this transition to a safer state,
        # and independently verify the physical torque register afterwards.
        bus._set_torque(sdk, packet, port, row["servo_id"], False)
        torque, _flags = read(sdk, packet, port, row["servo_id"], 40)
        if torque != 0:
            raise RuntimeError("Torque release was not verified; keep the arm supported")
        row["torque_enabled"] = False
        return {"released": True, "actuators": rows,
                "report": "Torque verified off. Scan again before assigning the ID."}


def assign(config, expected, new_id, sdk=None):
    """Reinspect immediately before writing; never torque-enable or command a pose."""
    if isinstance(new_id, bool) or not isinstance(new_id, int) or not 1 <= new_id <= 253:
        raise ValueError("The destination ID must be an integer from 1 to 253")
    with connection(config, sdk) as (sdk, packet, port):
        rows = inspect_bus(sdk, packet, port)
        if len(rows) != 1:
            raise ValueError("ID assignment requires exactly one isolated actuator; scan again")
        row = rows[0]
        old_id = row["servo_id"]
        if (old_id, row["model_number"]) != (expected["servo_id"], expected["model_number"]):
            raise ValueError("The actuator changed since discovery; scan again")
        if row["model_number"] != SUPPORTED_MODEL:
            raise ValueError("ID programming currently supports STS3215 only")
        if row["errors"] or row.get("reported_id") != old_id or row["hardware_error_flags"]:
            raise ValueError("ID programming requires complete, warning-free actuator feedback")
        if row.get("torque_enabled") is not False:
            raise ValueError("Torque is on or unknown. Support the robot and release torque before setup")
        if old_id == new_id:
            return {"assigned": True, "old_id": old_id, "new_id": new_id,
                    "actuators": rows, "report": f"ID {new_id} is already assigned; no write needed"}

        def write(servo_id, address, value):
            result, flags = packet.write1ByteTxRx(port, servo_id, address, value)
            if result != sdk.COMM_SUCCESS or flags:
                raise RuntimeError(f"ID {servo_id}: register {address} write was not acknowledged")

        # The ID write may succeed even if its acknowledgement uses the new ID.
        # Always locate and relock the isolated actuator, then verify all state.
        candidates = [old_id]
        try:
            write(old_id, EEPROM_LOCK, 0)
            candidates = [new_id, old_id]
            try:
                write(old_id, SERVO_ID, new_id)
            except RuntimeError:
                pass
        finally:
            locked = False
            for candidate in candidates:
                try:
                    actual, warning = read(sdk, packet, port, candidate, SERVO_ID)
                    if actual != candidate or warning:
                        continue
                    write(candidate, EEPROM_LOCK, 1)
                    lock, warning = read(sdk, packet, port, candidate, EEPROM_LOCK)
                    if lock == 1 and not warning:
                        locked = True
                        break
                except Exception:
                    continue
            if not locked:
                raise RuntimeError("ID state is uncertain and EEPROM relock could not be verified. "
                                   "Keep this actuator isolated and rescan before reconnecting the chain")
        actual, flags = read(sdk, packet, port, new_id, SERVO_ID)
        model, model_flags = read(sdk, packet, port, new_id, MODEL, 2)
        torque, torque_flags = read(sdk, packet, port, new_id, 40)
        _, old_result, _ = packet.ping(port, old_id)
        if (actual != new_id or model != SUPPORTED_MODEL or torque != 0
                or flags or model_flags or torque_flags
                or old_result != getattr(sdk, "COMM_RX_TIMEOUT", -6)):
            raise RuntimeError("ID change could not be fully verified; keep the actuator isolated and rescan")
        row.update(servo_id=new_id, reported_id=new_id)
        row["settings"]["eeprom_locked"] = 1
        return {"assigned": True, "old_id": old_id, "new_id": new_id, "actuators": [row],
                "report": f"ID {old_id} → {new_id} verified. EEPROM locked; torque remains off. "
                          "Power off, reconnect the chain, rescan, then recalibrate before motion."}


def build_test_context(config, state, row):
    """Translate captured raw limits into the existing single-joint motion contract."""
    if state.get("model_number") != SUPPORTED_MODEL:
        raise ValueError("Calibrated tests currently support STS3215")
    if (row.get("settings") or {}).get("operating_mode") != 0:
        raise ValueError("Position mode must be verified before slider testing")
    points = state["points"]
    low, home, high = (int(points[name]) for name in ("min", "home", "max"))
    margin = int(state["safety_margin_ticks"])
    if not 0 <= low < home < high < bus.TICKS_PER_REV or not low + margin < home < high - margin:
        raise ValueError("Captured points must form a safe range within one STS3215 revolution")
    scale = 360.0 / bus.TICKS_PER_REV
    joint = {"id": "actuator", "servo_id": state["servo_id"], "home_ticks": home,
             "safe_min_deg": (low + margin - home) * scale,
             "safe_max_deg": (high - margin - home) * scale, "velocity_limit": 180.0}
    profile_id = f"actuator_setup_{state['servo_id']}"
    profile = {"id": profile_id, "joints": [joint], "driver": {"baudrate": config["baudrate"]},
               "capability_bindings": {"joint_group": {"provider": {
                   "package": "blacknode-drivers", "component": "feetech"}}}}
    return {"profile": profile, "hardware_id": config["hardware_id"], "position_target_mode": True,
            "hardware": {"recommended": {"path": config["port"]}},
            "calibration": {"profile_id": profile_id, "hardware_id": config["hardware_id"],
                            "joints": {"actuator": dict(joint)}}, "degrees_per_tick": scale}


@node(name="FeetechActuatorSetupProvider", component="feetech", category="Drivers", hidden=True,
      inputs={}, outputs={"available": Bool, "provider": Dict, "report": Text})
def feetech_actuator_setup_provider(ctx):
    return {"available": True, "provider": {"package": "blacknode-drivers", "component": "feetech"},
            "report": "Feetech actuator discovery and isolated STS3215 ID assignment"}


feetech_actuator_setup_provider._bn_robot_actuator_setup_provider = {
    "package": "blacknode-drivers", "component": "feetech", "scan": scan, "assign": assign, "release": release,
    "match_hardware": _hardware_match_score,
    "read_position": read_position,
    "build_test_context": build_test_context,
}
