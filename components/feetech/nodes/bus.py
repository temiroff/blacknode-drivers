"""Transport-neutral Feetech STS/SMS serial-bus primitives.

The vendor SDK is imported only when hardware is accessed. Configuration,
parsing, conversion, and safety sequencing remain testable without hardware.
"""
from __future__ import annotations

import math
from dataclasses import asdict, dataclass
from typing import Any, Mapping

TICKS_PER_REV = 4096
DEFAULT_HOME_TICKS = 2048

ADDR_TORQUE_ENABLE = (40, 1)
ADDR_GOAL_POSITION = (42, 2)
ADDR_PRESENT_POSITION = (56, 2)
ADDR_PRESENT_VOLTAGE = (62, 1)

_HARDWARE_ERROR_BITS = {
    0x01: "voltage",
    0x02: "angle-sensor",
    0x04: "overheat",
    0x08: "overcurrent",
    0x20: "overload",
}


@dataclass(frozen=True)
class JointSpec:
    name: str
    servo_id: int
    min_deg: float
    max_deg: float
    home_ticks: int = DEFAULT_HOME_TICKS
    invert: bool = False

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


def parse_int_map(spec: str) -> dict[str, int]:
    """Parse ``name:value,name:value`` with actionable validation."""
    result: dict[str, int] = {}
    for chunk in (part.strip() for part in str(spec or "").split(",")):
        if not chunk:
            continue
        parts = [part.strip() for part in chunk.split(":")]
        if len(parts) != 2 or not parts[0]:
            raise ValueError(f"invalid integer-map entry '{chunk}'; expected name:value")
        result[parts[0]] = int(parts[1])
    return result


def parse_joint_map(
    spec: str,
    home_overrides: Mapping[str, int] | None = None,
    inverted: set[str] | None = None,
) -> dict[str, JointSpec]:
    """Parse ``name:servo_id:min_deg:max_deg`` entries."""
    homes = dict(home_overrides or {})
    inverted_names = set(inverted or set())
    joints: dict[str, JointSpec] = {}
    servo_ids: set[int] = set()
    for chunk in (part.strip() for part in str(spec or "").split(",")):
        if not chunk:
            continue
        parts = [part.strip() for part in chunk.split(":")]
        if len(parts) != 4 or not parts[0]:
            raise ValueError(
                f"invalid joint entry '{chunk}'; expected name:servo_id:min_deg:max_deg"
            )
        name, raw_servo_id, raw_min, raw_max = parts
        if name in joints:
            raise ValueError(f"joint name '{name}' is duplicated")
        servo_id = int(raw_servo_id)
        if not 1 <= servo_id <= 253:
            raise ValueError(f"joint '{name}' servo id must be between 1 and 253")
        if servo_id in servo_ids:
            raise ValueError(f"servo id {servo_id} is duplicated")
        min_deg = float(raw_min)
        max_deg = float(raw_max)
        if not math.isfinite(min_deg) or not math.isfinite(max_deg) or min_deg >= max_deg:
            raise ValueError(f"joint '{name}' minimum must be below its maximum")
        home_ticks = int(homes.get(name, DEFAULT_HOME_TICKS))
        if not 0 <= home_ticks < TICKS_PER_REV:
            raise ValueError(f"joint '{name}' home ticks must be between 0 and {TICKS_PER_REV - 1}")
        joints[name] = JointSpec(
            name=name,
            servo_id=servo_id,
            min_deg=min_deg,
            max_deg=max_deg,
            home_ticks=home_ticks,
            invert=name in inverted_names,
        )
        servo_ids.add(servo_id)
    if not joints:
        raise ValueError("at least one joint is required")
    unknown_homes = sorted(set(homes) - set(joints))
    unknown_inverted = sorted(inverted_names - set(joints))
    if unknown_homes:
        raise ValueError(f"home-ticks names are not in the joint map: {', '.join(unknown_homes)}")
    if unknown_inverted:
        raise ValueError(f"invert names are not in the joint map: {', '.join(unknown_inverted)}")
    return joints


def joints_from_profile(profile: Mapping[str, Any]) -> dict[str, JointSpec]:
    entries = profile.get("joints") if isinstance(profile.get("joints"), list) else []
    chunks: list[str] = []
    homes: dict[str, int] = {}
    inverted: set[str] = set()
    for index, raw_joint in enumerate(entries, start=1):
        if not isinstance(raw_joint, Mapping):
            raise ValueError(f"profile joint {index} must be an object")
        name = str(raw_joint.get("id") or raw_joint.get("name") or "").strip()
        servo_id = int(raw_joint.get("servo_id"))
        min_deg = float(raw_joint.get("safe_min_deg", raw_joint.get("min_deg", -180.0)))
        max_deg = float(raw_joint.get("safe_max_deg", raw_joint.get("max_deg", 180.0)))
        chunks.append(f"{name}:{servo_id}:{min_deg}:{max_deg}")
        homes[name] = int(raw_joint.get("home_ticks", DEFAULT_HOME_TICKS))
        if bool(raw_joint.get("invert")):
            inverted.add(name)
    return parse_joint_map(",".join(chunks), homes, inverted)


def joints_from_config(config: Mapping[str, Any]) -> dict[str, JointSpec]:
    entries = config.get("joints") if isinstance(config.get("joints"), list) else []
    joints: dict[str, JointSpec] = {}
    for raw_joint in entries:
        if not isinstance(raw_joint, Mapping):
            raise ValueError("configured joints must be objects")
        joint = JointSpec(
            name=str(raw_joint.get("name") or ""),
            servo_id=int(raw_joint.get("servo_id")),
            min_deg=float(raw_joint.get("min_deg")),
            max_deg=float(raw_joint.get("max_deg")),
            home_ticks=int(raw_joint.get("home_ticks", DEFAULT_HOME_TICKS)),
            invert=bool(raw_joint.get("invert")),
        )
        parsed = parse_joint_map(
            f"{joint.name}:{joint.servo_id}:{joint.min_deg}:{joint.max_deg}",
            {joint.name: joint.home_ticks},
            {joint.name} if joint.invert else set(),
        )
        if joint.name in joints:
            raise ValueError(f"joint name '{joint.name}' is duplicated")
        if any(existing.servo_id == joint.servo_id for existing in joints.values()):
            raise ValueError(f"servo id {joint.servo_id} is duplicated")
        joints.update(parsed)
    if not joints:
        raise ValueError("configuration contains no joints")
    return joints


def ticks_to_degrees(ticks: int, joint: JointSpec) -> float:
    degrees = (int(ticks) - joint.home_ticks) * 360.0 / TICKS_PER_REV
    return -degrees if joint.invert else degrees


def ticks_to_radians(ticks: int, joint: JointSpec) -> float:
    return math.radians(ticks_to_degrees(ticks, joint))


def degrees_to_ticks(degrees: float, joint: JointSpec) -> int:
    signed = -float(degrees) if joint.invert else float(degrees)
    ticks = joint.home_ticks + round(signed * TICKS_PER_REV / 360.0)
    return max(0, min(TICKS_PER_REV - 1, ticks))


def clamp_degrees(degrees: float, joint: JointSpec) -> float:
    return max(joint.min_deg, min(joint.max_deg, float(degrees)))


def load_sdk() -> Any:
    try:
        import scservo_sdk
    except Exception as exc:  # noqa: BLE001 - converted into an actionable package error
        raise RuntimeError(
            "Feetech SDK is unavailable; run: blacknode packages setup blacknode-drivers"
        ) from exc
    return scservo_sdk


def open_port(sdk: Any, port_name: str, baudrate: int) -> Any:
    if not str(port_name or "").strip():
        raise ValueError("serial port is required")
    port = sdk.PortHandler(str(port_name))
    try:
        if not port.openPort() or not port.setBaudRate(int(baudrate)):
            raise RuntimeError("SDK rejected the port or baud rate")
    except Exception as exc:
        try:
            port.closePort()
        except Exception:
            pass
        raise RuntimeError(
            f"could not open Feetech serial port {port_name} at {baudrate} baud: {exc}"
        ) from exc
    return port


def decode_hardware_errors(flags: int) -> list[str]:
    return [
        label
        for bit, label in _HARDWARE_ERROR_BITS.items()
        if int(flags) & bit
    ]


def read_position_status(
    sdk: Any,
    packet: Any,
    port: Any,
    servo_id: int,
) -> tuple[int | None, int]:
    """Return valid position data separately from servo hardware flags."""
    address, _width = ADDR_PRESENT_POSITION
    try:
        ticks, comm_result, servo_error = packet.read2ByteTxRx(port, servo_id, address)
    except Exception:
        return None, 0
    if comm_result != sdk.COMM_SUCCESS:
        return None, int(servo_error or 0)
    try:
        ticks_value = int(ticks)
    except (TypeError, ValueError):
        return None, int(servo_error or 0)
    if not 0 <= ticks_value < TICKS_PER_REV:
        return None, int(servo_error or 0)
    return ticks_value, int(servo_error or 0)


def read_position_or_none(sdk: Any, packet: Any, port: Any, servo_id: int) -> int | None:
    ticks, hardware_flags = read_position_status(
        sdk,
        packet,
        port,
        servo_id,
    )
    return ticks if hardware_flags == 0 else None


def read_torque_enabled_status(
    sdk: Any,
    packet: Any,
    port: Any,
    servo_id: int,
) -> tuple[bool | None, int]:
    """Return the physical torque bit separately from servo hardware flags."""
    address, _width = ADDR_TORQUE_ENABLE
    try:
        value, comm_result, servo_error = packet.read1ByteTxRx(
            port,
            servo_id,
            address,
        )
    except Exception:
        return None, 0
    if comm_result != sdk.COMM_SUCCESS:
        return None, int(servo_error or 0)
    return bool(value), int(servo_error or 0)


def read_torque_enabled_or_none(
    sdk: Any,
    packet: Any,
    port: Any,
    servo_id: int,
) -> bool | None:
    enabled, hardware_flags = read_torque_enabled_status(
        sdk,
        packet,
        port,
        servo_id,
    )
    return enabled if hardware_flags == 0 else None


def read_servo_diagnostics(
    sdk: Any,
    packet: Any,
    port: Any,
    servo_id: int,
) -> dict[str, Any] | None:
    """Read voltage, temperature, and status without changing servo state."""
    try:
        packed, comm_result, servo_error = packet.read4ByteTxRx(
            port,
            servo_id,
            ADDR_PRESENT_VOLTAGE[0],
        )
    except Exception:
        return None
    if comm_result != sdk.COMM_SUCCESS:
        return None
    raw = int(packed)
    status = int((raw >> 24) & 0xFF)
    flags = int(servo_error or 0) | status
    return {
        "voltage_v": float(raw & 0xFF) / 10.0,
        "temperature_c": float((raw >> 8) & 0xFF),
        "servo_status": status,
        "hardware_error_flags": flags,
        "hardware_errors": decode_hardware_errors(flags),
    }


def probe_bus(config: Mapping[str, Any], sdk: Any | None = None) -> dict[str, Any]:
    """Read position and health registers without performing any writes."""
    joints = joints_from_config(config)
    hardware_sdk = sdk or load_sdk()
    port = open_port(hardware_sdk, str(config.get("port") or ""), int(config.get("baudrate") or 1_000_000))
    packet = hardware_sdk.PacketHandler(0)
    readings: dict[str, dict[str, Any]] = {}
    errors: list[str] = []
    counters = {
        "operation_count": 0,
        "timeout_count": 0,
        "serial_packet_error_count": 0,
        "hardware_error_count": 0,
    }

    def record(comm_result: Any, servo_error: int) -> None:
        counters["operation_count"] += 1
        if comm_result != hardware_sdk.COMM_SUCCESS:
            counters["serial_packet_error_count"] += 1
            if comm_result == getattr(hardware_sdk, "COMM_RX_TIMEOUT", object()):
                counters["timeout_count"] += 1
        if servo_error:
            counters["hardware_error_count"] += 1

    try:
        for name, joint in joints.items():
            address, _width = ADDR_PRESENT_POSITION
            try:
                ticks, comm_result, position_error = packet.read2ByteTxRx(
                    port,
                    joint.servo_id,
                    address,
                )
            except Exception:
                counters["operation_count"] += 1
                counters["serial_packet_error_count"] += 1
                errors.append(
                    f"no valid Present_Position response from {name} "
                    f"(servo {joint.servo_id})"
                )
                continue
            record(comm_result, position_error)
            if comm_result != hardware_sdk.COMM_SUCCESS:
                errors.append(f"no valid Present_Position response from {name} (servo {joint.servo_id})")
                continue
            packed = 0
            diagnostic_error = 0
            try:
                packed, diagnostic_result, diagnostic_error = packet.read4ByteTxRx(
                    port,
                    joint.servo_id,
                    ADDR_PRESENT_VOLTAGE[0],
                )
                record(diagnostic_result, diagnostic_error)
            except Exception:
                diagnostic_result = None
                counters["operation_count"] += 1
                counters["serial_packet_error_count"] += 1
            status = int((int(packed) >> 24) & 0xFF) if diagnostic_result == hardware_sdk.COMM_SUCCESS else 0
            hardware_flags = int(position_error or 0) | int(diagnostic_error or 0) | status
            if status:
                counters["hardware_error_count"] += 1
            if diagnostic_result != hardware_sdk.COMM_SUCCESS:
                errors.append(
                    f"no valid voltage/temperature/status response from {name} "
                    f"(servo {joint.servo_id})"
                )
            if hardware_flags:
                errors.append(
                    f"{name} (servo {joint.servo_id}) reports hardware flags "
                    f"0x{hardware_flags:02x}"
                )
            readings[name] = {
                "servo_id": joint.servo_id,
                "ticks": int(ticks),
                "position_rad": ticks_to_radians(int(ticks), joint),
                "voltage_v": (
                    float(int(packed) & 0xFF) / 10.0
                    if diagnostic_result == hardware_sdk.COMM_SUCCESS
                    else None
                ),
                "temperature_c": (
                    float((int(packed) >> 8) & 0xFF)
                    if diagnostic_result == hardware_sdk.COMM_SUCCESS
                    else None
                ),
                "hardware_error_flags": hardware_flags,
                "hardware_errors": decode_hardware_errors(hardware_flags),
            }
    finally:
        try:
            port.closePort()
        except Exception:
            pass
    counters["serial_packet_error_rate"] = (
        counters["serial_packet_error_count"] / counters["operation_count"]
        if counters["operation_count"]
        else 0.0
    )
    return {
        "ok": not errors,
        "readings": readings,
        "diagnostics": counters,
        "errors": errors,
    }


def probe_raw_servos(
    config: Mapping[str, Any],
    sdk: Any | None = None,
) -> dict[str, Any]:
    """Discover and sample servo IDs with reads only and no profile assumptions."""
    hardware_sdk = sdk or load_sdk()
    port = open_port(
        hardware_sdk,
        str(config.get("port") or ""),
        int(config.get("baudrate") or 1_000_000),
    )
    packet = hardware_sdk.PacketHandler(0)
    configured_ids = config.get("servo_ids")
    if isinstance(configured_ids, (list, tuple, set)):
        parsed_ids: set[int] = set()
        for value in configured_ids:
            try:
                servo_id = int(value)
            except (TypeError, ValueError):
                continue
            if 1 <= servo_id <= 253:
                parsed_ids.add(servo_id)
        servo_ids = sorted(parsed_ids)
    else:
        maximum = max(1, min(253, int(config.get("max_servo_id") or 32)))
        servo_ids = list(range(1, maximum + 1))
    discovering = bool(config.get("discovering", configured_ids is None))
    readings: list[dict[str, Any]] = []
    warnings: list[str] = []
    errors: list[str] = []
    torque_states: list[bool] = []
    counters = {
        "operation_count": 0,
        "scan_miss_count": 0,
        "timeout_count": 0,
        "serial_packet_error_count": 0,
        "hardware_error_count": 0,
    }

    def failed_operation(comm_result: Any, *, scan_miss: bool = False) -> None:
        counters["operation_count"] += 1
        if scan_miss:
            counters["scan_miss_count"] += 1
            return
        counters["serial_packet_error_count"] += 1
        if comm_result == getattr(hardware_sdk, "COMM_RX_TIMEOUT", object()):
            counters["timeout_count"] += 1

    try:
        for servo_id in servo_ids:
            try:
                ticks, comm_result, position_flags = packet.read2ByteTxRx(
                    port,
                    servo_id,
                    ADDR_PRESENT_POSITION[0],
                )
            except Exception:
                failed_operation(None, scan_miss=discovering)
                continue
            if comm_result != hardware_sdk.COMM_SUCCESS:
                failed_operation(comm_result, scan_miss=discovering)
                continue
            counters["operation_count"] += 1
            try:
                ticks_value = int(ticks)
            except (TypeError, ValueError):
                counters["serial_packet_error_count"] += 1
                continue
            if not 0 <= ticks_value < TICKS_PER_REV:
                counters["serial_packet_error_count"] += 1
                continue

            torque_enabled: bool | None = None
            torque_flags = 0
            try:
                raw_torque, torque_result, torque_flags = packet.read1ByteTxRx(
                    port,
                    servo_id,
                    ADDR_TORQUE_ENABLE[0],
                )
                counters["operation_count"] += 1
                if torque_result == hardware_sdk.COMM_SUCCESS:
                    torque_enabled = bool(raw_torque)
                    torque_states.append(torque_enabled)
                else:
                    counters["serial_packet_error_count"] += 1
                    if torque_result == getattr(
                        hardware_sdk,
                        "COMM_RX_TIMEOUT",
                        object(),
                    ):
                        counters["timeout_count"] += 1
            except Exception:
                counters["operation_count"] += 1
                counters["serial_packet_error_count"] += 1

            diagnostics: dict[str, Any] = {}
            try:
                packed, diagnostic_result, diagnostic_flags = (
                    packet.read4ByteTxRx(
                        port,
                        servo_id,
                        ADDR_PRESENT_VOLTAGE[0],
                    )
                )
                counters["operation_count"] += 1
                if diagnostic_result == hardware_sdk.COMM_SUCCESS:
                    raw = int(packed)
                    status = int((raw >> 24) & 0xFF)
                    diagnostics = {
                        "voltage_v": float(raw & 0xFF) / 10.0,
                        "temperature_c": float((raw >> 8) & 0xFF),
                        "servo_status": status,
                        "hardware_error_flags": int(diagnostic_flags or 0)
                        | status,
                    }
                else:
                    counters["serial_packet_error_count"] += 1
                    if diagnostic_result == getattr(
                        hardware_sdk,
                        "COMM_RX_TIMEOUT",
                        object(),
                    ):
                        counters["timeout_count"] += 1
            except Exception:
                counters["operation_count"] += 1
                counters["serial_packet_error_count"] += 1

            hardware_flags = (
                int(position_flags or 0)
                | int(torque_flags or 0)
                | int(diagnostics.get("hardware_error_flags") or 0)
            )
            hardware_errors = decode_hardware_errors(hardware_flags)
            if hardware_flags:
                counters["hardware_error_count"] += 1
                warning = (
                    f"servo_{servo_id} (servo {servo_id}) hardware warning "
                    f"0x{hardware_flags:02x}: "
                    + (", ".join(hardware_errors) or "vendor status")
                )
                voltage = diagnostics.get("voltage_v")
                if (
                    "voltage" in hardware_errors
                    and isinstance(voltage, (int, float))
                    and not isinstance(voltage, bool)
                ):
                    warning += (
                        f"; measured input {float(voltage):.1f} V. "
                        "Check that the connected power supply matches the "
                        "servo voltage rating."
                    )
                warnings.append(warning)
            readings.append({
                "name": f"servo_{servo_id}",
                "semantic_name": f"Servo {servo_id}",
                "servo_id": servo_id,
                "position": float(ticks_value),
                "velocity": 0.0,
                "raw_position": ticks_value,
                "communication_ok": True,
                "torque_enabled": torque_enabled,
                "voltage_v": diagnostics.get("voltage_v"),
                "temperature_c": diagnostics.get("temperature_c"),
                "servo_status": diagnostics.get("servo_status"),
                "hardware_error_flags": hardware_flags,
                "hardware_errors": hardware_errors,
            })
    finally:
        try:
            port.closePort()
        except Exception:
            pass

    if not readings:
        errors.append(
            "no responding Feetech servos were found in the read-only scan"
        )
    counters["serial_packet_error_rate"] = (
        counters["serial_packet_error_count"] / counters["operation_count"]
        if counters["operation_count"]
        else 0.0
    )
    torque_enabled: bool | None = (
        torque_states[0]
        if torque_states and len(torque_states) == len(readings)
        and len(set(torque_states)) == 1
        else None
    )
    return {
        "ok": bool(readings),
        "joints": readings,
        "position_unit": "ticks",
        "velocity_unit": "ticks/s",
        "torque_enabled": torque_enabled,
        "warnings": warnings,
        "errors": errors,
        "diagnostics": counters,
    }


def _write_goal(sdk: Any, packet: Any, port: Any, servo_id: int, ticks: int) -> bool:
    address, _width = ADDR_GOAL_POSITION
    try:
        comm_result, servo_error = packet.write2ByteTxRx(port, servo_id, address, int(ticks))
    except Exception:
        return False
    return comm_result == sdk.COMM_SUCCESS and servo_error == 0


def _set_torque(sdk: Any, packet: Any, port: Any, servo_id: int, enabled: bool) -> bool:
    address, _width = ADDR_TORQUE_ENABLE
    desired = 1 if enabled else 0
    for _attempt in range(2):
        try:
            comm_result, servo_error = packet.write1ByteTxRx(
                port, servo_id, address, desired
            )
        except Exception:
            comm_result, servo_error = None, None
        if comm_result == sdk.COMM_SUCCESS and servo_error == 0:
            return True
        # A servo hardware warning does not prove a torque-off write failed.
        # Read the physical register back and accept only confirmed zero.
        # Enabling torque remains strict: any warning blocks authorization.
        if enabled and servo_error not in (None, 0):
            return False
        try:
            confirmed, read_result, read_error = packet.read1ByteTxRx(
                port, servo_id, address
            )
        except Exception:
            continue
        if (
            read_result == sdk.COMM_SUCCESS
            and int(confirmed) == desired
            and (read_error == 0 or not enabled)
        ):
            return True
    return False


def disable_all_torque(
    sdk: Any,
    packet: Any,
    port: Any,
    joints: Mapping[str, JointSpec],
) -> tuple[bool, str]:
    failed = [
        name
        for name, joint in joints.items()
        if not _set_torque(sdk, packet, port, joint.servo_id, False)
    ]
    if failed:
        return False, f"could not disable torque for: {', '.join(failed)}"
    return True, ""

def enable_all_torque_at_current_pose(
    sdk: Any,
    packet: Any,
    port: Any,
    joints: Mapping[str, JointSpec],
) -> tuple[bool, dict[str, int], str]:
    """Seed every current position before enabling any holding torque."""
    current_ticks: dict[str, int] = {}
    for name, joint in joints.items():
        ticks = read_position_or_none(sdk, packet, port, joint.servo_id)
        if ticks is None:
            disable_all_torque(sdk, packet, port, joints)
            return False, current_ticks, (
                f"could not read Present_Position for {name} (servo {joint.servo_id})"
            )
        current_ticks[name] = ticks
    for name, joint in joints.items():
        if not _write_goal(sdk, packet, port, joint.servo_id, current_ticks[name]):
            disable_all_torque(sdk, packet, port, joints)
            return False, current_ticks, (
                f"could not seed Goal_Position for {name} (servo {joint.servo_id})"
            )
    for name, joint in joints.items():
        if not _set_torque(sdk, packet, port, joint.servo_id, True):
            disable_all_torque(sdk, packet, port, joints)
            return False, current_ticks, (
                f"could not enable torque for {name} (servo {joint.servo_id})"
            )
    return True, current_ticks, ""


def write_joint_positions(
    sdk: Any,
    packet: Any,
    port: Any,
    joints: Mapping[str, JointSpec],
    degrees_by_name: Mapping[str, float],
) -> tuple[bool, str]:
    """Clamp every command at the hardware boundary and use confirmed writes."""
    for name, requested in degrees_by_name.items():
        joint = joints.get(name)
        if joint is None:
            return False, f"unknown joint '{name}'"
        target = degrees_to_ticks(clamp_degrees(float(requested), joint), joint)
        if not _write_goal(sdk, packet, port, joint.servo_id, target):
            return False, f"could not write Goal_Position for {name} (servo {joint.servo_id})"
    return True, ""
