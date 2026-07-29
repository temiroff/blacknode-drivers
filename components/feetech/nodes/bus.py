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


def read_position_or_none(sdk: Any, packet: Any, port: Any, servo_id: int) -> int | None:
    address, _width = ADDR_PRESENT_POSITION
    try:
        ticks, comm_result, servo_error = packet.read2ByteTxRx(port, servo_id, address)
    except Exception:
        return None
    if comm_result != sdk.COMM_SUCCESS or servo_error != 0:
        return None
    return int(ticks)


def probe_bus(config: Mapping[str, Any], sdk: Any | None = None) -> dict[str, Any]:
    """Read present positions only; this function performs no register writes."""
    joints = joints_from_config(config)
    hardware_sdk = sdk or load_sdk()
    port = open_port(hardware_sdk, str(config.get("port") or ""), int(config.get("baudrate") or 1_000_000))
    packet = hardware_sdk.PacketHandler(0)
    readings: dict[str, dict[str, Any]] = {}
    errors: list[str] = []
    try:
        for name, joint in joints.items():
            ticks = read_position_or_none(hardware_sdk, packet, port, joint.servo_id)
            if ticks is None:
                errors.append(f"no valid Present_Position response from {name} (servo {joint.servo_id})")
                continue
            readings[name] = {
                "servo_id": joint.servo_id,
                "ticks": ticks,
                "degrees": ticks_to_degrees(ticks, joint),
            }
    finally:
        try:
            port.closePort()
        except Exception:
            pass
    return {"ok": not errors, "readings": readings, "errors": errors}


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
        if servo_error not in (None, 0):
            return False
        try:
            confirmed, read_result, read_error = packet.read1ByteTxRx(
                port, servo_id, address
            )
        except Exception:
            continue
        if (
            read_result == sdk.COMM_SUCCESS
            and read_error == 0
            and int(confirmed) == desired
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
