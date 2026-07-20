#!/usr/bin/env python3
"""Feetech STS/SMS serial-bus servo driver -> Blacknode's native ROS 2 JointState contract.

Reusable across any Feetech-protocol robot, not just the SO-ARM101: pass a
different --joints map for a different arm. This script adds no new safety
logic of its own -- it publishes JointState on --state-topic, subscribes
JointState on --command-topic, and publishes one latched JSON config message
on --config-topic, exactly the contract already read by
packages/blacknode-ros2/nodes/ros2_native_runtime.py (ROS2NativeStatus /
ROS2NativeJointState / ROS2NativeSetJoint). Those nodes already sync-before-move
and clamp to the published limits; this script's own job is narrower: never
let the servos jump when torque switches on, and never write outside a
joint's calibrated range.

Hardware imports (rclpy, scservo_sdk) are deferred out of module top-level so
the pure parsing/math helpers below stay importable -- and unit-testable --
on a machine with neither ROS 2 nor the servo SDK installed.
"""
from __future__ import annotations

import argparse
import json
import math
import signal
import sys
import threading
import time
from dataclasses import dataclass
from typing import Any

_TICKS_PER_REV = 4096          # STS3215: 12-bit single-turn position range (0-4095)
_DEFAULT_HOME_TICKS = 2048     # protocol mid-point; override per-joint with --home-ticks
                                # if a joint's true mechanical zero differs after assembly

# Feetech STS/SMS control table addresses (register, byte width). Widely used
# across public Feetech/LeRobot/Waveshare STS3215 driver code, but VERIFY
# against real hardware with --dry-run before any write ever runs (see the
# package README's Safety section).
ADDR_TORQUE_ENABLE = (40, 1)
ADDR_GOAL_POSITION = (42, 2)
ADDR_PRESENT_POSITION = (56, 2)


@dataclass(frozen=True)
class JointSpec:
    name: str
    servo_id: int
    min_deg: float
    max_deg: float
    home_ticks: int = _DEFAULT_HOME_TICKS
    invert: bool = False


def parse_int_map(spec: str) -> dict[str, int]:
    """'name:ticks,name:ticks' -> {name: ticks}."""
    result: dict[str, int] = {}
    for chunk in (c.strip() for c in (spec or "").split(",")):
        if not chunk:
            continue
        name, _, ticks = chunk.partition(":")
        result[name.strip()] = int(ticks.strip())
    return result


def parse_joint_map(spec: str, home_overrides: dict[str, int], inverted: set[str]) -> dict[str, JointSpec]:
    """'shoulder_pan:1:-100:100,...' -> {name: JointSpec}."""
    joints: dict[str, JointSpec] = {}
    for chunk in (c.strip() for c in (spec or "").split(",")):
        if not chunk:
            continue
        name, sid, lo, hi = (part.strip() for part in chunk.split(":"))
        joints[name] = JointSpec(
            name=name,
            servo_id=int(sid),
            min_deg=float(lo),
            max_deg=float(hi),
            home_ticks=home_overrides.get(name, _DEFAULT_HOME_TICKS),
            invert=name in inverted,
        )
    return joints


def ticks_to_degrees(ticks: int, joint: JointSpec) -> float:
    deg = (ticks - joint.home_ticks) * 360.0 / _TICKS_PER_REV
    return -deg if joint.invert else deg


def degrees_to_ticks(deg: float, joint: JointSpec) -> int:
    signed = -deg if joint.invert else deg
    ticks = joint.home_ticks + round(signed * _TICKS_PER_REV / 360.0)
    return max(0, min(_TICKS_PER_REV - 1, ticks))


def clamp_degrees(deg: float, joint: JointSpec) -> float:
    lo, hi = min(joint.min_deg, joint.max_deg), max(joint.min_deg, joint.max_deg)
    return max(lo, min(hi, deg))


def _fail(message: str, code: int = 1) -> None:
    print(json.dumps({"ok": False, "error": message}), file=sys.stderr)
    sys.exit(code)


def _hardware_imports(transport: str = "native") -> dict[str, Any]:
    try:
        import scservo_sdk as sdk
        if transport == "sdk":
            return {"sdk": sdk}
        if transport == "rosbridge":
            import roslibpy
            return {"sdk": sdk, "roslibpy": roslibpy}
        import rclpy
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from sensor_msgs.msg import JointState
        from std_msgs.msg import String
    except Exception as exc:  # noqa: BLE001 - surfaced as a structured subprocess failure
        _fail(f"missing dependency: {type(exc).__name__}: {exc}", code=2)
        raise  # unreachable, keeps type-checkers happy
    return {
        "rclpy": rclpy,
        "JointState": JointState,
        "String": String,
        "QoSProfile": QoSProfile,
        "ReliabilityPolicy": ReliabilityPolicy,
        "DurabilityPolicy": DurabilityPolicy,
        "sdk": sdk,
    }


def _joint_state_payload(ticks_by_name: dict[str, int], joints: dict[str, JointSpec]) -> dict[str, Any]:
    now = time.time()
    return {
        "header": {
            "stamp": {"sec": int(now), "nanosec": int((now % 1) * 1_000_000_000)},
            "frame_id": "",
        },
        "name": list(ticks_by_name),
        "position": [math.radians(ticks_to_degrees(ticks, joints[name])) for name, ticks in ticks_by_name.items()],
        "velocity": [],
        "effort": [],
    }


def _config_payload(
    joints: dict[str, JointSpec],
    *,
    torque_enabled: bool = True,
    last_error: str = "",
) -> dict[str, Any]:
    return {
        "commands_allowed": torque_enabled,
        "torque_enabled": torque_enabled,
        "teach_mode": not torque_enabled,
        "mode": "hold" if torque_enabled else "teach",
        "last_error": last_error,
        "joints": {
            name: {
                "lower": math.radians(clamp_degrees(min(joint.min_deg, joint.max_deg), joint)),
                "upper": math.radians(clamp_degrees(max(joint.min_deg, joint.max_deg), joint)),
            }
            for name, joint in joints.items()
        },
    }


def _apply_command(
    message: Any,
    joints: dict[str, JointSpec],
    sdk: Any,
    packet: Any,
    port: Any,
) -> None:
    names = message.get("name", []) if isinstance(message, dict) else getattr(message, "name", [])
    positions = message.get("position", []) if isinstance(message, dict) else getattr(message, "position", [])
    goals: dict[int, int] = {}
    for name, position_rad in zip(names, positions):
        joint = joints.get(str(name))
        if joint is None:
            continue
        deg = clamp_degrees(math.degrees(float(position_rad)), joint)
        goals[joint.servo_id] = degrees_to_ticks(deg, joint)
    if goals:
        _sync_write_goals(sdk, packet, port, goals)


def _run_rosbridge(
    args: argparse.Namespace,
    imports: dict[str, Any],
    joints: dict[str, JointSpec],
    sdk: Any,
    packet: Any,
    port: Any,
    current_ticks: dict[str, int],
    stop_event: threading.Event,
) -> None:
    roslibpy = imports["roslibpy"]
    ros = roslibpy.Ros(host=args.host, port=args.rosbridge_port)
    # ``roslibpy`` uses a reconnecting Twisted client. Keep its reactor and
    # topic objects alive for the lifetime of the hardware driver: returning
    # from this function on the first dropped WebSocket used to leave a Python
    # process that looked healthy to Blacknode but no longer published state.
    while not stop_event.is_set() and not ros.is_connected:
        try:
            ros.run(timeout=max(1.0, args.connect_timeout))
        except Exception:
            if stop_event.wait(1.0):
                break
    if stop_event.is_set():
        ros.terminate()
        return
    state_pub = roslibpy.Topic(ros, args.state_topic, "sensor_msgs/msg/JointState")
    config_pub = roslibpy.Topic(ros, args.config_topic, "std_msgs/msg/String", latch=True)
    command_sub = roslibpy.Topic(ros, args.command_topic, "sensor_msgs/msg/JointState")
    control_sub = roslibpy.Topic(ros, args.control_topic, "std_msgs/msg/String")
    bus_lock = threading.Lock()
    control_state: dict[str, Any] = {"torque_enabled": True, "last_error": ""}
    last_known_ticks = dict(current_ticks)

    def publish_config() -> None:
        config_pub.publish(roslibpy.Message({
            "data": json.dumps(_config_payload(
                joints,
                torque_enabled=bool(control_state["torque_enabled"]),
                last_error=str(control_state["last_error"]),
            ))
        }))

    def apply_command_safely(message: Any) -> None:
        try:
            # roslibpy callbacks run on a reactor worker while the state loop
            # below reads the same half-duplex serial bus. Feetech packet
            # transactions must never overlap.
            with bus_lock:
                if not control_state["torque_enabled"]:
                    return
                _apply_command(message, joints, sdk, packet, port)
        except Exception as exc:  # keep one bad command from killing transport
            print(f"robot command rejected: {type(exc).__name__}: {exc}", file=sys.stderr, flush=True)

    def apply_control_safely(message: Any) -> None:
        action = _control_action(message)
        if not action:
            return
        try:
            with bus_lock:
                if _control_already_applied(action, control_state):
                    pass
                elif action == "enter_teach":
                    ok, error = _disable_all_torque(sdk, packet, port, joints)
                    control_state["torque_enabled"] = False
                    control_state["last_error"] = error
                else:
                    ok, seeded_ticks, error = _enable_all_torque_at_current_pose(sdk, packet, port, joints)
                    if ok:
                        last_known_ticks.update(seeded_ticks)
                    control_state["torque_enabled"] = ok
                    control_state["last_error"] = error
            publish_config()
        except Exception as exc:
            control_state["last_error"] = f"{type(exc).__name__}: {exc}"
            publish_config()

    state_pub.advertise()
    config_pub.advertise()
    command_sub.subscribe(apply_command_safely)
    control_sub.subscribe(apply_control_safely)
    try:
        publish_config()
        state_pub.publish(roslibpy.Message(_joint_state_payload(last_known_ticks, joints)))
        period = 1.0 / max(0.1, args.rate_hz)
        was_connected = True
        while not stop_event.wait(period):
            if not ros.is_connected:
                was_connected = False
                continue
            if not was_connected:
                # Topic reconnect hooks are scheduled one second after the
                # socket becomes ready. Wait for them, then refresh the
                # latched config and current pose for late subscribers.
                if stop_event.wait(1.1) or not ros.is_connected:
                    continue
                publish_config()
                state_pub.publish(roslibpy.Message(_joint_state_payload(last_known_ticks, joints)))
                was_connected = True
            with bus_lock:
                last_known_ticks.update(_sync_read_positions(sdk, packet, port, joints))
            state_pub.publish(roslibpy.Message(_joint_state_payload(last_known_ticks, joints)))
    finally:
        try:
            command_sub.unsubscribe()
            control_sub.unsubscribe()
            state_pub.unadvertise()
            config_pub.unadvertise()
        finally:
            ros.terminate()


def _read_position(sdk: Any, packet: Any, port: Any, servo_id: int) -> int:
    """Strict read used only during startup seeding: any failure here means
    the safety sequence cannot proceed, so it aborts the whole process rather
    than risk enabling torque against an unknown position."""
    ticks = _read_position_or_none(sdk, packet, port, servo_id)
    if ticks is None:
        _fail(f"could not read Present_Position for servo id {servo_id}")
    return ticks


def _read_position_or_none(sdk: Any, packet: Any, port: Any, servo_id: int) -> int | None:
    """Best-effort read used by the steady-state publish loop: a transient
    bus error on one poll should not take down an otherwise-healthy driver
    process, so this returns None instead of exiting."""
    address, _width = ADDR_PRESENT_POSITION
    try:
        ticks, comm_result, servo_error = packet.read2ByteTxRx(port, servo_id, address)
    except Exception:
        # scservo_sdk can raise IndexError when a serial response is shorter
        # than the protocol header promised. Treat malformed/transient packets
        # exactly like COMM_RX_CORRUPT and retain the last valid joint value.
        return None
    if comm_result != sdk.COMM_SUCCESS or servo_error != 0:
        return None
    return int(ticks)


def _sync_read_positions(
    sdk: Any,
    packet: Any,
    port: Any,
    joints: dict[str, JointSpec],
) -> dict[str, int]:
    """Read the whole daisy chain with the same group transaction as LeRobot.

    Older/test SDK shims may not expose GroupSyncRead; retain a sequential
    fallback so custom protocol adapters remain compatible.
    """
    group_type = getattr(sdk, "GroupSyncRead", None)
    if group_type is None:
        return {
            name: ticks
            for name, joint in joints.items()
            if (ticks := _read_position_or_none(sdk, packet, port, joint.servo_id)) is not None
        }
    address, width = ADDR_PRESENT_POSITION
    group = group_type(port, packet, address, width)
    for joint in joints.values():
        if not group.addParam(joint.servo_id):
            return {}
    try:
        if group.txRxPacket() != sdk.COMM_SUCCESS:
            return {}
        return {
            name: int(group.getData(joint.servo_id, address, width))
            for name, joint in joints.items()
            if group.isAvailable(joint.servo_id, address, width)
        }
    except Exception:
        return {}


def _sync_write_goals(
    sdk: Any,
    packet: Any,
    port: Any,
    goals: dict[int, int],
) -> bool:
    """Send all joint goals in one broadcast packet, matching LeRobot."""
    group_type = getattr(sdk, "GroupSyncWrite", None)
    if group_type is None:
        return all(
            _write_goal(sdk, packet, port, servo_id, ticks, confirm=True)
            for servo_id, ticks in goals.items()
        )
    address, width = ADDR_GOAL_POSITION
    group = group_type(port, packet, address, width)
    low_byte = getattr(sdk, "SCS_LOBYTE", lambda value: value & 0xFF)
    high_byte = getattr(sdk, "SCS_HIBYTE", lambda value: (value >> 8) & 0xFF)
    for servo_id, ticks in goals.items():
        if not group.addParam(servo_id, [low_byte(ticks), high_byte(ticks)]):
            return False
    try:
        return group.txPacket() == sdk.COMM_SUCCESS
    except Exception:
        return False


def _write_goal(sdk: Any, packet: Any, port: Any, servo_id: int, ticks: int, *, confirm: bool) -> bool:
    address, _width = ADDR_GOAL_POSITION
    if confirm:
        comm_result, servo_error = packet.write2ByteTxRx(port, servo_id, address, ticks)
        if comm_result != sdk.COMM_SUCCESS or servo_error != 0:
            return False
        return True
    packet.write2ByteTxOnly(port, servo_id, address, ticks)
    return True


def _set_torque(sdk: Any, packet: Any, port: Any, servo_id: int, enabled: bool) -> bool:
    address, _width = ADDR_TORQUE_ENABLE
    comm_result, servo_error = packet.write1ByteTxRx(port, servo_id, address, 1 if enabled else 0)
    return comm_result == sdk.COMM_SUCCESS and servo_error == 0


def _disable_all_torque(
    sdk: Any,
    packet: Any,
    port: Any,
    joints: dict[str, JointSpec],
) -> tuple[bool, str]:
    failed = [
        name
        for name, joint in joints.items()
        if not _set_torque(sdk, packet, port, joint.servo_id, False)
    ]
    if failed:
        return False, f"could not disable torque for: {', '.join(failed)}"
    return True, ""


def _enable_all_torque_at_current_pose(
    sdk: Any,
    packet: Any,
    port: Any,
    joints: dict[str, JointSpec],
) -> tuple[bool, dict[str, int], str]:
    """Read every joint, seed its goal while limp, then enable holding torque.

    Any read, goal-write, or torque-enable failure returns the whole arm to the
    safest available state (torque off) instead of leaving a partially holding
    robot.
    """
    current_ticks: dict[str, int] = {}
    for name, joint in joints.items():
        ticks = _read_position_or_none(sdk, packet, port, joint.servo_id)
        if ticks is None:
            _disable_all_torque(sdk, packet, port, joints)
            return False, current_ticks, f"could not read Present_Position for {name} (servo id {joint.servo_id})"
        current_ticks[name] = ticks

    for name, joint in joints.items():
        if not _write_goal(sdk, packet, port, joint.servo_id, current_ticks[name], confirm=True):
            _disable_all_torque(sdk, packet, port, joints)
            return False, current_ticks, f"could not seed Goal_Position for {name} (servo id {joint.servo_id})"

    for name, joint in joints.items():
        if not _set_torque(sdk, packet, port, joint.servo_id, True):
            _disable_all_torque(sdk, packet, port, joints)
            return False, current_ticks, f"could not enable torque for {name} (servo id {joint.servo_id})"
    return True, current_ticks, ""


def _control_action(message: Any) -> str:
    raw = message.get("data", "") if isinstance(message, dict) else getattr(message, "data", "")
    try:
        payload = json.loads(raw or "{}")
    except (TypeError, ValueError):
        return ""
    action = str(payload.get("action") or "").strip().lower() if isinstance(payload, dict) else ""
    return action if action in {"enter_teach", "exit_teach"} else ""


def _control_already_applied(action: str, control_state: dict[str, Any]) -> bool:
    """True when a repeated reliable-delivery command needs no bus writes."""
    if str(control_state.get("last_error") or ""):
        return False
    torque_enabled = bool(control_state.get("torque_enabled"))
    return (action == "enter_teach" and not torque_enabled) or (action == "exit_teach" and torque_enabled)


def _open_port(sdk: Any, port_name: str, baudrate: int) -> Any:
    """Open the serial port and set its baud rate, converting whatever this
    SDK does on failure (some paths return False, others raise straight from
    pyserial -- e.g. a nonexistent device raises SerialException) into one
    consistent structured _fail() so callers never see a raw traceback."""
    port = sdk.PortHandler(port_name)
    try:
        opened = port.openPort()
        if opened:
            opened = port.setBaudRate(baudrate)
    except Exception as exc:  # noqa: BLE001 - pyserial raises on open failure
        _fail(f"could not open serial port {port_name} at baud {baudrate}: {type(exc).__name__}: {exc}")
    if not opened:
        _fail(f"could not open serial port {port_name} at baud {baudrate}")
    return port


def _dry_run(sdk: Any, joints: dict[str, JointSpec], port_name: str, baudrate: int) -> int:
    """Read-only servo bus probe: pings every joint's Present_Position, never
    touches Goal_Position or Torque_Enable. Use this to validate wiring and
    the control-table addresses above before any write is ever attempted."""
    port = _open_port(sdk, port_name, baudrate)
    packet = sdk.PacketHandler(0)

    readings = []
    for joint in joints.values():
        address, _width = ADDR_PRESENT_POSITION
        ticks, comm_result, servo_error = packet.read2ByteTxRx(port, joint.servo_id, address)
        ok = comm_result == sdk.COMM_SUCCESS and servo_error == 0 and 0 <= ticks < _TICKS_PER_REV
        readings.append({
            "joint": joint.name,
            "servo_id": joint.servo_id,
            "ok": ok,
            "ticks": int(ticks) if ok else None,
            "degrees": ticks_to_degrees(int(ticks), joint) if ok else None,
            "comm_result": packet.getTxRxResult(comm_result) if comm_result != sdk.COMM_SUCCESS else "COMM_SUCCESS",
        })
    port.closePort()
    print(json.dumps({"ok": all(r["ok"] for r in readings), "readings": readings}, indent=2))
    return 0 if all(r["ok"] for r in readings) else 1


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True, help="serial device, e.g. /dev/ttyACM0")
    parser.add_argument("--baudrate", type=int, default=1_000_000)
    parser.add_argument("--joints", required=True, help="name:servo_id:min_deg:max_deg,...")
    parser.add_argument("--home-ticks", default="", help="name:ticks,... override for a joint's true mechanical zero")
    parser.add_argument("--invert", default="", help="comma-separated joint names whose sign should be flipped")
    parser.add_argument("--state-topic", default="/joint_states")
    parser.add_argument("--command-topic", default="/joint_commands")
    parser.add_argument("--config-topic", default="/joint_config")
    parser.add_argument("--control-topic", default="/robot_control")
    parser.add_argument("--rate-hz", type=float, default=60.0)
    parser.add_argument("--transport", choices=("native", "rosbridge"), default="native")
    parser.add_argument("--host", default="127.0.0.1", help="rosbridge host when --transport=rosbridge")
    parser.add_argument("--rosbridge-port", type=int, default=9090)
    parser.add_argument("--connect-timeout", type=float, default=10.0)
    parser.add_argument(
        "--torque-off-on-exit",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="disable torque on every servo when the driver stops (default: on, so the arm goes limp "
             "rather than holding position indefinitely with no watchdog)",
    )
    parser.add_argument("--dry-run", action="store_true", help="probe Present_Position only, no writes, no torque changes")
    args = parser.parse_args()

    home_overrides = parse_int_map(args.home_ticks)
    inverted = {name.strip() for name in args.invert.split(",") if name.strip()}
    joints = parse_joint_map(args.joints, home_overrides, inverted)
    if not joints:
        _fail("no joints parsed from --joints")

    imports = _hardware_imports("sdk" if args.dry_run else args.transport)
    sdk = imports["sdk"]

    if args.dry_run:
        return _dry_run(sdk, joints, args.port, args.baudrate)

    port = _open_port(sdk, args.port, args.baudrate)
    packet = sdk.PacketHandler(0)

    # --- Torque-enable safety sequence -------------------------------------
    # Feetech STS servos snap toward whatever is already sitting in
    # Goal_Position the instant Torque_Enable switches on. That register is
    # NOT guaranteed to already equal the servo's physical position (stale
    # value from a previous session, or a register default). So: read first,
    # seed Goal_Position with the just-read value while torque is still off,
    # THEN enable torque -- there is nothing left for the servo to snap toward.
    enabled, current_ticks, enable_error = _enable_all_torque_at_current_pose(sdk, packet, port, joints)
    if not enabled:
        _fail(enable_error)

    stop_event = threading.Event()

    def handle_stop(*_: Any) -> None:
        stop_event.set()

    signal.signal(signal.SIGTERM, handle_stop)
    signal.signal(signal.SIGINT, handle_stop)

    if args.transport == "rosbridge":
        try:
            _run_rosbridge(args, imports, joints, sdk, packet, port, current_ticks, stop_event)
        finally:
            if args.torque_off_on_exit:
                for joint in joints.values():
                    _set_torque(sdk, packet, port, joint.servo_id, False)
            port.closePort()
        return 0

    rclpy = imports["rclpy"]
    JointState = imports["JointState"]
    String = imports["String"]
    QoSProfile = imports["QoSProfile"]
    ReliabilityPolicy = imports["ReliabilityPolicy"]
    DurabilityPolicy = imports["DurabilityPolicy"]

    rclpy.init(args=None)
    node = rclpy.create_node("blacknode_feetech_bus_driver")
    state_pub = node.create_publisher(JointState, args.state_topic, 10)
    config_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    config_pub = node.create_publisher(String, args.config_topic, config_qos)
    control_state: dict[str, Any] = {"torque_enabled": True, "last_error": ""}
    bus_lock = threading.Lock()

    def publish_state(ticks_by_name: dict[str, int]) -> None:
        msg = JointState()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.name = list(ticks_by_name.keys())
        msg.position = [math.radians(ticks_to_degrees(ticks, joints[name])) for name, ticks in ticks_by_name.items()]
        msg.velocity = []
        msg.effort = []
        state_pub.publish(msg)

    # First /joint_states publish is the just-seeded pose (real hardware
    # position), so ROS2NativeSetJoint's "sync to current pose" has a real
    # value the instant it reads, not a startup race against an empty topic.
    publish_state(current_ticks)

    def publish_config() -> None:
        config_msg = String()
        config_msg.data = json.dumps(_config_payload(
            joints,
            torque_enabled=bool(control_state["torque_enabled"]),
            last_error=str(control_state["last_error"]),
        ))
        config_pub.publish(config_msg)

    publish_config()  # latched QoS keeps the latest torque state available to late subscribers

    def on_command(msg: Any) -> None:
        with bus_lock:
            if not control_state["torque_enabled"]:
                return
            _apply_command(msg, joints, sdk, packet, port)

    def on_control(msg: Any) -> None:
        action = _control_action(msg)
        if not action:
            return
        with bus_lock:
            if _control_already_applied(action, control_state):
                pass
            elif action == "enter_teach":
                _ok, error = _disable_all_torque(sdk, packet, port, joints)
                control_state["torque_enabled"] = False
                control_state["last_error"] = error
            else:
                ok, seeded_ticks, error = _enable_all_torque_at_current_pose(sdk, packet, port, joints)
                if ok:
                    last_known_ticks.update(seeded_ticks)
                control_state["torque_enabled"] = ok
                control_state["last_error"] = error
        publish_config()

    node.create_subscription(JointState, args.command_topic, on_command, 10)
    node.create_subscription(String, args.control_topic, on_control, 10)

    period = 1.0 / max(0.1, args.rate_hz)
    last_known_ticks = dict(current_ticks)
    try:
        last_publish = 0.0
        while rclpy.ok() and not stop_event.is_set():
            rclpy.spin_once(node, timeout_sec=min(0.05, period))
            now = time.monotonic()
            if now - last_publish >= period:
                last_publish = now
                with bus_lock:
                    last_known_ticks.update(_sync_read_positions(sdk, packet, port, joints))
                publish_state(last_known_ticks)
    finally:
        if args.torque_off_on_exit:
            for name, joint in joints.items():
                _set_torque(sdk, packet, port, joint.servo_id, False)  # best-effort; ignore result on shutdown
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        port.closePort()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())


