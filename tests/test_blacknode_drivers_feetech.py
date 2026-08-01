import math
import runpy
from pathlib import Path
import time
from types import SimpleNamespace

import pytest

import blacknode  # noqa: F401 - triggers extension package discovery
from blacknode.node import _NODE_REGISTRY
from blacknode.packages import (
    _PACKAGE_REGISTRY,
    adapter_dependency_plan,
    set_adapter_enabled,
    set_component_enabled,
)
from blacknode.pkg.blacknode_drivers.feetech import bus
from blacknode.pkg.blacknode_drivers.feetech import calibration


def test_feetech_component_registers_expected_nodes():
    info = _PACKAGE_REGISTRY["blacknode-drivers"]

    assert info.ok
    assert info.layer == "drivers"
    assert info.component_mode is True
    assert info.enabled_components == ["feetech"]
    assert set(info.components) == {"feetech"}
    assert not {
        "serial",
        "can",
        "usb",
        "motor-controllers",
        "sensor-drivers",
        "vendor-adapters",
    }.intersection(info.components)
    assert info.components["feetech"]["capabilities"] == [
        "driver.feetech",
        "robot.calibration-control",
        "robot.joint-driver",
        "robot.joint-motion-provider",
        "robot.raw-position-feedback",
    ]
    assert "feetech-servo-sdk>=1.0" in info.pip_dependencies
    assert {
        "FeetechBusConfig",
        "FeetechBusProbe",
        "FeetechCalibrationProvider",
        "FeetechRawMonitorProvider",
    }.issubset(info.node_types)
    assert _NODE_REGISTRY["FeetechBusConfig"]._bn_component == "feetech"
    provider = _NODE_REGISTRY["FeetechCalibrationProvider"]
    assert provider._bn_component == "feetech"
    assert provider._bn_hidden is True
    assert provider._bn_robot_calibration_provider["package"] == "blacknode-drivers"
    assert provider._bn_robot_calibration_provider["component"] == "feetech"
    assert provider._bn_robot_joint_motion_provider["package"] == "blacknode-drivers"
    assert provider._bn_robot_joint_motion_provider["component"] == "feetech"
    raw_provider = _NODE_REGISTRY["FeetechRawMonitorProvider"]
    assert raw_provider._bn_hidden is True
    assert (
        raw_provider._bn_robot_raw_monitor_provider["capability"]
        == "raw_position_feedback"
    )


def test_feetech_ros2_adapter_resolves_layer_dependencies_and_stays_disarmed():
    plan = adapter_dependency_plan("blacknode-drivers", "feetech", "ros2")
    assert [(item["package"], item["component"], item.get("adapter", "")) for item in plan["plan"]] == [
        ("blacknode-drivers", "feetech", ""),
        ("blacknode-robot", "contracts", ""),
        ("blacknode-ros2", "core", ""),
        ("blacknode-ros2", "rosbridge", ""),
        ("blacknode-drivers", "feetech", "ros2"),
    ]

    adapter_was_enabled = (
        "feetech/ros2"
        in _PACKAGE_REGISTRY["blacknode-drivers"].enabled_adapters
    )
    rosbridge_was_enabled = (
        "rosbridge" in _PACKAGE_REGISTRY["blacknode-ros2"].enabled_components
    )
    try:
        set_component_enabled("blacknode-ros2", "rosbridge", True)
        info = set_adapter_enabled("blacknode-drivers", "feetech", "ros2", True)
        assert "FeetechROS2Adapter" in info.node_types
        assert info.enabled_components == ["feetech"]
        assert info.enabled_adapters == ["feetech/ros2"]
        result = _NODE_REGISTRY["FeetechROS2Adapter"]({"config": {"port": "COM7"}})
        assert result["adapter"]["available"] is True
        assert result["adapter"]["motion_armed"] is False
        assert result["adapter"]["config"] == {"port": "COM7"}
        with pytest.raises(ValueError, match="blacknode-drivers/feetech adapter ros2"):
            set_component_enabled("blacknode-ros2", "rosbridge", False)
    finally:
        set_adapter_enabled(
            "blacknode-drivers",
            "feetech",
            "ros2",
            adapter_was_enabled,
        )
        set_component_enabled(
            "blacknode-ros2",
            "rosbridge",
            rosbridge_was_enabled,
        )


def test_feetech_config_is_inert_and_accepts_robot_profile():
    profile = {
        "id": "test_arm",
        "joints": [
            {
                "id": "shoulder",
                "servo_id": 1,
                "min_deg": -90,
                "max_deg": 90,
                "safe_min_deg": -75,
                "safe_max_deg": 70,
                "home_ticks": 2100,
                "invert": True,
            }
        ],
    }

    result = _NODE_REGISTRY["FeetechBusConfig"]({
        "profile": profile,
        "port": "COM7",
        "baudrate": 1_000_000,
    })

    assert result["ready"] is True
    assert result["config"]["driver"] == "feetech"
    assert result["config"]["motion_armed"] is False
    assert result["config"]["joints"] == [{
        "name": "shoulder",
        "servo_id": 1,
        "min_deg": -75.0,
        "max_deg": 70.0,
        "home_ticks": 2100,
        "invert": True,
    }]
    assert "configuration opens no hardware" in result["report"]


def test_feetech_probe_requires_explicit_confirmation(monkeypatch):
    monkeypatch.setattr(
        bus,
        "probe_bus",
        lambda _config: (_ for _ in ()).throw(AssertionError("hardware must stay closed")),
    )

    result = _NODE_REGISTRY["FeetechBusProbe"]({
        "config": {"port": "COM7"},
        "confirm_read_only": False,
    })

    assert result["connected"] is False
    assert "BLOCKED" in result["report"]


def test_feetech_calibration_provider_opens_normalized_session(monkeypatch):
    captured = []

    class FakeSession:
        def __init__(self, config):
            captured.append(config)

    provider = _NODE_REGISTRY[
        "FeetechCalibrationProvider"
    ]._bn_robot_calibration_provider
    monkeypatch.setitem(
        provider["open_session"].__globals__,
        "FeetechCalibrationSession",
        FakeSession,
    )
    session = provider["open_session"]({
        "profile": {
            "id": "test_arm",
            "driver": {"baudrate": 1_000_000},
            "joints": [{
                "id": "shoulder",
                "servo_id": 1,
                "safe_min_deg": -90,
                "safe_max_deg": 90,
            }],
        },
        "hardware": {"recommended": {"path": "COM7"}},
    })

    assert isinstance(session, FakeSession)
    assert captured[0]["port"] == "COM7"
    assert captured[0]["profile_id"] == "test_arm"
    assert list(captured[0]["joints"]) == ["shoulder"]


def test_feetech_calibration_provider_rejects_missing_hardware():
    provider = _NODE_REGISTRY[
        "FeetechCalibrationProvider"
    ]._bn_robot_calibration_provider

    with pytest.raises(ValueError, match="select a connected robot serial port"):
        provider["open_session"]({
            "profile": {
                "id": "test_arm",
                "joints": [{
                    "id": "shoulder",
                    "servo_id": 1,
                    "safe_min_deg": -90,
                    "safe_max_deg": 90,
                }],
            },
            "hardware": {},
        })


def test_monitor_only_calibration_close_does_not_change_torque(monkeypatch):
    torque_writes = []
    closed = []
    hardware_bus = calibration.FeetechCalibrationSession.__new__(
        calibration.FeetechCalibrationSession
    )
    hardware_bus.sdk = object()
    hardware_bus.packet = object()
    hardware_bus.port = SimpleNamespace(closePort=lambda: closed.append(True))
    hardware_bus.joints = {"shoulder": object()}
    hardware_bus._torque_touched = False
    monkeypatch.setattr(
        bus,
        "disable_all_torque",
        lambda *_args: torque_writes.append(True) or (True, ""),
    )

    hardware_bus.close()

    assert closed == [True]
    assert torque_writes == []


def test_feetech_motion_command_requires_armed_healthy_session(monkeypatch):
    writes = []
    hardware_bus = calibration.FeetechCalibrationSession.__new__(
        calibration.FeetechCalibrationSession
    )
    hardware_bus.sdk = object()
    hardware_bus.packet = object()
    hardware_bus.port = object()
    hardware_bus.joints = {"shoulder": object()}
    hardware_bus._armed = True
    hardware_bus._torque_touched = True
    healthy = {
        "pose": {"shoulder": 0.0},
        "torque_enabled": True,
        "errors": [],
        "warnings": [],
    }
    monkeypatch.setattr(hardware_bus, "sample", lambda: dict(healthy))
    monkeypatch.setattr(
        bus,
        "write_joint_positions",
        lambda _sdk, _packet, _port, _joints, positions: (
            writes.append(dict(positions)) or True,
            "",
        ),
    )

    result = hardware_bus.command(
        {"shoulder": 12.5},
        deadline=time.monotonic() + 1.0,
    )

    assert result["torque_enabled"] is True
    assert writes == [{"shoulder": 12.5}]
    hardware_bus._armed = False
    with pytest.raises(PermissionError, match="disarmed"):
        hardware_bus.command(
            {"shoulder": 0.0},
            deadline=time.monotonic() + 1.0,
        )


def test_joint_parsing_conversion_and_validation():
    joints = bus.parse_joint_map(
        "shoulder:1:-90:90,gripper:6:-10:80",
        {"shoulder": 2100},
        {"shoulder"},
    )
    shoulder = joints["shoulder"]

    assert shoulder.home_ticks == 2100
    assert shoulder.invert is True
    assert bus.ticks_to_degrees(shoulder.home_ticks, shoulder) == 0.0
    assert bus.degrees_to_ticks(10.0, shoulder) < shoulder.home_ticks
    assert bus.clamp_degrees(999.0, shoulder) == 90.0
    assert math.isclose(
        bus.ticks_to_degrees(bus.degrees_to_ticks(45.0, joints["gripper"]), joints["gripper"]),
        45.0,
    )

    with pytest.raises(ValueError, match="duplicated"):
        bus.parse_joint_map("a:1:-10:10,b:1:-10:10")
    with pytest.raises(ValueError, match="minimum"):
        bus.parse_joint_map("a:1:10:-10")


def test_read_only_probe_never_calls_write_methods():
    calls = []

    class Port:
        def openPort(self):
            calls.append("open")
            return True

        def setBaudRate(self, baudrate):
            calls.append(("baud", baudrate))
            return True

        def closePort(self):
            calls.append("close")

    class Packet:
        def read2ByteTxRx(self, _port, servo_id, address):
            calls.append(("read", servo_id, address))
            return 2048 + servo_id, 0, 0

        def read4ByteTxRx(self, _port, servo_id, address):
            calls.append(("diagnostics", servo_id, address))
            return 74 | (30 << 8), 0, 0

    fake_sdk = SimpleNamespace(
        COMM_SUCCESS=0,
        PortHandler=lambda _name: Port(),
        PacketHandler=lambda _protocol: Packet(),
    )
    config = {
        "port": "COM7",
        "baudrate": 1_000_000,
        "joints": [
            bus.JointSpec("shoulder", 1, -90, 90).to_dict(),
            bus.JointSpec("gripper", 6, -10, 80).to_dict(),
        ],
    }

    result = bus.probe_bus(config, sdk=fake_sdk)

    assert result["ok"] is True
    assert set(result["readings"]) == {"shoulder", "gripper"}
    assert result["readings"]["shoulder"]["position_rad"] == pytest.approx(
        math.radians(bus.ticks_to_degrees(2049, bus.JointSpec("shoulder", 1, -90, 90)))
    )
    assert result["readings"]["shoulder"]["voltage_v"] == 7.4
    assert result["readings"]["shoulder"]["temperature_c"] == 30.0
    assert result["diagnostics"]["serial_packet_error_count"] == 0
    assert calls == [
        "open",
        ("baud", 1_000_000),
        ("read", 1, bus.ADDR_PRESENT_POSITION[0]),
        ("diagnostics", 1, bus.ADDR_PRESENT_VOLTAGE[0]),
        ("read", 6, bus.ADDR_PRESENT_POSITION[0]),
        ("diagnostics", 6, bus.ADDR_PRESENT_VOLTAGE[0]),
        "close",
    ]


def test_raw_probe_discovers_responding_ids_without_writes():
    calls = []

    class Port:
        def openPort(self):
            calls.append("open")
            return True

        def setBaudRate(self, baudrate):
            calls.append(("baud", baudrate))
            return True

        def closePort(self):
            calls.append("close")

    class Packet:
        def read2ByteTxRx(self, _port, servo_id, address):
            calls.append(("position", servo_id, address))
            if servo_id not in {1, 6}:
                return 0, -1, 0
            return 2000 + servo_id, 0, 0

        def read1ByteTxRx(self, _port, servo_id, address):
            calls.append(("torque", servo_id, address))
            return 0, 0, 0

        def read4ByteTxRx(self, _port, servo_id, address):
            calls.append(("diagnostics", servo_id, address))
            return 120 | (31 << 8), 0, 0

        def __getattr__(self, name):
            if name.startswith("write"):
                raise AssertionError(f"raw probe attempted {name}")
            raise AttributeError(name)

    fake_sdk = SimpleNamespace(
        COMM_SUCCESS=0,
        COMM_RX_TIMEOUT=-1,
        PortHandler=lambda _name: Port(),
        PacketHandler=lambda _protocol: Packet(),
    )

    result = bus.probe_raw_servos({
        "port": "COM7",
        "baudrate": 1_000_000,
        "max_servo_id": 6,
        "discovering": True,
    }, sdk=fake_sdk)

    assert result["ok"] is True
    assert [joint["servo_id"] for joint in result["joints"]] == [1, 6]
    assert result["joints"][1]["raw_position"] == 2006
    assert result["joints"][0]["voltage_v"] == 12.0
    assert result["joints"][0]["temperature_c"] == 31.0
    assert result["torque_enabled"] is False
    assert result["diagnostics"]["scan_miss_count"] == 4
    assert result["diagnostics"]["serial_packet_error_count"] == 0
    assert not any(
        isinstance(call, tuple) and str(call[0]).startswith("write")
        for call in calls
    )


def test_torque_enable_reads_and_seeds_every_joint_before_holding():
    joints = bus.parse_joint_map("shoulder:1:-90:90,gripper:6:-10:80")
    calls = []

    class Packet:
        def read2ByteTxRx(self, _port, servo_id, _address):
            calls.append(("read", servo_id))
            return 2000 + servo_id, 0, 0

        def write2ByteTxRx(self, _port, servo_id, _address, ticks):
            calls.append(("goal", servo_id, ticks))
            return 0, 0

        def write1ByteTxRx(self, _port, servo_id, _address, enabled):
            calls.append(("torque", servo_id, enabled))
            return 0, 0

    ok, positions, error = bus.enable_all_torque_at_current_pose(
        SimpleNamespace(COMM_SUCCESS=0), Packet(), object(), joints
    )

    assert ok is True
    assert error == ""
    assert positions == {"shoulder": 2001, "gripper": 2006}
    assert calls == [
        ("read", 1),
        ("read", 6),
        ("goal", 1, 2001),
        ("goal", 6, 2006),
        ("torque", 1, 1),
        ("torque", 6, 1),
    ]


def test_partial_seed_failure_returns_all_joints_to_torque_off():
    joints = bus.parse_joint_map("shoulder:1:-90:90,gripper:6:-10:80")
    torque_writes = []

    class Packet:
        def read2ByteTxRx(self, _port, _servo_id, _address):
            return 2048, 0, 0

        def write2ByteTxRx(self, _port, servo_id, _address, _ticks):
            return (1, 0) if servo_id == 6 else (0, 0)

        def write1ByteTxRx(self, _port, servo_id, _address, enabled):
            torque_writes.append((servo_id, enabled))
            return 0, 0

    ok, _positions, error = bus.enable_all_torque_at_current_pose(
        SimpleNamespace(COMM_SUCCESS=0), Packet(), object(), joints
    )

    assert ok is False
    assert "could not seed Goal_Position for gripper" in error
    assert torque_writes == [(1, 0), (6, 0)]


def test_driver_startup_disables_torque_before_reading_pose():
    runtime_path = (
        Path(__file__).resolve().parents[1]
        / "components"
        / "feetech"
        / "adapters"
        / "ros2"
        / "runtime"
        / "feetech_bus_driver.py"
    )
    runtime = runpy.run_path(str(runtime_path))
    joints = runtime["parse_joint_map"](
        "shoulder:1:-90:90,gripper:6:-10:80",
        {},
        set(),
    )
    calls = []

    class Packet:
        def write1ByteTxRx(self, _port, servo_id, _address, enabled):
            calls.append(("torque", servo_id, enabled))
            return 0, 0

        def read2ByteTxRx(self, _port, servo_id, _address):
            calls.append(("read", servo_id))
            return 2000 + servo_id, 0, 0

    ok, positions, error = runtime["_prepare_released_startup"](
        SimpleNamespace(COMM_SUCCESS=0),
        Packet(),
        object(),
        joints,
    )

    assert ok is True
    assert positions == {"shoulder": 2001, "gripper": 2006}
    assert error == ""
    assert calls == [
        ("torque", 1, 0),
        ("torque", 6, 0),
        ("read", 1),
        ("read", 6),
    ]


def test_driver_goal_seed_accepts_readback_after_lost_write_ack():
    runtime_path = (
        Path(__file__).resolve().parents[1]
        / "components"
        / "feetech"
        / "adapters"
        / "ros2"
        / "runtime"
        / "feetech_bus_driver.py"
    )
    runtime = runpy.run_path(str(runtime_path))
    calls = []

    class Packet:
        def write2ByteTxRx(self, _port, servo_id, address, ticks):
            calls.append(("write", servo_id, address, ticks))
            return 1, 0

        def read2ByteTxRx(self, _port, servo_id, address):
            calls.append(("readback", servo_id, address))
            return 2048, 0, 0

    seeded = runtime["_write_goal"](
        SimpleNamespace(COMM_SUCCESS=0),
        Packet(),
        object(),
        5,
        2048,
        confirm=True,
    )

    assert seeded is True
    assert calls == [
        ("write", 5, runtime["ADDR_GOAL_POSITION"][0], 2048),
        ("readback", 5, runtime["ADDR_GOAL_POSITION"][0]),
    ]


def test_driver_torque_accepts_readback_after_lost_write_ack():
    runtime_path = (
        Path(__file__).resolve().parents[1]
        / "components"
        / "feetech"
        / "adapters"
        / "ros2"
        / "runtime"
        / "feetech_bus_driver.py"
    )
    runtime = runpy.run_path(str(runtime_path))
    calls = []

    class Packet:
        def write1ByteTxRx(self, _port, servo_id, address, enabled):
            calls.append(("write", servo_id, address, enabled))
            return 1, 0

        def read1ByteTxRx(self, _port, servo_id, address):
            calls.append(("readback", servo_id, address))
            return 1, 0, 0

    enabled = runtime["_set_torque"](
        SimpleNamespace(COMM_SUCCESS=0),
        Packet(),
        object(),
        3,
        True,
    )

    assert enabled is True
    assert calls == [
        ("write", 3, runtime["ADDR_TORQUE_ENABLE"][0], 1),
        ("readback", 3, runtime["ADDR_TORQUE_ENABLE"][0]),
    ]


def test_torque_change_does_not_mask_a_servo_reported_error():
    calls = []

    class Packet:
        def write1ByteTxRx(self, _port, servo_id, _address, enabled):
            calls.append(("write", servo_id, enabled))
            return 0, 4

        def read1ByteTxRx(self, *_args):
            calls.append(("readback",))
            return 1, 0, 0

    enabled = bus._set_torque(
        SimpleNamespace(COMM_SUCCESS=0),
        Packet(),
        object(),
        3,
        True,
    )

    assert enabled is False
    assert calls == [("write", 3, 1)]


def test_torque_release_accepts_confirmed_off_with_hardware_warning():
    calls = []

    class Packet:
        def write1ByteTxRx(self, _port, servo_id, _address, enabled):
            calls.append(("write", servo_id, enabled))
            return 0, 1

        def read1ByteTxRx(self, _port, servo_id, _address):
            calls.append(("readback", servo_id))
            return 0, 0, 1

    released = bus._set_torque(
        SimpleNamespace(COMM_SUCCESS=0),
        Packet(),
        object(),
        2,
        False,
    )

    assert released is True
    assert calls == [("write", 2, 0), ("readback", 2)]


def test_calibration_sample_preserves_warning_bearing_servo_feedback():
    joint = bus.JointSpec("shoulder_lift", 2, -100.0, 100.0)

    class Packet:
        def read2ByteTxRx(self, _port, servo_id, address):
            assert (servo_id, address) == (2, bus.ADDR_PRESENT_POSITION[0])
            return 833, 0, 1

        def read1ByteTxRx(self, _port, servo_id, address):
            assert (servo_id, address) == (2, bus.ADDR_TORQUE_ENABLE[0])
            return 0, 0, 1

        def read4ByteTxRx(self, _port, servo_id, address):
            assert (servo_id, address) == (2, bus.ADDR_PRESENT_VOLTAGE[0])
            return 119 | (32 << 8) | (1 << 24), 0, 1

    session = calibration.FeetechCalibrationSession.__new__(
        calibration.FeetechCalibrationSession
    )
    session.sdk = SimpleNamespace(COMM_SUCCESS=0)
    session.packet = Packet()
    session.port = object()
    session.joints = {"shoulder_lift": joint}

    sample = session.sample()

    assert sample["pose"]["shoulder_lift"] == pytest.approx(
        bus.ticks_to_degrees(833, joint)
    )
    assert sample["torque_enabled"] is False
    assert sample["errors"] == []
    assert sample["warnings"] == [
        "shoulder_lift (servo 2) hardware warning 0x01: voltage; "
        "measured input 11.9 V. Check that the connected power supply "
        "matches this robot and servo voltage rating."
    ]
    assert sample["servos"]["shoulder_lift"] == {
        "servo_id": 2,
        "communication_ok": True,
        "ticks": 833,
        "position_deg": pytest.approx(bus.ticks_to_degrees(833, joint)),
        "torque_enabled": False,
        "voltage_v": 11.9,
        "temperature_c": 32.0,
        "servo_status": 1,
        "hardware_error_flags": 1,
        "hardware_errors": ["voltage"],
    }


def test_driver_goal_seed_retries_then_fails_without_confirmation():
    runtime_path = (
        Path(__file__).resolve().parents[1]
        / "components"
        / "feetech"
        / "adapters"
        / "ros2"
        / "runtime"
        / "feetech_bus_driver.py"
    )
    runtime = runpy.run_path(str(runtime_path))
    writes = []

    class Packet:
        def write2ByteTxRx(self, _port, servo_id, _address, ticks):
            writes.append((servo_id, ticks))
            return 1, 0

        def read2ByteTxRx(self, _port, _servo_id, _address):
            return 1024, 0, 0

    seeded = runtime["_write_goal"](
        SimpleNamespace(COMM_SUCCESS=0),
        Packet(),
        object(),
        5,
        2048,
        confirm=True,
    )

    assert seeded is False
    assert writes == [(5, 2048), (5, 2048), (5, 2048)]


def test_driver_ros_node_name_is_unique_per_robot_topic_namespace():
    runtime_path = (
        Path(__file__).resolve().parents[1]
        / "components"
        / "feetech"
        / "adapters"
        / "ros2"
        / "runtime"
        / "feetech_bus_driver.py"
    )
    runtime = runpy.run_path(str(runtime_path))
    node_name = runtime["ros_node_name"]

    assert node_name("/leader/joint_states") == "blacknode_feetech_bus_driver_leader"
    assert node_name("/follower/joint_states") == "blacknode_feetech_bus_driver_follower"
    assert node_name("/joint_states") == "blacknode_feetech_bus_driver"
    assert node_name("/ignored", "arm 1/driver") == "arm_1_driver"
    assert node_name("/ignored", "123-driver") == "blacknode_123_driver"


def test_read_only_driver_advertises_no_command_authority():
    runtime_path = (
        Path(__file__).resolve().parents[1]
        / "components"
        / "feetech"
        / "adapters"
        / "ros2"
        / "runtime"
        / "feetech_bus_driver.py"
    )
    runtime = runpy.run_path(str(runtime_path))
    joint = runtime["JointSpec"]("shoulder", 1, -90.0, 90.0)

    config = runtime["_config_payload"](
        {"shoulder": joint},
        torque_enabled=True,
        commands_allowed=False,
    )

    assert config["torque_enabled"] is True
    assert config["commands_allowed"] is False


def test_position_writes_are_clamped_at_driver_boundary():
    joints = bus.parse_joint_map("shoulder:1:-30:40")
    writes = []
    packet = SimpleNamespace(
        write2ByteTxRx=lambda _port, servo_id, _address, ticks: writes.append(
            (servo_id, ticks)
        ) or (0, 0)
    )

    ok, error = bus.write_joint_positions(
        SimpleNamespace(COMM_SUCCESS=0),
        packet,
        object(),
        joints,
        {"shoulder": 999.0},
    )

    assert ok is True
    assert error == ""
    assert writes == [(1, bus.degrees_to_ticks(40.0, joints["shoulder"]))]


def test_deployed_driver_reports_read_only_joint_and_torque_telemetry():
    runtime_path = (
        Path(__file__).resolve().parents[1]
        / "components"
        / "feetech"
        / "adapters"
        / "ros2"
        / "runtime"
        / "feetech_bus_driver.py"
    )
    runtime = runpy.run_path(str(runtime_path))
    joint = runtime["JointSpec"]("shoulder", 1, -90.0, 90.0)
    published = []

    class Publisher:
        def publish_device_state(self, state):
            published.append(state)

    runtime["_publish_deployment_state"](
        Publisher(),
        {"shoulder": 2048},
        {"shoulder": joint},
        {"torque_enabled": False, "last_error": ""},
    )

    assert len(published) == 1
    state = published[0]
    assert state["kind"] == "blacknode.device-state"
    assert state["connected"] is True
    assert state["armed"] is False
    assert state["torque_enabled"] is False
    assert state["joint_state"]["kind"] == "blacknode.joint-state"
    assert state["joint_state"]["positions"] == {"shoulder": 0.0}
    assert state["joint_state"]["limits"]["shoulder"] == {
        "lower": -math.pi / 2,
        "upper": math.pi / 2,
    }
    assert state["values"]["servo_ids"] == {"shoulder": 1}
    assert state["values"]["raw_positions"] == {"shoulder": 2048}


def test_instrumented_packet_counts_timeouts_and_servo_hardware_flags():
    runtime_path = (
        Path(__file__).resolve().parents[1]
        / "components"
        / "feetech"
        / "adapters"
        / "ros2"
        / "runtime"
        / "feetech_bus_driver.py"
    )
    runtime = runpy.run_path(str(runtime_path))
    sdk = SimpleNamespace(COMM_SUCCESS=0, COMM_RX_TIMEOUT=-6)

    class Packet:
        def read2ByteTxRx(self, _port, _servo_id, _address):
            return 0, -6, 0

        def write1ByteTxRx(self, _port, _servo_id, _address, _value):
            return 0, 4

    telemetry = runtime["BusTelemetry"]()
    packet = runtime["InstrumentedPacket"](Packet(), sdk, telemetry)

    packet.read2ByteTxRx(object(), 1, 56)
    packet.write1ByteTxRx(object(), 2, 40, 1)

    assert telemetry.operation_count == 2
    assert telemetry.timeout_count == 1
    assert telemetry.serial_packet_error_count == 1
    assert telemetry.hardware_error_count == 1
    assert telemetry.protocol_error_flags == {2: 4}


def test_monitoring_position_read_keeps_valid_ticks_with_hardware_warning():
    runtime_path = (
        Path(__file__).resolve().parents[1]
        / "components"
        / "feetech"
        / "adapters"
        / "ros2"
        / "runtime"
        / "feetech_bus_driver.py"
    )
    runtime = runpy.run_path(str(runtime_path))
    sdk = SimpleNamespace(COMM_SUCCESS=0)
    packet = SimpleNamespace(
        read2ByteTxRx=lambda _port, _servo_id, _address: (833, 0, 1)
    )

    assert runtime["_read_position_or_none"](
        sdk,
        packet,
        object(),
        2,
    ) is None
    assert runtime["_read_position_or_none"](
        sdk,
        packet,
        object(),
        2,
        accept_hardware_warning=True,
    ) == 833


def test_servo_diagnostics_are_normalized_into_canonical_device_state():
    runtime_path = (
        Path(__file__).resolve().parents[1]
        / "components"
        / "feetech"
        / "adapters"
        / "ros2"
        / "runtime"
        / "feetech_bus_driver.py"
    )
    runtime = runpy.run_path(str(runtime_path))
    sdk = SimpleNamespace(COMM_SUCCESS=0)
    joint = runtime["JointSpec"]("shoulder", 1, -90.0, 90.0)
    joints = {"shoulder": joint}
    packed = 74 | (35 << 8) | (4 << 24)

    class Packet:
        def read4ByteTxRx(self, _port, servo_id, address):
            assert servo_id == 1
            assert address == runtime["ADDR_PRESENT_VOLTAGE"][0]
            return packed, 0, 0

    telemetry = runtime["BusTelemetry"]()
    packet = runtime["InstrumentedPacket"](Packet(), sdk, telemetry)
    runtime["_read_bus_diagnostics"](
        sdk,
        packet,
        object(),
        joints,
        telemetry,
    )
    telemetry.last_full_feedback_time = time.time()
    published = []

    class Publisher:
        def publish_device_state(self, state):
            published.append(state)

    runtime["_publish_deployment_state"](
        Publisher(),
        {"shoulder": 2048},
        joints,
        {
            "torque_enabled": False,
            "last_error": "",
            "stale_after": 0.25,
        },
        telemetry,
    )

    state = published[0]
    assert state["kind"] == "blacknode.device-state"
    assert state["connected"] is True
    assert state["temperatures_c"] == {"shoulder": 35.0}
    assert state["voltage_v"] == 7.4
    assert state["values"]["bus"]["hardware_error_flags"] == {"shoulder": 4}
    assert state["values"]["bus"]["hardware_errors"] == {
        "shoulder": ["overheat"]
    }
    assert state["faults"][0]["code"] == "feetech-hardware-shoulder"


def test_canonical_state_uses_last_complete_feedback_for_freshness():
    runtime_path = (
        Path(__file__).resolve().parents[1]
        / "components"
        / "feetech"
        / "adapters"
        / "ros2"
        / "runtime"
        / "feetech_bus_driver.py"
    )
    runtime = runpy.run_path(str(runtime_path))
    joint = runtime["JointSpec"]("shoulder", 1, -90.0, 90.0)
    telemetry = runtime["BusTelemetry"](
        last_full_feedback_time=time.time() - 2.0
    )
    published = []

    class Publisher:
        def publish_device_state(self, state):
            published.append(state)

    runtime["_publish_deployment_state"](
        Publisher(),
        {"shoulder": 2048},
        {"shoulder": joint},
        {
            "torque_enabled": False,
            "last_error": "",
            "stale_after": 0.25,
        },
        telemetry,
    )

    state = published[0]
    assert state["connected"] is False
    assert state["joint_state"]["source_time"] == pytest.approx(
        telemetry.last_full_feedback_time
    )
    assert any(fault["code"] == "feedback-timeout" for fault in state["faults"])
