"""Feetech provider for the generic robot calibration-control contract."""
from __future__ import annotations

import time
from typing import Any, Mapping

from blacknode.node import Bool, Dict, Text, node

from . import bus


def _profile_from_ctx(ctx: Mapping[str, Any]) -> dict[str, Any]:
    profile = ctx.get("profile") if isinstance(ctx.get("profile"), Mapping) else {}
    robot = ctx.get("robot") if isinstance(ctx.get("robot"), Mapping) else {}
    driver = robot.get("driver") if isinstance(robot.get("driver"), Mapping) else {}
    if not profile and isinstance(driver.get("profile"), Mapping):
        profile = driver["profile"]
    return dict(profile)


def _hardware_from_ctx(ctx: Mapping[str, Any]) -> dict[str, Any]:
    hardware = ctx.get("hardware") if isinstance(ctx.get("hardware"), Mapping) else {}
    robot = ctx.get("robot") if isinstance(ctx.get("robot"), Mapping) else {}
    if not hardware and isinstance(robot.get("usb"), Mapping):
        hardware = robot["usb"]
    return dict(hardware)


def _bus_config(ctx: Mapping[str, Any]) -> dict[str, Any]:
    profile = _profile_from_ctx(ctx)
    hardware = _hardware_from_ctx(ctx)
    recommended = (
        hardware.get("recommended")
        if isinstance(hardware.get("recommended"), Mapping)
        else {}
    )
    driver = profile.get("driver") if isinstance(profile.get("driver"), Mapping) else {}
    port = str(
        ctx.get("serial_port")
        or recommended.get("path")
        or hardware.get("port")
        or hardware.get("path")
        or ""
    ).strip()
    if not port:
        raise ValueError("select a connected robot serial port before calibration")
    return {
        "port": port,
        "baudrate": int(driver.get("baudrate") or 1_000_000),
        "joints": bus.joints_from_profile(profile),
        "profile_id": str(profile.get("id") or ""),
    }


class FeetechCalibrationSession:
    """Normalized calibration session backed by one Feetech serial bus."""

    def __init__(self, config: Mapping[str, Any]) -> None:
        self.config = dict(config)
        self.joints = dict(config["joints"])
        self.sdk = bus.load_sdk()
        self.port = bus.open_port(
            self.sdk,
            str(config["port"]),
            int(config["baudrate"]),
        )
        self.packet = self.sdk.PacketHandler(0)
        self._torque_touched = False
        self._armed = False

    def sample(self) -> dict[str, Any]:
        pose: dict[str, float] = {}
        torque_states: dict[str, bool] = {}
        errors: list[str] = []
        warnings: list[str] = []
        servos: dict[str, dict[str, Any]] = {}
        operation_count = 0
        packet_error_count = 0
        for name, joint in self.joints.items():
            ticks, position_flags = bus.read_position_status(
                self.sdk,
                self.packet,
                self.port,
                joint.servo_id,
            )
            operation_count += 1
            if ticks is None:
                packet_error_count += 1
                errors.append(
                    f"no position response from {name} (servo {joint.servo_id})"
                )
            else:
                pose[name] = bus.ticks_to_degrees(ticks, joint)
            torque_enabled, torque_flags = bus.read_torque_enabled_status(
                self.sdk,
                self.packet,
                self.port,
                joint.servo_id,
            )
            operation_count += 1
            if torque_enabled is None:
                packet_error_count += 1
                errors.append(
                    f"could not read torque state for {name} "
                    f"(servo {joint.servo_id})"
                )
            else:
                torque_states[name] = torque_enabled
            diagnostics = bus.read_servo_diagnostics(
                self.sdk,
                self.packet,
                self.port,
                joint.servo_id,
            )
            operation_count += 1
            if diagnostics is None:
                packet_error_count += 1
                diagnostics = {}
            hardware_flags = (
                int(position_flags)
                | int(torque_flags)
                | int(diagnostics.get("hardware_error_flags") or 0)
            )
            hardware_errors = bus.decode_hardware_errors(hardware_flags)
            if hardware_flags:
                warning = (
                    f"{name} (servo {joint.servo_id}) hardware warning "
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
                        "Check that the connected power supply matches this "
                        "robot and servo voltage rating."
                    )
                warnings.append(warning)
            servos[name] = {
                "servo_id": joint.servo_id,
                "communication_ok": ticks is not None,
                "ticks": ticks,
                "position_deg": pose.get(name),
                "torque_enabled": torque_enabled,
                "voltage_v": diagnostics.get("voltage_v"),
                "temperature_c": diagnostics.get("temperature_c"),
                "servo_status": diagnostics.get("servo_status"),
                "hardware_error_flags": hardware_flags,
                "hardware_errors": hardware_errors,
            }
        torque_enabled: bool | None = (
            any(torque_states.values())
            if len(torque_states) == len(self.joints)
            else None
        )
        if len(set(torque_states.values())) > 1:
            errors.append("mixed physical torque state across configured joints")
        return {
            "pose": pose,
            "torque_enabled": torque_enabled,
            "errors": errors,
            "warnings": warnings,
            "servos": servos,
            "diagnostics": {
                "operation_count": operation_count,
                "serial_packet_error_count": packet_error_count,
                "serial_packet_error_rate": (
                    packet_error_count / operation_count
                    if operation_count
                    else 0.0
                ),
                "hardware_warning_count": sum(
                    1
                    for servo in servos.values()
                    if servo["hardware_error_flags"]
                ),
            },
        }

    def release(self) -> dict[str, Any]:
        self._torque_touched = True
        self._armed = False
        ok, error = bus.disable_all_torque(
            self.sdk,
            self.packet,
            self.port,
            self.joints,
        )
        if not ok:
            raise RuntimeError(error)
        result = self.sample()
        if result["torque_enabled"] is not False:
            result.setdefault("errors", []).append(
                "torque release could not be verified on every configured joint"
            )
        return result

    def hold(self) -> dict[str, Any]:
        self._torque_touched = True
        self._armed = False
        ok, _positions, error = bus.enable_all_torque_at_current_pose(
            self.sdk,
            self.packet,
            self.port,
            self.joints,
        )
        if not ok:
            raise RuntimeError(error)
        result = self.sample()
        if result["torque_enabled"] is not True:
            bus.disable_all_torque(
                self.sdk,
                self.packet,
                self.port,
                self.joints,
            )
            result = self.sample()
            result.setdefault("errors", []).append(
                "holding torque could not be verified; torque was released"
            )
        else:
            self._armed = True
        return result

    def command(
        self,
        positions_deg: Mapping[str, float],
        *,
        deadline: float,
    ) -> dict[str, Any]:
        """Write bounded joint goals only for an armed, healthy, fresh session."""
        if not self._armed:
            raise PermissionError("joint motion is disarmed")
        if time.monotonic() > float(deadline):
            raise TimeoutError("joint command is stale")
        before = self.sample()
        if (
            before.get("torque_enabled") is not True
            or before.get("errors")
            or before.get("warnings")
        ):
            self.release()
            raise RuntimeError("live feedback or hardware health blocks motion")
        ok, error = bus.write_joint_positions(
            self.sdk,
            self.packet,
            self.port,
            self.joints,
            positions_deg,
        )
        if not ok:
            self.release()
            raise RuntimeError(error)
        after = self.sample()
        if (
            after.get("torque_enabled") is not True
            or after.get("errors")
            or after.get("warnings")
        ):
            self.release()
            raise RuntimeError("joint command could not be verified safely")
        return after

    def close(self) -> None:
        try:
            if self._torque_touched:
                bus.disable_all_torque(
                    self.sdk,
                    self.packet,
                    self.port,
                    self.joints,
                )
        finally:
            self._armed = False
            try:
                self.port.closePort()
            except Exception:
                pass


def open_calibration_session(ctx: Mapping[str, Any]) -> FeetechCalibrationSession:
    return FeetechCalibrationSession(_bus_config(ctx))


@node(
    name="FeetechCalibrationProvider",
    component="feetech",
    category="Drivers",
    hidden=True,
    description=(
        "Provider metadata for Feetech implementations of the generic robot "
        "calibration-control contract."
    ),
    inputs={"profile": Dict, "hardware": Dict},
    outputs={"available": Bool, "provider": Dict, "report": Text},
)
def feetech_calibration_provider(ctx: dict) -> dict:
    provider = {
        "package": "blacknode-drivers",
        "component": "feetech",
        "capability": "calibration_control",
    }
    return {
        "available": True,
        "provider": provider,
        "report": "Feetech calibration-control provider is available.",
    }


feetech_calibration_provider._bn_robot_calibration_provider = {
    "package": "blacknode-drivers",
    "component": "feetech",
    "capability": "calibration_control",
    "open_session": open_calibration_session,
}

feetech_calibration_provider._bn_robot_joint_motion_provider = {
    "package": "blacknode-drivers",
    "component": "feetech",
    "capability": "joint_group",
    "open_session": open_calibration_session,
}
