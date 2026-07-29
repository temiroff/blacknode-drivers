"""Blacknode nodes for the Feetech physical-driver component."""
from __future__ import annotations

from typing import Any, Mapping

from blacknode.node import Bool, Dict, Int, Text, node

from . import bus

_CATEGORY = "Drivers"
_DEFAULT_JOINTS = (
    "shoulder_pan:1:-100:100,shoulder_lift:2:-100:100,"
    "elbow_flex:3:-100:100,wrist_flex:4:-100:100,"
    "wrist_roll:5:-150:150,gripper:6:-10:90"
)


def _inverted_names(value: str) -> set[str]:
    return {name.strip() for name in str(value or "").split(",") if name.strip()}


def _build_config(ctx: Mapping[str, Any]) -> tuple[dict[str, Any], str]:
    profile = ctx.get("profile") if isinstance(ctx.get("profile"), Mapping) else {}
    if profile and profile.get("joints"):
        joints = bus.joints_from_profile(profile)
        source = f"profile {profile.get('id') or profile.get('name') or 'custom'}"
    else:
        joints = bus.parse_joint_map(
            str(ctx.get("joints") or _DEFAULT_JOINTS),
            bus.parse_int_map(str(ctx.get("home_ticks") or "")),
            _inverted_names(str(ctx.get("invert") or "")),
        )
        source = "joint map"
    port = str(ctx.get("port") or "").strip()
    config = {
        "schema_version": 1,
        "driver": "feetech",
        "port": port,
        "baudrate": int(ctx.get("baudrate") or 1_000_000),
        "joints": [joint.to_dict() for joint in joints.values()],
        "motion_armed": False,
    }
    report = (
        f"Feetech bus configured from {source}: {len(joints)} joint(s), "
        f"{port or 'serial port not selected'} at {config['baudrate']} baud\n"
        "motion: DISARMED; configuration opens no hardware"
    )
    return config, report


@node(
    name="FeetechBusConfig",
    category=_CATEGORY,
    description="Build and validate an inert Feetech serial-bus configuration from a robot profile or joint map.",
    inputs={
        "profile": Dict(default={}),
        "port": Text(default=""),
        "baudrate": Int(default=1_000_000),
        "joints": Text(default=_DEFAULT_JOINTS),
        "home_ticks": Text(default=""),
        "invert": Text(default=""),
    },
    outputs={"ready": Bool, "config": Dict, "report": Text},
    primary_inputs=["profile", "port"],
    primary_outputs=["config", "report"],
)
def feetech_bus_config(ctx: dict) -> dict:
    try:
        config, report = _build_config(ctx)
    except (TypeError, ValueError) as exc:
        return {"ready": False, "config": {}, "report": f"Feetech configuration invalid: {exc}"}
    return {
        "ready": bool(config["port"] and config["joints"]),
        "config": config,
        "report": report,
    }

@node(
    name="FeetechBusProbe",
    category=_CATEGORY,
    description="Perform an explicitly confirmed, read-only Present_Position probe of a configured Feetech bus.",
    inputs={
        "config": Dict,
        "confirm_read_only": Bool(default=False),
    },
    outputs={
        "connected": Bool,
        "readings": Dict,
        "diagnostics": Dict,
        "report": Text,
    },
)
def feetech_bus_probe(ctx: dict) -> dict:
    if not bool(ctx.get("confirm_read_only")):
        return {
            "connected": False,
            "readings": {},
            "diagnostics": {},
            "report": "Feetech probe BLOCKED: enable confirm_read_only after checking port, power, and wiring",
        }
    config = ctx.get("config") if isinstance(ctx.get("config"), Mapping) else {}
    try:
        result = bus.probe_bus(config)
    except Exception as exc:  # noqa: BLE001 - node returns structured hardware diagnostics
        return {
            "connected": False,
            "readings": {},
            "diagnostics": {},
            "report": f"Feetech read-only probe failed: {type(exc).__name__}: {exc}",
        }
    errors = result.get("errors") or []
    report = f"Feetech read-only probe received {len(result['readings'])} joint position(s)"
    diagnostics = result.get("diagnostics") or {}
    report += (
        f"; timeouts={diagnostics.get('timeout_count', 0)}, "
        f"packet_errors={diagnostics.get('serial_packet_error_count', 0)}, "
        f"hardware_errors={diagnostics.get('hardware_error_count', 0)}"
    )
    if errors:
        report += "\n" + "\n".join(f"- {error}" for error in errors)
    return {
        "connected": bool(result.get("ok")),
        "readings": result.get("readings") or {},
        "diagnostics": diagnostics,
        "report": report,
    }
