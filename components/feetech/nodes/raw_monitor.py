"""Read-only raw-servo discovery provider for Feetech STS/SMS buses."""
from __future__ import annotations

import threading
import time
from collections.abc import Mapping
from typing import Any

from blacknode.node import Bool, Dict, Text, node

from . import bus


_discovery_lock = threading.Lock()
_discovery_cache: dict[tuple[str, int, int], dict[str, Any]] = {}
_DISCOVERY_REFRESH_SECONDS = 300.0


def _hardware_port(hardware: Mapping[str, Any]) -> str:
    recommended = (
        hardware.get("recommended")
        if isinstance(hardware.get("recommended"), Mapping)
        else {}
    )
    return str(
        recommended.get("path")
        or hardware.get("port")
        or hardware.get("path")
        or ""
    ).strip()


def _hardware_match_score(hardware: Mapping[str, Any]) -> int:
    recommended = (
        hardware.get("recommended")
        if isinstance(hardware.get("recommended"), Mapping)
        else {}
    )
    protocol = str(
        hardware.get("protocol")
        or recommended.get("protocol")
        or ""
    ).strip().lower()
    if protocol == "feetech":
        return 1_000
    text = " ".join(
        str(
            recommended.get(field)
            or hardware.get(field)
            or ""
        ).lower()
        for field in ("manufacturer", "product", "description")
    )
    if "feetech" in text:
        return 900
    # USB serial adapters rarely identify the servo protocol. A low fallback
    # allows this provider when it is the sole installed raw-bus candidate;
    # the generic resolver rejects equal-score ambiguity as providers grow.
    return 10 if _hardware_port(hardware) else 0


def _raw_monitor_sample(ctx: Mapping[str, Any]) -> dict[str, Any]:
    hardware = (
        ctx.get("hardware")
        if isinstance(ctx.get("hardware"), Mapping)
        else {}
    )
    configured = (
        ctx.get("provider_config")
        if isinstance(ctx.get("provider_config"), Mapping)
        else {}
    )
    port = _hardware_port(hardware)
    if not port:
        raise ValueError("select a connected serial device for raw monitoring")
    baudrate = int(configured.get("baudrate") or 1_000_000)
    max_servo_id = max(
        1,
        min(253, int(configured.get("max_servo_id") or 32)),
    )
    key = (port, baudrate, max_servo_id)
    now = time.monotonic()
    with _discovery_lock:
        cached = dict(_discovery_cache.get(key) or {})
    known_ids = [
        int(value)
        for value in (cached.get("servo_ids") or [])
        if 1 <= int(value) <= 253
    ]
    discovering = (
        not known_ids
        or now - float(cached.get("scanned_at") or 0.0)
        >= _DISCOVERY_REFRESH_SECONDS
    )
    config = {
        "port": port,
        "baudrate": baudrate,
        "max_servo_id": max_servo_id,
        "discovering": discovering,
    }
    if not discovering:
        config["servo_ids"] = known_ids
    result = bus.probe_raw_servos(config)
    discovered_ids = [
        int(joint["servo_id"])
        for joint in (result.get("joints") or [])
        if isinstance(joint, Mapping) and joint.get("servo_id") is not None
    ]
    if discovering or discovered_ids:
        with _discovery_lock:
            _discovery_cache[key] = {
                "servo_ids": discovered_ids,
                "scanned_at": now if discovering else cached.get("scanned_at", now),
            }
    if not discovered_ids and known_ids and not discovering:
        # A previously detected chain disappeared. Forget it so the next
        # sample performs one bounded discovery pass.
        with _discovery_lock:
            _discovery_cache.pop(key, None)

    ids = ", ".join(str(value) for value in discovered_ids) or "none"
    scan_note = (
        f"scanned servo IDs 1-{max_servo_id}"
        if discovering
        else f"sampling discovered IDs {', '.join(str(value) for value in known_ids)}"
    )
    return {
        **result,
        "report": (
            f"Raw read-only Feetech monitor on {port}: {scan_note}; "
            f"responding IDs: {ids}. Positions are uncalibrated register ticks."
        ),
    }


@node(
    name="FeetechRawMonitorProvider",
    component="feetech",
    category="Drivers",
    hidden=True,
    description=(
        "Provider metadata for bounded, read-only raw Feetech servo discovery."
    ),
    inputs={"hardware": Dict, "provider_config": Dict},
    outputs={"available": Bool, "provider": Dict, "report": Text},
)
def feetech_raw_monitor_provider(ctx: dict) -> dict:
    provider = {
        "package": "blacknode-drivers",
        "component": "feetech",
        "capability": "raw_position_feedback",
    }
    return {
        "available": True,
        "provider": provider,
        "report": "Feetech raw-monitor provider is available.",
    }


feetech_raw_monitor_provider._bn_robot_raw_monitor_provider = {
    "package": "blacknode-drivers",
    "component": "feetech",
    "capability": "raw_position_feedback",
    "match_hardware": _hardware_match_score,
    "sample": _raw_monitor_sample,
}
