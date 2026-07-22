"""Descriptor for the Feetech process adapter exposed to ROS 2 transports."""
from __future__ import annotations

import sys
from pathlib import Path

from blacknode.node import Dict, Text, node


@node(
    name="FeetechROS2Adapter",
    category="Drivers",
    description="Resolve the Feetech joint driver process used by native ROS 2 and rosbridge transports. This node opens no hardware and never arms motion.",
    inputs={"config": Dict},
    outputs={"adapter": Dict, "report": Text},
)
def feetech_ros2_adapter(ctx: dict) -> dict:
    runtime = Path(__file__).resolve().parents[1] / "runtime" / "feetech_bus_driver.py"
    config = dict(ctx.get("config") or {})
    adapter = {
        "driver": "feetech",
        "integration": "ros2",
        "python": sys.executable,
        "runtime": str(runtime),
        "config": config,
        "motion_armed": False,
        "available": runtime.is_file(),
    }
    report = (
        f"Feetech ROS 2 adapter ready: {runtime} (motion disarmed)"
        if runtime.is_file()
        else f"Feetech ROS 2 runtime unavailable: {runtime}"
    )
    return {"adapter": adapter, "report": report}
