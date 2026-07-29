# blacknode-drivers Agent Instructions

This is an independent Blacknode extension-package repository. Check and
commit its Git state separately from the Blacknode core checkout.

## Scope

Keep physical bus protocols, vendor SDK adapters, firmware bridges, hardware
probes, and last-boundary motion safeguards here. Keep robot profiles and
calibration ownership in `blacknode-robot`, generic controllers in
`blacknode-motion`, and ROS graph/transport behavior in `blacknode-ros2`.

## Safety rules

- Keep motion and torque changes disarmed unless a user explicitly authorizes
  them.
- Before enabling torque, read every joint and seed its current position as
  the goal. Abort and release all joints if any read or write fails.
- Clamp commands at the driver boundary even when an upstream controller also
  applies limits.
- Serialize half-duplex bus transactions and make repeated commands
  idempotent.
- Default shutdown must disable torque. Warn that an unsupported arm may fall
  under gravity.
- Hardware probes must remain read-only and require an explicit confirmation.
- Never discover limits by driving into a physical hard stop.

## Development rules

- Public components represent concrete device or firmware driver families.
  Keep serial, CAN, USB, and other communication mechanisms private under
  `internal/transports/`; do not expose them as selectable components.
- Add a new driver component only with a real implementation, dependencies,
  safeguards, tests, and actionable unavailable-state reporting. Do not add
  generic buckets such as motor controllers, sensor drivers, or vendor
  adapters.
- Components are selectively loaded through `blacknode-package.toml`; keep
  optional vendor imports out of module top level.
- A hardware component may expose transport-neutral configuration and runtime
  primitives. Keep integration-specific adapters nested under their owning
  component, and give each optional adapter versioned dependencies such as
  `blacknode-ros2/core`.
- Preserve node names used by saved workflows or provide compatibility aliases.
- Keep every component independently mock-testable without attached hardware.

## Verification

From the Blacknode root:

```powershell
python -m pytest packages/blacknode-drivers/tests
python -m blacknode.cli validate packages/blacknode-drivers/templates/feetech-bus-config.json
```

Report physical serial, torque, shutdown, ROS, and motion paths as untested
unless they were deliberately exercised on supported hardware.
