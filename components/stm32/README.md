# STM32

Design notes for a future concrete firmware-bridge driver family.

STM32 is not a public package component yet. Serial, CAN, or USB helpers used
by an STM32 bridge belong under `internal/transports/`.

Add it to `blacknode-package.toml` when driver nodes, firmware protocol,
safeguards, actionable unavailable-state reporting, and tests are implemented.
