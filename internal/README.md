# Internal driver implementation

This tree contains implementation details shared by concrete hardware-driver
components. Nothing under `internal/` is a selectable Blacknode component or a
public capability.

Device-family components own their user-facing configuration, safeguards,
nodes, runtimes, and adapters. They may reuse private transport helpers from
`internal/transports/`.
