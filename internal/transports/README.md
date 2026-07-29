# Internal transports

Serial, CAN, and USB are communication mechanisms used by concrete drivers.
They are private implementation modules and are not declared in
`blacknode-package.toml`.

A driver selects and configures its transport internally. Users select a
device-family component such as `feetech`; they do not select a transport
component.
