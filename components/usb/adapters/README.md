# Adapters

Transport adapters for the `usb` component of `blacknode-drivers`.

One folder per transport, each mirroring the component layout:

    adapters/ros2/nodes/
    adapters/ros2/templates/

Declare it in `blacknode-package.toml`:

    [components.usb.adapters.ros2]
    description = "ROS 2 adapter for usb."
    default = false
    capabilities = ["adapter.usb.ros2"]
    nodes = ["components/usb/adapters/ros2/nodes"]

Adapters stay `default = false`: the capability package owns them, and
`blacknode-ros2` provides only the shared transport underneath.
