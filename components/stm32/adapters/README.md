# Adapters

Transport adapters for the `stm32` component of `blacknode-drivers`.

One folder per transport, each mirroring the component layout:

    adapters/ros2/nodes/
    adapters/ros2/templates/

Declare it in `blacknode-package.toml`:

    [components.stm32.adapters.ros2]
    description = "ROS 2 adapter for stm32."
    default = false
    capabilities = ["adapter.stm32.ros2"]
    nodes = ["components/stm32/adapters/ros2/nodes"]

Adapters stay `default = false`: the capability package owns them, and
`blacknode-ros2` provides only the shared transport underneath.
