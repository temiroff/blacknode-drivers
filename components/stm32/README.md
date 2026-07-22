# Stm32

Component of `blacknode-drivers`.

Node sources for this component belong in this folder. Until they move here,
nodes claim the component inline:

    @node(name="MyNode", component="stm32", ...)

Once sources live here, declare the folder in `blacknode-package.toml`:

    [components.stm32]
    nodes = ["components/stm32/nodes"]

and the inline `component=` argument can be dropped — the loader infers it
from the directory.
