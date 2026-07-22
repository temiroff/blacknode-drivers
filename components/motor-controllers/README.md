# Motor Controllers

Component of `blacknode-drivers`.

Node sources for this component belong in this folder. Until they move here,
nodes claim the component inline:

    @node(name="MyNode", component="motor-controllers", ...)

Once sources live here, declare the folder in `blacknode-package.toml`:

    [components.motor-controllers]
    nodes = ["components/motor-controllers/nodes"]

and the inline `component=` argument can be dropped — the loader infers it
from the directory.
