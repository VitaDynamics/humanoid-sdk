"""Validate the namespace actually selected by the pinned Aorta Core."""
import os


def require_node_group(node, requested):
    actual = node.group()
    if actual != requested:
        raise ValueError(
            f'effective Aorta group differs: requested={requested!r}, actual={actual!r}; '
            f'AORTA_GROUP={os.environ.get("AORTA_GROUP")!r}. '
            'Pinned Aorta Core resolves AORTA_GROUP/default, not the Node constructor group; '
            'set AORTA_GROUP to the requested group before creating nodes. '
            'SDK_GROUP is a demo/runner convention and does not override Core by itself.')
