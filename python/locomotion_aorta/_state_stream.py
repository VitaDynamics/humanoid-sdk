"""Aorta state-only adapter; deliberately does not construct control transport."""
import importlib

from .transport import AortaBindingsUnavailableError, BODY_STATE_TOPIC
from ._group import require_node_group


class _StateStream:
    def __init__(self, *, node_name, group, callback):
        aorta = importlib.import_module("aorta")
        decoder = importlib.import_module("lowlevel.BodyState").BodyState.GetRootAs
        self.node = aorta.Node(node_name, group)
        try:
            require_node_group(self.node, group)
            self.subscription = self.node.create_subscriber(
                BODY_STATE_TOPIC, lambda payload, _ctx: callback(payload, decoder),
                qos=aorta.QoS.sensor_data())
        except BaseException:
            self.node.close()
            raise

    def close(self):
        self.node.close()


def create_state_stream(**kwargs):
    try:
        return _StateStream(**kwargs)
    except (ImportError, OSError) as error:
        raise AortaBindingsUnavailableError(
            "the compatible Aorta body31 bindings are unavailable") from error
