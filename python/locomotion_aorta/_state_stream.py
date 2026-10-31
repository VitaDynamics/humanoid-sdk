"""Aorta state-only adapter; deliberately does not construct control transport."""
import importlib
import math
import threading

from .transport import AortaBindingsUnavailableError, BODY_STATE_TOPIC, _call_get_robot_info
from ._group import require_node_group


class _StateStream:
    def __init__(self, *, node_name, group, callback):
        aorta = importlib.import_module("aorta")
        body_state = importlib.import_module("lowlevel.BodyState").BodyState
        if (not callable(getattr(body_state, 'GetRootAs', None))
                or not callable(getattr(body_state, 'RuntimeId', None))
                or not callable(getattr(body_state, 'Joints', None))
                or hasattr(body_state, 'DescriptorRef')):
            raise AortaBindingsUnavailableError('incompatible BodyState direct-runtime bindings')
        decoder = body_state.GetRootAs
        self.node = aorta.Node(node_name, group)
        self._aorta = aorta
        self._info_lock = threading.Lock()
        self._info_clients = {}
        self._closed = False
        try:
            require_node_group(self.node, group)
            self.subscription = self.node.create_subscriber(
                BODY_STATE_TOPIC, lambda payload, _ctx: callback(payload, decoder),
                qos=aorta.QoS.sensor_data())
        except BaseException:
            self.node.close()
            raise

    def get_robot_info(self, timeout):
        if isinstance(timeout, bool) or not isinstance(timeout, (int, float)) or not math.isfinite(timeout) or timeout <= 0:
            raise ValueError('timeout must be finite and positive')
        with self._info_lock:
            if self._closed:
                raise RuntimeError('body state source is closed')
            timeout_ms = math.ceil(timeout * 1000)
            request = importlib.import_module('aorta.services.humanoid_locomotion.GetRobotInfoRequest')
            response = importlib.import_module('aorta.services.humanoid_locomotion.GetRobotInfoResponse')
            meta = importlib.import_module('get_robot_info_schema_meta')
            return _call_get_robot_info(self.node, self._aorta, self._info_clients,
                                        timeout_ms, request, response, meta)

    def close(self):
        with self._info_lock:
            if not self._closed:
                self._closed = True
                self.node.close()


def create_state_stream(**kwargs):
    try:
        return _StateStream(**kwargs)
    except (ImportError, OSError) as error:
        raise AortaBindingsUnavailableError(
            "the compatible Aorta body31 bindings are unavailable") from error
