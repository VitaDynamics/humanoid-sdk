from collections.abc import Callable
from typing import Any


class FakeSubscription:
    def __init__(self) -> None:
        self.close_count = 0

    def close(self) -> None:
        self.close_count += 1


class FakeTransport:
    def __init__(self) -> None:
        self.callbacks: dict[str, list[Callable[[Any], None]]] = {}
        self.subscriptions: list[FakeSubscription] = []
        self.published: list[tuple[str, Any]] = []
        self.on_publish: Callable[[str, Any], None] | None = None
        self.close_count = 0
        self.closed = False

    def _ensure_open(self) -> None:
        if self.closed:
            from locomotion_aorta import ControllerClosedError
            raise ControllerClosedError('Aorta transport is closed')

    def subscribe(
        self, topic: str, callback: Callable[[Any], None]
    ) -> FakeSubscription:
        subscription = FakeSubscription()
        self.subscriptions.append(subscription)
        self.callbacks.setdefault(topic, []).append(callback)
        return subscription

    def publish(self, topic: str, message: Any) -> None:
        self.published.append((topic, message))
        if self.on_publish is not None:
            self.on_publish(topic, message)

    def emit(self, topic: str, message: Any) -> None:
        for callback in list(self.callbacks.get(topic, [])):
            callback(message)

    def get_robot_info(self, timeout):
        from body_fixtures import robot_info_reply
        return robot_info_reply()

    def close(self) -> None:
        self.closed = True
        self.close_count += 1
