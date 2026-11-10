"""Mechanical cleanup for a single test call blocked at an explicit gate."""
from contextlib import contextmanager
import threading


@contextmanager
def running_call(call, *, release, join_timeout):
    errors = []

    def run():
        try:
            call()
        except BaseException as error:
            errors.append(error)

    worker = threading.Thread(target=run)
    worker.start()
    try:
        yield errors
    finally:
        release.set()
        worker.join(join_timeout)
        if worker.is_alive():
            raise AssertionError('test worker did not terminate after gate release')
