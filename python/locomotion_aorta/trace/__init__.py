"""Optional SDK-local FBS tracing. Importing this module starts no resources.

All submitted time values are integer microseconds. Register producer/stage
handles before starting control; each producer belongs to one actual thread.
"""
from .session import Kind, Outcome, Session, disabled, monotonic_us
from ._sdk import Trace

__all__ = ['Trace', 'Kind', 'Outcome', 'Session', 'disabled', 'monotonic_us']
