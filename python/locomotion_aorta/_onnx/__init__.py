"""Private demo inference; imported only when a model is constructed."""

from ._native import ORT_VERSION, Session, TensorInfo

__all__ = ["ORT_VERSION", "Session", "TensorInfo"]
__version__ = "0.2.0"
