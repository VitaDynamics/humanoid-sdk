"""Optional demo inference, separate from the locomotion_aorta core package."""

from ._native import ORT_VERSION, Session, TensorInfo

__all__ = ["ORT_VERSION", "Session", "TensorInfo"]
__version__ = "0.2.0"
