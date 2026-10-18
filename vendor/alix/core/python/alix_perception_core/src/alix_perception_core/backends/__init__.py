"""Inference backends. `local` is the default and is registered on import."""

from .base import (Backend, BackendError, DEFAULT_BACKEND, available, create,
                   register)

__all__ = ["Backend", "BackendError", "DEFAULT_BACKEND", "available", "create",
           "register"]
