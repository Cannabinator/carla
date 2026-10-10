"""``install()`` makes ``import carla`` resolve to the deterministic fake (``serve --fake``, frontend tests)."""

import sys

from .sim import build_module


def install(seed: int = 0):
    """Register the fake as the ``carla`` module. Refuses to replace a real, already imported CARLA."""
    existing = sys.modules.get("carla")
    if existing is not None and not getattr(existing, "IS_FAKE", False):
        raise RuntimeError("the real carla module is already imported; --fake must be chosen before it is")
    module = build_module(seed)
    sys.modules["carla"] = module
    return module
