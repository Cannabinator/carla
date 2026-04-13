"""Legacy visualization namespace.

The compact application layout now exposes LiDAR functionality from
``src.lidar``. This module remains as a compatibility layer for older imports
by re-exporting the underlying visualization package.
"""

from .lidar import LiDARDataCollector, ConnectionManager, app, manager, set_collector

__all__ = [
    'LiDARDataCollector',
    'ConnectionManager',
    'app',
    'manager',
    'set_collector'
]
