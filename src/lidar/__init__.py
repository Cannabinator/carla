"""Compact LiDAR application namespace.

This package provides a shorter import path for the LiDAR collector,
streaming API, and FastAPI server while preserving the existing
visualization package for backward compatibility.
"""

from src.visualization.lidar import (
    LiDARDataCollector,
    ConnectionManager,
    app,
    manager,
    set_collector,
    LiDARStreamingAPI,
    create_ego_lidar_stream,
)

__all__ = [
    "LiDARDataCollector",
    "ConnectionManager",
    "app",
    "manager",
    "set_collector",
    "LiDARStreamingAPI",
    "create_ego_lidar_stream",
]