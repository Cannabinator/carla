#!/usr/bin/env python3
"""
Flexible LiDAR Streaming API
Provides reusable interface for V2V LiDAR visualization across different scenarios.
"""

import asyncio
import logging
import threading
import time
from typing import Optional, Callable
from pathlib import Path

import carla

from .collector import LiDARDataCollector
from .server import app, set_collector

logger = logging.getLogger(__name__)


class LiDARStreamingAPI:
    """
    Flexible API for streaming LiDAR data from CARLA vehicles.
    
    Features:
    - Single or multi-vehicle LiDAR streaming
    - Configurable sensor parameters
    - Web server management
    - Automatic cleanup
    
    Example:
        >>> api = LiDARStreamingAPI(world, web_port=8000)
        >>> api.register_vehicle(ego_vehicle, vehicle_id=0)  # Only ego
        >>> api.start_server()
        >>> # ... run simulation ...
        >>> api.stop()
    """
    
    def __init__(
        self,
        world: carla.World,
        web_host: str = '0.0.0.0',
        web_port: int = 8000,
        downsample_factor: int = 1,
        channels: int = 64,
        points_per_second: int = 1000000,
        lidar_range: float = 100.0,
        rotation_frequency: float = 20.0,
        v2v_network = None,
    ):
        """
        Initialize LiDAR streaming API.
        
        Args:
            world: CARLA world instance
            web_host: Web server host (0.0.0.0 for all interfaces)
            web_port: Web server port
            downsample_factor: Point downsampling (1=no downsampling)
            channels: Number of LiDAR laser channels
            points_per_second: LiDAR point generation rate
            lidar_range: LiDAR maximum range in meters
            rotation_frequency: LiDAR rotation speed in Hz
            v2v_network: Optional V2VNetworkEnhanced instance for combined visualization
        """
        self.world = world
        self.web_host = web_host
        self.web_port = web_port
        self.v2v_network = v2v_network
        
        # LiDAR configuration
        self.lidar_config = {
            'channels': channels,
            'points_per_second': points_per_second,
            'range': lidar_range,
            'rotation_frequency': rotation_frequency,
            'upper_fov': 15.0,
            'lower_fov': -25.0,
            'horizontal_fov': 360.0,
        }
        
        # Create collector with custom config
        self.collector = LiDARDataCollector(
            world=world,
            downsample_factor=downsample_factor,
            **self.lidar_config
        )
        
        # Server state
        self.server_thread: Optional[threading.Thread] = None
        self.is_running = False
        
        logger.info(f"LiDAR API initialized: {channels}ch, {points_per_second}pts/s, {lidar_range}m range")
    
    def register_vehicle(self, vehicle: carla.Actor, vehicle_id: int = 0):
        """
        Register a vehicle for LiDAR streaming.
        
        Args:
            vehicle: CARLA vehicle actor
            vehicle_id: Unique identifier for this vehicle
        """
        self.collector.register_vehicle(vehicle_id, vehicle)
        logger.info(f"Registered vehicle {vehicle_id} for LiDAR streaming")
    
    def register_ego_only(self, ego_vehicle: carla.Actor):
        """
        Convenience method to register only the ego vehicle.
        
        Args:
            ego_vehicle: The ego/leading vehicle
        """
        self.register_vehicle(ego_vehicle, vehicle_id=0)
        logger.info("Ego vehicle registered (single-vehicle mode)")
    
    def start_server(self, background: bool = True):
        """
        Start the web server for visualization.
        
        Args:
            background: Run server in background thread (default True)
        """
        if self.is_running:
            logger.warning("Server already running")
            return
        
        # Register collector and V2V network with FastAPI
        set_collector(self.collector)
        if self.v2v_network:
            from .server import set_v2v_network
            set_v2v_network(self.v2v_network)
        
        if background:
            self.server_thread = threading.Thread(
                target=self._run_server,
                daemon=True
            )
            self.server_thread.start()
            time.sleep(2)  # Give server time to start
        else:
            self._run_server()
        
        self.is_running = True
        
        # Get network IP
        import socket
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(("8.8.8.8", 80))
            network_ip = s.getsockname()[0]
            s.close()
        except:
            network_ip = "localhost"
        
        print(f"\n{'='*80}")
        if self.v2v_network:
            print(f"🌐 Unified Visualization Server:")
            print(f"   Dashboard: http://localhost:{self.web_port}")
            print(f"   Network:   http://{network_ip}:{self.web_port}")
            print(f"   Endpoints: /lidar (3D viewer) | /v2v (V2V dashboard)")
        else:
            print(f"🌐 LiDAR Viewer URLs:")
            print(f"   Local:   http://localhost:{self.web_port}")
            print(f"   Network: http://{network_ip}:{self.web_port}")
        print(f"{'='*80}\n")
        logger.info(f"Web server started on {self.web_host}:{self.web_port}")
    
    def _run_server(self):
        """Internal method to run uvicorn server."""
        import uvicorn
        uvicorn.run(app, host=self.web_host, port=self.web_port, log_level="info")
    
    def stop(self):
        """Stop streaming and cleanup resources."""
        logger.info("Stopping LiDAR streaming...")
        self.collector.cleanup()
        self.is_running = False
        logger.info("✓ LiDAR API stopped")
    
    def get_point_count(self) -> int:
        """Get current total point count across all vehicles."""
        data = self.collector.get_combined_pointcloud()
        return data.get('num_points', 0) if data else 0
    
    def get_vehicle_count(self) -> int:
        """Get number of registered vehicles."""
        return len(self.collector.vehicles)


def create_ego_lidar_stream(
    world: carla.World,
    ego_vehicle: carla.Actor,
    web_port: int = 8000,
    high_quality: bool = True
) -> LiDARStreamingAPI:
    """
    Quick setup for ego-only LiDAR streaming (most common use case).
    
    Args:
        world: CARLA world instance
        ego_vehicle: The ego/leading vehicle
        web_port: Web server port
        high_quality: Use high-quality settings (True) or fast settings (False)
    
    Returns:
        LiDARStreamingAPI instance (already started)
    
    Example:
        >>> api = create_ego_lidar_stream(world, ego_vehicle)
        >>> # ... run simulation ...
        >>> api.stop()
    """
    if high_quality:
        # High quality: Dense point cloud
        api = LiDARStreamingAPI(
            world=world,
            web_port=web_port,
            downsample_factor=1,
            channels=64,
            points_per_second=1000000,
            lidar_range=100.0,
            rotation_frequency=20.0,
        )
    else:
        # Fast mode: Lower resolution but faster
        api = LiDARStreamingAPI(
            world=world,
            web_port=web_port,
            downsample_factor=2,
            channels=32,
            points_per_second=500000,
            lidar_range=80.0,
            rotation_frequency=20.0,
        )
    
    api.register_ego_only(ego_vehicle)
    api.start_server()
    
    logger.info(f"Ego LiDAR stream created ({'high quality' if high_quality else 'fast mode'})")
    return api
