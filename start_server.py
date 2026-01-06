#!/usr/bin/env python3
"""
Standalone V2V Visualization Server

Starts the unified web server on port 8000 without running a simulation.
Control simulations from the web interface at http://localhost:8000
"""

import sys
import logging
from pathlib import Path

# Add project root to path
sys.path.insert(0, str(Path(__file__).parent.parent))

from src.visualization.lidar.server import run_server

# Setup logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


def main():
    """Start the visualization server."""
    host = '0.0.0.0'
    port = 8000
    
    print("=" * 80)
    print("🌐 CARLA V2V Visualization Server")
    print("=" * 80)
    print(f"Starting server on {host}:{port}")
    print()
    print(f"📊 Control Panel:   http://localhost:{port}/")
    print(f"🎯 LiDAR Viewer:    http://localhost:{port}/lidar")
    print(f"📡 V2V Dashboard:   http://localhost:{port}/v2v")
    print()
    print("💡 Use keyboard shortcuts:")
    print("   1 = Control Panel  |  2 = LiDAR Viewer  |  3 = V2V Dashboard")
    print()
    print("🎮 Control simulations from the web interface!")
    print("   Configure parameters and click 'Start Simulation'")
    print()
    print("Press Ctrl+C to stop the server")
    print("=" * 80)
    print()
    
    try:
        run_server(host=host, port=port)
    except KeyboardInterrupt:
        print("\n\n✓ Server stopped")
        logger.info("Server shutdown")


if __name__ == "__main__":
    main()
