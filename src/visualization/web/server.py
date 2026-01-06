#!/usr/bin/env python3
"""
Standalone V2V LiDAR Web Viewer Server
Runs the web frontend without requiring CARLA connection.
Useful for development or viewing recorded data.
"""

import argparse
import logging
from pathlib import Path
import sys

# Add project root to path (web/server.py -> web -> visualization -> src -> carla)
sys.path.insert(0, str(Path(__file__).parent.parent.parent.parent))
from src.visualization.lidar import app

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


def run_standalone_server(host: str = '0.0.0.0', port: int = 8000):
    """Run the web server standalone without CARLA connection.
    
    Args:
        host: Server host (0.0.0.0 for all interfaces)
        port: Server port
    """
    import uvicorn
    import socket
    
    # Get actual network IP
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        network_ip = s.getsockname()[0]
        s.close()
    except:
        network_ip = "your-ip-address"
    
    print(f"\n{'='*80}")
    print(f"🌐 V2V LiDAR Web Viewer - Standalone Mode")
    print(f"{'='*80}\n")
    print(f"📡 Server URLs:")
    print(f"   Local (Ubuntu):  http://localhost:{port}")
    print(f"   Network (Windows): http://{network_ip}:{port}")
    print(f"\n💡 Access from Windows:")
    print(f"   1. Open browser on Windows")
    print(f"   2. Navigate to: http://{network_ip}:{port}")
    print(f"   3. Make sure Windows Firewall allows the connection\n")
    print(f"⏹️  Press Ctrl+C to stop\n")
    print(f"{'='*80}\n")
    
    logger.info(f"Starting standalone web server on {host}:{port}")
    logger.info(f"Network IP: {network_ip}")
    
    try:
        uvicorn.run(app, host=host, port=port, log_level="info")
    except KeyboardInterrupt:
        print("\n\n👋 Shutting down server...")
        logger.info("Server stopped by user")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description='Standalone V2V LiDAR Web Viewer Server'
    )
    parser.add_argument(
        '--host',
        default='0.0.0.0',
        help='Server host (0.0.0.0 for all interfaces)'
    )
    parser.add_argument(
        '--port',
        type=int,
        default=8000,
        help='Server port (default: 8000)'
    )
    
    args = parser.parse_args()
    
    run_standalone_server(host=args.host, port=args.port)
