#!/bin/bash
# Start V2V LiDAR Web Viewer (Standalone - No CARLA Required)
# Access from Windows: http://192.168.xxx.xxx:8000

set -e

cd /home/workstation/carla

echo "╔════════════════════════════════════════════════════════════════╗"
echo "║       V2V LiDAR Web Viewer - Standalone Mode                 ║"
echo "╚════════════════════════════════════════════════════════════════╝"
echo ""

# Activate virtual environment
echo "Activating virtual environment..."
source venv/bin/activate

# Get network IP
NETWORK_IP=$(hostname -I | awk '{print $1}')

echo ""
echo "╔════════════════════════════════════════════════════════════════╗"
echo "║                   Access URLs                              ║"
echo "╚════════════════════════════════════════════════════════════════╝"
echo ""
echo "From Ubuntu (local):"
echo "   http://localhost:8000"
echo ""
echo "From Windows (network):"
echo "   http://${NETWORK_IP}:8000"
echo ""
echo " Instructions for Windows:"
echo "   1. Open any web browser (Chrome, Edge, Firefox)"
echo "   2. Enter URL: http://${NETWORK_IP}:8000"
echo "   3. If blocked, check Windows Firewall settings"
echo ""
echo "Press Ctrl+C to stop server"
echo ""
echo "════════════════════════════════════════════════════════════════"
echo ""

# Run server using main entry point
python start_server.py
