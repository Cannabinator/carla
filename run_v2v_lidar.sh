#!/bin/bash
# Quick start script for V2V LiDAR Visualization

echo "╔════════════════════════════════════════════════════════════════╗"
echo "║       V2V LiDAR Visualization - Quick Start                   ║"
echo "╚════════════════════════════════════════════════════════════════╝"
echo ""

# Check if venv exists
if [ ! -d "venv" ]; then
    echo "❌ Virtual environment not found. Please create it first:"
    echo "   python3 -m venv venv"
    exit 1
fi

# Activate venv
echo "🔧 Activating virtual environment..."
source venv/bin/activate

# Install dependencies
echo "📦 Installing dependencies..."
pip install -q fastapi uvicorn[standard] websockets

# Check CARLA server
echo ""
echo "🔍 Checking CARLA server connection..."
python3 -c "
import carla
import sys
try:
    client = carla.Client('192.168.1.110', 2000)
    client.set_timeout(5.0)
    world = client.get_world()
    print(f'✓ Connected to CARLA: {world.get_map().name}')
except Exception as e:
    print(f'❌ Cannot connect to CARLA server: {e}')
    print('   Make sure CARLA is running on 192.168.1.110:2000')
    sys.exit(1)
"

if [ $? -ne 0 ]; then
    exit 1
fi

# Run tests
echo ""
echo "🧪 Running tests..."
python tests/test_v2v_lidar.py 2>&1 | grep -E "(test_|^=|Ran|OK|FAILED)"

if [ $? -ne 0 ]; then
    echo "⚠️  Some tests failed, but continuing..."
fi

# Get local IP
LOCAL_IP=$(hostname -I | awk '{print $1}')

echo ""
echo "╔════════════════════════════════════════════════════════════════╗"
echo "║                   🚀 Starting V2V LiDAR Scenario               ║"
echo "╚════════════════════════════════════════════════════════════════╝"
echo ""
echo "📡 Web Viewer URLs:"
echo "   Local:    http://localhost:8000"
echo "   Network:  http://$LOCAL_IP:8000"
echo ""
echo "🎮 Controls:"
echo "   - Left Mouse:  Rotate view"
echo "   - Right Mouse: Pan view"
echo "   - Scroll:      Zoom"
echo ""
echo "⏹️  Press Ctrl+C to stop"
echo ""
echo "════════════════════════════════════════════════════════════════"
echo ""

# Run scenario
python src/scenarios/v2v_complete_demo.py \
    --carla-host 192.168.1.110 \
    --duration 120
