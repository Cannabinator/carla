#!/bin/bash
# Test network connectivity for Windows access

echo "╔════════════════════════════════════════════════════════════════╗"
echo "║       Network Connectivity Test for Windows Access           ║"
echo "╚════════════════════════════════════════════════════════════════╝"
echo ""

# Get network info
UBUNTU_IP=$(hostname -I | awk '{print $1}')
WINDOWS_IP="192.168.1.110"

echo "🔍 Network Configuration:"
echo "   Ubuntu IP:  $UBUNTU_IP"
echo "   Windows IP: $WINDOWS_IP"
echo ""

# Test 1: Ping Windows
echo "📡 Test 1: Can Ubuntu reach Windows?"
if ping -c 2 -W 2 $WINDOWS_IP &>/dev/null; then
    echo "   ✅ SUCCESS - Can ping Windows at $WINDOWS_IP"
else
    echo "   ❌ FAILED - Cannot ping Windows"
fi
echo ""

# Test 2: Check if web server can start
echo "📡 Test 2: Starting web server on port 8000..."
cd /home/workstation/carla
source venv/bin/activate

# Start server in background
python src/visualization/web/server.py --host 0.0.0.0 --port 8000 > /tmp/webserver.log 2>&1 &
SERVER_PID=$!
sleep 3

# Check if server is running
if ss -tlnp 2>/dev/null | grep -q ":8000"; then
    echo "   ✅ SUCCESS - Server listening on port 8000"
    
    # Get the binding address
    BIND_ADDR=$(ss -tlnp 2>/dev/null | grep :8000 | awk '{print $4}')
    echo "   📍 Binding: $BIND_ADDR"
    
    if echo "$BIND_ADDR" | grep -q "0.0.0.0"; then
        echo "   ✅ Server accepts connections from any IP (0.0.0.0)"
    else
        echo "   ⚠️  Server may only accept local connections"
    fi
else
    echo "   ❌ FAILED - Server not listening"
    echo "   📝 Server log:"
    cat /tmp/webserver.log 2>/dev/null | tail -5
fi
echo ""

# Test 3: Local access
echo "📡 Test 3: Testing local access..."
if curl -s --max-time 2 http://localhost:8000 >/dev/null 2>&1; then
    echo "   ✅ SUCCESS - Can access http://localhost:8000"
else
    echo "   ❌ FAILED - Cannot access locally"
fi
echo ""

# Test 4: Network access from Ubuntu
echo "📡 Test 4: Testing network access from Ubuntu IP..."
if curl -s --max-time 2 http://$UBUNTU_IP:8000 >/dev/null 2>&1; then
    echo "   ✅ SUCCESS - Can access http://$UBUNTU_IP:8000"
else
    echo "   ❌ FAILED - Cannot access via network IP"
fi
echo ""

# Test 5: Check firewall
echo "📡 Test 5: Checking Ubuntu firewall..."
if command -v ufw >/dev/null 2>&1; then
    UFW_STATUS=$(sudo ufw status 2>/dev/null | grep -i "status:" | awk '{print $2}')
    if [ "$UFW_STATUS" = "inactive" ]; then
        echo "   ✅ UFW firewall is inactive (not blocking)"
    else
        echo "   ⚠️  UFW is active - checking port 8000..."
        if sudo ufw status | grep -q "8000"; then
            echo "   ✅ Port 8000 is allowed in firewall"
        else
            echo "   ⚠️  Port 8000 not explicitly allowed"
            echo "   💡 Run: sudo ufw allow 8000/tcp"
        fi
    fi
else
    echo "   ✅ UFW not installed (no firewall)"
fi
echo ""

# Summary
echo "╔════════════════════════════════════════════════════════════════╗"
echo "║                   📋 Summary & Next Steps                     ║"
echo "╚════════════════════════════════════════════════════════════════╝"
echo ""
echo "🌐 Access URL for Windows:"
echo "   http://$UBUNTU_IP:8000"
echo ""
echo "📝 To access from Windows:"
echo "   1. Open any browser (Chrome, Edge, Firefox)"
echo "   2. Enter: http://$UBUNTU_IP:8000"
echo "   3. If blocked, check Windows Firewall"
echo ""
echo "🛑 To stop test server:"
echo "   kill $SERVER_PID"
echo ""
echo "📖 For detailed troubleshooting, see: WINDOWS_ACCESS.md"
echo ""

# Keep server running or kill
read -p "Keep server running? (y/n): " -n 1 -r
echo ""
if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    kill $SERVER_PID 2>/dev/null
    echo "✓ Server stopped"
else
    echo "✓ Server running (PID: $SERVER_PID)"
    echo "  Access at: http://$UBUNTU_IP:8000"
    echo "  Stop with: kill $SERVER_PID"
fi
