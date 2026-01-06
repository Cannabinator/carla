#!/usr/bin/env python3
"""
Quick Visual Test for LiDAR Frontend
Starts server and opens browser for manual testing
"""

import asyncio
import webbrowser
import argparse
import sys
from pathlib import Path

# Add parent directory to path
sys.path.insert(0, str(Path(__file__).parent.parent))


def print_test_checklist():
    """Print manual testing checklist"""
    print("\n" + "="*70)
    print("LIDAR FRONTEND VISUAL TEST CHECKLIST")
    print("="*70)
    print("\n📋 CAMERA CONTROLS:")
    print("  [ ] Orbit Mode (default)")
    print("      - Left mouse drag rotates around center")
    print("      - Right mouse drag pans")
    print("      - Mouse wheel zooms")
    print("  [ ] Follow Mode")
    print("      - Click 'Enable Follow'")
    print("      - Camera smoothly tracks ego vehicle")
    print("      - Camera follows vehicle rotation (yaw)")
    print("  [ ] Free-Fly Mode")
    print("      - Click 'Enable Free-Fly'")
    print("      - Click canvas to lock pointer")
    print("      - WASD keys move camera")
    print("      - Space/Shift for up/down")
    print("      - Mouse looks around")
    print("      - ESC exits free-fly")
    print("  [ ] Mode Switching")
    print("      - Only one mode active at a time")
    print("      - Buttons update correctly")
    print("      - Switching disables other modes")
    print("  [ ] Reset Camera")
    print("      - Returns to (-25, 0, 20)")
    print("      - Disables all special modes")
    
    print("\n🎨 RENDERING:")
    print("  [ ] Point Cloud Visible")
    print("      - Points render correctly")
    print("      - Semantic colors applied")
    print("      - Points visible at all zoom levels")
    print("  [ ] Point Size Toggle")
    print("      - Cycles: 0.5 → 1.0 → 0.2 → 0.5")
    print("      - Button shows current size")
    print("      - Visible change in point size")
    print("  [ ] Grid Helper")
    print("      - Toggle on/off works")
    print("      - Grid in XY plane (Z-up)")
    print("  [ ] Depth Testing")
    print("      - Points properly occlude each other")
    print("      - No z-fighting artifacts")
    
    print("\n📡 WEBSOCKET:")
    print("  [ ] Initial Connection")
    print("      - Status indicator green")
    print("      - 'Loading...' disappears")
    print("      - Data starts flowing")
    print("  [ ] Reconnection")
    print("      - Shows 'Reconnecting...' on disconnect")
    print("      - Automatically reconnects")
    print("      - Status indicator updates")
    
    print("\n📊 UI ELEMENTS:")
    print("  [ ] Statistics Display")
    print("      - FPS updates (> 0)")
    print("      - Point count updates")
    print("      - Vehicle count updates")
    print("      - Latency shows reasonable value (< 100ms)")
    print("  [ ] All Buttons Present")
    print("      - Reset Camera")
    print("      - Enable Follow / Disable Follow")
    print("      - Enable Free-Fly / Disable Free-Fly")
    print("      - Point Size: X.X")
    print("      - Toggle Grid")
    print("  [ ] Legend Visible")
    print("      - All semantic categories listed")
    print("      - Colors match point cloud")
    
    print("\n⚡ PERFORMANCE:")
    print("  [ ] Smooth Rendering")
    print("      - FPS > 30 (preferably 60)")
    print("      - No stuttering or lag")
    print("      - Camera movement smooth")
    print("  [ ] Large Point Clouds")
    print("      - Handles 50k+ points")
    print("      - Performance acceptable")
    print("      - No browser freezing")
    
    print("\n🔧 TROUBLESHOOTING:")
    print("  [ ] Browser Console")
    print("      - F12 to open developer tools")
    print("      - Check for errors (red text)")
    print("      - Verify WebSocket messages")
    print("  [ ] Network Tab")
    print("      - WebSocket connection visible")
    print("      - Messages flowing")
    print("      - No connection errors")
    
    print("\n" + "="*70)
    print("TESTING PROCEDURE:")
    print("="*70)
    print("1. Ensure CARLA server is running on 192.168.1.110:2000")
    print("2. Run a scenario with LiDAR (e.g., v2v_scenario_perf.py)")
    print("3. Browser should auto-open to http://localhost:8000/viewer.html")
    print("4. Test each item in checklist above")
    print("5. Report any issues or unexpected behavior")
    print("="*70)
    
    print("\n💡 TIPS:")
    print("  - Test each camera mode thoroughly")
    print("  - Try switching modes mid-movement")
    print("  - Test with different point cloud sizes")
    print("  - Check behavior at extreme zoom levels")
    print("  - Verify smooth interpolation in follow mode")
    print("  - Ensure free-fly pointer lock works reliably")
    
    print("\n🐛 KNOWN ISSUES TO VERIFY FIXED:")
    print("  ✅ Camera can only orbit (not move freely) - SHOULD BE FIXED")
    print("  ✅ Points disappear when zoomed out - SHOULD BE FIXED")
    print("  ✅ Follow mode doesn't track rotation - SHOULD BE FIXED")
    print("  ✅ Point size too small (0.1) - SHOULD BE FIXED (now 0.5)")
    
    print("\n" + "="*70)


def print_quick_test_instructions():
    """Print quick test instructions for automated run"""
    print("\n🚀 QUICK FRONTEND TEST")
    print("="*70)
    print("This will:")
    print("1. Start a test WebSocket server")
    print("2. Generate sample point cloud data")
    print("3. Open viewer in browser")
    print("4. Stream test data for 60 seconds")
    print("\nYou can manually test all camera controls and features.")
    print("="*70)


async def run_test_server(duration=60):
    """
    Run a test WebSocket server with sample data
    
    Args:
        duration: How long to run server (seconds)
    """
    try:
        import uvicorn
        from fastapi import FastAPI, WebSocket
        import numpy as np
    except ImportError:
        print("❌ Missing dependencies. Install with:")
        print("   pip install fastapi uvicorn numpy")
        return False
    
    app = FastAPI()
    
    @app.websocket("/ws")
    async def websocket_endpoint(websocket: WebSocket):
        await websocket.accept()
        print("✅ WebSocket client connected")
        
        try:
            t = 0
            while True:
                # Generate sample point cloud
                num_points = np.random.randint(40000, 50000)
                
                # Create a simple moving point cloud
                x = np.random.randn(num_points) * 20 + np.sin(t * 0.1) * 10
                y = np.random.randn(num_points) * 20 + np.cos(t * 0.1) * 10
                z = np.random.randn(num_points) * 5 + 1
                tags = np.random.randint(0, 23, num_points)
                
                # Ego transform (moving in circle)
                ego_x = np.sin(t * 0.1) * 10
                ego_y = np.cos(t * 0.1) * 10
                ego_yaw = (t * 0.1) * 180 / np.pi
                
                data = {
                    "num_points": num_points,
                    "num_vehicles": 5,
                    "ego_transform": {
                        "x": float(ego_x),
                        "y": float(ego_y),
                        "z": 0.5,
                        "yaw": float(ego_yaw)
                    },
                    "points": {
                        "x": x.tolist(),
                        "y": y.tolist(),
                        "z": z.tolist(),
                        "tag": tags.tolist()
                    }
                }
                
                await websocket.send_json(data)
                await asyncio.sleep(0.05)  # 20 Hz update rate
                
                t += 0.05
                
        except Exception as e:
            print(f"❌ WebSocket error: {e}")
        finally:
            print("❌ WebSocket client disconnected")
    
    # Serve static files
    from fastapi.staticfiles import StaticFiles
    from fastapi.responses import HTMLResponse, FileResponse
    
    # Serve viewer.html
    @app.get("/")
    async def root():
        viewer_path = Path(__file__).parent.parent / "src" / "visualization" / "web" / "viewer.html"
        if not viewer_path.exists():
            return HTMLResponse(f"<h1>Viewer not found</h1><p>Expected at: {viewer_path}</p><p>Exists: {viewer_path.exists()}</p>", status_code=404)
        return FileResponse(viewer_path)
    
    @app.get("/viewer.html")
    async def get_viewer():
        viewer_path = Path(__file__).parent.parent / "src" / "visualization" / "web" / "viewer.html"
        if not viewer_path.exists():
            return HTMLResponse(f"<h1>Viewer not found</h1><p>Expected at: {viewer_path}</p><p>Exists: {viewer_path.exists()}</p>", status_code=404)
        return FileResponse(viewer_path)
    
    print("\n🌐 Starting test server on http://localhost:8000")
    print(f"⏱️  Will run for {duration} seconds")
    print("📊 Generating sample point cloud data...")
    
    # Verify viewer exists before starting
    viewer_path = Path(__file__).parent.parent / "src" / "visualization" / "web" / "viewer.html"
    print(f"📁 Viewer path: {viewer_path}")
    print(f"✅ Viewer exists: {viewer_path.exists()}")
    
    # Open browser
    webbrowser.open("http://localhost:8000/")
    
    # Run server
    config = uvicorn.Config(app, host="0.0.0.0", port=8000, log_level="warning")
    server = uvicorn.Server(config)
    
    # Run with timeout
    try:
        await asyncio.wait_for(server.serve(), timeout=duration)
    except asyncio.TimeoutError:
        print(f"\n⏱️  Test duration ({duration}s) reached. Stopping server.")
    
    return True


def main():
    parser = argparse.ArgumentParser(description="LiDAR Frontend Visual Testing")
    parser.add_argument('--checklist', action='store_true', 
                       help='Print testing checklist')
    parser.add_argument('--run', action='store_true',
                       help='Run test server with sample data')
    parser.add_argument('--duration', type=int, default=60,
                       help='Test duration in seconds (default: 60)')
    
    args = parser.parse_args()
    
    if args.checklist:
        print_test_checklist()
    elif args.run:
        print_quick_test_instructions()
        asyncio.run(run_test_server(duration=args.duration))
    else:
        # Default: show both
        print_test_checklist()
        print("\n\n")
        
        choice = input("Run automated test server? (y/n): ").lower()
        if choice == 'y':
            print_quick_test_instructions()
            asyncio.run(run_test_server(duration=args.duration))


if __name__ == "__main__":
    main()
