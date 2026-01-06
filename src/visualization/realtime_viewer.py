#!/usr/bin/env python3
"""
Real-time LIDAR and Camera Visualization for CARLA
Uses Open3D for 3D point clouds and OpenCV for camera feeds
"""

import carla
import numpy as np
import queue
import time
import argparse
from threading import Thread

try:
    import open3d as o3d
    OPEN3D_AVAILABLE = True
except ImportError:
    OPEN3D_AVAILABLE = False
    print("⚠️  Open3D not available. Install with: pip install open3d")

try:
    import cv2
    CV2_AVAILABLE = True
except ImportError:
    CV2_AVAILABLE = False
    print("⚠️  OpenCV not available. Install with: pip install opencv-python")


class LidarVisualizer:
    """Real-time LIDAR point cloud visualizer using Open3D."""
    
    def __init__(self):
        if not OPEN3D_AVAILABLE:
            raise ImportError("Open3D is required for LIDAR visualization")
            
        self.vis = o3d.visualization.Visualizer()
        self.vis.create_window(window_name='CARLA LIDAR', width=1280, height=720)
        
        self.pcd = o3d.geometry.PointCloud()
        self.first_frame = True
        
        # Setup camera view
        self.setup_view()
        
    def setup_view(self):
        """Setup optimal viewing angle."""
        opt = self.vis.get_render_option()
        opt.background_color = np.asarray([0.05, 0.05, 0.05])
        opt.point_size = 2.0
        opt.show_coordinate_frame = True
        
    def update(self, points):
        """Update point cloud visualization.
        
        Args:
            points: Nx4 numpy array (x, y, z, intensity)
        """
        # Extract XYZ coordinates
        xyz = points[:, :3]
        
        # Color by height (Z coordinate) for better visualization
        colors = np.zeros_like(xyz)
        z_norm = (xyz[:, 2] - xyz[:, 2].min()) / (xyz[:, 2].max() - xyz[:, 2].min() + 1e-6)
        colors[:, 0] = z_norm  # Red channel
        colors[:, 2] = 1 - z_norm  # Blue channel
        
        self.pcd.points = o3d.utility.Vector3dVector(xyz)
        self.pcd.colors = o3d.utility.Vector3dVector(colors)
        
        if self.first_frame:
            self.vis.add_geometry(self.pcd)
            self.first_frame = False
        else:
            self.vis.update_geometry(self.pcd)
            
        self.vis.poll_events()
        self.vis.update_renderer()
        
    def close(self):
        """Close the visualizer."""
        self.vis.destroy_window()


class CameraVisualizer:
    """Real-time camera feed visualizer using OpenCV."""
    
    def __init__(self, window_name='CARLA Camera'):
        if not CV2_AVAILABLE:
            raise ImportError("OpenCV is required for camera visualization")
            
        self.window_name = window_name
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.window_name, 960, 540)
        
    def update(self, image_data):
        """Update camera feed.
        
        Args:
            image_data: CARLA image sensor data
        """
        # Convert CARLA image to numpy array
        array = np.frombuffer(image_data.raw_data, dtype=np.uint8)
        array = np.reshape(array, (image_data.height, image_data.width, 4))
        array = array[:, :, :3]  # Remove alpha channel
        array = cv2.cvtColor(array, cv2.COLOR_RGB2BGR)  # Convert to BGR for OpenCV
        
        # Add timestamp overlay
        cv2.putText(array, f"Frame: {image_data.frame}", (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        
        cv2.imshow(self.window_name, array)
        cv2.waitKey(1)
        
    def close(self):
        """Close the visualizer."""
        cv2.destroyWindow(self.window_name)


class VisualizationScenario:
    """CARLA scenario with real-time visualization."""
    
    def __init__(self, host='192.168.1.110', port=2000, enable_lidar=True, enable_camera=True):
        self.host = host
        self.port = port
        self.enable_lidar = enable_lidar
        self.enable_camera = enable_camera
        
        self.client = None
        self.world = None
        self.ego_vehicle = None
        self.sensors = []
        self.actor_list = []
        
        # Queues for sensor data
        self.lidar_queue = queue.Queue(maxsize=2)
        self.camera_queue = queue.Queue(maxsize=2)
        
        # Visualizers
        self.lidar_viz = None
        self.camera_viz = None
        
    def connect(self):
        """Connect to CARLA server."""
        print(f"🔄 Connecting to CARLA server at {self.host}:{self.port}")
        self.client = carla.Client(self.host, self.port)
        self.client.set_timeout(30.0)
        self.world = self.client.get_world()
        print(f"✓ Connected! Map: {self.world.get_map().name}\n")
        
    def setup_world(self):
        """Configure world settings."""
        settings = self.world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.05  # 20 FPS
        self.world.apply_settings(settings)
        print("✓ Synchronous mode enabled")
        
    def spawn_ego_vehicle(self):
        """Spawn ego vehicle."""
        print("🚗 Spawning ego vehicle...")
        blueprint_library = self.world.get_blueprint_library()
        vehicle_bp = blueprint_library.filter('vehicle.tesla.model3')[0]
        spawn_points = self.world.get_map().get_spawn_points()
        
        self.ego_vehicle = self.world.spawn_actor(vehicle_bp, spawn_points[0])
        self.actor_list.append(self.ego_vehicle)
        print("✓ Ego vehicle spawned")
        
    def attach_sensors(self):
        """Attach sensors for visualization."""
        print("\n📡 Attaching sensors...")
        blueprint_library = self.world.get_blueprint_library()
        
        # LIDAR
        if self.enable_lidar and OPEN3D_AVAILABLE:
            lidar_bp = blueprint_library.find('sensor.lidar.ray_cast')
            lidar_bp.set_attribute('channels', '32')
            lidar_bp.set_attribute('points_per_second', '280000')
            lidar_bp.set_attribute('rotation_frequency', '10')
            lidar_bp.set_attribute('range', '100')
            
            lidar_transform = carla.Transform(carla.Location(x=0, z=2.4))
            lidar = self.world.spawn_actor(lidar_bp, lidar_transform, attach_to=self.ego_vehicle)
            self.sensors.append(lidar)
            
            def lidar_callback(data):
                if not self.lidar_queue.full():
                    self.lidar_queue.put(data)
                    
            lidar.listen(lidar_callback)
            print("✓ LIDAR attached (32 channels)")
            
            self.lidar_viz = LidarVisualizer()
            
        # Camera
        if self.enable_camera and CV2_AVAILABLE:
            camera_bp = blueprint_library.find('sensor.camera.rgb')
            camera_bp.set_attribute('image_size_x', '1280')
            camera_bp.set_attribute('image_size_y', '720')
            camera_bp.set_attribute('fov', '90')
            
            camera_transform = carla.Transform(
                carla.Location(x=2.5, z=0.7),
                carla.Rotation(pitch=0)
            )
            camera = self.world.spawn_actor(camera_bp, camera_transform, attach_to=self.ego_vehicle)
            self.sensors.append(camera)
            
            def camera_callback(image):
                if not self.camera_queue.full():
                    self.camera_queue.put(image)
                    
            camera.listen(camera_callback)
            print("✓ RGB Camera attached (1280x720)")
            
            self.camera_viz = CameraVisualizer()
            
    def setup_spectator(self):
        """Setup spectator camera to follow vehicle."""
        spectator = self.world.get_spectator()
        
        def update_spectator():
            if self.ego_vehicle:
                transform = self.ego_vehicle.get_transform()
                spectator_transform = carla.Transform(
                    transform.location + carla.Location(x=-8, z=4),
                    carla.Rotation(pitch=-15, yaw=transform.rotation.yaw)
                )
                spectator.set_transform(spectator_transform)
                
        return update_spectator
        
    def run(self, duration=300):
        """Run scenario with visualization."""
        print(f"\n▶️  Running visualization for {duration} seconds...")
        print("=" * 60)
        print("Controls:")
        print("  - Mouse: Rotate/pan LIDAR view (in Open3D window)")
        print("  - ESC: Exit")
        print("=" * 60)
        
        self.ego_vehicle.set_autopilot(True, 8000)
        update_spectator = self.setup_spectator()
        
        start_time = time.time()
        frame = 0
        
        try:
            while time.time() - start_time < duration:
                self.world.tick()
                frame += 1
                
                # Update spectator
                if frame % 2 == 0:
                    update_spectator()
                
                # Update LIDAR visualization
                if self.lidar_viz and not self.lidar_queue.empty():
                    lidar_data = self.lidar_queue.get()
                    points = np.frombuffer(lidar_data.raw_data, dtype=np.float32)
                    points = np.reshape(points, (-1, 4))
                    self.lidar_viz.update(points)
                    
                # Update camera visualization
                if self.camera_viz and not self.camera_queue.empty():
                    camera_data = self.camera_queue.get()
                    self.camera_viz.update(camera_data)
                
                # Progress update
                if frame % 100 == 0:
                    elapsed = time.time() - start_time
                    fps = frame / elapsed if elapsed > 0 else 0
                    print(f"[{elapsed:.1f}s] Frame: {frame} | FPS: {fps:.1f}")
                    
        except KeyboardInterrupt:
            print("\n⚠️  Visualization interrupted by user")
            
        elapsed = time.time() - start_time
        print(f"\n✓ Visualization completed! Duration: {elapsed:.1f}s, Frames: {frame}")
        
    def cleanup(self):
        """Clean up all resources."""
        print("\n🧹 Cleaning up...")
        
        # Close visualizers
        if self.lidar_viz:
            self.lidar_viz.close()
        if self.camera_viz:
            self.camera_viz.close()
            
        # Stop sensors
        for sensor in self.sensors:
            if sensor.is_alive:
                sensor.stop()
                sensor.destroy()
                
        # Destroy actors
        self.client.apply_batch([carla.command.DestroyActor(x) for x in self.actor_list])
        
        # Restore settings
        settings = self.world.get_settings()
        settings.synchronous_mode = False
        settings.fixed_delta_seconds = None
        self.world.apply_settings(settings)
        
        print("✓ Cleanup complete")


def main():
    parser = argparse.ArgumentParser(description='Real-time CARLA sensor visualization')
    parser.add_argument('--host', default='192.168.1.110', help='CARLA server host')
    parser.add_argument('--port', type=int, default=2000, help='CARLA server port')
    parser.add_argument('--duration', type=int, default=300, help='Duration in seconds (default: 5 min)')
    parser.add_argument('--no-lidar', action='store_true', help='Disable LIDAR visualization')
    parser.add_argument('--no-camera', action='store_true', help='Disable camera visualization')
    
    args = parser.parse_args()
    
    scenario = VisualizationScenario(
        host=args.host,
        port=args.port,
        enable_lidar=not args.no_lidar,
        enable_camera=not args.no_camera
    )
    
    try:
        scenario.connect()
        scenario.setup_world()
        scenario.spawn_ego_vehicle()
        scenario.attach_sensors()
        
        # Wait for initialization
        print("\n⏱️  Initializing sensors...")
        for _ in range(40):
            scenario.world.tick()
            
        scenario.run(duration=args.duration)
        
    except Exception as e:
        print(f"\n❌ Error: {e}")
        import traceback
        traceback.print_exc()
        
    finally:
        scenario.cleanup()
        print("\n✅ Visualization finished!")


if __name__ == "__main__":
    main()
