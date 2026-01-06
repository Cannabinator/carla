#!/usr/bin/env python3
"""
Comprehensive Frontend Testing Suite for LiDAR Viewer
Tests camera controls, WebSocket connectivity, rendering, and user interactions.
"""

import asyncio
import json
import time
from pathlib import Path
from typing import Dict, List, Optional
import unittest
import sys

# Add parent directory to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent))

try:
    from selenium import webdriver
    from selenium.webdriver.common.by import By
    from selenium.webdriver.common.keys import Keys
    from selenium.webdriver.common.action_chains import ActionChains
    from selenium.webdriver.support.ui import WebDriverWait
    from selenium.webdriver.support import expected_conditions as EC
    from selenium.common.exceptions import TimeoutException
    SELENIUM_AVAILABLE = True
except ImportError:
    SELENIUM_AVAILABLE = False
    print("⚠️  Selenium not installed. Install with: pip install selenium")
    print("⚠️  Also install Chrome/Firefox driver for full frontend testing")


class FrontendTestBase(unittest.TestCase):
    """Base class for frontend tests with common setup/teardown"""
    
    @classmethod
    def setUpClass(cls):
        """Setup browser and load viewer page"""
        if not SELENIUM_AVAILABLE:
            cls.skipTest(cls, "Selenium not available")
        
        try:
            # Try Chrome first
            options = webdriver.ChromeOptions()
            options.add_argument('--headless')  # Run in background
            options.add_argument('--no-sandbox')
            options.add_argument('--disable-dev-shm-usage')
            cls.driver = webdriver.Chrome(options=options)
        except Exception:
            try:
                # Fallback to Firefox
                options = webdriver.FirefoxOptions()
                options.add_argument('--headless')
                cls.driver = webdriver.Firefox(options=options)
            except Exception as e:
                cls.skipTest(cls, f"No browser driver available: {e}")
        
        cls.driver.set_window_size(1920, 1080)
        cls.base_url = "http://localhost:8000"
        cls.wait = WebDriverWait(cls.driver, 10)
    
    @classmethod
    def tearDownClass(cls):
        """Cleanup browser"""
        if hasattr(cls, 'driver'):
            cls.driver.quit()
    
    def load_viewer(self):
        """Load the viewer page"""
        self.driver.get(f"{self.base_url}/viewer.html")
        # Wait for Three.js to load
        self.wait.until(
            lambda d: d.execute_script("return typeof THREE !== 'undefined'")
        )


class CameraControlTests(FrontendTestBase):
    """Test camera control modes and interactions"""
    
    def setUp(self):
        """Load viewer before each test"""
        self.load_viewer()
    
    def test_initial_camera_position(self):
        """Test camera starts at correct default position"""
        camera_pos = self.driver.execute_script("""
            return {
                x: camera.position.x,
                y: camera.position.y,
                z: camera.position.z
            };
        """)
        
        self.assertAlmostEqual(camera_pos['x'], -25, delta=1)
        self.assertAlmostEqual(camera_pos['y'], 0, delta=1)
        self.assertAlmostEqual(camera_pos['z'], 20, delta=1)
    
    def test_reset_camera_button(self):
        """Test reset camera button returns to default position"""
        # Move camera away
        self.driver.execute_script("camera.position.set(100, 100, 100);")
        
        # Click reset button
        reset_btn = self.wait.until(
            EC.element_to_be_clickable((By.XPATH, "//button[contains(text(), 'Reset Camera')]"))
        )
        reset_btn.click()
        
        time.sleep(0.5)  # Wait for animation
        
        # Check position reset
        camera_pos = self.driver.execute_script("""
            return {
                x: camera.position.x,
                y: camera.position.y,
                z: camera.position.z
            };
        """)
        
        self.assertAlmostEqual(camera_pos['x'], -25, delta=1)
        self.assertAlmostEqual(camera_pos['y'], 0, delta=1)
        self.assertAlmostEqual(camera_pos['z'], 20, delta=1)
    
    def test_follow_mode_toggle(self):
        """Test follow mode can be enabled and disabled"""
        follow_btn = self.wait.until(
            EC.element_to_be_clickable((By.ID, "follow-btn"))
        )
        
        # Initially disabled
        initial_text = follow_btn.text
        self.assertIn("Enable", initial_text)
        
        # Enable follow mode
        follow_btn.click()
        time.sleep(0.2)
        
        enabled_text = follow_btn.text
        self.assertIn("Disable", enabled_text)
        
        # Check JavaScript state
        follow_mode = self.driver.execute_script("return followMode;")
        self.assertTrue(follow_mode)
        
        # Disable follow mode
        follow_btn.click()
        time.sleep(0.2)
        
        disabled_text = follow_btn.text
        self.assertIn("Enable", disabled_text)
        
        follow_mode = self.driver.execute_script("return followMode;")
        self.assertFalse(follow_mode)
    
    def test_freefly_mode_toggle(self):
        """Test free-fly mode can be enabled and disabled"""
        freefly_btn = self.wait.until(
            EC.element_to_be_clickable((By.ID, "freefly-btn"))
        )
        
        # Initially disabled
        initial_text = freefly_btn.text
        self.assertIn("Enable", initial_text)
        
        # Enable free-fly mode
        freefly_btn.click()
        time.sleep(0.2)
        
        enabled_text = freefly_btn.text
        self.assertIn("Disable", enabled_text)
        
        # Check JavaScript state
        freefly_mode = self.driver.execute_script("return freeFlyMode;")
        self.assertTrue(freefly_mode)
    
    def test_control_mode_exclusivity(self):
        """Test that only one control mode is active at a time"""
        # Enable follow mode
        follow_btn = self.driver.find_element(By.ID, "follow-btn")
        follow_btn.click()
        time.sleep(0.2)
        
        modes = self.driver.execute_script("""
            return {
                follow: followMode,
                freefly: freeFlyMode,
                orbit: orbitControls.enabled
            };
        """)
        
        self.assertTrue(modes['follow'])
        self.assertFalse(modes['freefly'])
        self.assertFalse(modes['orbit'])
        
        # Switch to free-fly mode
        freefly_btn = self.driver.find_element(By.ID, "freefly-btn")
        freefly_btn.click()
        time.sleep(0.2)
        
        modes = self.driver.execute_script("""
            return {
                follow: followMode,
                freefly: freeFlyMode,
                orbit: orbitControls.enabled
            };
        """)
        
        self.assertFalse(modes['follow'])
        self.assertTrue(modes['freefly'])
        self.assertFalse(modes['orbit'])
    
    def test_orbit_controls_work(self):
        """Test orbit controls are functional in default mode"""
        # Ensure we're in orbit mode
        self.driver.execute_script("""
            followMode = false;
            freeFlyMode = false;
            orbitControls.enabled = true;
        """)
        
        # Get initial camera position
        initial_pos = self.driver.execute_script("""
            return {
                x: camera.position.x,
                y: camera.position.y,
                z: camera.position.z
            };
        """)
        
        # Simulate mouse drag (orbit)
        canvas = self.driver.find_element(By.TAG_NAME, "canvas")
        actions = ActionChains(self.driver)
        actions.move_to_element(canvas)
        actions.click_and_hold()
        actions.move_by_offset(100, 0)
        actions.release()
        actions.perform()
        
        time.sleep(0.5)  # Wait for animation
        
        # Camera should have moved
        final_pos = self.driver.execute_script("""
            return {
                x: camera.position.x,
                y: camera.position.y,
                z: camera.position.z
            };
        """)
        
        # Position should be different (at least one coordinate changed significantly)
        position_changed = (
            abs(final_pos['x'] - initial_pos['x']) > 1 or
            abs(final_pos['y'] - initial_pos['y']) > 1 or
            abs(final_pos['z'] - initial_pos['z']) > 1
        )
        self.assertTrue(position_changed, "Camera position should change with orbit controls")


class PointCloudRenderingTests(FrontendTestBase):
    """Test point cloud rendering and visibility"""
    
    def setUp(self):
        """Load viewer before each test"""
        self.load_viewer()
    
    def test_point_cloud_exists(self):
        """Test point cloud object is created"""
        point_cloud = self.driver.execute_script("return pointCloud;")
        self.assertIsNotNone(point_cloud)
    
    def test_point_size_toggle(self):
        """Test point size cycles correctly"""
        size_btn = self.wait.until(
            EC.element_to_be_clickable((By.ID, "size-btn"))
        )
        
        # Initial size: 0.5
        initial_size = self.driver.execute_script("return pointCloud.material.size;")
        self.assertAlmostEqual(initial_size, 0.5, delta=0.01)
        
        # Click -> 0.5 -> 1.0
        size_btn.click()
        time.sleep(0.1)
        size = self.driver.execute_script("return pointCloud.material.size;")
        self.assertAlmostEqual(size, 1.0, delta=0.01)
        
        # Click -> 1.0 -> 0.2
        size_btn.click()
        time.sleep(0.1)
        size = self.driver.execute_script("return pointCloud.material.size;")
        self.assertAlmostEqual(size, 0.2, delta=0.01)
        
        # Click -> 0.2 -> 0.5
        size_btn.click()
        time.sleep(0.1)
        size = self.driver.execute_script("return pointCloud.material.size;")
        self.assertAlmostEqual(size, 0.5, delta=0.01)
    
    def test_point_material_settings(self):
        """Test point cloud material has correct settings for visibility"""
        material_props = self.driver.execute_script("""
            return {
                size: pointCloud.material.size,
                vertexColors: pointCloud.material.vertexColors,
                sizeAttenuation: pointCloud.material.sizeAttenuation,
                transparent: pointCloud.material.transparent,
                depthTest: pointCloud.material.depthTest,
                depthWrite: pointCloud.material.depthWrite
            };
        """)
        
        self.assertEqual(material_props['size'], 0.5)
        self.assertTrue(material_props['vertexColors'])
        self.assertTrue(material_props['sizeAttenuation'])
        self.assertFalse(material_props['transparent'])
        self.assertTrue(material_props['depthTest'])
        self.assertTrue(material_props['depthWrite'])
    
    def test_grid_toggle(self):
        """Test grid visibility can be toggled"""
        # Grid should be visible initially
        grid_visible = self.driver.execute_script("return gridHelper.visible;")
        self.assertTrue(grid_visible)
        
        # Toggle off
        toggle_btn = self.driver.find_element(By.XPATH, "//button[contains(text(), 'Toggle Grid')]")
        toggle_btn.click()
        time.sleep(0.1)
        
        grid_visible = self.driver.execute_script("return gridHelper.visible;")
        self.assertFalse(grid_visible)
        
        # Toggle back on
        toggle_btn.click()
        time.sleep(0.1)
        
        grid_visible = self.driver.execute_script("return gridHelper.visible;")
        self.assertTrue(grid_visible)


class WebSocketConnectionTests(FrontendTestBase):
    """Test WebSocket connection handling"""
    
    def setUp(self):
        """Load viewer before each test"""
        self.load_viewer()
    
    def test_websocket_initialization(self):
        """Test WebSocket is initialized"""
        # Wait for WebSocket to be created
        time.sleep(1)
        
        ws_state = self.driver.execute_script("""
            return {
                exists: typeof ws !== 'undefined' && ws !== null,
                readyState: ws ? ws.readyState : null
            };
        """)
        
        self.assertTrue(ws_state['exists'], "WebSocket should be initialized")
        # readyState: 0=CONNECTING, 1=OPEN, 2=CLOSING, 3=CLOSED
        self.assertIn(ws_state['readyState'], [0, 1, 3])  # Valid states
    
    def test_status_indicator_exists(self):
        """Test connection status indicator is present"""
        status_indicator = self.wait.until(
            EC.presence_of_element_located((By.ID, "status-indicator"))
        )
        self.assertIsNotNone(status_indicator)
    
    def test_reconnection_attempt(self):
        """Test automatic reconnection on disconnect"""
        # Simulate disconnect
        self.driver.execute_script("""
            if (ws && ws.readyState === 1) {
                ws.close();
            }
        """)
        
        time.sleep(0.5)
        
        # Check loading message
        loading_element = self.driver.find_element(By.ID, "loading")
        loading_text = loading_element.text
        
        # Should show reconnecting message
        self.assertIn("Reconnecting", loading_text)


class UIElementTests(FrontendTestBase):
    """Test UI elements and layout"""
    
    def setUp(self):
        """Load viewer before each test"""
        self.load_viewer()
    
    def test_all_control_buttons_present(self):
        """Test all control buttons are present"""
        buttons = {
            "Reset Camera": "//button[contains(text(), 'Reset Camera')]",
            "Follow": "//button[@id='follow-btn']",
            "Free-Fly": "//button[@id='freefly-btn']",
            "Point Size": "//button[@id='size-btn']",
            "Toggle Grid": "//button[contains(text(), 'Toggle Grid')]"
        }
        
        for name, xpath in buttons.items():
            with self.subTest(button=name):
                button = self.wait.until(
                    EC.presence_of_element_located((By.XPATH, xpath))
                )
                self.assertIsNotNone(button, f"{name} button not found")
    
    def test_stats_display_present(self):
        """Test statistics display elements are present"""
        stats_elements = ['fps', 'point-count', 'vehicle-count', 'latency']
        
        for element_id in stats_elements:
            with self.subTest(element=element_id):
                element = self.wait.until(
                    EC.presence_of_element_located((By.ID, element_id))
                )
                self.assertIsNotNone(element, f"{element_id} element not found")
    
    def test_legend_present(self):
        """Test semantic color legend is present"""
        legend = self.wait.until(
            EC.presence_of_element_located((By.CLASS_NAME, "legend"))
        )
        self.assertIsNotNone(legend)
        
        # Check for some legend items
        legend_html = legend.get_attribute('innerHTML')
        self.assertIn("Vehicle", legend_html)
        self.assertIn("Pedestrian", legend_html)
        self.assertIn("Road", legend_html)


class PerformanceTests(FrontendTestBase):
    """Test rendering performance and responsiveness"""
    
    def setUp(self):
        """Load viewer before each test"""
        self.load_viewer()
    
    def test_animation_loop_running(self):
        """Test animation loop is running"""
        time.sleep(2)  # Let animation run
        
        fps = self.driver.execute_script("return parseInt(document.getElementById('fps').textContent);")
        self.assertGreater(fps, 0, "FPS should be greater than 0")
        self.assertLess(fps, 200, "FPS should be reasonable (< 200)")
    
    def test_window_resize_handling(self):
        """Test renderer handles window resize"""
        initial_size = self.driver.execute_script("""
            return {
                width: renderer.domElement.width,
                height: renderer.domElement.height
            };
        """)
        
        # Resize window
        self.driver.set_window_size(1280, 720)
        time.sleep(0.5)
        
        new_size = self.driver.execute_script("""
            return {
                width: renderer.domElement.width,
                height: renderer.domElement.height
            };
        """)
        
        # Size should have changed
        self.assertNotEqual(initial_size, new_size)


class KeyboardControlTests(FrontendTestBase):
    """Test keyboard controls for free-fly mode"""
    
    def setUp(self):
        """Load viewer and enable free-fly mode"""
        self.load_viewer()
        
        # Enable free-fly mode
        freefly_btn = self.wait.until(
            EC.element_to_be_clickable((By.ID, "freefly-btn"))
        )
        freefly_btn.click()
        time.sleep(0.2)
    
    def test_wasd_movement_registered(self):
        """Test WASD keys register in movement state"""
        canvas = self.driver.find_element(By.TAG_NAME, "canvas")
        
        # Press W key
        actions = ActionChains(self.driver)
        actions.move_to_element(canvas)
        actions.send_keys('w')
        actions.perform()
        
        time.sleep(0.1)
        
        # Note: We can't easily test actual movement without pointer lock,
        # but we can verify the mode is active
        freefly_active = self.driver.execute_script("return freeFlyMode;")
        self.assertTrue(freefly_active)


def run_frontend_tests(verbose=True):
    """
    Run all frontend tests
    
    Args:
        verbose: Print detailed test output
    
    Returns:
        bool: True if all tests passed
    """
    if not SELENIUM_AVAILABLE:
        print("❌ Cannot run frontend tests - Selenium not installed")
        print("   Install with: pip install selenium")
        print("   Also install Chrome/Firefox driver")
        return False
    
    # Create test suite
    loader = unittest.TestLoader()
    suite = unittest.TestSuite()
    
    # Add all test classes
    suite.addTests(loader.loadTestsFromTestCase(CameraControlTests))
    suite.addTests(loader.loadTestsFromTestCase(PointCloudRenderingTests))
    suite.addTests(loader.loadTestsFromTestCase(WebSocketConnectionTests))
    suite.addTests(loader.loadTestsFromTestCase(UIElementTests))
    suite.addTests(loader.loadTestsFromTestCase(PerformanceTests))
    suite.addTests(loader.loadTestsFromTestCase(KeyboardControlTests))
    
    # Run tests
    verbosity = 2 if verbose else 1
    runner = unittest.TextTestRunner(verbosity=verbosity)
    result = runner.run(suite)
    
    # Print summary
    print("\n" + "="*70)
    print("FRONTEND TEST SUMMARY")
    print("="*70)
    print(f"Tests run: {result.testsRun}")
    print(f"Successes: {result.testsRun - len(result.failures) - len(result.errors)}")
    print(f"Failures: {len(result.failures)}")
    print(f"Errors: {len(result.errors)}")
    print(f"Skipped: {len(result.skipped)}")
    print("="*70)
    
    return result.wasSuccessful()


if __name__ == "__main__":
    import argparse
    
    parser = argparse.ArgumentParser(description="Frontend testing suite for LiDAR viewer")
    parser.add_argument('-v', '--verbose', action='store_true', help='Verbose output')
    parser.add_argument('--test', type=str, help='Run specific test class')
    
    args = parser.parse_args()
    
    if args.test:
        # Run specific test class
        suite = unittest.TestLoader().loadTestsFromName(f"__main__.{args.test}")
        runner = unittest.TextTestRunner(verbosity=2 if args.verbose else 1)
        result = runner.run(suite)
        sys.exit(0 if result.wasSuccessful() else 1)
    else:
        # Run all tests
        success = run_frontend_tests(verbose=args.verbose)
        sys.exit(0 if success else 1)
