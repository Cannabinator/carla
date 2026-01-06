#!/usr/bin/env python3
"""
Quick test of V2V one-line output format
Demonstrates the console output without running full CARLA scenario
"""

from dataclasses import dataclass
import time


@dataclass
class MockBSM:
    """Mock BSM for testing"""
    vehicle_id: int
    speed: float
    heading: float


def format_one_line_status(vehicle_id: int, speed: float, heading: float, 
                           neighbors: int, threats: int, max_threat: int, 
                           msg_count: int) -> str:
    """
    Format one-line V2V status output.
    
    Args:
        vehicle_id: Vehicle identifier
        speed: Speed in m/s
        heading: Heading in degrees (0-360)
        neighbors: Count of neighbor vehicles
        threats: Count of threat vehicles
        max_threat: Highest threat level (0-4)
        msg_count: Total BSM messages sent
    
    Returns:
        Formatted status string
    """
    speed_kmh = speed * 3.6
    
    # Format threat info
    if threats > 0:
        threat_str = f"{threats}(L{max_threat})"
    else:
        threat_str = "0"
    
    return (
        f"V2V: {speed:.1f}m/s ({speed_kmh:.1f}km/h) | "
        f"Heading:{heading:.1f}° | "
        f"Neighbors:{neighbors} | "
        f"Threats:{threat_str} | "
        f"Msgs:{msg_count}"
    )


def demo_output():
    """Demonstrate one-line output format"""
    print("V2V One-Line Output Format Demo")
    print("=" * 120)
    print()
    
    # Scenario 1: Normal driving, no threats
    print("Scenario 1: Normal driving")
    for i in range(5):
        speed = 10.0 + i * 2.0
        heading = 45.0 + i * 5.0
        neighbors = 3
        msg_count = i + 1
        
        status = format_one_line_status(0, speed, heading, neighbors, 0, 0, msg_count)
        print(f"\r{status}", end='')
        time.sleep(0.5)
    print()
    print()
    
    # Scenario 2: Approaching vehicle (increasing threat)
    print("Scenario 2: Approaching vehicle (increasing threat)")
    threat_levels = [0, 1, 2, 3, 4]
    for level in threat_levels:
        speed = 15.0
        heading = 90.0
        neighbors = 4
        threats = 1 if level > 0 else 0
        msg_count = level + 10
        
        status = format_one_line_status(0, speed, heading, neighbors, threats, level, msg_count)
        print(f"\r{status}", end='')
        time.sleep(0.5)
    print()
    print()
    
    # Scenario 3: High-speed with multiple neighbors
    print("Scenario 3: High-speed highway (multiple neighbors)")
    for i in range(5):
        speed = 25.0 + i * 1.0
        heading = 180.0
        neighbors = 5 + i
        threats = 2
        max_threat = 2
        msg_count = i + 20
        
        status = format_one_line_status(0, speed, heading, neighbors, threats, max_threat, msg_count)
        print(f"\r{status}", end='')
        time.sleep(0.5)
    print()
    print()
    
    print("=" * 120)
    print("\nFormat explanation:")
    print("  V2V: <speed_m/s> (<speed_km/h>)")
    print("  Heading: <degrees> (0-360)")
    print("  Neighbors: <count>")
    print("  Threats: <count>(L<max_level>) where level is 0-4")
    print("  Msgs: <total_bsm_count>")
    print()
    print("Threat Levels:")
    print("  L0: No threat (TTC > 5s or distance > 50m)")
    print("  L1: Monitoring (3s < TTC <= 5s)")
    print("  L2: Caution (2s < TTC <= 3s)")
    print("  L3: Warning (1s < TTC <= 2s)")
    print("  L4: Critical (TTC <= 1s)")
    print()


if __name__ == '__main__':
    demo_output()
