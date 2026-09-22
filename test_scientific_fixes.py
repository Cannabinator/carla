#!/usr/bin/env python3
"""Test script to validate the scientific accuracy fixes."""

from src.v2v.dsrc_channel import DSRCChannel, DSRCConfig
from src.utils.potholes import PotholeManager

def test_dsrc_channel():
    """Test the new DSRC channel model."""
    print("=" * 70)
    print("TEST 1: DSRC Channel Model - CSMA/CA Contention")
    print("=" * 70)
    
    cfg = DSRCConfig()
    channel = DSRCChannel(cfg)
    
    print("\nTesting CSMA/CA contention model:")
    test_cases = [
        (1, "Single vehicle (no contention)"),
        (5, "Light traffic"),
        (15, "Moderate traffic"),
        (20, "Dense traffic"),
        (50, "Heavy traffic"),
        (100, "Very heavy traffic"),
        (200, "Saturation"),
    ]
    
    all_passed = True
    for n_vehicles, description in test_cases:
        p_csma = channel._csma_success_prob(n_vehicles)
        print(f"  N={n_vehicles:3d} ({description:25s}) → P_csma = {p_csma:.3f}")
        
        # Validate expected behavior
        if n_vehicles <= 20 and p_csma < 0.90:
            print(f"    ⚠️  WARNING: Expected PRR > 0.90 for N ≤ 20")
            all_passed = False
        if n_vehicles == 50 and not (0.75 < p_csma < 0.82):
            print(f"    ⚠️  WARNING: Expected PRR ≈ 0.78 for N ≈ 50")
            all_passed = False
        if n_vehicles >= 200 and p_csma < 0.35:
            print(f"    ⚠️  WARNING: Expected PRR floor at 0.35")
            all_passed = False
    
    print("\nRF Model Parameters:")
    print(f"  Path loss exponent (LOS):  {cfg.path_loss_exponent_los} (was 2.0)")
    print(f"  Path loss exponent (NLOS): {cfg.path_loss_exponent_nlos} (was 2.75)")
    print(f"  Shadowing std (LOS):       {cfg.shadowing_std_los_db} dB (was 4.0)")
    print(f"  Shadowing std (NLOS):      {cfg.shadowing_std_nlos_db} dB (was 6.0)")
    
    if all_passed:
        print("\n✅ DSRC Channel Model: PASSED")
    else:
        print("\n❌ DSRC Channel Model: FAILED")
    
    return all_passed


def test_pothole_manager():
    """Test the new PotholeManager manual registration mode."""
    print("\n" + "=" * 70)
    print("TEST 2: PotholeManager - Manual Registration Mode")
    print("=" * 70)
    
    mgr = PotholeManager()
    
    # Test 1: Initial state should be empty
    if len(mgr) != 0:
        print(f"❌ FAILED: Expected 0 potholes initially, got {len(mgr)}")
        return False
    print("✓ Initial state: 0 potholes (no auto-generation)")
    
    # Test 2: Add potholes manually
    p1 = mgr.add_pothole(x=100.0, y=50.0, z=0.1, severity=2, radius=0.35)
    p2 = mgr.add_pothole(x=150.0, y=75.0, z=0.15, severity=3)
    p3 = mgr.add_pothole(x=200.0, y=100.0, z=0.08, severity=1)
    
    if len(mgr) != 3:
        print(f"❌ FAILED: Expected 3 potholes after adding, got {len(mgr)}")
        return False
    print(f"✓ Manual registration: Added 3 potholes")
    
    # Test 3: Spatial query
    nearby = mgr.get_nearby(140.0, 70.0, radius=30.0)
    if len(nearby) != 1:
        print(f"❌ FAILED: Expected 1 nearby pothole, got {len(nearby)}")
        return False
    print(f"✓ Spatial query: Found {len(nearby)} pothole(s) within 30m")
    
    # Test 4: Serialization
    pothole_dicts = mgr.to_dict_list()
    if len(pothole_dicts) != 3:
        print(f"❌ FAILED: Expected 3 serialized potholes, got {len(pothole_dicts)}")
        return False
    print(f"✓ Serialization: {len(pothole_dicts)} potholes serialized")
    
    # Test 5: Verify no POTHOLE_SEED constant exists
    if hasattr(mgr, 'POTHOLE_SEED'):
        print(f"❌ FAILED: POTHOLE_SEED constant should be removed")
        return False
    print("✓ No auto-generation seed constant (as expected)")
    
    print("\n✅ PotholeManager: PASSED")
    return True


def test_config_consistency():
    """Test that config.py and dsrc_channel.py have matching defaults."""
    print("\n" + "=" * 70)
    print("TEST 3: Configuration Consistency")
    print("=" * 70)
    
    from src.config import DEFAULT_DSRC_CONFIG
    from src.v2v.dsrc_channel import DSRCConfig
    
    cfg1 = DEFAULT_DSRC_CONFIG
    cfg2 = DSRCConfig()
    
    checks = [
        ('path_loss_exponent_los', cfg1.path_loss_exponent_los, cfg2.path_loss_exponent_los),
        ('path_loss_exponent_nlos', cfg1.path_loss_exponent_nlos, cfg2.path_loss_exponent_nlos),
        ('shadowing_std_los_db', cfg1.shadowing_std_los_db, cfg2.shadowing_std_los_db),
        ('shadowing_std_nlos_db', cfg1.shadowing_std_nlos_db, cfg2.shadowing_std_nlos_db),
        ('channel_busy_ratio', cfg1.channel_busy_ratio, cfg2.channel_busy_ratio),
    ]
    
    all_match = True
    for name, val1, val2 in checks:
        if val1 != val2:
            print(f"❌ MISMATCH: {name}: config.py={val1}, dsrc_channel.py={val2}")
            all_match = False
        else:
            print(f"✓ {name}: {val1}")
    
    if all_match:
        print("\n✅ Configuration Consistency: PASSED")
    else:
        print("\n❌ Configuration Consistency: FAILED")
    
    return all_match


if __name__ == "__main__":
    print("\n" + "=" * 70)
    print("SCIENTIFIC ACCURACY FIXES - VALIDATION TEST SUITE")
    print("=" * 70)
    
    results = []
    results.append(("DSRC Channel Model", test_dsrc_channel()))
    results.append(("PotholeManager", test_pothole_manager()))
    results.append(("Config Consistency", test_config_consistency()))
    
    print("\n" + "=" * 70)
    print("TEST SUMMARY")
    print("=" * 70)
    for name, passed in results:
        status = "✅ PASSED" if passed else "❌ FAILED"
        print(f"{status:12s} - {name}")
    
    all_passed = all(passed for _, passed in results)
    print("=" * 70)
    if all_passed:
        print("✅ ALL TESTS PASSED - Ready to commit")
    else:
        print("❌ SOME TESTS FAILED - Requires rework")
    print("=" * 70)
