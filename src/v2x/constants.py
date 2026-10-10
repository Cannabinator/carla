"""Message priorities, transmission rates, ranges and sharing thresholds."""




# Message priority constants
PRIORITY_ROUTINE = 0


PRIORITY_HIGH = 1


PRIORITY_EMERGENCY = 2


# Transmission rates (Hz)
BSM_RATE_NORMAL = 2  # project default (not a standard's value; see V2XNetwork)


BSM_RATE_HIGH = 10  # nominal BSM rate per SAE J2945/1 (unverified); not used by the simulation


CAM_RATE = 2  # project default; ETSI CAM is 1-10 Hz and event-triggered (unverified); not used


# Communication range (meters)
RANGE_SHORT_M = 50


RANGE_MEDIUM_M = 150


RANGE_LONG_M = 300


# Data sharing thresholds
SHARE_SENSOR_DATA_DISTANCE = 50  # Share sensor data within 50m


SHARE_LIDAR_DISTANCE = 30  # Share LiDAR data within 30m


EMERGENCY_BROADCAST_DISTANCE = 300  # Emergency messages go further
