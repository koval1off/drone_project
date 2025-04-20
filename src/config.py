# Configuration constants for DroneController

# RC channel values
RC_NEUTRAL = 1500  # Neutral position for throttle, yaw, pitch, roll
THROTTLE_TAKEOFF = 1900  # Throttle value for takeoff
PITCH_MAX_SPEED = 1200  # Maximum pitch speed for long-distance flight
PITCH_MEDIUM_SPEED = 1470  # Pitch speed for medium-distance flight
PITCH_SLOW_SPEED = 1477  # Pitch speed for short-distance flight

# Yaw control
YAW_MAX_SPEED = 300  # Maximum yaw rotation speed
YAW_MIN_SPEED = 25  # Minimum yaw rotation speed
YAW_KP = 0.7  # Proportional gain for yaw control
YAW_TOLERANCE = 0.1  # Acceptable yaw error in degrees
YAW_TIMEOUT = 200  # Timeout for yaw rotation in seconds

# Loop delays
CONTROL_LOOP_DELAY = 0.1  # Delay for RC channel update loop (seconds)
AUTOCORRECT_LOOP_DELAY = 0.1  # Delay for yaw autocorrection loop (seconds)

# Flight parameters
ALTITUDE_TOLERANCE = 1.0  # Acceptable altitude error in meters
DISTANCE_STOP_THRESHOLD = 0.5  # Distance threshold to stop flight in meters
KP_DISTANCE = 1.0  # Proportional gain for distance-based speed control

# Geographic conversions
METERS_PER_DEGREE_LAT = 111320  # Meters per degree of latitude
EARTH_CIRCUMFERENCE = 40075000  # Earth's circumference in meters