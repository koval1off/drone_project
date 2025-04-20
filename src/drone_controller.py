import threading
import time
import math
import logging
from dronekit import VehicleMode
from config import (
    RC_NEUTRAL, THROTTLE_TAKEOFF, PITCH_MAX_SPEED, PITCH_MEDIUM_SPEED, PITCH_SLOW_SPEED,
    YAW_MAX_SPEED, YAW_MIN_SPEED, YAW_KP, YAW_TOLERANCE, YAW_TIMEOUT,
    CONTROL_LOOP_DELAY, AUTOCORRECT_LOOP_DELAY, ALTITUDE_TOLERANCE,
    DISTANCE_STOP_THRESHOLD, KP_DISTANCE, METERS_PER_DEGREE_LAT, EARTH_CIRCUMFERENCE
)

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [%(levelname)s] %(message)s',
    datefmt='%Y-%m-%d %H:%M:%S'
)
logger = logging.getLogger(__name__)


class DroneController:
    def __init__(self, vehicle):
        self.vehicle = vehicle
        self.roll = 0
        self.pitch = 0
        self.throttle = 1000
        self.yaw = 0
        self.target_lat = None
        self.target_lon = None
        self.target_alt = None
        self._running_loops = True
        self._autocorrect_target = None
        self._autocorrect_active = False
        self._start_control_threads()

    def _start_control_threads(self):
        """Start background threads for controlling drone channels and autocorrection."""
        threading.Thread(target=self._control_loop, daemon=True).start()
        threading.Thread(target=self._autocorrect_heading_loop, daemon=True).start()

    def _control_loop(self):
        """Continuously update RC channel overrides."""
        while self._running_loops:
            try:
                self.vehicle.channels.overrides = {
                    "1": int(self.roll),
                    "2": int(self.pitch),
                    "3": int(self.throttle),
                    "4": int(self.yaw)
                }
            except Exception as e:
                logger.error(f"Failed to update channels: {e}")
            time.sleep(CONTROL_LOOP_DELAY)

    def _autocorrect_heading_loop(self):
        """Adjust yaw to maintain heading towards the target coordinates."""
        while self._running_loops:
            if self._autocorrect_active and self._autocorrect_target:
                current_yaw = self.get_current_yaw()
                target_yaw = self.calculate_target_yaw(*self._autocorrect_target)
                raw_diff = target_yaw - current_yaw
                yaw_diff = (raw_diff + 180) % 360 - 180

                if abs(yaw_diff) < 0.5:
                    self.yaw = RC_NEUTRAL
                else:
                    self.yaw = self._calculate_turn_speed(yaw_diff)
            time.sleep(AUTOCORRECT_LOOP_DELAY)

    def _calculate_turn_speed(self, yaw_diff, max_speed=YAW_MAX_SPEED, min_speed=YAW_MIN_SPEED, k_p=YAW_KP):
        """Calculate the yaw rotation speed based on the yaw difference."""
        if abs(yaw_diff) < YAW_TOLERANCE:
            return RC_NEUTRAL
        turn_speed = min(max(abs(yaw_diff) * k_p, min_speed), max_speed)
        return RC_NEUTRAL + turn_speed if yaw_diff > 0 else RC_NEUTRAL - turn_speed

    def stop_control(self):
        """Stop all control loops and threads."""
        self._running_loops = False
        logger.info("RC control threads stopped.")

    def arm_and_prepare(self):
        """Arm the drone and set ALT_HOLD mode, ensuring readiness for flight."""
        logger.info("Checking drone readiness for arming...")
        while not self.vehicle.is_armable:
            logger.warning("Drone not ready for arming. Waiting...")
            time.sleep(1)

        logger.info("Setting ALT_HOLD mode...")
        self.vehicle.mode = VehicleMode("ALT_HOLD")
        while self.vehicle.mode.name != "ALT_HOLD":
            logger.info("Waiting for ALT_HOLD mode...")
            time.sleep(1)

        logger.info("Arming the drone...")
        self.vehicle.armed = True
        while not self.vehicle.armed:
            logger.info("Waiting for arming completion...")
            time.sleep(1)

        logger.info("Drone armed and ready for flight!")

    def takeoff_to_altitude(self, target_relative_alt: float):
        """Take off to the specified relative altitude."""
        if not isinstance(target_relative_alt, float):
            logger.error("Altitude must be a float!")
            return
        if target_relative_alt <= 0:
            logger.error("Target altitude must be greater than 0!")
            return

        while True:
            current_alt = self.vehicle.location.global_relative_frame.alt
            logger.info(f"Current altitude: {current_alt:.2f} m")
            if current_alt >= target_relative_alt - ALTITUDE_TOLERANCE:
                self.throttle = RC_NEUTRAL
                logger.info("Target altitude reached.")
                break
            else:
                self.throttle = THROTTLE_TAKEOFF
            time.sleep(0.5)

    def get_current_yaw(self):
        """Get the current yaw of the drone in degrees."""
        yaw_rad = self.vehicle.attitude.yaw
        yaw_deg = math.degrees(yaw_rad)
        return (yaw_deg + 360) % 360

    def calculate_target_yaw(self, target_lat: float, target_lon: float) -> float:
        """Calculate the yaw angle to face the target coordinates."""
        current_location = self.vehicle.location.global_relative_frame
        current_lat = current_location.lat
        current_lon = current_location.lon

        lat1 = math.radians(current_lat)
        lon1 = math.radians(current_lon)
        lat2 = math.radians(target_lat)
        lon2 = math.radians(target_lon)

        d_lon = lon2 - lon1
        x = math.sin(d_lon) * math.cos(lat2)
        y = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(d_lon)
        initial_bearing = math.atan2(x, y)
        initial_bearing_deg = math.degrees(initial_bearing)
        compass_bearing = (initial_bearing_deg + 360) % 360
        return compass_bearing

    def rotate_to_target_yaw(self, target_yaw, tolerance=YAW_TOLERANCE, timeout=YAW_TIMEOUT):
        """Rotate the drone to the specified yaw angle."""
        start_time = time.time()
        logger.info(f"Current yaw: {self.get_current_yaw():.2f}")
        logger.info(f"Target yaw: {target_yaw:.2f}")

        while time.time() - start_time < timeout:
            current_yaw = self.get_current_yaw()
            raw_diff = target_yaw - current_yaw
            yaw_diff = (raw_diff + 180) % 360 - 180

            logger.info(f"Yaw difference: {yaw_diff:.2f}")
            if abs(yaw_diff) <= tolerance:
                self.yaw = RC_NEUTRAL
                logger.info(f"Drone aligned to target yaw. Difference: {yaw_diff:.2f}")
                return True

            self.yaw = self._calculate_turn_speed(yaw_diff)
            time.sleep(CONTROL_LOOP_DELAY)

        logger.error(f"Timeout ({timeout}s) while rotating drone.")
        self.yaw = RC_NEUTRAL
        return False

    def get_distance_to(self, target_lat: float, target_lon: float) -> float:
        """Calculate the horizontal distance to the target coordinates."""
        current_lat = self.vehicle.location.global_relative_frame.lat
        current_lon = self.vehicle.location.global_relative_frame.lon

        delta_lat = (target_lat - current_lat) * METERS_PER_DEGREE_LAT
        delta_lon = (target_lon - current_lon) * EARTH_CIRCUMFERENCE * math.cos(math.radians(current_lat)) / 360
        distance = math.sqrt(delta_lat**2 + delta_lon**2)
        return distance

    def fly_forward_to(self, target_lat: float, target_lon: float):
        """Fly the drone to the target coordinates in ALT_HOLD mode."""
        logger.info("Flying to target coordinates...")
        while self._running_loops:
            distance = self.get_distance_to(target_lat, target_lon)
            logger.info(f"Distance to target: {distance:.2f} m")

            if distance > 30:
                self.pitch = PITCH_MAX_SPEED
            elif distance > 5:
                self.pitch = PITCH_MEDIUM_SPEED
            elif distance > DISTANCE_STOP_THRESHOLD:
                self.pitch = PITCH_SLOW_SPEED
            else:
                logger.info(f"Target reached. Position: {self.get_current_position()}")
                break
            time.sleep(CONTROL_LOOP_DELAY)
        self.pitch = RC_NEUTRAL

    def enable_autocorrect_to_target(self, target_lat, target_lon):
        """Enable yaw autocorrection to face the target coordinates. """
        self._autocorrect_target = (target_lat, target_lon)
        self._autocorrect_active = True
        logger.info("Yaw autocorrection enabled.")

    def disable_autocorrect_to_target(self):
        """Disable yaw autocorrection."""
        self._autocorrect_target = None
        self._autocorrect_active = False
        logger.info("Yaw autocorrection disabled.")

    def land(self):
        """Land the drone by switching to LAND mode."""
        logger.info("Initiating landing...")
        self.vehicle.mode = VehicleMode("LAND")
        while self.vehicle.armed:
            logger.info("Waiting for landing completion...")
            time.sleep(1)
        logger.info("Landing completed.")

    def get_current_position(self):
        """Get the current position of the drone."""
        loc = self.vehicle.location.global_relative_frame
        return loc.lat, loc.lon, loc.alt

