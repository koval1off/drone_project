import threading
import time
import math
from dronekit import VehicleMode


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
        """Update RC channel overrides every 0.1 seconds."""
        while self._running_loops:
            try:
                self.vehicle.channels.overrides = {
                    "1": int(self.roll),
                    "2": int(self.pitch),
                    "3": int(self.throttle),
                    "4": int(self.yaw)
                }
            except Exception as e:
                print(f"[ERROR] Failed to update channels: {e}")
            time.sleep(0.1)

    def _autocorrect_heading_loop(self):
        """Adjust yaw to maintain heading towards the target coordinates every 0.1 seconds."""
        while self._running_loops:
            if self._autocorrect_active and self._autocorrect_target:
                current_yaw = self.get_current_yaw()
                target_yaw = self.calculate_target_yaw(*self._autocorrect_target)
                raw_diff = target_yaw - current_yaw
                yaw_diff = (raw_diff + 180) % 360 - 180

                if abs(yaw_diff) < 0.5:
                    self.yaw = 1500
                else:
                    self.yaw = self._calculate_turn_speed(yaw_diff)
            time.sleep(0.1)

    def _calculate_turn_speed(self, yaw_diff, max_speed=300, min_speed=25, k_p=0.7):
        """Calculate the yaw rotation speed."""
        if abs(yaw_diff) < 0.1:
            return 1500
        turn_speed = min(max(abs(yaw_diff) * k_p, min_speed), max_speed)
        return 1500 + turn_speed if yaw_diff > 0 else 1500 - turn_speed

    def stop_control(self):
        """Stop all control loops and threads."""
        self._running_loops = False
        print("[INFO] RC control threads stopped.")

    def arm_and_prepare(self):
        """Arm the drone and set ALT_HOLD mode, ready to flight check."""
        print("[INFO] Checking drone readiness for arming...")
        while not self.vehicle.is_armable:
            print("[WARN] Drone not ready for arming. Waiting...", end="\r")
            time.sleep(1)

        print("[INFO] Setting ALT_HOLD mode...")
        self.vehicle.mode = VehicleMode("ALT_HOLD")
        while self.vehicle.mode.name != "ALT_HOLD":
            print("[INFO] Waiting for ALT_HOLD mode...")
            time.sleep(1)

        print("[INFO] Arming the drone...")
        self.vehicle.armed = True
        while not self.vehicle.armed:
            print("[INFO] Waiting for arming completion...")
            time.sleep(1)

        print("[INFO] Drone armed and ready for flight!")

    def takeoff_to_altitude(self, target_relative_alt: float):
        """Take off to the specified relative altitude."""
        if not isinstance(target_relative_alt, float):
            print("[ERROR] Altitude must be a float!")
            return
        if target_relative_alt <= 0:
            print("[ERROR] Target altitude must be greater than 0!")
            return

        while True:
            current_alt = self.vehicle.location.global_relative_frame.alt
            print(f"[INFO] Current altitude: {current_alt:.2f} m", end="\r")
            if current_alt >= target_relative_alt - 1:
                self.throttle = 1500
                print("[INFO] Target altitude reached.")
                break
            else:
                self.throttle = 1900
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

    def rotate_to_target_yaw(self, target_yaw, tolerance=0.1, timeout=200):
        """Rotate the drone to the specified yaw angle."""
        start_time = time.time()
        print(f"[INFO] Current yaw: {self.get_current_yaw():.2f}")
        print(f"[INFO] Target yaw: {target_yaw:.2f}")

        while time.time() - start_time < timeout:
            current_yaw = self.get_current_yaw()
            raw_diff = target_yaw - current_yaw
            yaw_diff = (raw_diff + 180) % 360 - 180

            print(f"[INFO] Yaw difference: {yaw_diff:.2f}", end="\r")
            if abs(yaw_diff) <= tolerance:
                self.yaw = 1500
                print(f"[INFO] Drone aligned to target yaw. Difference: {yaw_diff:.2f}")
                return True

            self.yaw = self._calculate_turn_speed(yaw_diff)
            time.sleep(0.1)

        print(f"[ERROR] Timeout ({timeout}s) while rotating drone.")
        self.yaw = 1500
        return False

    def get_distance_to(self, target_lat: float, target_lon: float) -> float:
        """Calculate the horizontal distance to the target coordinates."""
        current_lat = self.vehicle.location.global_relative_frame.lat
        current_lon = self.vehicle.location.global_relative_frame.lon

        delta_lat = (target_lat - current_lat) * 111320
        delta_lon = (target_lon - current_lon) * 40075000 * math.cos(math.radians(current_lat)) / 360
        distance = math.sqrt(delta_lat**2 + delta_lon**2)
        return distance

    def fly_forward_to(self, target_lat, target_lon, max_speed=1200, min_speed=1480, k_p=1, stop_threshold=0.1):
        """Fly the drone to the target coordinates."""
        print("[INFO] Flying to target coordinates...")
        while self._running_loops:
            distance = self.get_distance_to(target_lat, target_lon)
            print(f"[INFO] Distance to target: {distance:.2f} m", end="\r")

            if distance > 30:
                self.pitch = max_speed
            elif distance > 5:
                self.pitch = 1470
            elif distance > 0.5:
                self.pitch = 1477
            else:
                print(f"[INFO] Target reached. Position: {self.get_current_position()}")
                break
            time.sleep(0.1)
        self.pitch = 1500

    def enable_autocorrect_to_target(self, target_lat, target_lon):
        """Enable yaw autocorrection."""
        self._autocorrect_target = (target_lat, target_lon)
        self._autocorrect_active = True

    def disable_autocorrect_to_target(self):
        """Disable yaw autocorrection."""
        self._autocorrect_target = None
        self._autocorrect_active = False

    def land(self):
        """Land the drone by switching to LAND mode."""
        print("[INFO] Initiating landing...")
        self.vehicle.mode = VehicleMode("LAND")
        while self.vehicle.armed:
            print("[INFO] Waiting for landing completion...", end="\r")
            time.sleep(1)
        print("[INFO] Landing completed.")

    def get_current_position(self):
        """Get the current position of the drone."""
        loc = self.vehicle.location.global_relative_frame
        return loc.lat, loc.lon, loc.alt

