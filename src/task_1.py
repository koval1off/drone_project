from dronekit import connect
from drone_controller import DroneController


target_altitude = 100.0
target_lat = 50.443326
target_lon = 30.448078
target_yaw_final = 350


def main():
    """Execute the drone mission."""
    vehicle = None
    try:
        print("[INFO] Connecting to the drone...")
        vehicle = connect("tcp:127.0.0.1:5762", wait_ready=True)
        drone_controller = DroneController(vehicle)
        drone_controller.arm_and_prepare()
        drone_controller.takeoff_to_altitude(target_altitude)
        drone_controller.rotate_to_target_yaw(drone_controller.calculate_target_yaw(target_lat, target_lon))
        drone_controller.enable_autocorrect_to_target(target_lat, target_lon)
        drone_controller.fly_forward_to(target_lat, target_lon)
        drone_controller.disable_autocorrect_to_target()
        drone_controller.rotate_to_target_yaw(target_yaw_final)
        drone_controller.land()
    except Exception as e:
        print(f"[ERROR] Mission failed: {e}")
    finally:
        if vehicle:
            vehicle.close()
            print("[INFO] Drone connection closed.")

if __name__ == "__main__":
    main()

