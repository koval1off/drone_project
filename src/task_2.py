import math

# Input data
control_point_lat = 50.603694
control_point_lon = 30.650625
azimuth = 335  # Azimuth "up" in degrees
center_x, center_y = 320, 256  # Image center
point_x_center, point_y_center = 558, 328  # Control point center
scale = 0.38  # 1 pixel = 0.38 meters

# Step 1: Calculate pixel offsets
delta_x = center_x - point_x_center  # 320 - 558 = -238
delta_y = center_y - point_y_center  # 256 - 328 = -72

# Step 2: Convert pixel offsets to meters
delta_x_m = delta_x * scale  # -238 * 0.38 = -90.44
delta_y_m = delta_y * scale  # -72 * 0.38 = -27.36
# Y-axis is top-down, so negate for "up" direction (335°)
delta_y_m = -delta_y_m  # 27.36

# Step 3: Account for image orientation
azimuth_x = (azimuth + 90) % 360  # X-axis: 335 + 90 = 65°
azimuth_y = azimuth  # Y-axis: 335°

# Convert angles to radians
azimuth_x_rad = math.radians(azimuth_x)
azimuth_y_rad = math.radians(azimuth_y)

# Calculate offset components
delta_lat_x = delta_x_m * math.cos(azimuth_x_rad)  # -90.44 * cos(65°)
delta_lon_x = delta_x_m * math.sin(azimuth_x_rad)  # -90.44 * sin(65°)
delta_lat_y = delta_y_m * math.cos(azimuth_y_rad)  # 27.36 * cos(335°)
delta_lon_y = delta_y_m * math.sin(azimuth_y_rad)  # 27.36 * sin(335°)

# Total offset in meters
delta_lat_m = delta_lat_x + delta_lat_y
delta_lon_m = delta_lon_x + delta_lon_y

# Step 4: Convert to geographic coordinates
meters_per_degree_lat = 111320
meters_per_degree_lon = 111320 * math.cos(math.radians(control_point_lat))

delta_lat = delta_lat_m / meters_per_degree_lat
delta_lon = delta_lon_m / meters_per_degree_lon

# Step 5: Calculate final coordinates
center_lat = control_point_lat + delta_lat
center_lon = control_point_lon + delta_lon

print(f"Image center coordinates: ({center_lat:.6f}, {center_lon:.6f})")

