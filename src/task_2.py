import math

# Вхідні дані
control_point_lat = 50.603694
control_point_lon = 30.650625
azimuth = 335  # Азимут "вгору" у градусах
center_x, center_y = 320, 256  # Центр зображення
point_x_center, point_y_center = 558, 328  # Центр контрольної точки
scale = 0.38  # 1 піксель = 0.38 м

# Крок 1: Зміщення в пікселях
delta_x = center_x - point_x_center  # 320 - 558 = -238
delta_y = center_y - point_y_center  # 256 - 328 = -72

# Крок 2: Зміщення в метрах
delta_x_m = delta_x * scale  # -238 * 0.38 = -90.44
delta_y_m = delta_y * scale  # -72 * 0.38 = -27.36
# Вісь Y зверху вниз, тому для напряму "вгору" (335°) беремо протилежне
delta_y_m = -delta_y_m  # 27.36

# Крок 3: Орієнтація зображення
azimuth_x = (azimuth + 90) % 360  # Вісь X = 335 + 90 = 65°
azimuth_y = azimuth  # Вісь Y = 335°

# Переведення кутів у радіани
azimuth_x_rad = math.radians(azimuth_x)
azimuth_y_rad = math.radians(azimuth_y)

# Компоненти зміщення
delta_lat_x = delta_x_m * math.cos(azimuth_x_rad)  # -90.44 * cos(65°)
delta_lon_x = delta_x_m * math.sin(azimuth_x_rad)  # -90.44 * sin(65°)
delta_lat_y = delta_y_m * math.cos(azimuth_y_rad)  # 27.36 * cos(335°)
delta_lon_y = delta_y_m * math.sin(azimuth_y_rad)  # 27.36 * sin(335°)

# Загальне зміщення в метрах
delta_lat_m = delta_lat_x + delta_lat_y
delta_lon_m = delta_lon_x + delta_lon_y

# Крок 4: Переведення в географічні координати
meters_per_degree_lat = 111320
meters_per_degree_lon = 111320 * math.cos(math.radians(control_point_lat))

delta_lat = delta_lat_m / meters_per_degree_lat
delta_lon = delta_lon_m / meters_per_degree_lon

# Крок 5: Нові координати
center_lat = control_point_lat + delta_lat
center_lon = control_point_lon + delta_lon

print(f"Координати центру зображення: ({center_lat:.6f}, {center_lon:.6f})")
# Результат: (50.603573, 30.650493)

