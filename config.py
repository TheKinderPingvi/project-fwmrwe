# config.py

# Основные редактируемые параметры необходимые для миссии

# Зона сканирования
SCAN_AREA_X = 1.5
X_TOLERANCE = 0.5
SCAN_Y_MIN = -0.8
SCAN_Y_MAX = 10.8
SCAN_HEIGHT = 2.85
SCAN_START_DELAY = 2.0

# Кабели (несущие тросы)
CABLE_HEIGHT_TOL = 0.07
EXPECTED_HIGH_CABLE_HEIGHT = 1.55 
EXPECTED_LOW_CABLE_HEIGHT = 1.25 
MIN_Z = 0.5

# Консоли
CONSOLE_HEIGHT = 1.8
CONSOLE_HEIGHT_TOL = 0.15
CONSOLE_CLUSTER_THRESH = 0.05
CONSOLE_MIN_POINTS = 5

# Размеры дрона (из документации)
DRONE_LENGTH = 0.355  # м
DRONE_WIDTH = 0.355   # м
DRONE_HEIGHT = 0.195  # м

# Обычно не редактируемые параметры

# Безопасные расстояния
SAFE_DIST = 0.3
CROSSING_OFFSET = 0.78
TARGET_HEIGHT_ABOVE = 0.15
SAFETY_MARGIN = 0.5

# Навигация
DIST_TOL = 0.1
WAYPOINT_STEP = 0.1
SPEED_HORIZONTAL = 0.3
SPEED_VERTICAL = 0.1
SPEED_ASCENT = 0.4

# Для live-коррекции 
LIVE_RADIUS = 0.35      
LIVE_TIMEOUT = 0.2     
MAX_BUFFER_AGE = 1.0    