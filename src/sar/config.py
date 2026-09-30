"""로봇·지도 공통 상수. 매직 넘버는 이 파일에만 정의."""

# TurtleBot3 Burger (Webots R2025a TurtleBot3Burger.proto)
WHEEL_RADIUS = 0.033  # m
WHEEL_SEPARATION = 0.160  # m
MAX_WHEEL_SPEED = 6.67  # rad/s, RotationalMotor maxVelocity
ROBOT_RADIUS = 0.105  # m, Intro 과정 노트북 핵심 파라미터

# 장치 이름 (Intro 실습 월드 기준)
LIDAR_NAME = "LDS-01"
CAMERA_NAME = "camera"
LEFT_MOTOR_NAME = "left wheel motor"
RIGHT_MOTOR_NAME = "right wheel motor"

# 지도 (허브 좌표·단위 규칙)
GRID_RESOLUTION = 0.05  # m/cell

# 미션 주기
MISSION_HZ = 10
