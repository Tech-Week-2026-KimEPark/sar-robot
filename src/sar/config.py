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

# 카메라 (Intro 실습 월드 TurtleBot3 Burger 기준)
CAMERA_FOV = 1.0472  # rad, 수평 화각

# 대상 인식 (과제와 구현 기준 9~11장)
TARGET_COLOR = "red"  # HSV_RANGES의 키
YOLO_MODEL = "../../models/YOLO/yolo11n.pt"  # 컨트롤러 폴더 기준 상대 경로
YOLO_DEVICE = "cpu"  # NVIDIA GPU면 "cuda", Apple Silicon이면 "mps"
YOLO_CLASSES = [47, 49, 32]  # COCO apple, orange, sports ball
YOLO_CONF = 0.20
YOLO_EVERY = 4  # step, YOLO 추론 주기
COLOR_RATIO_MIN = 0.25  # YOLO 상자 안 대상 색 픽셀 비율 하한
USE_COLOR_FALLBACK = True  # YOLO 미검출 시 색 분할로 대체 검출
TARGET_DIAMETER = 0.095  # m, 사과 지름
MIN_BLOB_AREA = 60  # px, 색 분할 덩어리 면적 하한
MIN_CIRCULARITY = 0.6  # 색 분할 덩어리 원형도(4πA/P²) 하한
MASK_KERNEL_SIZE = 3  # px, 색 마스크 잡음 제거(열림 연산) 커널 크기
HORIZON_MARGIN = 20  # px, 중심이 화면 가운데선보다 이만큼 위인 검출은 식탁 위 물체로 제외
CONFIRM_FRAMES = 4  # 연속 검출 프레임 수
CONFIRM_MATCH_RADIUS = 0.5  # m, 연속 검출을 같은 대상으로 보는 위치 차이 상한
CONFIRM_MIN_DIST = 0.3  # m, 위치 가중 평균의 거리 하한
APPROACH_DIST = 0.35  # m
FOUND_EXCLUDE_RADIUS = 0.6  # m, 구조 완료 위치 주변 검출 제외 반경

# HSV 범위 (OpenCV: H 0~179, S·V 0~255). 대회 조명에서 hsv_tuner로 재조정
HSV_RANGES = {
    "red": [((0, 120, 60), (8, 255, 255)), ((170, 120, 60), (179, 255, 255))],
    "orange": [((12, 170, 120), (28, 255, 255))],
    "purple": [((125, 90, 50), (160, 255, 255))],
    "green": [((30, 90, 50), (60, 255, 255))],
}

# 지도 시각화
MAP_SAVE_DIR = "output"  # 컨트롤러 폴더 기준
MAP_SAVE_PERIOD = 5.0  # s
MAP_VIEW_MARGIN = 20  # cell, 확인된 영역 바깥 여백
MAP_VIEW_SCALE = 2  # 격자 1칸의 픽셀 수

# HSV 튜닝 도구 (controllers/hsv_tuner)
TUNER_V = 0.10  # m/s, 키보드 전진 속도
TUNER_W = 1.0  # rad/s, 키보드 회전 속도
TUNER_FRAME_DIR = "frames"  # 컨트롤러 폴더 기준
