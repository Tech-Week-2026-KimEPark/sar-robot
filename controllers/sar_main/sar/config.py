"""모든 설정값. 숫자 상수는 이 파일에만 정의.

기준: docs 저장소 human/reference/sar-과제-구현-기준.md 10장 (CONTEXT.md 9장)
"""

import math

# TurtleBot3 Burger (Webots R2025a TurtleBot3Burger.proto)
WHEEL_RADIUS = 0.033  # m
WHEEL_SEPARATION = 0.160  # m
ROBOT_RADIUS = 0.105  # m, Intro 과정 노트북 핵심 파라미터
MAX_WHEEL_SPEED = 6.67  # rad/s, RotationalMotor maxVelocity

# 장치 이름 (Intro 실습 월드 기준)
LIDAR_NAME = "LDS-01"
CAMERA_NAME = "camera"
COMPASS_NAME = "compass"
LEFT_MOTOR_NAME = "left wheel motor"
RIGHT_MOTOR_NAME = "right wheel motor"

# 과제
START_X, START_Y, START_THETA = -0.3, -7.5, math.pi  # 대회 당일 입력
TARGET_COLOR = "red"  # red | green | purple | orange | custom
TARGET_COUNT = 2  # 대회 공지: 빨간 사과 2개
GOAL_XY = None  # 대회 공지: 목적지는 시작 지점
TIME_LIMIT = 900.0  # s, 시뮬레이션 시간. 대회 공지 확인 후 수정
RETURN_RESERVE = 150.0  # s

# 주행
V_MAX = 0.18  # m/s
V_APPROACH = 0.10  # m/s
W_MAX = 1.8  # rad/s
LOOKAHEAD = 0.35  # m
GOAL_TOL = 0.15  # m
SAFETY_MARGIN = 0.06  # m, 몸체 바깥 여유
STOP_DIST = 0.20  # m, 정면 즉시 정지 거리

# 지도
MAP_RES = 0.05  # m
L_OCC = 0.9  # 맞은 칸 log-odds 증분 (확률 약 0.71)
L_FREE = -0.4  # 지나간 칸 log-odds 증분 (확률 약 0.40)
L_MIN, L_MAX = -2.0, 3.5  # 확률 약 0.12 ~ 0.97에서 고정
OCC_THRESHOLD = 0.3  # 이보다 크면 장애물 (확률 약 0.57)
LIDAR_MIN, LIDAR_MAX = 0.12, 3.5  # m
INFLATE = ROBOT_RADIUS + 0.07  # m
MIN_FRONTIER_CELLS = 6

# 위치 추정
HEADING_Q = 0.01**2  # 방향 예측 잡음 (한 스텝, rad^2)
HEADING_R = 0.05**2  # 나침반 관측 잡음 (rad^2)
SCAN_MATCH = False  # 오차 0.3 m 이상 확인 시 True
SCAN_MATCH_PERIOD = 1.5  # s

# 인식
CAMERA_FOV = 1.0472  # rad
YOLO_MODEL = "../../models/YOLO/yolo11n.pt"  # 컨트롤러 폴더 기준
YOLO_DEVICE = "cpu"  # NVIDIA GPU면 "cuda", Apple Silicon이면 "mps"
YOLO_CLASSES = [47, 49, 32]  # apple, orange, sports ball
YOLO_CONF = 0.20
YOLO_EVERY = 4  # 몇 스텝마다 추론
COLOR_RATIO_MIN = 0.25  # 상자 안 대상 색 픽셀 비율 하한
USE_COLOR_FALLBACK = True  # YOLO 미검출 시 색 분할로 대체
TARGET_DIAMETER = 0.095  # m
MIN_BLOB_AREA = 60  # px
MIN_CIRCULARITY = 0.6  # 색 분할 덩어리 원형도(4πA/P²) 하한
MASK_KERNEL_SIZE = 3  # px, 색 마스크 잡음 제거(열림 연산) 커널 크기
HORIZON_MARGIN = 20  # px, 중심이 화면 가운데선보다 이만큼 위인 검출은 식탁 위 물체로 제외
CONFIRM_FRAMES = 4
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

# 출력
MAP_SAVE_DIR = "output"  # 지도 이미지 저장 폴더 (컨트롤러 폴더 기준)
MAP_SAVE_PERIOD = 5.0  # s
MAP_VIEW_MARGIN = 20  # cell, 확인된 영역 바깥 여백
MAP_VIEW_SCALE = 2  # 격자 1칸의 픽셀 수
LOG_INTERVAL = 1.0  # s, 주기 로그 간격

# HSV 튜닝 도구 (controllers/hsv_tuner)
TUNER_V = 0.10  # m/s, 키보드 전진 속도
TUNER_W = 1.0  # rad/s, 키보드 회전 속도
TUNER_FRAME_DIR = "frames"  # 컨트롤러 폴더 기준
