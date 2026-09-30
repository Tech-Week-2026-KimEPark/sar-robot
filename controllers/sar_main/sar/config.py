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
KEYBOARD_MAX_KEYS = 7  # Webots 키보드 동시 입력 최대 개수
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
MAP_SIZE = 32.0  # m, 시작점 중심 정사각형 지도 한 변 (640 x 640칸)
RAY_STEP = MAP_RES / 2  # m, 라이다 광선 추적 표본 간격
WALL_BAND = 0.20  # m, 팽창 영역 바깥에서 벽 근처 추가 비용을 주는 폭

# 경로 계획
WALL_COST = 2.0  # 벽 근처 칸 비용 배수의 최대 증가량 (팽창 경계에서 1 + WALL_COST)
UNKNOWN_COST = 1.5  # allow_unknown일 때 모르는 칸 비용 배수
PLAN_MARGIN = 1.0  # m, 계획 영역 = 확인한 영역 + 시작·목표를 포함하는 여백
SNAP_RADIUS = 0.5  # m, 시작·목표가 통과 불가 칸이면 이 반경 안의 가장 가까운 통과 가능 칸 사용
PATH_SMOOTH = True  # 시야선 기반 경로 다듬기. False면 격자 경로를 그대로 사용
PATH_STEP = 0.10  # m, 반환 경로의 점 간격
BLACKLIST_RADIUS = 0.5  # m, 블랙리스트 좌표 주변 프론티어 제외 반경
FRONTIER_MIN_DIST = 0.3  # m, 이보다 가까운 프론티어 칸은 목표에서 제외 (제자리 목표 방지)

# 미션 상태 머신 (CONTEXT.md 7장)
INIT_SPIN_W = 0.8  # rad/s, 시작 제자리 회전 속도 (1회전 약 7.9초)
COMPASS_SCALE_TOL = 0.5  # 오도메트리/나침반 회전 비율이 1에서 이만큼 벗어나면 나침반 미사용
REPLAN_PERIOD = 2.0  # s, 프론티어 선택·경로 재계획 주기
FRONTIER_TIMEOUT = 30.0  # s, 같은 프론티어 목표에 도달하지 못하면 블랙리스트
APPROACH_TIMEOUT = 60.0  # s, 대상 접근 제한 시간
RESCUE_HOLD = 2.0  # s, 구조 정지 시간
RETURN_TOL = 0.12  # m, 시작점 도착 판정 반경
FACE_TOL = 0.1  # rad, 대상·목표 정면 정렬 허용 오차
FACE_GAIN = 1.5  # 1/s, 정렬 각속도 = 이득 × 방위각
CANDIDATE_DROP_DIST = 0.8  # m, 후보 위치에 이 거리까지 접근해도 확정되지 않으면 후보 삭제
STUCK_TIME, STUCK_DIST = 4.0, 0.05  # s, m. 이 시간 동안 이 거리 미만 이동이면 RECOVERY
BLOCKED_TIME = 3.0  # s, 안전 필터 연속 차단 시 재계획
RECOVERY_BACK_TIME = 1.0  # s, RECOVERY 후진 시간
RECOVERY_TURN_TIME = 1.5  # s, RECOVERY 회전 시간
TRAJ_STEP = 0.10  # m, 주행 궤적 기록 간격

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
USE_COLOR_FALLBACK = True  # YOLO 대상 상자 밖의 색 분할 덩어리를 추가 검출
TARGET_DIAMETER = 0.095  # m
MIN_BLOB_AREA = 60  # px
MIN_CIRCULARITY = 0.75  # 색 분할 덩어리 원형도(4πA/P²) 하한. 꼭지 제거 후 판정
MIN_BLOB_FILL = 0.72  # 색 분할 덩어리 면적 / 외접원 면적 하한. 정사각형 0.64, 원 1.0
BLOB_OPEN_RATIO = 0.15  # 꼭지 제거 열림 연산 커널 크기 / 덩어리 짧은 변
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
TUNER_V = 0.12  # m/s, 키보드 전진 속도
TUNER_W = 0.6  # rad/s, 키보드 회전 속도
TUNER_FINE_SCALE = 0.3  # Shift(Webots 창) 또는 대문자(OpenCV 창) 입력 시 속도 배율
TUNER_KEY_HOLD = 0.6  # s (실제 시간), OpenCV 창 키 1회 입력의 유지 시간. 키 반복 지연보다 길게
TUNER_FRAME_DIR = "frames"  # 컨트롤러 폴더 기준
TUNER_LOG_FILE = "output/measurements.csv"  # 컨트롤러 폴더 기준, m 키 측정 기록
TUNER_SCAN_W = 0.4  # rad/s, 자동 주행 제자리 회전 속도
TUNER_SCAN_TURNS = 1.0  # 회전, 자동 접근에서 대상을 찾지 못하면 멈추는 누적 회전량
TUNER_SURVEY_STEP = 0.5236  # rad (30°), 회전 측정 간격
TUNER_ALIGN_TOL = 0.05  # rad, 정렬 완료 방위각 오차
TUNER_ALIGN_GAIN = 1.5  # 1/s, 정렬 각속도 = 이득 × 방위각
TUNER_APPROACH_V = 0.08  # m/s, 자동 접근 전진 속도
TUNER_MEASURE_DISTS = (3.0, 2.0, 1.0)  # m, 자동 접근 측정 지점
TUNER_STOP_DIST = 0.5  # m, 자동 접근 정지 거리
TUNER_LOST_TIMEOUT = 2.0  # s, 대상 미검출이 이 시간 이상이면 다시 탐색
