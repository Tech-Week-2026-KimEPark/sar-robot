"""Webots 컨트롤러 진입점. 모듈을 생성하고 미션 루프를 실행한다.

팀 모듈은 같은 폴더의 sar/ 패키지에 있다. Webots는 컨트롤러 폴더를 import 경로에 포함하므로
별도 설치 없이 import된다. 상태 머신은 sar/mission.py에 있다.
"""

from sar import config
from sar.grid_map import GridMap
from sar.mission import Mission
from sar.odometry import Odometry
from sar.perception import TargetDetector
from sar.robot_io import RobotIO


def main() -> None:
    io = RobotIO()
    odom = Odometry(config.START_X, config.START_Y, config.START_THETA)
    detector = TargetDetector(config.TARGET_COLOR, config.YOLO_MODEL)
    if detector.load_error:
        print(f"YOLO 로드 실패, 색 분할만 사용: {detector.load_error}")
    mission = Mission(io, odom, GridMap(), detector, log=print)
    while io.step():
        mission.tick()


if __name__ == "__main__":
    main()
