"""Webots 컨트롤러 진입점. RobotIO를 생성하고 미션 루프를 실행한다.

팀 모듈은 같은 폴더의 sar/ 패키지에 있다. Webots는 컨트롤러 폴더를 import 경로에 포함하므로
별도 설치 없이 import된다. Mission 구현 전까지는 정지 상태에서 위치와 라이다 값만 출력한다.
"""

from sar import config
from sar.odometry import Odometry
from sar.robot_io import RobotIO


def main() -> None:
    io = RobotIO()
    odom = Odometry(config.START_X, config.START_Y, config.START_THETA)
    last_log = -config.LOG_INTERVAL
    while io.step():
        odom.update(*io.encoders())
        io.drive(0.0, 0.0)  # Mission 구현 전 정지 상태 유지
        if io.time() - last_log >= config.LOG_INTERVAL:
            x, y, theta = odom.pose()
            ranges = io.lidar()
            front = ranges[len(ranges) // 2] if ranges else float("nan")
            print(
                f"[t={io.time():.1f}s] pose=({x:.2f}, {y:.2f}, {theta:.2f}) "
                f"lidar_points={len(ranges) if ranges else 0} front={front:.2f}m"
            )
            last_log = io.time()


if __name__ == "__main__":
    main()
