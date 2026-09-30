"""Webots 컨트롤러 진입점. 로직은 src/sar 패키지에 작성한다.

Webots Preferences의 Python command를 저장소 .venv의 Python으로 설정해야 한다.
sar 패키지는 .venv에 editable 모드(pip install -e .)로 설치되어 있다.
"""

from sar import config
from sar.localization.odometry import WheelOdometry
from sar.robot_io import RobotIO

LOG_INTERVAL = 1.0  # s


def main() -> None:
    io = RobotIO()
    odom = WheelOdometry(config.WHEEL_RADIUS, config.WHEEL_SEPARATION)
    last_log = -LOG_INTERVAL
    while io.step():
        pose = odom.update(*io.wheel_angles())
        io.set_wheel_speeds(0.0, 0.0)  # 미션 구현 전 정지 상태 유지
        if io.time() - last_log >= LOG_INTERVAL:
            ranges = io.lidar_ranges()
            print(
                f"t={io.time():.1f}s pose=({pose.x:.2f}, {pose.y:.2f}, {pose.theta:.2f}) "
                f"lidar_points={len(ranges)} front={ranges[len(ranges) // 2]:.2f}m"
            )
            last_log = io.time()


if __name__ == "__main__":
    main()
