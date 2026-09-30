"""Webots 컨트롤러 진입점. 로직은 src/sar 패키지에 작성한다.

Webots는 환경설정의 Python(기본 python3)으로 이 파일을 실행한다. 이 파일은
저장소 루트 .venv의 Python으로 자신을 다시 실행한 뒤 src/를 import 경로에 추가한다.
재실행 전 단계는 macOS 기본 Python 3.9에서도 동작해야 하므로 3.10 이상 문법을 쓰지 않는다.
"""

import os
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
VENV = ROOT / ".venv"
VENV_PYTHON = VENV / ("Scripts/python.exe" if os.name == "nt" else "bin/python")
LOG_INTERVAL = 1.0  # s


def use_venv_python() -> None:
    """현재 Python이 저장소 .venv가 아니면 .venv Python으로 재실행."""
    if Path(sys.prefix).resolve() == VENV.resolve():
        return
    if not VENV_PYTHON.exists():
        sys.exit(
            f"{VENV_PYTHON} 없음. docs/human/how-to/dev-setup.md의 가상환경 생성 절차 실행 필요"
        )
    args = [str(VENV_PYTHON), str(Path(__file__).resolve())]
    if os.name == "nt":  # Windows execv는 새 프로세스를 만들고 종료하므로 종료 코드를 전달
        sys.exit(subprocess.call(args))
    os.execv(args[0], args)


def main() -> None:
    from sar import config
    from sar.localization.odometry import WheelOdometry
    from sar.robot_io import RobotIO

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
    use_venv_python()
    sys.path.insert(0, str(ROOT / "src"))
    main()
