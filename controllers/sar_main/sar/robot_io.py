"""Webots 장치 래핑. Webots API는 이 파일에서만 호출한다.

`controller` 모듈은 RobotIO 생성 시 import하므로 wheel_speeds()는 Webots 없이 테스트할 수 있다.
"""

import numpy as np

from sar import config


def wheel_speeds(v: float, w: float) -> tuple[float, float]:
    """(v [m/s], w [rad/s])를 좌우 바퀴 각속도 [rad/s]로 변환. 최대값 초과 시 비율 유지하며 축소."""
    half = w * config.WHEEL_SEPARATION / 2
    wl = (v - half) / config.WHEEL_RADIUS
    wr = (v + half) / config.WHEEL_RADIUS
    peak = max(abs(wl), abs(wr))
    if peak > config.MAX_WHEEL_SPEED:
        scale = config.MAX_WHEEL_SPEED / peak
        wl, wr = wl * scale, wr * scale
    return wl, wr


class RobotIO:
    def __init__(self) -> None:
        from controller import Robot

        self.robot = Robot()
        self.timestep = int(self.robot.getBasicTimeStep())
        self._lidar = self._enable(config.LIDAR_NAME)
        self._camera = self._enable(config.CAMERA_NAME)
        self._compass = self._enable(config.COMPASS_NAME)
        self._left = self.robot.getDevice(config.LEFT_MOTOR_NAME)
        self._right = self.robot.getDevice(config.RIGHT_MOTOR_NAME)
        self._left_enc = self._left.getPositionSensor()
        self._right_enc = self._right.getPositionSensor()
        for motor, encoder in ((self._left, self._left_enc), (self._right, self._right_enc)):
            motor.setPosition(float("inf"))  # 속도 제어 모드
            motor.setVelocity(0.0)
            encoder.enable(self.timestep)

    def _enable(self, name: str):
        """장치를 켜서 반환. 월드에 없으면 None."""
        device = self.robot.getDevice(name)
        if device is not None:
            device.enable(self.timestep)
        return device

    def step(self) -> bool:
        """시뮬레이션 1 step 진행. 종료 시 False."""
        return self.robot.step(self.timestep) != -1

    def time(self) -> float:
        """시뮬레이션 시간 [s]."""
        return self.robot.getTime()

    def encoders(self) -> tuple[float, float]:
        """좌우 바퀴 누적 회전각 [rad]."""
        return self._left_enc.getValue(), self._right_enc.getValue()

    def compass(self) -> tuple | None:
        """나침반 벡터. 장치가 없으면 None."""
        return tuple(self._compass.getValues()) if self._compass else None

    def lidar(self) -> list[float] | None:
        """거리 360개 [m]. 인덱스 180 정면, 90 왼쪽, 270 오른쪽, 0 뒤. 장치가 없으면 None."""
        return list(self._lidar.getRangeImage()) if self._lidar else None

    def camera_bgr(self) -> np.ndarray | None:
        """카메라 이미지 (480, 640, 3) BGR uint8. 장치나 이미지가 없으면 None."""
        image = self._camera.getImage() if self._camera else None
        if not image:
            return None
        h, w = self._camera.getHeight(), self._camera.getWidth()
        return np.frombuffer(image, np.uint8).reshape((h, w, 4))[:, :, :3].copy()

    def drive(self, v: float, w: float) -> None:
        """속도 명령 (v [m/s], w [rad/s])를 바퀴 각속도로 변환해 설정."""
        wl, wr = wheel_speeds(v, w)
        self._left.setVelocity(wl)
        self._right.setVelocity(wr)


if __name__ == "__main__":
    # Webots 없이 실행: controllers/sar_main에서 python -m sar.robot_io
    straight = 0.1 / config.WHEEL_RADIUS
    assert wheel_speeds(0.1, 0.0) == (straight, straight)
    wl, wr = wheel_speeds(0.0, 1.0)
    assert wl == -wr and wr > 0  # 제자리 좌회전
    wl, wr = wheel_speeds(1.0, 0.5)  # 최대값 초과 시 비율 유지 축소
    assert max(abs(wl), abs(wr)) <= config.MAX_WHEEL_SPEED + 1e-9
    print("robot_io self-test ok")
