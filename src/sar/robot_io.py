"""Webots 장치 접근 래퍼. Webots 밖에서는 import하지 않는다."""

import numpy as np
from controller import Robot

from sar import config


class RobotIO:
    def __init__(self) -> None:
        self.robot = Robot()
        self.timestep = int(self.robot.getBasicTimeStep())

        self.lidar = self.robot.getDevice(config.LIDAR_NAME)
        self.lidar.enable(self.timestep)

        self.camera = self.robot.getDevice(config.CAMERA_NAME)
        self.camera.enable(self.timestep)

        self.left_motor = self.robot.getDevice(config.LEFT_MOTOR_NAME)
        self.right_motor = self.robot.getDevice(config.RIGHT_MOTOR_NAME)
        self.left_encoder = self.left_motor.getPositionSensor()
        self.right_encoder = self.right_motor.getPositionSensor()
        for motor, encoder in (
            (self.left_motor, self.left_encoder),
            (self.right_motor, self.right_encoder),
        ):
            motor.setPosition(float("inf"))  # 속도 제어 모드
            motor.setVelocity(0.0)
            encoder.enable(self.timestep)

    def step(self) -> bool:
        """시뮬레이션 1 step 진행. 종료 시 False."""
        return self.robot.step(self.timestep) != -1

    def time(self) -> float:
        return self.robot.getTime()

    def lidar_ranges(self) -> list[float]:
        return self.lidar.getRangeImage()

    def wheel_angles(self) -> tuple[float, float]:
        """좌우 바퀴 누적 회전각 [rad]."""
        return self.left_encoder.getValue(), self.right_encoder.getValue()

    def camera_bgr(self) -> np.ndarray:
        """카메라 이미지를 (height, width, 3) BGR 배열로 반환."""
        h, w = self.camera.getHeight(), self.camera.getWidth()
        bgra = np.frombuffer(self.camera.getImage(), np.uint8).reshape((h, w, 4))
        return bgra[:, :, :3].copy()

    def set_wheel_speeds(self, v_left: float, v_right: float) -> None:
        """좌우 바퀴 선속도 [m/s]를 모터 각속도 [rad/s]로 변환해 설정."""
        limit = config.MAX_WHEEL_SPEED
        for motor, v in ((self.left_motor, v_left), (self.right_motor, v_right)):
            motor.setVelocity(max(-limit, min(limit, v / config.WHEEL_RADIUS)))
