"""오도메트리 정확도 진단. Intro 원본 tb3_ground_truth 월드(breakroom_ground_truth.wbt)에서 사용.

Supervisor로 얻은 실제 위치와 순수 엔코더 Odometry 추정치를 비교해 사각형 한 바퀴
주행 후 오차를 출력한다. 나침반 보정 없이 엔코더만 사용한 기준값이다.
CONTEXT.md 9장 기준: 오차 0.3 m 이상이면 SCAN_MATCH 도입을 검토한다.

controllers/sar_main/sar/*를 재사용하므로 Webots Python command가 저장소 .venv를
가리켜야 한다 (dev-setup.md 4절).
"""

import math
import sys
from pathlib import Path

from controller import Supervisor

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "sar_main"))

from sar import config
from sar.local_control import safety_filter
from sar.odometry import Odometry
from sar.robot_io import wheel_speeds

robot = Supervisor()
timestep = int(robot.getBasicTimeStep())

left_motor = robot.getDevice(config.LEFT_MOTOR_NAME)
right_motor = robot.getDevice(config.RIGHT_MOTOR_NAME)
left_motor.setPosition(float("inf"))
right_motor.setPosition(float("inf"))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)
left_encoder = left_motor.getPositionSensor()
right_encoder = right_motor.getPositionSensor()
left_encoder.enable(timestep)
right_encoder.enable(timestep)

lidar = robot.getDevice(config.LIDAR_NAME)
if lidar:
    lidar.enable(timestep)

compass_device = robot.getDevice(config.COMPASS_NAME)
gyro_device = robot.getDevice("gyro")
print(f"[odom_check] compass 장치: {'있음' if compass_device else '없음'}")
print(f"[odom_check] gyro 장치: {'있음' if gyro_device else '없음'}")

robot_node = robot.getSelf()


def ground_truth_pose() -> tuple[float, float, float]:
    x, y, _ = robot_node.getPosition()
    orientation = robot_node.getOrientation()  # row-major 3x3
    theta = math.atan2(orientation[3], orientation[0])
    return x, y, theta


start_x, start_y, start_theta = ground_truth_pose()
odom = Odometry(start_x, start_y, start_theta)
# 첫 robot.step() 전에는 센서값이 NaN이므로 기준값은 루프 첫 반복에서 읽는다
# (Odometry.update()는 self._last가 None인 첫 호출을 기준값으로만 씀)

# 사각형 한 바퀴: (전진 -> 제자리 좌회전 90도) x 4, 엔코더만 사용(나침반 보정 없음)
FORWARD_SPEED = config.V_APPROACH  # m/s, 좁은 방에서도 안전한 속도
SIDE_TIME = 3.0  # s
TURN_WHEEL_SPEED = 2.0  # rad/s, 좌우 반대 부호로 제자리 회전
YAW_RATE = config.WHEEL_RADIUS * 2 * TURN_WHEEL_SPEED / config.WHEEL_SEPARATION
TURN_TIME = (math.pi / 2) / YAW_RATE

phases = []
for _ in range(4):
    phases.append(("forward", SIDE_TIME))
    phases.append(("turn", TURN_TIME))

phase_index = 0
phase_start = robot.getTime()
blocked_stop = False

while robot.step(timestep) != -1:
    now = robot.getTime()
    enc_l, enc_r = left_encoder.getValue(), right_encoder.getValue()
    odom.update(enc_l, enc_r)

    if phase_index >= len(phases):
        break

    name, duration = phases[phase_index]
    if now - phase_start >= duration:
        phase_index += 1
        phase_start = now
        left_motor.setVelocity(0.0)
        right_motor.setVelocity(0.0)
        continue

    if name == "forward":
        ranges = lidar.getRangeImage() if lidar else None
        v, w, blocked = safety_filter(FORWARD_SPEED, 0.0, ranges)
        if blocked:
            blocked_stop = True
            print(f"[odom_check] t={now:.1f}s 전방 장애물로 정지, 사각형 중단")
            break
    else:
        v, w = 0.0, YAW_RATE

    wl, wr = wheel_speeds(v, w)
    left_motor.setVelocity(wl)
    right_motor.setVelocity(wr)

left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

gt_x, gt_y, gt_theta = ground_truth_pose()
odom_x, odom_y, odom_theta = odom.pose()
error = math.hypot(gt_x - odom_x, gt_y - odom_y)

print(f"[odom_check] 중단 여부: {'예' if blocked_stop else '아니오 (사각형 완주)'}")
print(f"[odom_check] ground truth pose = ({gt_x:.3f}, {gt_y:.3f}, {gt_theta:.3f})")
print(f"[odom_check] odometry pose     = ({odom_x:.3f}, {odom_y:.3f}, {odom_theta:.3f})")
print(f"[odom_check] 위치 오차 = {error:.3f} m (기준: 0.3 m 이상이면 SCAN_MATCH 검토)")
