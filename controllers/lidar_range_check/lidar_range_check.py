"""LDS-01 라이다가 minRange(0.12m)보다 가까운 물체에 어떤 값을 반환하는지 확인.

worlds/lidar_range_check.wbt 전용 진단 컨트롤러. 정면(index 180)에 0.09m(최소범위
미만), 후면(index 0)에 0.19m(최소범위 이상) 장애물을 고정 배치해 두 값을 비교한다.
"""

from controller import Robot

robot = Robot()
timestep = int(robot.getBasicTimeStep())

lidar = robot.getDevice("LDS-01")
lidar.enable(timestep)
lidar.enablePointCloud()

while robot.step(timestep) != -1:
    if robot.getTime() >= 2.0:
        break

ranges = lidar.getRangeImage()
n = len(ranges)
front = ranges[n // 2]  # index 180, 0.09m 장애물 (minRange 미만)
back = ranges[0]  # index 0, 0.19m 장애물 (minRange 이상)

print(f"[lidar_range_check] 포인트 수: {n}")
print(f"[lidar_range_check] 정면(0.09m, minRange 미만) 반환값: {front}")
print(f"[lidar_range_check] 후면(0.19m, minRange 이상) 반환값: {back}")
