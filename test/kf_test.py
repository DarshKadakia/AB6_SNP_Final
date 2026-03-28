# main.py
from robot_core.snp_model import Localization, RobotModel
import time
robot = RobotModel("COM7")
# fusion = ScalarPositionFusion(arena_length_cm=90.0)
loc = Localization()
# inside your robot loop:
while True:
    sensors = robot.get_sensors()
    x, y, theta = loc.compute_odometry(robot.sensors)
    print(x)
    # print(y)
    # print(theta)
    robot.goto_pose(loc, 24, 0, 0)
    # robot.send_velocity(0.6, 0, 0)
    # position = fusion.step(sensors, sensors.encoder_left, sensors.encoder_right)
    # print({fusion.position})
    # print({robot.sensors.ultrasonic} | {robot.sensors.waterPultrasonic} | {robot.sensors.sharp_ir_distance} | {fusion.position}  | {fusion.uncertainty})
    # time.sleep(0.2)
    # if fusion.position >= 80.0:
    #     robot.close()
    #     fusion.reset()   # back at home