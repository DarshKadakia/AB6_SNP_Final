from robot_core.snp_model import RobotModel, Plotter, ScalarPositionFusion
import time
import threading
import teleop as teleop

robot = RobotModel("COM7")
stop_event = threading.Event()
fusion = ScalarPositionFusion(arena_length_cm=100.0)


plotter = Plotter(max_points=100, interval_ms=50, title="SHARP IR")
# kalman_1d = DistanceKalman()


def control_loop():
    while True:
        sensors = robot.get_sensors()
        # position = fusion.step(sensors, sensors.encoder_left, sensors.encoder_right)
        teleop.update()

        plotter.plot(
            "sharp ir", sensors.sharp_ir_distance,
            "water us",sensors.waterPultrasonic,
            "us", sensors.ultrasonic,
            # "sharpir", sensors.sharp_ir_distance + 11.5,
            # "position", position,
        )
        time.sleep(0.09)
        robot.send_velocity(teleop.vx, teleop.vy, teleop.vw)

t = threading.Thread(target=control_loop, daemon=True)
t.start()

try:
    plotter.start(on_close=stop_event.set)
except KeyboardInterrupt:
    print("\nInterrupted by user")
finally:
    stop_event.set()
    robot.close()
