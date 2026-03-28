# import json
# import math
# import time
# import keyboard
# import teleop
# from robot_core.snp_model import RobotModel, Localization

# robot = RobotModel("COM7")
# loc = Localization()

# recording = False
# replaying = False
# trajectory = []
# start_time = None
# file_name = "trajectory.json"

# key_last_pressed = {"w": 0, "q": 0, "r": 0, "h": 0}
# DEBOUNCE_S = 0.3

# def debounced(key):
#     now = time.time()
#     if keyboard.is_pressed(key) and (now - key_last_pressed[key]) > DEBOUNCE_S:
#         key_last_pressed[key] = now
#         return True
#     return False

# replay_index = 0
# replay_trajectory = []
# replay_start = 0

# # Stored start pose for "return home"
# home_pose = None

# print('W: record | Q: stop/abort | R: replay | H: return to start')

# while True:
#     x, y, theta = loc.compute_odometry(robot.sensors)

#     # --- START RECORDING ---
#     if debounced('w') and not recording and not replaying:
#         print("Recording started...")
#         recording = True
#         trajectory = []
#         start_time = time.time()
#         print(x)
#         home_pose = (x, y, theta)   # save start pose at record time

#     # --- TELEOP + RECORD ---
#     if recording:
#         teleop.update()
#         robot.send_velocity(teleop.vx, teleop.vy, teleop.vw)
#         trajectory.append({
#             "t": time.time() - start_time,
#             "x": x, "y": y, "theta": theta
#         })

#     # --- STOP RECORDING ---
#     if debounced('q') and recording:
#         recording = False
#         robot.send_velocity(0, 0, 0)
#         with open(file_name, "w") as f:
#             json.dump({
#                 "home": {"x": home_pose[0], "y": home_pose[1], "theta": home_pose[2]},
#                 "trajectory": trajectory
#             }, f, indent=4)
#         print(f"Saved {len(trajectory)} points to {file_name}")

#     # --- START REPLAY ---
#     if debounced('r') and not replaying and not recording:
#         with open(file_name, "r") as f:
#             data = json.load(f)
#             # print(data)
#         replay_trajectory = data["trajectory"]
#         home_pose = (data["home"]["x"], data["home"]["y"], data["home"]["theta"])
#         if replay_trajectory:
#             print("Replaying...")
#             replaying = True
#             replay_index = 0
#             replay_start = time.time()

#     # --- NON-BLOCKING REPLAY TICK ---
#     if replaying:
#         now = time.time() - replay_start

#         # Advance target waypoint index based on elapsed time
#         while replay_index < len(replay_trajectory) - 1 and \
#               replay_trajectory[replay_index + 1]["t"] <= now:
#             replay_index += 1

#         pt = replay_trajectory[replay_index]
#         cx, cy, ctheta = loc.compute_odometry(robot.sensors)

#         dx = pt["x"] - cx
#         dy = pt["y"] - cy
#         dist_err = math.hypot(dx, dy)
#         final_heading_err = (pt["theta"] - ctheta + math.pi) % (2 * math.pi) - math.pi

#         # Check if we've reached the final waypoint
#         if replay_index >= len(replay_trajectory) - 1 and \
#            dist_err < 0.02 and abs(final_heading_err) < 0.05:
#             robot.send_velocity(0, 0, 0)
#             print("Replay done.")
#             replaying = False
#         else:
#             angle_to_goal = math.atan2(dy, dx)
#             heading_err = (angle_to_goal - ctheta + math.pi) % (2 * math.pi) - math.pi

#             if dist_err > 0.02:
#                 vx = max(-0.3, min(0.3, 1.5 * dist_err * math.cos(heading_err)))
#                 vy = max(-0.3, min(0.3, 1.5 * dist_err * math.sin(heading_err)))
#                 vw = max(-1.0, min(1.0, 2.0 * heading_err))
#             else:
#                 vx, vy = 0.0, 0.0
#                 vw = max(-1.0, min(1.0, 2.0 * final_heading_err))

#             robot.send_velocity(vx, vy, vw)

#     # --- ABORT REPLAY ---
#     if debounced('q') and replaying:
#         robot.send_velocity(0, 0, 0)
#         print("Replay aborted.")
#         replaying = False

#     # --- RETURN TO START ---
#     if debounced('h') and not recording and not replaying:
#         if home_pose is not None:
#             print("Returning to start pose...")
#             print(loc)
#             robot.goto_pose(loc, 0, 0, 0)
#             print(x)
#             print("Back at start.")
#         else:
#             print("No home pose saved yet. Record a trajectory first.")

#     time.sleep(0.01)



import json
import time
import keyboard
import teleop as teleop
from robot_core.snp_model import RobotModel, Localization

robot = RobotModel("COM7")
loc = Localization()

recording = False
replaying = False
trajectory = []      # list of {vx, vy, vw, duration}
file_name = "trajectory.json"

key_last_pressed = {"w": 0, "q": 0, "r": 0, "h": 0}
DEBOUNCE_S = 0.3

def debounced(key):
    now = time.time()
    if keyboard.is_pressed(key) and (now - key_last_pressed[key]) > DEBOUNCE_S:
        key_last_pressed[key] = now
        return True
    return False

# --- REPLAY STATE ---
replay_index = 0
replay_trajectory = []
replay_step_start = 0.0

print('W: record | Q: stop/abort | R: replay | H: return to start (reverses keystrokes)')

last_tick = time.time()

while True:
    now = time.time()
    dt = now - last_tick
    last_tick = now

    # --- START RECORDING ---
    if debounced('w') and not recording and not replaying:
        print("Recording started...")
        recording = True
        trajectory = []

    # --- TELEOP + RECORD ---
    if recording:
        teleop.update()
        robot.send_velocity(teleop.vx, teleop.vy, teleop.vw)

        if trajectory:
            last = trajectory[-1]
            same_input = (
                last["vx"] == teleop.vx and
                last["vy"] == teleop.vy and
                last["vw"] == teleop.vw
            )
            if same_input:
                # Extend duration of current segment instead of adding a new one
                last["duration"] += dt
            else:
                trajectory.append({
                    "vx": teleop.vx,
                    "vy": teleop.vy,
                    "vw": teleop.vw,
                    "duration": dt
                })
        else:
            trajectory.append({
                "vx": teleop.vx,
                "vy": teleop.vy,
                "vw": teleop.vw,
                "duration": dt
            })

    # --- STOP RECORDING ---
    if debounced('q') and recording:
        recording = False
        robot.send_velocity(0, 0, 0)
        with open(file_name, "w") as f:
            json.dump({"trajectory": trajectory}, f, indent=4)
        total = sum(s["duration"] for s in trajectory)
        print(f"Saved {len(trajectory)} segments ({total:.2f}s) to {file_name}")

    # --- START REPLAY ---
    if debounced('r') and not replaying and not recording:
        with open(file_name, "r") as f:
            data = json.load(f)
        replay_trajectory = data["trajectory"]
        if replay_trajectory:
            print("Replaying...")
            replaying = True
            replay_index = 0
            replay_step_start = time.time()

    # --- NON-BLOCKING REPLAY TICK ---
    if replaying:
        if replay_index >= len(replay_trajectory):
            robot.send_velocity(0, 0, 0)
            print("Replay done.")
            replaying = False
        else:
            step = replay_trajectory[replay_index]
            robot.send_velocity(step["vx"], step["vy"], step["vw"])
            if time.time() - replay_step_start >= step["duration"]:
                replay_index += 1
                replay_step_start = time.time()

    # --- ABORT REPLAY ---
    if debounced('q') and replaying:
        robot.send_velocity(0, 0, 0)
        print("Replay aborted.")
        replaying = False

    # --- RETURN TO START (reverse the trajectory) ---
    if debounced('h') and not recording and not replaying:
        try:
            with open(file_name, "r") as f:
                data = json.load(f)
            fwd = data["trajectory"]
            if not fwd:
                print("Trajectory is empty.")
            else:
                print("Returning to start by reversing keystrokes...")
                for step in reversed(fwd):
                    robot.send_velocity(-step["vx"], -step["vy"], -step["vw"])
                    time.sleep(step["duration"])
                robot.send_velocity(0, 0, 0)
                print("Back at start.")
        except FileNotFoundError:
            print("No trajectory file found. Record one first.")

    time.sleep(0.01)