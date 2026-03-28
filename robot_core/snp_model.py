import serial
import struct
import threading
import time
import numpy as np
import math
import json
import collections
import matplotlib.pyplot as plt
from collections import deque
from matplotlib import animation

# SENSOR CLASS
class Sensors:
    STRUCT_FORMAT = "<3f4B2i9fB4f"
    SIZE = struct.calcsize(STRUCT_FORMAT)

    def __init__(self):
        self.sharp_ir_distance = 0.0
        self.waterPultrasonic = 0.0
        self.ultrasonic = 0.0
        # self.hall1 = 0
        # self.hall2 = 0
        self.limit_switch_1 = 0
        self.limit_switch_2 = 0
        self.ir_right = 0
        self.ir_left = 0

        self.encoder_left = 0
        self.encoder_right = 0

        self.accel_x = 0.0
        self.accel_y = 0.0
        self.accel_z = 0.0

        self.gyro_x = 0.0
        self.gyro_y = 0.0
        self.gyro_z = 0.0

        self.temperature = 0.0

        self.pitch = 0.0
        self.roll = 0.0

        self.mpu_available = 0
        self.ch0 = 0
        self.ch1 = 0 
        self.ch2 = 0
        self.ch3 = 0

        self._sma_buffers = {}
        self._ema_values = {}
        self._median_buffers = {}

    def update_from_bytes(self, data_bytes):
        unpacked = struct.unpack(self.STRUCT_FORMAT, data_bytes)

        (
            self.sharp_ir_distance,
            self.waterPultrasonic,
            self.ultrasonic,
            # self.hall1,
            # self.hall2,
            self.limit_switch_1,
            self.limit_switch_2,
            self.ir_right,
            self.ir_left,
            self.encoder_left,
            self.encoder_right,
            self.accel_x,
            self.accel_y,
            self.accel_z,
            self.gyro_x,
            self.gyro_y,
            self.gyro_z,
            self.temperature,
            self.pitch,
            self.roll,
            self.mpu_available,
            self.ch0,
            self.ch1,
            self.ch2,
            self.ch3,
        ) = unpacked

    def MEDIAN(self, sensor_name, new_value, window_size):
        """
        sensor_name : string identifier (e.g., "ir", "ultra")
        new_value   : latest sensor reading
        window_size : number of samples in sliding window
        """

        # Create buffer for this sensor if not exists
        if sensor_name not in self._median_buffers:
            self._median_buffers[sensor_name] = collections.deque(maxlen=window_size)

        buffer = self._median_buffers[sensor_name]
        buffer.append(new_value)

        # Sort current buffer and return middle value
        sorted_vals = sorted(buffer)
        mid = len(sorted_vals) // 2

        # If odd window → exact middle
        if len(sorted_vals) % 2 == 1:
            return sorted_vals[mid]
        # If even window → average of two middle values
        else:
            return (sorted_vals[mid - 1] + sorted_vals[mid]) / 2

    def SMA(self, sensor_name, new_value, window_size):
        """
        sensor_name : string identifier (e.g., "ir", "ultra")
        new_value   : latest sensor reading
        window_size : number of samples to average
        """

        if sensor_name not in self._sma_buffers:
            self._sma_buffers[sensor_name] = collections.deque(maxlen=window_size)

        buffer = self._sma_buffers[sensor_name]
        buffer.append(new_value)

        return sum(buffer) / len(buffer)
        
    def EMA(self, sensor_name, new_value, alpha):
        """
        sensor_name : string identifier
        new_value   : latest reading
        alpha       : smoothing factor (0 < alpha <= 1)
        """

        if sensor_name not in self._ema_values:
            # Initialize with first value
            self._ema_values[sensor_name] = new_value
            return new_value

        prev = self._ema_values[sensor_name]
        ema_value = alpha * new_value + (1 - alpha) * prev

        self._ema_values[sensor_name] = ema_value

        return ema_value
    
    def low_pass(self, sensor_name, new_value, tau, dt):
        """
        tau : time constant (seconds)
        dt  : time step (seconds)
        """

        alpha = dt / (tau + dt)

        if sensor_name not in self._ema_values:
            self._ema_values[sensor_name] = new_value
            return new_value

        prev = self._ema_values[sensor_name]
        filtered = alpha * new_value + (1 - alpha) * prev

        self._ema_values[sensor_name] = filtered
        return filtered
    
    def complementary_filter(self, accel_angle, gyro_rate, dt, alpha=0.98):
        """
        accel_angle : angle computed from accelerometer
        gyro_rate   : angular velocity (deg/sec)
        dt          : timestep
        alpha       : fusion weight (0.95–0.99 typical)
        """

        if not hasattr(self, "_comp_angle"):
            self._comp_angle = accel_angle
            return accel_angle

        gyro_angle = self._comp_angle + gyro_rate * dt

        fused = alpha * gyro_angle + (1 - alpha) * accel_angle

        self._comp_angle = fused
        return fused

class PositionFusion:
    """
    1-D Kalman filter fusing all 5 sensors into a single position estimate.

    State vector:  Xk = [position_cm, velocity_cm_per_s]

    Equations (your notation):
        Xkp = A·Xk-1 + B·uk + wk          ← predict state
        Pkp = A·Pk-1·Aᵀ + Q               ← predict covariance
        K   = Pkp·Hᵀ / (H·Pkp·Hᵀ + R)    ← Kalman gain  (per sensor)
        Xk  = Xkp + K·[Yk - H·Xkp]       ← correct state (per sensor)
        Pk  = (I - K·H)·Pkp               ← correct covariance (per sensor)

    Access after calling step():
        fusion.position    → fused position in cm from home
        fusion.velocity    → fused velocity in cm/s
        fusion.uncertainty → 1-sigma position uncertainty in cm
    """

    def __init__(self,
                 arena_length_cm  = 90.0,
                 counts_per_rev   = 360,
                 wheel_diameter_cm= 6.5,
                 dt               = 0.05):

        self.arena = arena_length_cm
        self.dt    = dt
        self.counts_per_cm = counts_per_rev / (math.pi * wheel_diameter_cm)

        # ── State vector Xk = [position, velocity]ᵀ ──
        self.Xk = np.array([[0.0],   # position  (cm)
                             [0.0]])  # velocity  (cm/s)

        # ── State transition matrix A ──
        # position += velocity * dt
        # velocity unchanged (constant-velocity model)
        self.A = np.array([[1, dt],
                           [0,  1]])

        # ── Control input matrix B (for IMU acceleration) ──
        # position += 0.5 * a * dt²
        # velocity += a * dt
        self.B = np.array([[0.5 * dt**2],
                           [dt         ]])

        # ── Initial covariance Pk-1 ──
        self.Pk = np.eye(2) * 1.0

        # ── Process noise Q (how much we distrust the motion model) ──
        self.Q = np.array([[0.5, 0.0],
                           [0.0, 2.0]])

        # ── Per-sensor measurement noise R ──
        # These are scalar variances — tune from real logged data
        self.R = {
            "encoder":   0.3,    # very trusted on clean surface
            "imu":       1.5,    # trusted for short intervals
            "us_air":    1.0,    # good, occasional spikes
            "us_water":  0.8,    # slightly more stable than air US
            "ir":        2.0,    # nonlinear curve, less trusted
        }

        # ── Observation matrices H ──
        # H_pos picks out position from state vector → [1, 0]
        # H_vel picks out velocity from state vector → [0, 1]
        self.H_pos = np.array([[1.0, 0.0]])
        self.H_vel = np.array([[0.0, 1.0]])

        # ── Internal tracking ──
        self._prev_enc_left  = None
        self._prev_enc_right = None
        self._imu_vel        = 0.0   # integrated IMU velocity

    # ── Private: one Kalman update step for any sensor ──────────────────────
    def _update(self, z_scalar, H, R_scalar):
        """
        Xk  = Xkp + K · [Yk - H·Xkp]
        Pk  = (I - K·H) · Pkp
        z_scalar : raw scalar measurement
        H        : 1×2 observation matrix
        R_scalar : scalar measurement noise variance
        """
        z   = np.array([[z_scalar]])
        R   = np.array([[R_scalar]])

        # Innovation:  Yk - H·Xkp
        innov = z - H @ self.Xk

        # S = H·Pkp·Hᵀ + R
        S = H @ self.Pk @ H.T + R

        # K = Pkp·Hᵀ / S
        K = self.Pk @ H.T @ np.linalg.inv(S)

        # Correct state and covariance
        self.Xk = self.Xk + K @ innov
        self.Pk = (np.eye(2) - K @ H) @ self.Pk

    # ── Public: call once per control loop cycle ─────────────────────────────
    def step(self, sensors, enc_left=None, enc_right=None):
        """
        sensors   : your Sensors object (already updated from serial)
        enc_left  : cumulative left encoder count  (int, optional)
        enc_right : cumulative right encoder count (int, optional)

        Returns fused position in cm.
        """

        # ════════════════════════════════════════════════
        # STEP 1 — PREDICT
        # Xkp = A·Xk-1 + B·uk
        # Pkp = A·Pk-1·Aᵀ + Q
        # ════════════════════════════════════════════════

        # uk = forward acceleration from IMU (cm/s²)
        uk = sensors.accel_x * 100.0   # m/s² → cm/s²

        self.Xk = self.A @ self.Xk + self.B * uk
        self.Pk = self.A @ self.Pk @ self.A.T + self.Q

        # ════════════════════════════════════════════════
        # STEP 2 — UPDATE (one call per sensor)
        # Each sensor has its own H and R
        # ════════════════════════════════════════════════

        # ── UPDATE 1: Wheel encoders → position ──
        if enc_left is not None and enc_right is not None:
            if self._prev_enc_left is not None:
                dl = (enc_left  - self._prev_enc_left)  / self.counts_per_cm
                dr = (enc_right - self._prev_enc_right) / self.counts_per_cm
                delta_cm     = (dl + dr) / 2.0
                pos_from_enc = float(self.Xk[0][0]) + delta_cm
                self._update(pos_from_enc, self.H_pos, self.R["encoder"])
            self._prev_enc_left  = enc_left
            self._prev_enc_right = enc_right

        # ── UPDATE 2: IMU velocity correction ──
        # Integrates acceleration → velocity, corrects Xk[1]
        self._imu_vel += uk * self.dt
        self._imu_vel *= 0.98              # dampen integration drift
        self._update(self._imu_vel, self.H_vel, self.R["imu"])

        # ── UPDATE 3: Front air ultrasonic → position ──
        # sensor reads distance-to-wall → convert to distance-from-home
        us_air = sensors.ultrasonic
        if 2.0 < us_air < self.arena - 2.0:
            pos_from_us = self.arena - us_air
            self._update(pos_from_us, self.H_pos, self.R["us_air"])

        # ── UPDATE 4: Front waterproof ultrasonic → position ──
        us_water = sensors.waterPultrasonic
        if 2.0 < us_water < self.arena - 2.0:
            pos_from_water = self.arena - us_water
            self._update(pos_from_water, self.H_pos, self.R["us_water"])

        # ── UPDATE 5: Sharp IR (back) → position ──
        # IR reads distance from back wall = distance from home directly
        # Uncomment when sharp_ir_distance is enabled in Sensors
        # ir = sensors.sharp_ir_distance
        # if 2.0 < ir < self.arena - 2.0:
        #     self._update(ir, self.H_pos, self.R["ir"])

        # Clamp position to arena bounds
        self.Xk[0] = np.clip(self.Xk[0], 0.0, self.arena)

        return float(self.Xk[0][0])

    # ── Readable properties ──────────────────────────────────────────────────
    @property
    def position(self) -> float:
        """Fused position in cm from home. Access as robot.fusion.position"""
        return float(self.Xk[0][0])

    @property
    def velocity(self) -> float:
        """Fused velocity in cm/s. Access as robot.fusion.velocity"""
        return float(self.Xk[1][0])

    @property
    def uncertainty(self) -> float:
        """1-sigma position uncertainty in cm. Shrinks as sensors agree."""
        return float(np.sqrt(self.Pk[0, 0]))

    def reset(self):
        """Call this when robot returns to home position."""
        self.Xk = np.array([[0.0], [0.0]])
        self.Pk = np.eye(2) * 1.0
        self._prev_enc_left  = None
        self._prev_enc_right = None
        self._imu_vel        = 0.0

    # ── Challenge 2: degrade individual sensors ──────────────────────────────
    def degrade(self, sensor: str, factor: float = 10.0):
        """
        Increase R for a sensor to simulate degraded arena conditions.
        sensor : "encoder" | "imu" | "us_air" | "us_water" | "ir"
        factor : multiplier on noise (default 10× = mostly ignored)

        Example:
            robot.fusion.degrade("encoder")    # slippery mat
            robot.fusion.degrade("us_air")     # angled reflector
            robot.fusion.degrade("ir")         # retroreflective tape
        """
        if sensor in self.R:
            self.R[sensor] *= factor

    def restore(self):
        """Reset all R values to defaults after challenge."""
        self.R = {
            "encoder":  0.3,
            "imu":      1.5,
            "us_air":   1.0,
            "us_water": 0.8,
            "ir":       2.0,
        }

class RobotModel:
    HEADER = 0xAA

    def __init__(self, port, baudrate=115200):
        self.ser = serial.Serial(port, baudrate, timeout=0.1)

        self.sensors = Sensors()
        self.fusion = PositionFusion()
        self.scalar_fusion = ScalarPositionFusion()
        self.plotter = Plotter()
        self.weighted_fusion = WeightedPositionFusion()

        self.running = True
        self.rx_thread = threading.Thread(target=self._receive_loop)
        self.rx_thread.start()


    # TRANSMIT VELOCITY
    def send_velocity(self, vx, vy, vw):
        packet = struct.pack("<Bfff", self.HEADER, vx, vy, vw)
        self.ser.write(packet)

    # RECEIVE LOOP
    def _receive_loop(self):
        while self.running:

            # Read 1 byte at a time until header is found
            byte = self.ser.read(1)

            if not byte:
                continue

            if byte[0] == self.HEADER:

                # Now read full packet
                data = self.ser.read(self.sensors.SIZE)

                if len(data) == self.sensors.SIZE:
                    self.sensors.update_from_bytes(data)
                    self.fusion.step(self.sensors)
                    self.scalar_fusion.step(self.sensors)
                    self.weighted_fusion.step(self.sensors,
                          self.sensors.encoder_left,
                          self.sensors.encoder_right)
                    

    def goto_pose(self, loc, target_x, target_y, target_theta,
                xy_tol=1, theta_tol=2,
                kp_lin=1.5, kp_ang=2.0,
                max_lin=0.3, max_ang=1.0):

        TIMEOUT = 5.0
        start = time.time()

        while True:
            cx, cy, ctheta = loc.compute_odometry(self.sensors)

            dx = target_x - cx
            dy = target_y - cy
            dist_err = dx

            angle_to_goal = math.atan2(dy, dx)
            heading_err = self._wrap_angle(angle_to_goal - ctheta)
            final_heading_err = self._wrap_angle(target_theta - ctheta)

            # ── Termination ──────────────────────────────────────────────────
            if dist_err < xy_tol and abs(final_heading_err) < theta_tol:
                self.send_velocity(0, 0, 0)
                return True

            if time.time() - start > TIMEOUT:
                self.send_velocity(0, 0, 0)
                print(f"[goto_pose] Timeout! dist={dist_err:.3f}m")
                return False

            # ── Control law ──────────────────────────────────────────────────
            if dist_err > xy_tol:
                # First rotate to face the goal, then drive forward
                vx = self._clamp(kp_lin * dist_err * math.cos(heading_err), -max_lin, max_lin)
                vw = self._clamp(kp_ang * heading_err, -max_ang, max_ang)
            else:
                # At goal XY — spin to final heading
                vx = 0.0
                vw = self._clamp(kp_ang * final_heading_err, -max_ang, max_ang)

            self.send_velocity(vx, 0, vw)
            time.sleep(0.01)

    @staticmethod
    def _wrap_angle(angle):
        return (angle + math.pi) % (2 * math.pi) - math.pi

    @staticmethod
    def _clamp(val, lo, hi):
        return max(lo, min(hi, val))

    # GET SENSOR OBJECT
    def get_sensors(self):
        return self.sensors
    
    def plot(self, *args):
        self.plotter.plot(*args)

    def start_plot(self, on_close=None):
        self.plotter.start(on_close=on_close)
    
    # CLEAN SHUTDOWN
    def close(self):
        self.running = False
        self.rx_thread.join()
        self.ser.close()
  
    
class Plotter:
    COLORS = ["steelblue", "tomato", "mediumseagreen", "mediumpurple", "orange", "deeppink"]

    def __init__(self, max_points=100, interval_ms=50, y_range=None, title="Serial Plotter", ylabel="Value"):
        self.max_points  = max_points
        self.interval_ms = interval_ms
        self.y_range     = y_range
        self.title       = title
        self.ylabel      = ylabel

        self._signals = {}   # name -> deque
        self._lines   = {}   # name -> Line2D
        self._fig     = None
        self._ax      = None
        self._ani     = None
        self._started = False
        self._stop_event = threading.Event()

    def plot(self, *args):
        """
        Call this anywhere in your loop:
            robot.plot("gyro_x", sensors.gyro_x, "gyro_y", sensors.gyro_y)
        Automatically registers new signals on first sight.
        """
        if len(args) % 2 != 0:
            raise ValueError("Arguments must be name/value pairs")

        for i in range(0, len(args), 2):
            name  = args[i]
            value = float(args[i + 1])

            if name not in self._signals:
                self._signals[name] = deque([0.0] * self.max_points, maxlen=self.max_points)

            self._signals[name].append(value)

    def start(self, on_close=None):
        """
        Call once from your main thread AFTER your control loop is running.
        Blocks until the window is closed.
        """
        self._fig, self._ax = plt.subplots()
        self._ax.set_xlim(0, self.max_points)
        self._ax.set_title(self.title)
        self._ax.set_xlabel("Samples")
        self._ax.set_ylabel(self.ylabel)
        self._ax.grid(True, alpha=0.3)

        if self.y_range:
            self._ax.set_ylim(*self.y_range)

        if on_close:
            self._fig.canvas.mpl_connect("close_event", lambda _: on_close())

        self._ani = animation.FuncAnimation(
            self._fig,
            self._animate,
            interval=self.interval_ms,
            blit=False,
            cache_frame_data=False,
        )

        plt.tight_layout()
        plt.show()
        self._stop_event.set()

    def _animate(self, _frame):
        # Register any new signals that appeared since last frame
        for name in self._signals:
            if name not in self._lines:
                color = self.COLORS[len(self._lines) % len(self.COLORS)]
                line, = self._ax.plot([], [], label=name, color=color, linewidth=1.5)
                self._lines[name] = line
                self._ax.legend(loc="upper right")

        x = list(range(self.max_points))
        for name, line in self._lines.items():
            line.set_data(x, list(self._signals[name]))

        if not self.y_range:
            all_vals = [v for d in self._signals.values() for v in d]
            if all_vals:
                mn, mx = min(all_vals), max(all_vals)
                pad = max(1.0, (mx - mn) * 0.15)
                self._ax.set_ylim(mn - pad, mx + pad)

    def stop(self):
        self._stop_event.set()
        plt.close(self._fig)

class ScalarPositionFusion:
    """
    Scalar (1D) Kalman filter for position estimation.
    No velocity state — designed for slow robots needing
    a steady, trustworthy distance reading.

    Same 5 Kalman equations as PositionFusion but:
        - State is a single float (position only)
        - No matrices, no velocity, no IMU acceleration
        - Predict simply holds position, grows uncertainty
        - Update pulls estimate toward each sensor reading

    Access:
        fusion = ScalarPositionFusion()
        fusion.step(sensors)
        fusion.position     → clean fused position in cm
        fusion.uncertainty  → ±1σ confidence in cm
    """

    def __init__(self, arena_length_cm=90.0):
        self.arena = arena_length_cm

        # ── State: single position float ──
        self.Xk = 0.0    # position in cm from home
        self.Pk = 1.0    # uncertainty (scalar, not matrix)

        # ── Process noise: how much uncertainty grows each cycle ──
        self.Q  = 0.5

        # ── Measurement noise per sensor (lower = more trusted) ──
        self.R  = {
            "encoder":  0.3,
            "us_air":   1.0,
            "us_water": 0.8,
            "ir":       2.0,
        }
        self.OFFSETS = {"us_air":-11.5, "us_water": 11.5, "ir": -11.5}
        self._has_moved = False

        self._prev_enc_left  = None
        self._prev_enc_right = None

    # ── Single sensor update (pure scalar math) ──────────────────────────────
    def _update(self, z, R):
        """
        Xkp = Xk  (predict — static model)
        Pkp = Pk + Q

        K   = Pkp / (Pkp + R)
        Xk  = Xkp + K * (z - Xkp)
        Pk  = (1 - K) * Pkp
        """
        K       = self.Pk / (self.Pk + R)       # Kalman gain
        self.Xk = self.Xk + K * (z - self.Xk)  # correct estimate
        self.Pk = (1 - K) * self.Pk             # shrink uncertainty

    # ── Main update — call once per loop cycle ───────────────────────────────
    def step(self, sensors, enc_left=None, enc_right=None,
             counts_per_cm=20):
        """
        sensors        : Sensors object
        enc_left/right : cumulative encoder counts (optional)
        counts_per_cm  : encoder calibration value
        """

        # ── PREDICT ──
        # position assumed static — holds last estimate
        # uncertainty grows by Q each cycle
        self.Pk += self.Q

        # ── UPDATE 1: Encoders → position ──
        if enc_left is not None and enc_right is not None:
            if self._prev_enc_left is not None:
                dl = (enc_left  - self._prev_enc_left)  / counts_per_cm
                dr = (enc_right - self._prev_enc_right) / counts_per_cm
                if abs(dl + dr) > 0.1:          # only flag if real movement
                    self._has_moved = True
                pos_from_enc = self.Xk + (dl + dr) / 2.0
                self._update(pos_from_enc, self.R["encoder"])
            self._prev_enc_left  = enc_left
            self._prev_enc_right = enc_right

        if not self._has_moved:
            return self.Xk
        # ── UPDATE 2: Front air ultrasonic → position ──
        us_air = sensors.ultrasonic
        if 2.0 < us_air < self.arena - 2.0:
            self._update(us_air - self.OFFSETS["us_air"], self.R["us_air"])

        # ── UPDATE 3: Front waterproof ultrasonic → position ──
        us_water = sensors.waterPultrasonic
        if 2.0 < us_water < self.arena - 2.0:
            self._update(self.arena - us_water - self.OFFSETS["us_water"], self.R["us_water"])

        # ── UPDATE 4: Sharp IR (back) → position ──
        ir = sensors.sharp_ir_distance
        if 2.0 < ir < self.arena - 2.0:
            self._update(ir - self.OFFSETS["ir"], self.R["ir"])

        self.Xk = max(0.0, min(self.arena, self.Xk))
        return self.Xk

    @property
    def position(self) -> float:
        """Clean fused position in cm from home."""
        return self.Xk

    @property
    def uncertainty(self) -> float:
        """±1σ confidence in cm. Grows when no sensors fire, shrinks on update."""
        return math.sqrt(self.Pk)

    def reset(self):
        """Call when robot returns to home."""
        self.Xk = 0.0
        self.Pk = 1.0
        self._prev_enc_left  = None
        self._prev_enc_right = None
        self._has_moved = False

    def degrade(self, sensor: str, factor: float = 10.0):
        """Challenge 2 — increase noise for a sensor."""
        if sensor in self.R:
            self.R[sensor] *= factor

    def restore(self):
        """Reset sensor noise to defaults."""
        self.R = {"encoder": 0.3, "us_air": 1.0, "us_water": 0.8, "ir": 2.0}


class WeightedPositionFusion:
    """
    Simple weighted-average fusion of encoder, ultrasonic, and IR.
    Weights are based on 1/R (inverse noise variance) — same R values as
    ScalarPositionFusion so the two are directly comparable in the GUI.

    Access after calling step():
        fusion.position      → weighted-average position in cm
        fusion.weights       → dict of normalised weights used last step
        fusion.contributions → dict of each sensor's cm contribution
    """

    ARENA = 90.0
    OFFSETS = {"us_air": -11.5, "us_water": 11.5, "ir": -11.5}

    # Same R as ScalarPositionFusion for fair comparison
    R_DEFAULT = {
        "encoder":  0.3,
        "us_air":   1.0,
        "us_water": 0.8,
        "ir":       2.0,
    }

    def __init__(self, arena_length_cm=90.0, counts_per_cm=20):
        self.arena        = arena_length_cm
        self.counts_per_cm = counts_per_cm

        self.R = dict(self.R_DEFAULT)

        self.position      = 0.0
        self.weights       = {}
        self.contributions = {}

        self._prev_enc_left  = None
        self._prev_enc_right = None
        self._enc_pos        = 0.0   # dead-reckoned encoder position

    def step(self, sensors, enc_left=None, enc_right=None):
        """
        Call once per loop cycle.  Returns weighted-average position in cm.
        """
        readings = {}

        # ── Encoder dead-reckoning ──────────────────────────────────────
        if enc_left is not None and enc_right is not None:
            if self._prev_enc_left is not None:
                dl = (enc_left  - self._prev_enc_left)  / self.counts_per_cm
                dr = (enc_right - self._prev_enc_right) / self.counts_per_cm
                self._enc_pos += (dl + dr) / 2.0
            self._prev_enc_left  = enc_left
            self._prev_enc_right = enc_right
            readings["encoder"] = self._enc_pos

        # ── Ultrasonic (front, air) ─────────────────────────────────────
        us = sensors.ultrasonic
        if 2.0 < us < self.arena - 2.0:
            readings["us_air"] = us - self.OFFSETS["us_air"]

        # ── Waterproof ultrasonic ───────────────────────────────────────
        usw = sensors.waterPultrasonic
        if 2.0 < usw < self.arena - 2.0:
            readings["us_water"] = self.arena - usw - self.OFFSETS["us_water"]

        # ── Sharp IR (back) ─────────────────────────────────────────────
        ir = sensors.sharp_ir_distance
        if 2.0 < ir < self.arena - 2.0:
            readings["ir"] = ir - self.OFFSETS["ir"]

        if not readings:
            return self.position   # nothing to fuse

        # ── Weighted average: w_i = 1 / R_i ────────────────────────────
        raw_weights = {k: 1.0 / self.R[k] for k in readings}
        total_w     = sum(raw_weights.values())
        norm_w      = {k: raw_weights[k] / total_w for k in raw_weights}

        self.position      = sum(norm_w[k] * readings[k] for k in readings)
        self.position      = max(0.0, min(self.arena, self.position))
        self.weights       = norm_w
        self.contributions = {k: norm_w[k] * readings[k] for k in readings}
        return self.position

    def reset(self):
        self.position     = 0.0
        self._prev_enc_left  = None
        self._prev_enc_right = None
        self._enc_pos     = 0.0
        self.weights      = {}
        self.contributions = {}

    def set_weight(self, sensor: str, r_value: float):
        """Manually override R for a sensor (used by GUI sliders)."""
        if sensor in self.R:
            self.R[sensor] = max(r_value, 0.01)

    def restore(self):
        self.R = dict(self.R_DEFAULT)

class Localization:
    # Robot physical parameters
    CPR            = 460                          # encoder counts per revolution
    WHEEL_DIAMETER = 8.0                          # cm
    WHEEL_BASE     = 15.0                         # cm (150 mm between wheels)
    DIST_PER_TICK  = math.pi * WHEEL_DIAMETER / CPR   # cm per encoder tick
 
    def __init__(self):
        self.prev_left  = None
        self.prev_right = None
 
        # Full 2D pose
        self.x     = 0.0   # cm
        self.y     = 0.0   # cm
        self.theta = 0.0   # radians
 
    def compute_odometry(self, sensors):
        """
        Call once per sensor packet.
        Updates and returns (x, y, theta) — position in cm, heading in radians.
        """
        left  = sensors.encoder_left
        right = sensors.encoder_right
 
        # First call — initialise, no movement yet
        if self.prev_left is None:
            self.prev_left  = left
            self.prev_right = right
            return self.x, self.y, self.theta
 
        # Ticks since last call → distance each wheel travelled (cm)
        dl = (left  - self.prev_left)  * self.DIST_PER_TICK
        dr = (right - self.prev_right) * self.DIST_PER_TICK
 
        self.prev_left  = left
        self.prev_right = right
 
        # Differential drive equations
        d      = (dl + dr) / 2.0               # forward distance (cm)
        dtheta = (dr - dl) / self.WHEEL_BASE   # change in heading (radians)
 
        # Update pose
        self.theta += dtheta
        self.x     += d * math.cos(self.theta)
        self.y     += d * math.sin(self.theta)
 
        return self.x, self.y, self.theta
 
    def reset(self):
        """Reset pose to origin."""
        self.x = self.y = self.theta = 0.0
        self.prev_left = self.prev_right = None


class WallTracker:
    """
    Tracks wall segments detected by the right IR sensor using encoder
    odometry — mirrors the logic in the manual measurement script.
 
    Usage (called automatically inside RobotModel._receive_loop):
        tracker = WallTracker(cpr=460, wheel_diameter_cm=8)
        tracker.update(ir_right_state, avg_encoder_ticks)
 
    Completed walls are queued in tracker.completed_walls as dicts:
        {
            "length_cm": float,          # measured wall length
            "start_encoder": float,      # avg encoder at wall start
            "end_encoder":   float,      # avg encoder at wall end
            "timestamp":     float,      # time.time() when wall ended
        }
 
    Active-wall state is exposed for the live GUI:
        tracker.wall_active       → bool
        tracker.current_length_cm → float (grows while wall is detected)
    """
 
    def __init__(self, cpr: int = 460, wheel_diameter_cm: float = 8.0):
        self.distance_per_tick = math.pi * wheel_diameter_cm / cpr
 
        # internal state
        self._ir_prev         = 0
        self._encoder_start   = 0.0
        self._wall_active     = False
 
        # public read-only state
        self.wall_active        = False
        self.current_length_cm  = 0.0
 
        # completed wall queue — consumed by get_maze_state()
        self.completed_walls: list = []
        self._lock = threading.Lock()
 
    def update(self, ir_state: int, avg_encoder: float):
        """
        Call once per sensor packet.
 
        ir_state    : 1 = wall present, 0 = no wall  (matches sensors.ir_right)
        avg_encoder : (encoder_left + encoder_right) / 2  — cumulative ticks
        """
 
        # ── WALL RISING EDGE ────────────────────────────────────────
        if ir_state == 1 and self._ir_prev == 0:
            self._encoder_start = avg_encoder
            self._wall_active   = True
            self.current_length_cm = 0.0
 
        # ── WALL FALLING EDGE ───────────────────────────────────────
        if ir_state == 0 and self._ir_prev == 1 and self._wall_active:
            ticks       = avg_encoder - self._encoder_start
            wall_length = ticks * self.distance_per_tick
            entry = {
                "length_cm":      wall_length,
                "start_encoder":  self._encoder_start,
                "end_encoder":    avg_encoder,
                "timestamp":      time.time(),
            }
            with self._lock:
                self.completed_walls.append(entry)
            self._wall_active      = False
            self.current_length_cm = 0.0
 
        # ── WHILE WALL IS ACTIVE — update live length ───────────────
        if self._wall_active:
            ticks = avg_encoder - self._encoder_start
            self.current_length_cm = ticks * self.distance_per_tick
 
        # expose cleaned state
        self.wall_active = self._wall_active
        self._ir_prev    = ir_state
 
    def pop_completed_walls(self) -> list:
        """
        Return and clear all completed wall entries.
        Thread-safe — call from the GUI / main thread.
        """
        with self._lock:
            walls = list(self.completed_walls)
            self.completed_walls.clear()
        return walls