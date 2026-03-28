# """
# Robot Sensor Challenge — PyQt5 GUI  (v3)
# =========================================
# Educational robot challenge: move 25 cm using one sensor at a time.

# Key improvements in this version
# ─────────────────────────────────
# • Chart shows DISTANCE (cm) for both Current and Actual — always apples-to-apples
# • Live conversion formula shown per sensor  (e.g. "ticks ÷ 41.2 = cm")
# • Sensor Report Card appears after STOP:
#     – Prediction accuracy
#     – Noise / jitter rating (live variance of last 10 samples)
#     – A one-liner limitation hint per sensor
# • Session log panel: records each attempt across all sensors for comparison
# • Layout uses the right column space fully

# Run  :  python robot_sensor_gui.py
# Needs:  pip install PyQt5
# """

# import sys, random, collections, math, os
# from PyQt5.QtWidgets import (
#     QApplication, QMainWindow, QWidget, QLabel, QPushButton,
#     QComboBox, QLineEdit, QVBoxLayout, QHBoxLayout, QGridLayout,
#     QFrame, QSizePolicy, QMessageBox, QScrollArea
# )
# from PyQt5.QtCore import Qt, QTimer, QRectF, QPointF
# from PyQt5.QtGui  import QPainter, QColor, QPen, QBrush, QFont, QLinearGradient, QPolygonF

# # ── Import fusion from snp_model (same directory) ─────────────────────────────
# _HERE = os.path.dirname(os.path.abspath(__file__))
# if _HERE not in sys.path:
#     sys.path.insert(0, _HERE)
# try:
#     from robot_core.snp_model import ScalarPositionFusion
#     _FUSION_AVAILABLE = True
# except ImportError:
#     _FUSION_AVAILABLE = False
#     ScalarPositionFusion = None

# # ── Constants ──────────────────────────────────────────────────────────────────
# ARENA_CM  = 90.0
# TARGET_CM = 25.0
# TOLERANCE = 0.5          # ±0.5 cm = ±5 mm

# # Conversion: raw reading → cm distance travelled
# # distance_cm = (reading - start) / scale  (scale can be negative for ultrasonic)
# SENSORS = {
#     "Encoder — Left Wheel": {
#         "key":        "encoder_left",
#         "unit":       "ticks",
#         "hint":       "Count encoder ticks. How many ticks equal 25 cm of travel?",
#         "sim_scale":  41.2,
#         "sim_start":  0.0,
#         "formula":    "ticks ÷ 41.2  =  cm travelled",
#         "to_cm":      lambda r, start=0.0,  scale=41.2: (r - start) / scale,
#         "limitation": "Encoders accumulate drift — small slip errors add up over distance.",
#         "noise_char": "low",
#     },
#     "Ultrasonic — Front": {
#         "key":        "ultrasonic",
#         "unit":       "cm",
#         "hint":       "Arena is 90 cm. Sensor reads distance to front wall. After 25 cm moved → what reading?",
#         "sim_scale":  -1.0,
#         "sim_start":  90.0,
#         "formula":    "90 − reading  =  cm travelled",
#         "to_cm":      lambda r, start=90.0, scale=-1.0: (start - r),
#         "limitation": "Ultrasonic can't distinguish surfaces < 3 cm apart and bounces off angled walls.",
#         "noise_char": "medium",
#     },
#     "Sharp IR — Back": {
#         "key":        "sharp_ir_distance",
#         "unit":       "cm",
#         "hint":       "Measures distance behind the robot. Starts ~5 cm. What after 25 cm forward?",
#         "sim_scale":  1.0,
#         "sim_start":  5.0,
#         "formula":    "reading − 5  =  cm travelled",
#         "to_cm":      lambda r, start=5.0,  scale=1.0:  (r - start),
#         "limitation": "Sharp IR is highly nonlinear beyond 30 cm and sensitive to surface colour.",
#         "noise_char": "high",
#     },
# }

# NOISE_COLORS = {"low": "#00e676", "medium": "#ffd740", "high": "#ff4d6d"}

# # ── Palette ────────────────────────────────────────────────────────────────────
# C_BG      = "#0b0e17"
# C_SURFACE = "#111520"
# C_CARD    = "#161b2e"
# C_BORDER  = "#1e2540"
# C_ACCENT  = "#00d4ff"
# C_ACCENT2 = "#ff4d6d"
# C_SUCCESS = "#00e676"
# C_FAIL    = "#ff1744"
# C_YELLOW  = "#ffd740"
# C_TEXT    = "#dce6f5"
# C_MUTED   = "#4a5a7a"
# C_CURRENT = "#00d4ff"
# C_ACTUAL  = "#ff4d6d"
# FONT      = "Consolas"


# # ══════════════════════════════════════════════════════════════════════════════
# class SimRobot:
#     def __init__(self):
#         self.position_cm = 0.0

#     def move(self, delta):
#         self.position_cm = max(0.0, min(ARENA_CM, self.position_cm + delta))

#     def get_reading(self, name):
#         m = SENSORS[name]
#         noise = {"Encoder — Left Wheel": random.uniform(-2, 2),
#                  "Ultrasonic — Front":   random.uniform(-0.5, 0.5),
#                  "Sharp IR — Back":      random.uniform(-0.8, 0.8)}[name]
#         return round(m["sim_start"] + m["sim_scale"] * self.position_cm + noise, 1)

#     def get_position(self):
#         return self.position_cm

#     def reset(self):
#         self.position_cm = 0.0


# # ══════════════════════════════════════════════════════════════════════════════
# class SimSensors:
#     """
#     Minimal Sensors-compatible object for use with ScalarPositionFusion
#     in simulation mode.  Mirrors the attribute names expected by step().
#     """
#     def __init__(self):
#         self.sharp_ir_distance = 0.0
#         self.waterPultrasonic  = 0.0
#         self.ultrasonic        = 0.0
#         self.encoder_left      = 0
#         self.encoder_right     = 0

#     def update(self, robot: "SimRobot", name: str):
#         """Populate from SimRobot state so fusion gets realistic readings."""
#         pos = robot.get_position()
#         # Ultrasonic: reads distance to front wall
#         self.ultrasonic        = round(ARENA_CM - pos + random.uniform(-0.3, 0.3), 1)
#         # Waterproof ultrasonic: reads distance from back wall (same as pos)
#         self.waterPultrasonic  = round(pos + random.uniform(-0.3, 0.3), 1)
#         # Sharp IR: reads distance behind robot (approx pos + 5 cm offset)
#         self.sharp_ir_distance = round(pos + 5.0 + random.uniform(-0.5, 0.5), 1)
#         # Encoders: cumulative ticks
#         self.encoder_left      = int(pos * 41.2 + random.uniform(-1, 1))
#         self.encoder_right     = int(pos * 41.2 + random.uniform(-1, 1))


# # ══════════════════════════════════════════════════════════════════════════════
# class RobotTopView(QWidget):
#     """
#     Top-down (bird's eye) view of the robot inside its 90 cm arena.

#     Layout (landscape orientation):
#       ┌─────────────────────────────────────┐
#       │  WALL          arena 90 cm         │
#       │  ┌──────────────────────────────┐  │
#       │  │  [US]  ╔══════╗  [US_W]     │  │
#       │  │        ║ BODY ║             │  │
#       │  │   [IR] ╚══════╝             │  │
#       │  └──────────────────────────────┘  │
#       │  HOME ──────────────→ 25 cm target │
#       └─────────────────────────────────────┘

#     The robot body (rectangle) slides right as position increases.
#     A dashed target line shows the 25 cm goal.
#     Sensor cone arcs are drawn to show what each sensor sees.
#     """

#     def __init__(self, parent=None):
#         super().__init__(parent)
#         self.setMinimumSize(300, 180)
#         self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
#         self._pos_cm      = 0.0   # robot position (cm)
#         self._fusion_cm   = 0.0   # fusion position (shown separately)
#         self._active_sensor = ""  # currently selected sensor name

#     def set_position(self, pos_cm: float, fusion_cm: float = None):
#         self._pos_cm    = max(0.0, min(ARENA_CM, pos_cm))
#         if fusion_cm is not None:
#             self._fusion_cm = max(0.0, min(ARENA_CM, fusion_cm))
#         self.update()

#     def set_sensor(self, name: str):
#         self._active_sensor = name
#         self.update()

#     def paintEvent(self, _):
#         p = QPainter(self)
#         p.setRenderHint(QPainter.Antialiasing)

#         W, H  = self.width(), self.height()

#         # ── Margins & arena geometry ──────────────────────────────────────
#         PAD_L  = 10
#         PAD_R  = 10
#         PAD_T  = 32     # room for title + labels above
#         PAD_B  = 26     # room for distance ruler below
#         AW     = W - PAD_L - PAD_R   # arena pixel width
#         AH     = H - PAD_T - PAD_B   # arena pixel height

#         def cm_to_px(cm):
#             return PAD_L + int(AW * cm / ARENA_CM)

#         # ── Background ────────────────────────────────────────────────────
#         p.fillRect(0, 0, W, H, QColor(C_CARD))

#         # Title
#         p.setPen(QColor(C_MUTED))
#         p.setFont(QFont(FONT, 8))
#         p.drawText(0, 6, W, 18, Qt.AlignCenter, "ARENA — TOP VIEW")

#         # ── Arena floor ───────────────────────────────────────────────────
#         p.setBrush(QColor("#0d1120"))
#         p.setPen(QPen(QColor(C_BORDER), 1))
#         p.drawRect(PAD_L, PAD_T, AW, AH)

#         # Grid lines every 10 cm
#         p.setPen(QPen(QColor(C_BORDER), 1, Qt.DotLine))
#         for cm in range(10, int(ARENA_CM), 10):
#             x = cm_to_px(cm)
#             p.drawLine(x, PAD_T, x, PAD_T + AH)

#         # ── Target line at 25 cm ──────────────────────────────────────────
#         tx = cm_to_px(TARGET_CM)
#         p.setPen(QPen(QColor(C_YELLOW), 2, Qt.DashLine))
#         p.drawLine(tx, PAD_T, tx, PAD_T + AH)
#         p.setPen(QColor(C_YELLOW))
#         p.setFont(QFont(FONT, 7, QFont.Bold))
#         p.drawText(tx - 20, PAD_T + 3, 40, 14, Qt.AlignCenter, "25 cm")

#         # ── Robot geometry ────────────────────────────────────────────────
#         # Robot body: 12 cm × 10 cm → scaled to pixels
#         R_W_CM = 12.0
#         R_H_CM = 10.0
#         r_px_w = int(AW * R_W_CM / ARENA_CM)
#         r_px_h = int(AH * R_H_CM / ARENA_CM) if AH > 0 else 30
#         r_px_h = max(r_px_h, 28)   # minimum visible height

#         robot_x = cm_to_px(self._pos_cm)             # left edge of robot
#         robot_cy = PAD_T + AH // 2                   # vertical centre

#         # ── Sensor cones ──────────────────────────────────────────────────
#         sensor_key = SENSORS.get(self._active_sensor, {}).get("key", "")

#         # Front ultrasonic — cyan cone pointing right
#         if "ultrasonic" in sensor_key and "water" not in sensor_key:
#             cone_x   = robot_x + r_px_w
#             cone_len = int(AW * (ARENA_CM - self._pos_cm - R_W_CM) / ARENA_CM)
#             cone_len = max(cone_len, 4)
#             cone_half = r_px_h // 3
#             pts = QPolygonF([
#                 QPointF(cone_x, robot_cy),
#                 QPointF(cone_x + cone_len, robot_cy - cone_half),
#                 QPointF(cone_x + cone_len, robot_cy + cone_half),
#             ])
#             cone_col = QColor(C_ACCENT); cone_col.setAlpha(55)
#             p.setBrush(QBrush(cone_col))
#             p.setPen(QPen(QColor(C_ACCENT), 1, Qt.DotLine))
#             p.drawPolygon(pts)

#         # Sharp IR — red cone pointing left (back sensor)
#         elif sensor_key == "sharp_ir_distance":
#             cone_x   = robot_x
#             cone_len = int(AW * max(self._pos_cm - R_W_CM * 0.5, 2) / ARENA_CM)
#             cone_len = max(cone_len, 4)
#             cone_half = r_px_h // 4
#             pts = QPolygonF([
#                 QPointF(cone_x, robot_cy),
#                 QPointF(cone_x - cone_len, robot_cy - cone_half),
#                 QPointF(cone_x - cone_len, robot_cy + cone_half),
#             ])
#             cone_col = QColor(C_ACCENT2); cone_col.setAlpha(55)
#             p.setBrush(QBrush(cone_col))
#             p.setPen(QPen(QColor(C_ACCENT2), 1, Qt.DotLine))
#             p.drawPolygon(pts)

#         # ── Draw wheels (4 corner rectangles) ────────────────────────────
#         w_w = max(r_px_w // 7, 4)
#         w_h = max(r_px_h // 4, 6)
#         wheel_col = QColor("#2a3050")
#         p.setBrush(QBrush(wheel_col)); p.setPen(Qt.NoPen)
#         # Front wheels
#         p.drawRect(robot_x + r_px_w - w_w,
#                    robot_cy - r_px_h // 2 - w_h // 2, w_w, w_h)
#         p.drawRect(robot_x + r_px_w - w_w,
#                    robot_cy + r_px_h // 2 - w_h // 2, w_w, w_h)
#         # Rear wheels
#         p.drawRect(robot_x,
#                    robot_cy - r_px_h // 2 - w_h // 2, w_w, w_h)
#         p.drawRect(robot_x,
#                    robot_cy + r_px_h // 2 - w_h // 2, w_w, w_h)

#         # ── Robot body ────────────────────────────────────────────────────
#         body_grad = QLinearGradient(robot_x, robot_cy - r_px_h // 2,
#                                     robot_x, robot_cy + r_px_h // 2)
#         body_grad.setColorAt(0.0, QColor("#1e2a45"))
#         body_grad.setColorAt(1.0, QColor("#141d33"))
#         p.setBrush(QBrush(body_grad))
#         p.setPen(QPen(QColor(C_ACCENT), 1))
#         p.drawRoundedRect(robot_x, robot_cy - r_px_h // 2, r_px_w, r_px_h, 3, 3)

#         # Forward arrow on body
#         arr_x = robot_x + r_px_w // 2
#         p.setPen(QPen(QColor(C_ACCENT), 2))
#         p.drawLine(arr_x - 4, robot_cy, arr_x + 5, robot_cy)
#         p.drawLine(arr_x + 5, robot_cy, arr_x + 2, robot_cy - 3)
#         p.drawLine(arr_x + 5, robot_cy, arr_x + 2, robot_cy + 3)

#         # ── Sensor dot labels ─────────────────────────────────────────────
#         # Front ultrasonic dot (right side)
#         us_x = robot_x + r_px_w - 2
#         p.setBrush(QColor(C_ACCENT)); p.setPen(Qt.NoPen)
#         p.drawEllipse(us_x - 3, robot_cy - r_px_h // 2 + 3, 6, 6)
#         p.setPen(QColor(C_ACCENT)); p.setFont(QFont(FONT, 6))
#         p.drawText(us_x - 10, robot_cy - r_px_h // 2 + 10, 20, 10,
#                    Qt.AlignCenter, "US")

#         # Back IR dot (left side)
#         ir_x = robot_x + 2
#         p.setBrush(QColor(C_ACCENT2)); p.setPen(Qt.NoPen)
#         p.drawEllipse(ir_x - 3, robot_cy + r_px_h // 2 - 9, 6, 6)
#         p.setPen(QColor(C_ACCENT2)); p.setFont(QFont(FONT, 6))
#         p.drawText(ir_x - 10, robot_cy + r_px_h // 2 - 14, 20, 10,
#                    Qt.AlignCenter, "IR")

#         # ── Encoder wheel highlight when encoder selected ─────────────────
#         if sensor_key == "encoder_left":
#             p.setBrush(Qt.NoBrush)
#             p.setPen(QPen(QColor(C_YELLOW), 2))
#             p.drawRect(robot_x,
#                        robot_cy + r_px_h // 2 - w_h // 2, w_w, w_h)
#             p.setPen(QColor(C_YELLOW)); p.setFont(QFont(FONT, 6))
#             p.drawText(robot_x - 16, robot_cy + r_px_h // 2 + 2, 32, 10,
#                        Qt.AlignCenter, "ENC")

#         # ── Fusion ghost position ─────────────────────────────────────────
#         if abs(self._fusion_cm - self._pos_cm) > 0.5:
#             fx = cm_to_px(self._fusion_cm)
#             p.setPen(QPen(QColor(C_ACTUAL), 1, Qt.DashLine))
#             p.setBrush(Qt.NoBrush)
#             p.drawRect(fx, robot_cy - r_px_h // 2, r_px_w, r_px_h)

#         # ── Distance ruler below arena ────────────────────────────────────
#         ruler_y = PAD_T + AH + 4
#         p.setPen(QColor(C_MUTED))
#         p.setFont(QFont(FONT, 7))
#         for cm in range(0, int(ARENA_CM) + 1, 10):
#             x = cm_to_px(cm)
#             p.drawLine(x, ruler_y, x, ruler_y + 4)
#             p.drawText(x - 12, ruler_y + 5, 24, 12, Qt.AlignCenter, str(cm))

#         # Position readout
#         p.setPen(QColor(C_ACCENT))
#         p.setFont(QFont(FONT, 8, QFont.Bold))
#         p.drawText(W - 80, PAD_T + 4, 76, 16, Qt.AlignRight | Qt.AlignVCenter,
#                    f"{self._pos_cm:.1f} cm")

#         p.end()


# # ══════════════════════════════════════════════════════════════════════════════
# class DistanceBarChart(QWidget):
#     """
#     Two vertical bars always in centimetres:
#       • Current  – distance estimated from selected sensor reading
#       • Actual   – sensor-fusion ground truth (hidden until STOP)
#     A horizontal target line marks 25 cm.
#     """
#     def __init__(self, parent=None):
#         super().__init__(parent)
#         self.setMinimumSize(280, 300)
#         self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
#         self.current_cm   = 0.0
#         self.actual_cm    = 0.0
#         self._show_actual = False

#     def set_values(self, current_cm, actual_cm):
#         self.current_cm = current_cm
#         self.actual_cm  = actual_cm
#         self.update()

#     def reveal(self, show: bool):
#         self._show_actual = show
#         self.update()

#     def paintEvent(self, _):
#         p = QPainter(self)
#         p.setRenderHint(QPainter.Antialiasing)
#         W, H   = self.width(), self.height()
#         PAD_L  = 48
#         PAD_R  = 16
#         PAD_T  = 36
#         PAD_B  = 50
#         CW     = W - PAD_L - PAD_R
#         CH     = H - PAD_T - PAD_B
#         MAX_CM = 35.0   # chart tops out at 35 cm (target is 25)

#         p.fillRect(0, 0, W, H, QColor(C_CARD))

#         # Grid lines + Y labels
#         for i in range(6):
#             val = MAX_CM * i / 5
#             y   = PAD_T + CH - int(CH * i / 5)
#             p.setPen(QPen(QColor(C_BORDER), 1))
#             p.drawLine(PAD_L, y, W - PAD_R, y)
#             p.setPen(QColor(C_MUTED))
#             p.setFont(QFont(FONT, 8))
#             p.drawText(2, y - 8, PAD_L - 6, 18,
#                        Qt.AlignRight | Qt.AlignVCenter, f"{val:.0f}")

#         # Y-axis spine
#         p.setPen(QPen(QColor(C_MUTED), 1))
#         p.drawLine(PAD_L, PAD_T, PAD_L, PAD_T + CH)

#         # TARGET LINE at 25 cm
#         ty = PAD_T + CH - int(CH * TARGET_CM / MAX_CM)
#         p.setPen(QPen(QColor(C_YELLOW), 2, Qt.DashLine))
#         p.drawLine(PAD_L, ty, W - PAD_R, ty)
#         p.setPen(QColor(C_YELLOW))
#         p.setFont(QFont(FONT, 8, QFont.Bold))
#         p.drawText(W - PAD_R - 48, ty - 14, 60, 14,
#                    Qt.AlignLeft, "← 25 cm")

#         # Bars
#         GAP = 20
#         BW  = (CW - GAP * 3) // 2

#         bars = [
#             ("Current\n(sensor)",  self.current_cm, C_CURRENT),
#             ("Actual\n(fusion)",   self.actual_cm,  C_ACTUAL),
#         ]

#         for i, (lbl, val, col) in enumerate(bars):
#             x = PAD_L + GAP + i * (BW + GAP)

#             if lbl.startswith("Actual") and not self._show_actual:
#                 p.setPen(QColor(C_MUTED))
#                 p.setFont(QFont(FONT, 18, QFont.Bold))
#                 p.drawText(x, PAD_T, BW, CH, Qt.AlignCenter, "?")
#                 p.setFont(QFont(FONT, 8))
#                 p.drawText(x, PAD_T + CH + 8,  BW, 18, Qt.AlignCenter, "cm")
#                 for j, line in enumerate(lbl.split("\n")):
#                     p.drawText(x, PAD_T + CH + 22 + j * 14, BW, 14,
#                                Qt.AlignCenter, line)
#                 continue

#             clipped = max(min(val, MAX_CM), 0.0)
#             frac    = clipped / MAX_CM
#             bar_h   = max(int(CH * frac), 3)
#             y_top   = PAD_T + CH - bar_h

#             grad = QLinearGradient(x, y_top, x, y_top + bar_h)
#             c  = QColor(col); c2 = QColor(col); c2.setAlpha(120)
#             grad.setColorAt(0.0, c); grad.setColorAt(1.0, c2)
#             p.setBrush(QBrush(grad)); p.setPen(Qt.NoPen)
#             p.drawRoundedRect(x, y_top, BW, bar_h, 5, 5)

#             # Value label
#             p.setPen(QColor(col))
#             p.setFont(QFont(FONT, 10, QFont.Bold))
#             p.drawText(x, max(y_top - 20, PAD_T), BW, 18,
#                        Qt.AlignCenter, f"{val:.1f} cm")

#             # Axis labels
#             p.setPen(QColor(C_MUTED))
#             p.setFont(QFont(FONT, 8))
#             p.drawText(x, PAD_T + CH + 8,  BW, 18, Qt.AlignCenter, "cm")
#             for j, line in enumerate(lbl.split("\n")):
#                 p.drawText(x, PAD_T + CH + 22 + j * 14, BW, 14,
#                            Qt.AlignCenter, line)

#         # Chart title
#         p.setPen(QColor(C_MUTED))
#         p.setFont(QFont(FONT, 8))
#         p.drawText(0, 8, W, 18, Qt.AlignCenter, "DISTANCE TRAVELLED  (cm)")
#         p.end()


# # ══════════════════════════════════════════════════════════════════════════════
# class NoiseBar(QWidget):
#     """Compact horizontal bar showing live sensor jitter level."""
#     def __init__(self, label="Noise", parent=None):
#         super().__init__(parent)
#         self.setFixedHeight(22)
#         self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
#         self._label  = label
#         self._level  = 0.0   # 0–1

#     def set_level(self, level: float):
#         self._level = max(0.0, min(level, 1.0))
#         self.update()

#     def paintEvent(self, _):
#         p = QPainter(self)
#         p.setRenderHint(QPainter.Antialiasing)
#         W, H = self.width(), self.height()
#         LW   = 80
#         BH   = 10
#         by   = (H - BH) // 2

#         p.fillRect(0, 0, W, H, QColor(C_CARD))

#         # Label
#         p.setPen(QColor(C_MUTED))
#         p.setFont(QFont(FONT, 8))
#         p.drawText(0, 0, LW, H, Qt.AlignVCenter | Qt.AlignLeft, self._label)

#         bx = LW
#         bw = W - LW - 4
#         # Track
#         p.setBrush(QColor(C_BORDER)); p.setPen(Qt.NoPen)
#         p.drawRoundedRect(bx, by, bw, BH, 3, 3)
#         # Fill
#         fw = int(bw * self._level)
#         if fw > 0:
#             lvl = self._level
#             col = QColor("#00e676") if lvl < 0.35 else \
#                   QColor("#ffd740") if lvl < 0.65 else QColor("#ff4d6d")
#             p.setBrush(col)
#             p.drawRoundedRect(bx, by, fw, BH, 3, 3)
#         p.end()


# # ══════════════════════════════════════════════════════════════════════════════
# class KeyWidget(QLabel):
#     def __init__(self, text, parent=None):
#         super().__init__(text, parent)
#         self.setFixedSize(40, 40)
#         self.setAlignment(Qt.AlignCenter)
#         self.set_active(False)

#     def set_active(self, v):
#         if v:
#             self.setStyleSheet(
#                 f"background:{C_ACCENT};color:#000;border-radius:5px;"
#                 f"font-family:{FONT};font-size:14px;font-weight:bold;")
#         else:
#             self.setStyleSheet(
#                 f"background:{C_BORDER};color:{C_TEXT};border-radius:5px;"
#                 f"font-family:{FONT};font-size:14px;font-weight:bold;"
#                 f"border:1px solid {C_MUTED};")


# # ── Widget helpers ─────────────────────────────────────────────────────────────
# def lbl(text, size=10, color=C_TEXT, bold=False):
#     l = QLabel(text)
#     l.setStyleSheet(
#         f"color:{color};font-family:{FONT};font-size:{size}px;"
#         f"font-weight:{'bold' if bold else 'normal'};"
#         f"background:transparent;border:none;")
#     return l

# def btn(text, bg=C_ACCENT, fg="#000", size=11):
#     b = QPushButton(text)
#     b.setStyleSheet(f"""
#         QPushButton{{background:{bg};color:{fg};border:none;border-radius:7px;
#                      padding:8px 16px;font-family:{FONT};font-size:{size}px;font-weight:bold;}}
#         QPushButton:hover   {{background:{bg}cc;}}
#         QPushButton:pressed {{background:{bg}77;}}
#         QPushButton:disabled{{background:{C_MUTED};color:{C_BORDER};}}
#     """)
#     return b

# def card():
#     f = QFrame()
#     f.setStyleSheet(
#         f"QFrame{{background:{C_CARD};border:1px solid {C_BORDER};border-radius:10px;}}")
#     return f

# def divider():
#     d = QFrame(); d.setFrameShape(QFrame.HLine)
#     d.setStyleSheet(f"background:{C_BORDER};border:none;max-height:1px;")
#     return d


# # ══════════════════════════════════════════════════════════════════════════════
# class MainWindow(QMainWindow):
#     def __init__(self):
#         super().__init__()
#         self.setWindowTitle("Robot Sensor Challenge")
#         self.setMinimumSize(1100, 740)
#         self.setStyleSheet(f"QMainWindow{{background:{C_BG};}}")

#         self.sim          = SimRobot()
#         self.real_robot   = None
#         self.hw_connected = False

#         # ScalarPositionFusion used in both sim and live modes
#         if _FUSION_AVAILABLE:
#             self.fusion     = ScalarPositionFusion(arena_length_cm=ARENA_CM)
#             self.sim_fusion = ScalarPositionFusion(arena_length_cm=ARENA_CM)
#         else:
#             self.fusion     = None
#             self.sim_fusion = None
#         self.sim_sensors = SimSensors()

#         self.running      = False
#         self.keys_held    = set()
#         self.user_pred    = 0.0
#         self.speed_cm     = 1.2
#         self._noise_buf   = collections.deque(maxlen=12)  # last N raw readings
#         self._session_log = []  # list of dicts per attempt

#         # Pre-create widgets referenced by _sensor_changed before full UI build
#         self.pred_unit_lbl = lbl("", 12, C_MUTED)
#         self.unit_lbl      = lbl("", 11, C_MUTED)
#         self.hint_lbl      = lbl("", 9, C_YELLOW)
#         self.hint_lbl.setWordWrap(True)
#         self.formula_lbl   = lbl("", 10, C_ACCENT)

#         self._build_ui()

#         self.drive_timer = QTimer(); self.drive_timer.setInterval(100)
#         self.drive_timer.timeout.connect(self._drive_tick)

#         self.poll_timer = QTimer(); self.poll_timer.setInterval(200)
#         self.poll_timer.timeout.connect(self._poll)
#         self.poll_timer.start()

#     # ══════════════════════════════════════════════════════════════════════════
#     def _build_ui(self):
#         root = QWidget(); root.setStyleSheet(f"background:{C_BG};")
#         self.setCentralWidget(root)
#         ml = QVBoxLayout(root); ml.setContentsMargins(0, 0, 0, 0); ml.setSpacing(0)
#         ml.addWidget(self._build_header())

#         body = QWidget(); body.setStyleSheet(f"background:{C_BG};")
#         bl = QHBoxLayout(body); bl.setContentsMargins(22, 16, 22, 16); bl.setSpacing(18)
#         bl.addWidget(self._build_left(),  stretch=5)
#         bl.addWidget(self._build_right(), stretch=5)
#         ml.addWidget(body)

#     # ── Header ─────────────────────────────────────────────────────────────────
#     def _build_header(self):
#         h = QWidget(); h.setFixedHeight(68)
#         h.setStyleSheet(f"background:{C_SURFACE};border-bottom:2px solid {C_ACCENT};")
#         hl = QHBoxLayout(h); hl.setContentsMargins(26, 0, 26, 0); hl.setSpacing(12)

#         title = QLabel("ROBOT SENSOR CHALLENGE")
#         title.setStyleSheet(
#             f"color:{C_ACCENT};font-family:Courier New;font-size:20px;"
#             f"font-weight:bold;background:transparent;")
#         sub = QLabel("Use one sensor at a time to move the robot exactly 25 cm")
#         sub.setStyleSheet(
#             f"color:{C_MUTED};font-family:{FONT};font-size:9px;background:transparent;")
#         tcol = QVBoxLayout(); tcol.setSpacing(2)
#         tcol.addWidget(title); tcol.addWidget(sub)

#         self.conn_badge = QLabel("  SIM MODE  ")
#         self.conn_badge.setStyleSheet(
#             f"background:{C_YELLOW};color:#000;font-family:{FONT};"
#             f"font-size:9px;font-weight:bold;border-radius:4px;padding:3px 8px;")

#         self.conn_btn = btn("⚡  Connect to COM7", C_SURFACE, C_ACCENT, 9)
#         self.conn_btn.setStyleSheet(
#             self.conn_btn.styleSheet() +
#             f"QPushButton{{border:1px solid {C_ACCENT};}}")
#         self.conn_btn.clicked.connect(self._toggle_connection)

#         hl.addLayout(tcol)
#         hl.addStretch()
#         hl.addWidget(self.conn_btn)
#         hl.addWidget(self.conn_badge)
#         return h

#     # ── Left column ────────────────────────────────────────────────────────────
#     def _build_left(self):
#         w = QWidget(); w.setStyleSheet("background:transparent;")
#         l = QVBoxLayout(w); l.setContentsMargins(0, 0, 0, 0); l.setSpacing(12)
#         l.addWidget(self._build_sensor_card())
#         l.addWidget(self._build_prediction_card())
#         l.addWidget(self._build_controls_card())
#         l.addWidget(self._build_result_card())
#         l.addStretch()
#         return w

#     def _build_sensor_card(self):
#         f = card()
#         l = QVBoxLayout(f); l.setContentsMargins(16, 12, 16, 12); l.setSpacing(7)
#         l.addWidget(lbl("① SELECT SENSOR", 8, C_MUTED))
#         l.addWidget(divider())

#         self.combo = QComboBox()
#         self.combo.addItems(list(SENSORS.keys()))
#         self.combo.setStyleSheet(f"""
#             QComboBox{{background:{C_BG};color:{C_TEXT};border:1px solid {C_BORDER};
#                        border-radius:6px;padding:7px 12px;font-family:{FONT};font-size:11px;}}
#             QComboBox::drop-down{{border:none;width:22px;}}
#             QComboBox QAbstractItemView{{background:{C_CARD};color:{C_TEXT};
#                 selection-background-color:{C_ACCENT};selection-color:#000;
#                 border:1px solid {C_BORDER};font-family:{FONT};}}
#         """)
#         self.combo.currentTextChanged.connect(self._sensor_changed)
#         l.addWidget(self.combo)

#         # Formula row
#         formula_row = QHBoxLayout()
#         formula_row.addWidget(lbl("FORMULA:", 8, C_MUTED))
#         formula_row.addWidget(self.formula_lbl)
#         formula_row.addStretch()
#         l.addLayout(formula_row)

#         # Live reading row
#         read_row = QHBoxLayout()
#         read_row.addWidget(lbl("RAW READING:", 9, C_MUTED))
#         self.reading_lbl = lbl("—", 20, C_ACCENT, bold=True)
#         read_row.addWidget(self.reading_lbl)
#         read_row.addWidget(self.unit_lbl)
#         read_row.addStretch()
#         l.addLayout(read_row)

#         # Noise bar
#         self.noise_bar = NoiseBar("Sensor noise:")
#         l.addWidget(self.noise_bar)

#         l.addWidget(self.hint_lbl)
#         self._sensor_changed(self.combo.currentText())
#         return f

#     def _build_prediction_card(self):
#         f = card()
#         l = QVBoxLayout(f); l.setContentsMargins(16, 12, 16, 12); l.setSpacing(7)
#         l.addWidget(lbl("② YOUR PREDICTION", 8, C_MUTED))
#         l.addWidget(divider())
#         l.addWidget(lbl("Enter the raw sensor reading you expect when the robot has moved 25 cm:", 10))

#         row = QHBoxLayout()
#         self.pred_entry = QLineEdit()
#         self.pred_entry.setPlaceholderText("e.g.  65.0")
#         self.pred_entry.setStyleSheet(f"""
#             QLineEdit{{background:{C_BG};color:{C_ACCENT};border:1px solid {C_ACCENT};
#                        border-radius:6px;padding:7px 14px;font-family:{FONT};
#                        font-size:17px;font-weight:bold;}}
#         """)
#         row.addWidget(self.pred_entry, stretch=3)
#         row.addWidget(self.pred_unit_lbl)
#         row.addStretch()
#         l.addLayout(row)
#         return f

#     def _build_controls_card(self):
#         f = card()
#         l = QVBoxLayout(f); l.setContentsMargins(16, 12, 16, 12); l.setSpacing(8)
#         l.addWidget(lbl("③ CHALLENGE CONTROLS", 8, C_MUTED))
#         l.addWidget(divider())

#         row = QHBoxLayout()
#         self.start_btn = btn("▶  START",     C_ACCENT,  "#000")
#         self.stop_btn  = btn("■  STOP",      C_ACCENT2, "#fff")
#         self.retry_btn = btn("↺  TRY AGAIN", C_BORDER,  C_TEXT)
#         self.stop_btn.setEnabled(False)
#         self.retry_btn.setEnabled(False)

#         self.start_btn.clicked.connect(self._start)
#         self.stop_btn.clicked.connect(self._stop)
#         self.retry_btn.clicked.connect(self._retry)

#         row.addWidget(self.start_btn)
#         row.addWidget(self.stop_btn)
#         row.addWidget(self.retry_btn)
#         l.addLayout(row)

#         self.status_lbl = lbl("● WAITING FOR START", 10, C_MUTED, bold=True)
#         l.addWidget(self.status_lbl)
#         return f

#     def _build_result_card(self):
#         self.res_card = card(); self.res_card.hide()
#         l = QVBoxLayout(self.res_card); l.setContentsMargins(18, 12, 18, 12)
#         self.res_lbl = QLabel("")
#         self.res_lbl.setAlignment(Qt.AlignCenter)
#         self.res_lbl.setWordWrap(True)
#         self.res_lbl.setStyleSheet(
#             f"color:{C_TEXT};font-family:{FONT};font-size:13px;"
#             f"font-weight:bold;background:transparent;")
#         l.addWidget(self.res_lbl)
#         return self.res_card

#     # ── Right column ───────────────────────────────────────────────────────────
#     def _build_right(self):
#         w = QWidget(); w.setStyleSheet("background:transparent;")
#         l = QVBoxLayout(w); l.setContentsMargins(0, 0, 0, 0); l.setSpacing(12)

#         # ── TOP PANEL: Robot top-view ─────────────────────────────────────
#         tv_card = card()
#         tvl = QVBoxLayout(tv_card); tvl.setContentsMargins(10, 8, 10, 8); tvl.setSpacing(4)
#         tvl.addWidget(lbl("ARENA  —  TOP VIEW", 8, C_MUTED))
#         tvl.addWidget(divider())

#         self.top_view = RobotTopView()
#         self.top_view.setMinimumHeight(160)
#         tvl.addWidget(self.top_view, stretch=1)

#         # Legend row for top view
#         tv_leg = QHBoxLayout()
#         for col, txt in [(C_ACCENT,  "Robot (sim)"),
#                          (C_ACTUAL,  "Fusion estimate"),
#                          (C_YELLOW,  "25 cm target")]:
#             dot = QLabel("●")
#             dot.setStyleSheet(
#                 f"color:{col};background:transparent;border:none;font-size:11px;")
#             tv_leg.addWidget(dot)
#             tv_leg.addWidget(lbl(txt, 8, C_MUTED))
#             tv_leg.addSpacing(6)
#         tv_leg.addStretch()
#         tvl.addLayout(tv_leg)
#         l.addWidget(tv_card, stretch=3)

#         # Chart card — tall
#         cc = card()
#         cl = QVBoxLayout(cc); cl.setContentsMargins(12, 10, 12, 10); cl.setSpacing(6)
#         cl.addWidget(lbl("DISTANCE COMPARISON", 8, C_MUTED))
#         cl.addWidget(divider())

#         self.chart = DistanceBarChart()
#         cl.addWidget(self.chart, stretch=1)

#         # Legend
#         leg = QHBoxLayout()
#         for col, txt in [(C_CURRENT, "Current (from sensor)"),
#                          (C_ACTUAL,  "Actual (fusion) — revealed on STOP"),
#                          (C_YELLOW,  "25 cm target")]:
#             dot = QLabel("●")
#             dot.setStyleSheet(
#                 f"color:{col};background:transparent;border:none;font-size:12px;")
#             leg.addWidget(dot)
#             leg.addWidget(lbl(txt, 8, C_MUTED))
#             leg.addSpacing(6)
#         leg.addStretch()
#         cl.addLayout(leg)
#         l.addWidget(cc, stretch=3)

#         # Sensor report card — hidden until STOP
#         l.addWidget(self._build_report_card(), stretch=0)

#         # Teleop card
#         l.addWidget(self._build_teleop_card(), stretch=0)

#         # Session log
#         l.addWidget(self._build_log_card(), stretch=2)

#         return w

#     def _build_report_card(self):
#         self.report_card = card()
#         self.report_card.hide()
#         l = QVBoxLayout(self.report_card); l.setContentsMargins(16, 12, 16, 12); l.setSpacing(6)

#         hrow = QHBoxLayout()
#         hrow.addWidget(lbl("SENSOR REPORT", 8, C_MUTED))
#         hrow.addStretch()
#         self.report_sensor_lbl = lbl("", 9, C_ACCENT, bold=True)
#         hrow.addWidget(self.report_sensor_lbl)
#         l.addLayout(hrow)
#         l.addWidget(divider())

#         # Stats row
#         stats = QHBoxLayout(); stats.setSpacing(20)

#         acc_col = QVBoxLayout()
#         acc_col.addWidget(lbl("Prediction accuracy", 8, C_MUTED))
#         self.report_acc_lbl = lbl("—", 16, C_TEXT, bold=True)
#         acc_col.addWidget(self.report_acc_lbl)
#         stats.addLayout(acc_col)

#         err_col = QVBoxLayout()
#         err_col.addWidget(lbl("Distance error", 8, C_MUTED))
#         self.report_err_lbl = lbl("—", 16, C_TEXT, bold=True)
#         err_col.addWidget(self.report_err_lbl)
#         stats.addLayout(err_col)

#         noise_col = QVBoxLayout()
#         noise_col.addWidget(lbl("Signal noise", 8, C_MUTED))
#         self.report_noise_lbl = lbl("—", 12, C_TEXT, bold=True)
#         noise_col.addWidget(self.report_noise_lbl)
#         stats.addLayout(noise_col)

#         stats.addStretch()
#         l.addLayout(stats)

#         # Limitation hint
#         self.report_limit_lbl = lbl("", 9, C_YELLOW)
#         self.report_limit_lbl.setWordWrap(True)
#         l.addWidget(self.report_limit_lbl)

#         return self.report_card

#     def _build_teleop_card(self):
#         f = card()
#         l = QVBoxLayout(f); l.setContentsMargins(16, 10, 16, 10); l.setSpacing(6)
#         l.addWidget(lbl("④ KEYBOARD TELEOP", 8, C_MUTED))
#         l.addWidget(divider())

#         self.kw = KeyWidget("W"); self.ka = KeyWidget("A")
#         self.ks = KeyWidget("S"); self.kd = KeyWidget("D")

#         kg = QGridLayout(); kg.setSpacing(4)
#         kg.addWidget(self.kw, 0, 1)
#         kg.addWidget(self.ka, 1, 0)
#         kg.addWidget(self.ks, 1, 1)
#         kg.addWidget(self.kd, 1, 2)

#         row = QHBoxLayout(); row.addStretch()
#         row.addLayout(kg); row.addSpacing(14)

#         hints = QVBoxLayout()
#         for t in ["W / ↑  Forward", "S / ↓  Backward",
#                   "A / ←  Left", "D / →  Right"]:
#             hints.addWidget(lbl(t, 9, C_MUTED))
#         row.addLayout(hints); row.addStretch()
#         l.addLayout(row)
#         return f

#     def _build_log_card(self):
#         f = card()
#         l = QVBoxLayout(f); l.setContentsMargins(16, 10, 16, 10); l.setSpacing(6)

#         hrow = QHBoxLayout()
#         hrow.addWidget(lbl("SESSION LOG", 8, C_MUTED))
#         hrow.addStretch()
#         clr = btn("clear", C_BORDER, C_MUTED, 8)
#         clr.setFixedHeight(22)
#         clr.clicked.connect(self._clear_log)
#         hrow.addWidget(clr)
#         l.addLayout(hrow)
#         l.addWidget(divider())

#         self.log_container = QWidget()
#         self.log_container.setStyleSheet("background:transparent;")
#         self.log_layout = QVBoxLayout(self.log_container)
#         self.log_layout.setContentsMargins(0, 0, 0, 0)
#         self.log_layout.setSpacing(3)
#         self.log_layout.addStretch()

#         scroll = QScrollArea()
#         scroll.setWidget(self.log_container)
#         scroll.setWidgetResizable(True)
#         scroll.setStyleSheet(
#             f"QScrollArea{{background:transparent;border:none;}}"
#             f"QScrollBar:vertical{{background:{C_BORDER};width:6px;border-radius:3px;}}"
#             f"QScrollBar::handle:vertical{{background:{C_MUTED};border-radius:3px;}}")
#         scroll.setFixedHeight(100)
#         l.addWidget(scroll)

#         self.log_empty_lbl = lbl("No attempts yet — complete a challenge to see results here.",
#                                  9, C_MUTED)
#         self.log_empty_lbl.setAlignment(Qt.AlignCenter)
#         l.addWidget(self.log_empty_lbl)

#         return f

#     # ── Hardware connection ────────────────────────────────────────────────────
#     def _toggle_connection(self):
#         self._disconnect() if self.hw_connected else self._connect()

#     def _connect(self):
#         try:
#             from snp_model import ScalarPositionFusion, RobotModel
#             self.real_robot   = RobotModel("COM7")
#             if self.fusion is None and _FUSION_AVAILABLE:
#                 self.fusion   = ScalarPositionFusion(arena_length_cm=ARENA_CM)
#             elif self.fusion is None:
#                 from snp_model import ScalarPositionFusion as SPF
#                 self.fusion   = SPF(arena_length_cm=ARENA_CM)
#             self.hw_connected = True
#             self.conn_badge.setText("  LIVE — COM7  ")
#             self.conn_badge.setStyleSheet(
#                 f"background:{C_SUCCESS};color:#000;font-family:{FONT};"
#                 f"font-size:9px;font-weight:bold;border-radius:4px;padding:3px 8px;")
#             self.conn_btn.setText("✕  Disconnect")
#         except ImportError:
#             QMessageBox.critical(self, "Import Error",
#                 "snp_model.py not found.\n"
#                 "Ensure snp_model.py is in the same directory.")
#         except Exception as e:
#             QMessageBox.critical(self, "Connection Failed",
#                 f"Could not connect to COM7:\n{e}")

#     def _disconnect(self):
#         if self.real_robot:
#             try: self.real_robot.close()
#             except: pass
#         self.real_robot = None; self.fusion = None; self.hw_connected = False
#         self.conn_badge.setText("  SIM MODE  ")
#         self.conn_badge.setStyleSheet(
#             f"background:{C_YELLOW};color:#000;font-family:{FONT};"
#             f"font-size:9px;font-weight:bold;border-radius:4px;padding:3px 8px;")
#         self.conn_btn.setText("⚡  Connect to COM7")

#     # ── Sensor helpers ─────────────────────────────────────────────────────────
#     def _sensor_changed(self, name):
#         meta = SENSORS[name]
#         self.hint_lbl.setText(f"💡  {meta['hint']}")
#         self.formula_lbl.setText(meta["formula"])
#         self.pred_unit_lbl.setText(meta["unit"])
#         self.unit_lbl.setText(meta["unit"])
#         self._noise_buf.clear()
#         if hasattr(self, "top_view"):
#             self.top_view.set_sensor(name)

#     def _get_reading(self):
#         name = self.combo.currentText()
#         if self.hw_connected and self.real_robot:
#             try:
#                 s = self.real_robot.get_sensors()
#                 return round(float(getattr(s, SENSORS[name]["key"], 0.0)), 2)
#             except: return 0.0
#         return self.sim.get_reading(name)

#     def _reading_to_cm(self, reading):
#         return SENSORS[self.combo.currentText()]["to_cm"](reading)

#     def _get_position(self):
#         if self.hw_connected and self.real_robot and self.fusion:
#             try:
#                 s = self.real_robot.get_sensors()
#                 self.fusion.step(s, s.encoder_left, s.encoder_right)
#                 return self.fusion.position
#             except: return 0.0
#         # Sim mode: update sim_sensors then step ScalarPositionFusion
#         self.sim_sensors.update(self.sim, self.combo.currentText())
#         if self.sim_fusion is not None:
#             self.sim_fusion.step(
#                 self.sim_sensors,
#                 self.sim_sensors.encoder_left,
#                 self.sim_sensors.encoder_right,
#             )
#             return self.sim_fusion.position
#         return self.sim.get_position()

#     # ── Poll ───────────────────────────────────────────────────────────────────
#     def _poll(self):
#         reading = self._get_reading()
#         name    = self.combo.currentText()
#         self.reading_lbl.setText(str(reading))
#         self.unit_lbl.setText(SENSORS[name]["unit"])

#         # Noise buffer
#         self._noise_buf.append(reading)
#         if len(self._noise_buf) >= 3:
#             vals   = list(self._noise_buf)
#             mean   = sum(vals) / len(vals)
#             var    = sum((v - mean) ** 2 for v in vals) / len(vals)
#             # Normalise roughly: encoder variance up to ~50, IR up to ~2, ultrasonic up to ~1
#             maxvar = {"Encoder — Left Wheel": 50.0,
#                       "Ultrasonic — Front":    1.0,
#                       "Sharp IR — Back":       2.0}.get(name, 5.0)
#             level = min(var / maxvar, 1.0)
#             self.noise_bar.set_level(level)

#         # Update top view regardless of running state
#         sim_pos    = self.sim.get_position()
#         fusion_pos = self._get_position()
#         self.top_view.set_position(sim_pos, fusion_pos)

#         if self.running:
#             current_cm = max(self._reading_to_cm(reading), 0.0)
#             actual_cm  = fusion_pos
#             self.chart.set_values(current_cm, actual_cm)

#     # ── Drive loop ─────────────────────────────────────────────────────────────
#     def _drive_tick(self):
#         fwd  = Qt.Key_W in self.keys_held or Qt.Key_Up    in self.keys_held
#         back = Qt.Key_S in self.keys_held or Qt.Key_Down  in self.keys_held
#         left = Qt.Key_A in self.keys_held or Qt.Key_Left  in self.keys_held
#         rght = Qt.Key_D in self.keys_held or Qt.Key_Right in self.keys_held

#         if fwd:    self._move( self.speed_cm)
#         elif back: self._move(-self.speed_cm)
#         else:
#             # send zero velocity so robot actually stops
#             if self.hw_connected and self.real_robot:
#                 try: self.real_robot.send_velocity(0.0, 0.0, 0.0)
#                 except: pass

#         # turning
#         if self.hw_connected and self.real_robot and (left or rght):
#             try: self.real_robot.send_velocity(0.0, 0.0, 0.3 if left else -0.3)
#             except: pass

#         self.kw.set_active(fwd);  self.ks.set_active(back)
#         self.ka.set_active(left); self.kd.set_active(rght)
#         self._set_status("● DRIVING" if (fwd or back) else "● HOLDING",
#                         C_SUCCESS   if (fwd or back) else C_YELLOW)

#     def _move(self, d):
#         if self.hw_connected and self.real_robot:
#             try:
#                 vx = 0.15 if d > 0 else -0.15
#                 self.real_robot.send_velocity(vx, 0.0, 0.0)
#             except: pass
#         else:
#             self.sim.move(d)

#     # ── Challenge flow ─────────────────────────────────────────────────────────
#     def _start(self):
#         try:
#             self.user_pred = float(self.pred_entry.text().strip())
#         except ValueError:
#             self._show_result("⚠  Please enter a valid number first.", C_YELLOW)
#             return

#         self.sim.reset()
#         if self.fusion:
#             try: self.fusion.reset()
#             except: pass
#         if self.sim_fusion:
#             try: self.sim_fusion.reset()
#             except: pass

#         self.running = True
#         self.chart.reveal(False)
#         self.chart.set_values(0, 0)
#         self.res_card.hide()
#         self.report_card.hide()
#         self._noise_buf.clear()

#         self.start_btn.setEnabled(False)
#         self.stop_btn.setEnabled(True)
#         self.retry_btn.setEnabled(False)
#         self.combo.setEnabled(False)
#         self.pred_entry.setEnabled(False)
#         self.conn_btn.setEnabled(False)

#         self._set_status("● DRIVING", C_SUCCESS)
#         self.drive_timer.start()
#         self.setFocus()

#     def _stop(self):
#         if not self.running: return
#         self.running = False
#         self.drive_timer.stop()
#         self.keys_held.clear()
#         for k in (self.kw, self.ka, self.ks, self.kd): k.set_active(False)

#         self.stop_btn.setEnabled(False)
#         self.retry_btn.setEnabled(True)
#         self.conn_btn.setEnabled(True)
#         self._set_status("● STOPPED", C_MUTED)

#         name      = self.combo.currentText()
#         actual    = self._get_position()
#         reading   = self._get_reading()
#         current_cm = max(self._reading_to_cm(reading), 0.0)
#         err_cm    = abs(actual - TARGET_CM)
#         success   = err_cm <= TOLERANCE

#         # Final chart
#         self.chart.reveal(True)
#         self.chart.set_values(current_cm, actual)

#         # Prediction accuracy (how close was their predicted reading vs ideal)
#         meta      = SENSORS[name]
#         # Ideal reading at exactly 25 cm
#         ideal_rdg = meta["sim_start"] + meta["sim_scale"] * TARGET_CM
#         pred_err  = abs(self.user_pred - ideal_rdg)
#         pred_pct  = max(0, 100 - (pred_err / max(abs(ideal_rdg), 1)) * 100)

#         # Noise level from buffer
#         noise_char = meta["noise_char"]
#         noise_col  = NOISE_COLORS[noise_char]

#         # Populate report card
#         self.report_sensor_lbl.setText(name)
#         self.report_acc_lbl.setText(f"{pred_pct:.0f}%")
#         self.report_acc_lbl.setStyleSheet(
#             f"color:{C_SUCCESS if pred_pct >= 80 else C_YELLOW if pred_pct >= 50 else C_FAIL};"
#             f"font-family:{FONT};font-size:16px;font-weight:bold;background:transparent;")
#         self.report_err_lbl.setText(f"{err_cm*10:.1f} mm")
#         self.report_err_lbl.setStyleSheet(
#             f"color:{C_SUCCESS if success else C_FAIL};"
#             f"font-family:{FONT};font-size:16px;font-weight:bold;background:transparent;")
#         self.report_noise_lbl.setText(noise_char.upper())
#         self.report_noise_lbl.setStyleSheet(
#             f"color:{noise_col};font-family:{FONT};"
#             f"font-size:12px;font-weight:bold;background:transparent;")
#         self.report_limit_lbl.setText(f"⚠  {meta['limitation']}")
#         self.report_card.show()

#         # Result banner
#         if success:
#             self._show_result(
#                 f"✓  SUCCESS!   Stopped at {actual:.2f} cm  ·  error = {err_cm*10:.1f} mm",
#                 C_SUCCESS)
#         else:
#             self._show_result(
#                 f"✗  NOT QUITE   Stopped at {actual:.2f} cm  ·  error = {err_cm*10:.1f} mm",
#                 C_FAIL)

#         # Session log entry
#         self._add_log_entry(name, actual, err_cm, success, pred_pct)

#     def _retry(self):
#         self.running = False
#         self.drive_timer.stop()
#         self.keys_held.clear()
#         for k in (self.kw, self.ka, self.ks, self.kd): k.set_active(False)

#         self.sim.reset()
#         if self.fusion:
#             try: self.fusion.reset()
#             except: pass
#         if self.sim_fusion:
#             try: self.sim_fusion.reset()
#             except: pass

#         self.start_btn.setEnabled(True)
#         self.stop_btn.setEnabled(False)
#         self.retry_btn.setEnabled(False)
#         self.combo.setEnabled(True)
#         self.pred_entry.setEnabled(True)
#         self.pred_entry.clear()
#         self.conn_btn.setEnabled(True)

#         self.chart.reveal(False)
#         self.chart.set_values(0, 0)
#         self.res_card.hide()
#         self.report_card.hide()
#         self._set_status("● WAITING FOR START", C_MUTED)

#     # ── Session log ────────────────────────────────────────────────────────────
#     def _add_log_entry(self, sensor_name, actual_cm, err_cm, success, pred_pct):
#         attempt = len(self._session_log) + 1
#         self._session_log.append({
#             "sensor": sensor_name, "actual": actual_cm,
#             "err": err_cm, "success": success, "pred_pct": pred_pct
#         })

#         short = sensor_name.split("—")[0].strip()
#         col   = C_SUCCESS if success else C_FAIL
#         mark  = "✓" if success else "✗"

#         row_w = QWidget(); row_w.setStyleSheet("background:transparent;")
#         row   = QHBoxLayout(row_w); row.setContentsMargins(0, 0, 0, 0); row.setSpacing(8)

#         row.addWidget(lbl(f"#{attempt}", 8, C_MUTED))
#         row.addWidget(lbl(short, 8, C_TEXT))
#         row.addStretch()
#         row.addWidget(lbl(f"{actual_cm:.1f} cm", 8, C_TEXT))
#         row.addWidget(lbl(f"Δ {err_cm*10:.1f} mm", 8, C_MUTED))
#         row.addWidget(lbl(f"{mark}", 11, col, bold=True))

#         # Insert before the stretch at end
#         self.log_layout.insertWidget(self.log_layout.count() - 1, row_w)
#         self.log_empty_lbl.hide()

#     def _clear_log(self):
#         self._session_log.clear()
#         while self.log_layout.count() > 1:
#             item = self.log_layout.takeAt(0)
#             if item.widget(): item.widget().deleteLater()
#         self.log_empty_lbl.show()

#     # ── Utilities ──────────────────────────────────────────────────────────────
#     def _set_status(self, txt, color):
#         self.status_lbl.setText(txt)
#         self.status_lbl.setStyleSheet(
#             f"color:{color};font-family:{FONT};font-size:10px;"
#             f"font-weight:bold;background:transparent;border:none;")

#     def _show_result(self, msg, color):
#         self.res_lbl.setText(msg)
#         self.res_lbl.setStyleSheet(
#             f"color:{color};font-family:{FONT};font-size:13px;"
#             f"font-weight:bold;background:transparent;")
#         border = C_SUCCESS if color == C_SUCCESS else \
#                  C_FAIL    if color == C_FAIL    else C_YELLOW
#         self.res_card.setStyleSheet(
#             f"QFrame{{background:{C_CARD};border:2px solid {border};border-radius:10px;}}")
#         self.res_card.show()

#     # ── Keyboard ───────────────────────────────────────────────────────────────
#     def keyPressEvent(self, e):
#         if self.running: self.keys_held.add(e.key())

#     def keyReleaseEvent(self, e):
#         self.keys_held.discard(e.key())

#     def closeEvent(self, e):
#         self.drive_timer.stop(); self.poll_timer.stop()
#         self._disconnect(); e.accept()


# # ══════════════════════════════════════════════════════════════════════════════
# if __name__ == "__main__":
#     app = QApplication(sys.argv)
#     app.setStyle("Fusion")
#     win = MainWindow()
#     win.show()
#     sys.exit(app.exec_())







"""
Sensor Fusion Explorer — PyQt5 Educational GUI
================================================
3-page interactive lesson on multi-sensor fusion and Kalman filtering.

Page 1 : Robot World View  — see sensors disagree, fusion chooses best
Page 2 : Weighted Average  — sliders show how weights change the estimate
Page 3 : Kalman Filter     — step-through with maths, predict/update cycle

Run  :  python sensor_fusion_explainer.py
Needs:  pip install PyQt5
"""

import sys, math, random, os
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QLabel, QPushButton,
    QVBoxLayout, QHBoxLayout, QGridLayout, QFrame, QSizePolicy,
    QSlider, QStackedWidget, QScrollArea
)
from PyQt5.QtCore  import Qt, QTimer, QPointF, QRectF
from PyQt5.QtGui   import (
    QPainter, QColor, QPen, QBrush, QFont, QLinearGradient,
    QRadialGradient, QPainterPath, QPolygonF, QFontMetrics
)

# ── Import fusion classes from snp_model.py (same directory) ──────────────────
_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)
try:
    from snp_model import (
        RobotModel,
        WeightedPositionFusion,
        ScalarPositionFusion
    )
    HW_AVAILABLE = True
except Exception:
    HW_AVAILABLE = False
    # Stub classes so Page1/Page2/Page3 instantiate without crashing
    class _FusionStub:
        position = 0.0; uncertainty = 1.0; weights = {}; contributions = {}
        def step(self, *a, **kw): return 0.0
        def _update(self, z, R): pass
        def reset(self): pass
        def set_weight(self, *a): pass
        def restore(self): pass
        def degrade(self, *a): pass
        # Scalar-specific attributes
        Xk = 0.0; Pk = 1.0
    class WeightedPositionFusion(_FusionStub):
        _enc_pos = 0.0; _prev_enc_left = None; _prev_enc_right = None
    class ScalarPositionFusion(_FusionStub): pass
    class RobotModel: pass

# ── Palette ────────────────────────────────────────────────────────────────────
C_BG       = "#080c14"
C_SURFACE  = "#0e1422"
C_CARD     = "#121826"
C_BORDER   = "#1a2236"
C_TEXT     = "#d8e4f0"
C_MUTED    = "#3d5068"
C_ACCENT   = "#00c8ff"

C_ENCODER  = "#4488ff"   # blue
C_ULTRA    = "#00e676"   # green
C_IR       = "#ffd740"   # yellow
C_FUSION   = "#ff4444"   # red
C_KALMAN   = "#cc44ff"   # purple

C_WALL     = "#2a3a50"
C_GRID     = "#0f1825"
C_ROBOT    = "#00c8ff"

FONT_MONO  = "Consolas"
FONT_TITLE = "Courier New"

ARENA_CM   = 90.0   # total corridor length
TARGET_CM  = 25.0


# ══════════════════════════════════════════════════════════════════════════════
# Shared helpers
# ══════════════════════════════════════════════════════════════════════════════
def lbl(text, size=13, color=C_TEXT, bold=False, wrap=False):
    l = QLabel(text)
    l.setStyleSheet(
        f"color:{color};font-family:{FONT_MONO};font-size:{size}px;"
        f"font-weight:{'bold' if bold else 'normal'};"
        f"background:transparent;border:none;")
    if wrap:
        l.setWordWrap(True)
    return l


def title_lbl(text, size=16):
    l = QLabel(text)
    l.setStyleSheet(
        f"color:{C_ACCENT};font-family:{FONT_TITLE};font-size:{size}px;"
        f"font-weight:bold;background:transparent;border:none;letter-spacing:2px;")
    return l


def card(pad=14):
    f = QFrame()
    f.setStyleSheet(
        f"QFrame{{background:{C_CARD};border:1px solid {C_BORDER};border-radius:10px;}}")
    return f


def divider():
    d = QFrame(); d.setFrameShape(QFrame.HLine)
    d.setStyleSheet(f"background:{C_BORDER};border:none;max-height:1px;")
    return d


def nav_btn(text, active=False):
    b = QPushButton(text)
    bg = C_ACCENT if active else C_CARD
    fg = "#000"   if active else C_MUTED
    b.setStyleSheet(f"""
        QPushButton{{background:{bg};color:{fg};border:1px solid {C_BORDER};
                     border-radius:6px;padding:7px 18px;
                     font-family:{FONT_MONO};font-size:13px;font-weight:bold;}}
        QPushButton:hover{{background:{C_ACCENT}44;color:{C_ACCENT};}}
    """)
    return b


def colored_dot(color, size=14):
    l = QLabel("●")
    l.setStyleSheet(
        f"color:{color};font-size:{size}px;background:transparent;border:none;")
    return l


# ══════════════════════════════════════════════════════════════════════════════
# PAGE 1 — World View Canvas
# ══════════════════════════════════════════════════════════════════════════════
class WorldCanvas(QWidget):
    """
    Top-down corridor — LIGHT MODE.
    LEFT wall  = back wall  (IR measures from back wall to robot)
    RIGHT wall = front wall (Ultrasonic measures from robot to front wall)
    Robot triangle faces RIGHT.
    Sensor lines are HORIZONTAL along the corridor axis.
    """
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumHeight(300)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)

        self.robot_cm    = 30.0
        self.encoder_cm  = 30.0
        self.ultra_cm    = 30.0
        self.ir_cm       = 30.0
        self.fusion_cm   = 30.0
        self.enc_sigma   = 3.0
        self.ultra_sigma = 5.0
        self.fuse_sigma  = 1.5

    def _px(self, cm):
        PAD = 80
        return PAD + (cm / ARENA_CM) * (self.width() - 2 * PAD)

    def paintEvent(self, _):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)

        W, H    = self.width(), self.height()
        PAD     = 80
        WALL_W  = 32
        cy      = H // 2
        CORR_T  = 28
        CORR_B  = H - 28

        # ── Light background ────────────────────────────────────────────
        p.fillRect(0, 0, W, H, QColor("#eef2f7"))

        # Subtle horizontal guide lines
        p.setPen(QPen(QColor("#d8e0ea"), 1))
        for y in range(CORR_T, CORR_B, 32):
            p.drawLine(PAD + WALL_W, y, W - PAD - WALL_W, y)

        # Corridor floor (white)
        p.fillRect(PAD + WALL_W, CORR_T,
                   W - 2*PAD - 2*WALL_W, CORR_B - CORR_T,
                   QColor("#ffffff"))
        p.setPen(QPen(QColor("#c8d4e0"), 1))
        p.drawRect(PAD + WALL_W, CORR_T,
                   W - 2*PAD - 2*WALL_W, CORR_B - CORR_T)

        # cm ruler ticks along bottom of corridor
        p.setPen(QColor("#90a0b0"))
        p.setFont(QFont(FONT_MONO, 13))
        for i in range(10):
            x   = int(self._px(i * ARENA_CM / 9))
            val = int(i * ARENA_CM / 9)
            p.drawLine(x, CORR_B, x, CORR_B + 6)
            p.drawText(x - 16, CORR_B + 6, 32, 18, Qt.AlignCenter, str(val))
        p.drawText(PAD + WALL_W, CORR_B + 20, 40, 16, Qt.AlignLeft, "cm")

        # ── LEFT WALL (back wall) ───────────────────────────────────────
        lw_grad = QLinearGradient(0, 0, PAD + WALL_W, 0)
        lw_grad.setColorAt(0.0, QColor("#5a6e84"))
        lw_grad.setColorAt(1.0, QColor("#8899aa"))
        p.fillRect(0, 0, PAD + WALL_W, H, QBrush(lw_grad))
        p.setPen(QPen(QColor("#445566"), 2))
        p.drawLine(PAD + WALL_W, 0, PAD + WALL_W, H)
        # Label
        p.setPen(QColor("#ffffff"))
        p.setFont(QFont(FONT_MONO, 14, QFont.Bold))
        p.save()
        p.translate(PAD // 2 + 4, H // 2)
        p.rotate(-90)
        p.drawText(-52, -8, 104, 22, Qt.AlignCenter, "BACK WALL")
        p.restore()

        # ── RIGHT WALL (front wall) ─────────────────────────────────────
        rw_grad = QLinearGradient(W - PAD - WALL_W, 0, W, 0)
        rw_grad.setColorAt(0.0, QColor("#8899aa"))
        rw_grad.setColorAt(1.0, QColor("#5a6e84"))
        p.fillRect(W - PAD - WALL_W, 0, PAD + WALL_W, H, QBrush(rw_grad))
        p.setPen(QPen(QColor("#445566"), 2))
        p.drawLine(W - PAD - WALL_W, 0, W - PAD - WALL_W, H)
        p.setPen(QColor("#ffffff"))
        p.setFont(QFont(FONT_MONO, 14, QFont.Bold))
        p.save()
        p.translate(W - PAD // 2 - 4, H // 2)
        p.rotate(90)
        p.drawText(-56, -8, 112, 22, Qt.AlignCenter, "FRONT WALL")
        p.restore()

        # ── Uncertainty zones ───────────────────────────────────────────
        def draw_zone(cm, sigma, col):
            cx     = self._px(cm)
            half_x = (sigma / ARENA_CM) * (W - 2*PAD - 2*WALL_W) * 2.4
            half_y = (CORR_B - CORR_T) * 0.40
            grad   = QRadialGradient(cx, cy, half_x)
            c  = QColor(col); c.setAlpha(50)
            c2 = QColor(col); c2.setAlpha(0)
            grad.setColorAt(0.0, c); grad.setColorAt(1.0, c2)
            p.setBrush(QBrush(grad)); p.setPen(Qt.NoPen)
            p.drawEllipse(QRectF(cx - half_x, cy - half_y, half_x*2, half_y*2))

        draw_zone(self.encoder_cm, self.enc_sigma,   C_ENCODER)
        draw_zone(self.ultra_cm,   self.ultra_sigma,  C_ULTRA)
        draw_zone(self.fusion_cm,  self.fuse_sigma,   C_FUSION)

        rx_robot = self._px(self.robot_cm)
        rx_front = W - PAD - WALL_W
        rx_back  = PAD + WALL_W

        # Robot body pixel dimensions (needed for sensor line anchors)
        R_W    = 36
        R_H    = 44
        r_left = int(rx_robot) - R_W // 2
        r_right= r_left + R_W
        r_top  = cy - R_H // 2

        # ── ULTRASONIC line: robot FRONT face → right (front) wall ─────
        us_y = cy - 36
        p.setPen(QPen(QColor(C_ULTRA), 2, Qt.DashLine))
        p.drawLine(r_right, us_y, rx_front, us_y)
        p.setPen(QPen(QColor(C_ULTRA), 2, Qt.SolidLine))
        p.drawLine(rx_front, us_y, rx_front - 12, us_y - 7)
        p.drawLine(rx_front, us_y, rx_front - 12, us_y + 7)
        mid_us = int((r_right + rx_front) / 2)
        p.setPen(QColor("#007a50"))
        p.setFont(QFont(FONT_MONO, 13, QFont.Bold))
        us_dist = ARENA_CM - self.robot_cm
        p.drawText(mid_us - 54, us_y - 22, 108, 18,
                   Qt.AlignCenter, f"US {us_dist:.1f} cm")

        # ── IR line: left (back) wall → robot BACK face ─────────────────
        ir_y = cy + 36
        p.setPen(QPen(QColor(C_IR), 2, Qt.DashLine))
        p.drawLine(rx_back, ir_y, r_left, ir_y)
        p.setPen(QPen(QColor(C_IR), 2, Qt.SolidLine))
        p.drawLine(rx_back, ir_y, rx_back + 12, ir_y - 7)
        p.drawLine(rx_back, ir_y, rx_back + 12, ir_y + 7)
        mid_ir = int((rx_back + r_left) / 2)
        p.setPen(QColor("#a07800"))
        p.setFont(QFont(FONT_MONO, 13, QFont.Bold))
        ir_dist = self.ir_cm   # actual IR reading from fusion
        p.drawText(mid_ir - 54, ir_y + 4, 108, 18,
                   Qt.AlignCenter, f"IR {ir_dist:.1f} cm")

        # ── Encoder dot (draw BEFORE robot body so body sits on top) ────
        ex = self._px(self.encoder_cm)
        p.setBrush(QColor(C_ENCODER))
        p.setPen(QPen(QColor("#ffffff"), 2))
        p.drawEllipse(QPointF(ex, cy), 11, 11)
        p.setPen(QColor(C_ENCODER))
        p.setFont(QFont(FONT_MONO, 13, QFont.Bold))
        p.drawText(int(ex) - 36, cy - 30, 72, 18,
                   Qt.AlignCenter, f"ENC {self.encoder_cm:.1f}")

        # ── Fusion dot (draw BEFORE robot body so body sits on top) ─────
        fx = self._px(self.fusion_cm)
        p.setBrush(QColor(C_FUSION))
        p.setPen(QPen(QColor("#ffffff"), 2))
        p.drawEllipse(QPointF(fx, cy), 14, 14)
        p.setPen(QColor(C_FUSION))
        p.setFont(QFont(FONT_MONO, 13, QFont.Bold))
        p.drawText(int(fx) - 42, cy + 22, 84, 18,
                   Qt.AlignCenter, f"FUSED {self.fusion_cm:.1f}")

        # ── Robot top-down body (rectangle + wheels, nose → RIGHT) ──────
        W_W = 8; W_H = 14

        # Wheels (4 corners)
        wheel_col = QColor("#3a4a60")
        p.setBrush(QBrush(wheel_col)); p.setPen(Qt.NoPen)
        for wx, wy in [
            (r_left - W_W,  r_top - W_H // 2),
            (r_left - W_W,  r_top + R_H - W_H // 2),
            (r_left + R_W,  r_top - W_H // 2),
            (r_left + R_W,  r_top + R_H - W_H // 2),
        ]:
            p.drawRoundedRect(wx, wy, W_W, W_H, 2, 2)

        # Body gradient (dark blue)
        body_grad = QLinearGradient(r_left, r_top, r_left, r_top + R_H)
        body_grad.setColorAt(0.0, QColor("#1a3a9c"))
        body_grad.setColorAt(1.0, QColor("#112266"))
        p.setBrush(QBrush(body_grad))
        p.setPen(QPen(QColor("#ffffff"), 2))
        p.drawRoundedRect(r_left, r_top, R_W, R_H, 5, 5)

        # Forward arrow on body
        ax = r_left + R_W // 2; ay = cy
        p.setPen(QPen(QColor("#ffffff"), 2))
        p.drawLine(ax - 6, ay, ax + 8, ay)
        p.drawLine(ax + 8, ay, ax + 4, ay - 4)
        p.drawLine(ax + 8, ay, ax + 4, ay + 4)

        # Sensor dots on robot body edges
        p.setBrush(QColor(C_ULTRA)); p.setPen(Qt.NoPen)
        p.drawEllipse(r_right - 4, cy - 4, 8, 8)   # front US dot
        p.setBrush(QColor(C_IR)); p.setPen(Qt.NoPen)
        p.drawEllipse(r_left - 4,  cy - 4, 8, 8)   # back IR dot

        # ── Corner hint ──────────────────────────────────────────────────
        p.setPen(QColor("#90a0b0"))
        p.setFont(QFont(FONT_MONO, 11))
        p.drawText(W - 260, CORR_T + 2, 250, 16,
                   Qt.AlignRight, "shaded region = uncertainty zone")

        p.end()


# ══════════════════════════════════════════════════════════════════════════════
# PAGE 1 — full widget
# ══════════════════════════════════════════════════════════════════════════════
class Page1(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setStyleSheet(f"background:{C_BG};")

        # Simulation state
        self._true_pos  = 30.0
        self._enc_drift = 0.0
        self._step      = 0
        self._animating = False
        self._robot     = None   # set via set_hardware()

        # Fusion objects used in sim mode (same classes as snp_model)
        self._sim_wfusion = WeightedPositionFusion()
        self._sim_kfusion = ScalarPositionFusion()

        self._build()

        self._timer = QTimer(); self._timer.setInterval(120)
        self._timer.timeout.connect(self._anim_tick)

    def _build(self):
        l = QVBoxLayout(self); l.setContentsMargins(20, 14, 20, 14); l.setSpacing(12)

        # Title row
        tr = QHBoxLayout()
        tr.addWidget(title_lbl("PAGE 1 — ROBOT WORLD VIEW", 13))
        tr.addStretch()
        self.anim_btn = QPushButton("▶  Animate sensors")
        self.anim_btn.setStyleSheet(f"""
            QPushButton{{background:{C_ACCENT};color:#000;border:none;border-radius:6px;
                         padding:7px 16px;font-family:{FONT_MONO};font-size:13px;font-weight:bold;}}
            QPushButton:hover{{background:{C_ACCENT}cc;}}
        """)
        self.anim_btn.clicked.connect(self._toggle_anim)
        tr.addWidget(self.anim_btn)
        l.addLayout(tr)

        l.addWidget(lbl(
            "A top-down view of the robot corridor.  Each sensor gives a different estimate of position — "
            "fusion combines them into one best guess.", 9, C_MUTED, wrap=True))

        # World canvas
        wc = card()
        wl = QVBoxLayout(wc); wl.setContentsMargins(0, 0, 0, 0)
        self.canvas = WorldCanvas()
        wl.addWidget(self.canvas)
        l.addWidget(wc)

        # Legend row
        leg = QHBoxLayout(); leg.setSpacing(20)
        items = [
            (C_ENCODER, "Encoder estimate (blue dot)"),
            (C_ULTRA,   "Ultrasonic measurement (green line)"),
            (C_IR,      "IR measurement (yellow line)"),
            (C_FUSION,  "Fused estimate (red dot)"),
            (C_ROBOT,   "True robot position (triangle)"),
        ]
        for col, txt in items:
            leg.addWidget(colored_dot(col, 10))
            leg.addWidget(lbl(txt, 9, C_MUTED))
            leg.addSpacing(4)
        leg.addStretch()
        l.addLayout(leg)

        l.addWidget(divider())

        # Reading panel
        rp = QHBoxLayout(); rp.setSpacing(16)
        self.enc_lbl  = self._reading_card("ENCODER PREDICTION",  C_ENCODER, "—  cm")
        self.ult_lbl  = self._reading_card("ULTRASONIC READING",  C_ULTRA,   "—  cm")
        self.ir_lbl   = self._reading_card("IR READING",          C_IR,      "—  cm")
        self.fuse_lbl = self._reading_card("FUSED ESTIMATE",      C_FUSION,  "—  cm")
        self.true_lbl = self._reading_card("TRUE POSITION",       C_ROBOT,   "—  cm")
        for w in (self.enc_lbl, self.ult_lbl, self.ir_lbl, self.fuse_lbl, self.true_lbl):
            rp.addWidget(w)
        l.addLayout(rp)

        # Insight box
        self.insight = QLabel(
            "💡  Press Animate to watch sensors report different values — "
            "notice how fusion tracks closer to truth than any single sensor.")
        self.insight.setWordWrap(True)
        self.insight.setStyleSheet(
            f"color:{C_YELLOW};font-family:{FONT_MONO};font-size:12px;"
            f"background:{C_CARD};border:1px solid {C_BORDER};"
            f"border-radius:6px;padding:10px 14px;")
        l.addWidget(self.insight)

    def _reading_card(self, title, color, init):
        f = card()
        fl = QVBoxLayout(f); fl.setContentsMargins(12, 8, 12, 8); fl.setSpacing(4)
        fl.addWidget(lbl(title, 7, C_MUTED))
        val = lbl(init, 18, color, bold=True)
        fl.addWidget(val)
        f._val_lbl = val
        return f

    def _update_readings(self):
        enc = self.canvas.encoder_cm
        ult = self.canvas.ultra_cm
        ir  = self.canvas.ir_cm
        fus = self.canvas.fusion_cm
        tr  = self.canvas.robot_cm
        self.enc_lbl._val_lbl.setText(f"{enc:.1f} cm")
        self.ult_lbl._val_lbl.setText(f"{ult:.1f} cm")
        self.ir_lbl._val_lbl.setText(f"{ir:.1f} cm")
        self.fuse_lbl._val_lbl.setText(f"{fus:.1f} cm")
        self.true_lbl._val_lbl.setText(f"{tr:.1f} cm")

        # Insight text when sensors disagree significantly
        spread = max(enc, ult, ir, fus) - min(enc, ult, ir, fus)
        if spread > 4:
            self.insight.setText(
                f"⚠  Sensors disagree by {spread:.1f} cm — "
                f"fusion resolves the conflict using weighted confidence.")
        else:
            self.insight.setText(
                f"✓  Sensors agree closely (spread {spread:.1f} cm) — "
                f"fusion estimate is very confident.")

    def _toggle_anim(self):
        if self._animating:
            self._timer.stop()
            self._animating = False
            self.anim_btn.setText("▶  Animate sensors")
        else:
            self._timer.start()
            self._animating = True
            self.anim_btn.setText("■  Stop")

    def set_hardware(self, robot):
        """Pass live RobotModel to use real sensors, or None to return to sim."""
        self._robot = robot

    def _anim_tick(self):
        if self._robot is not None:
            # ── LIVE: rx thread already calls fusion.step() every packet ──
            # Just read the already-updated results — do NOT call .step() again
            # as that would double-count encoder deltas.
            s   = self._robot.get_sensors()
            kf  = self._robot.scalar_fusion
            wf  = self._robot.weighted_fusion

            # Recover per-sensor position estimates directly from raw readings
            # using the same geometry as snp_model (OFFSETS applied there)
            OFFSETS_US  = -11.5   # ScalarPositionFusion.OFFSETS["us_air"]
            OFFSETS_IR  = -11.5   # ScalarPositionFusion.OFFSETS["ir"]

            enc_pos = wf._enc_pos  # dead-reckoned encoder position (cm)

            us_raw = s.ultrasonic
            us_pos = (us_raw + OFFSETS_US) if 2.0 < us_raw < ARENA_CM - 2.0 \
                     else enc_pos

            ir_raw = s.sharp_ir_distance
            ir_pos = (ir_raw + OFFSETS_IR) if 2.0 < ir_raw < ARENA_CM - 2.0 \
                     else enc_pos

            fused = kf.position
            true  = fused   # best proxy for truth on live hardware
        else:
            # ── SIM: synthesise noisy readings, call the real fusion API ────
            self._true_pos  = 20 + 20 * (0.5 + 0.5 * math.sin(self._step * 0.04))
            self._enc_drift += random.uniform(-0.05, 0.12)
            self._step += 1
            true = self._true_pos

            # Build a Sensors-like object with simulated raw values
            class _S: pass
            s = _S()
            s.ultrasonic        = (ARENA_CM - true) + random.gauss(0, 2.0)
            s.waterPultrasonic  = (ARENA_CM - true) + random.gauss(0, 2.5)
            s.sharp_ir_distance = true + random.gauss(0, 1.5)
            enc_ticks = int((true + self._enc_drift) * 20)
            s.encoder_left  = enc_ticks
            s.encoder_right = enc_ticks

            # Call WeightedPositionFusion and ScalarPositionFusion from snp_model
            self._sim_wfusion.step(s, s.encoder_left, s.encoder_right)
            self._sim_kfusion.step(s, s.encoder_left, s.encoder_right)

            # Recover per-sensor cm values: contributions[k] / weights[k]
            contrib = self._sim_wfusion.contributions
            _wu     = self._sim_wfusion.weights
            enc_pos = (contrib["encoder"] / _wu["encoder"]
                       if _wu.get("encoder", 0) > 0 else true + self._enc_drift)
            us_pos  = (contrib["us_air"]  / _wu["us_air"]
                       if _wu.get("us_air",  0) > 0 else ARENA_CM - s.ultrasonic)
            ir_pos  = (contrib["ir"]      / _wu["ir"]
                       if _wu.get("ir",      0) > 0 else s.sharp_ir_distance)
            fused   = self._sim_kfusion.position   # Kalman output

        self.canvas.robot_cm   = true
        self.canvas.encoder_cm = enc_pos
        self.canvas.ultra_cm   = us_pos
        self.canvas.ir_cm      = ir_pos
        self.canvas.fusion_cm  = fused
        self.canvas.update()
        self._update_readings()


C_YELLOW = "#ffd740"   # make accessible at module level


# ══════════════════════════════════════════════════════════════════════════════
# PAGE 2 — Weighted Average Fusion
# ══════════════════════════════════════════════════════════════════════════════
class WeightedCanvas(QWidget):
    """Number line showing three sensor estimates and the weighted average."""
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumHeight(120)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        self.enc   = 26.0
        self.ultra = 23.0
        self.ir    = 24.5
        self.w_enc = 0.33
        self.w_ult = 0.33
        self.w_ir  = 0.34
        self.fused = 24.5

    def _px(self, cm):
        W = self.width(); PAD = 100
        return PAD + (cm / 40.0) * (W - 2*PAD)

    def paintEvent(self, _):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        W, H = self.width(), self.height()
        p.fillRect(0, 0, W, H, QColor(C_SURFACE))

        PAD = 60
        cy  = H // 2

        # Axis
        p.setPen(QPen(QColor(C_BORDER), 2))
        p.drawLine(PAD, cy, W - PAD, cy)
        # Ticks every 5 cm up to 40
        p.setPen(QColor(C_MUTED))
        p.setFont(QFont(FONT_MONO, 11))
        for v in range(0, 41, 5):
            x = int(self._px(v))
            p.drawLine(x, cy - 4, x, cy + 4)
            p.drawText(x - 12, cy + 8, 24, 14, Qt.AlignCenter, str(v))

        # Uncertainty blobs
        def blob(cm, sigma_cm, color, y_off):
            cx   = self._px(cm)
            half = (sigma_cm / 40.0) * (W - 2*PAD)
            grad = QRadialGradient(cx, cy + y_off, half)
            c  = QColor(color); c.setAlpha(60)
            c2 = QColor(color); c2.setAlpha(0)
            grad.setColorAt(0, c); grad.setColorAt(1, c2)
            p.setBrush(QBrush(grad)); p.setPen(Qt.NoPen)
            p.drawEllipse(QRectF(cx - half, cy + y_off - 18, half*2, 36))

        blob(self.enc,   3.0, C_ENCODER, -18)
        blob(self.ultra, 5.0, C_ULTRA,     0)
        blob(self.ir,    2.5, C_IR,       18)

        # Sensor dots + labels
        def dot_label(cm, color, label, y_off):
            x = int(self._px(cm))
            p.setBrush(QColor(color)); p.setPen(Qt.NoPen)
            p.drawEllipse(QPointF(x, cy + y_off), 7, 7)
            p.setPen(QColor(color))
            p.setFont(QFont(FONT_MONO, 11, QFont.Bold))
            p.drawText(x - 30, cy + y_off - 22, 60, 14, Qt.AlignCenter,
                       f"{label}: {cm:.1f}")

        dot_label(self.enc,   C_ENCODER, "ENC",   -18)
        dot_label(self.ultra, C_ULTRA,   "ULTRA",   0)
        dot_label(self.ir,    C_IR,      "IR",     18)

        # Fused dot (larger)
        fx = int(self._px(self.fused))
        p.setBrush(QColor(C_FUSION)); p.setPen(Qt.NoPen)
        p.drawEllipse(QPointF(fx, cy), 11, 11)
        p.setPen(QColor(C_FUSION))
        p.setFont(QFont(FONT_MONO, 12, QFont.Bold))
        p.drawText(fx - 34, cy - 32, 68, 16, Qt.AlignCenter,
                   f"FUSED: {self.fused:.2f} cm")

        p.end()


class Page2(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setStyleSheet(f"background:{C_BG};")
        self._robot = None
        self._local_wfusion = WeightedPositionFusion()   # used when no hardware
        self._build()

    def _build(self):
        l = QVBoxLayout(self); l.setContentsMargins(20, 14, 20, 14); l.setSpacing(12)
        l.addWidget(title_lbl("PAGE 2 — WEIGHTED AVERAGE FUSION", 13))
        l.addWidget(lbl(
            "Each sensor has a different accuracy (σ).  A sensor you trust more gets a higher weight.\n"
            "Drag the sliders to change sensor values or weights — watch the fused estimate move.",
            9, C_MUTED, wrap=True))

        # Canvas
        wc = card()
        wl = QVBoxLayout(wc); wl.setContentsMargins(6, 6, 6, 6)
        self.wcanvas = WeightedCanvas()
        wl.addWidget(self.wcanvas)
        l.addWidget(wc)

        # ── Equation display ──
        eq_card = card()
        eq_l = QVBoxLayout(eq_card); eq_l.setContentsMargins(16, 12, 16, 12); eq_l.setSpacing(6)
        eq_l.addWidget(lbl("WEIGHTED AVERAGE FORMULA", 8, C_MUTED))
        eq_l.addWidget(divider())

        self.eq_lbl = QLabel()
        self.eq_lbl.setWordWrap(True)
        self.eq_lbl.setStyleSheet(
            f"color:{C_TEXT};font-family:{FONT_MONO};font-size:14px;"
            f"background:transparent;border:none;line-height:1.6;")
        eq_l.addWidget(self.eq_lbl)

        self.result_lbl = lbl("", 14, C_FUSION, bold=True)
        eq_l.addWidget(self.result_lbl)
        l.addWidget(eq_card)

        # ── Sliders ──
        sl_card = card()
        sl_l = QGridLayout(sl_card)
        sl_l.setContentsMargins(16, 12, 16, 12); sl_l.setSpacing(8)
        sl_l.addWidget(lbl("SENSOR CONTROLS", 8, C_MUTED), 0, 0, 1, 4)
        sl_l.addWidget(divider(), 1, 0, 1, 4)

        sensors = [
            ("Encoder value",    C_ENCODER, 10, 35, 260, "enc_val"),
            ("Encoder weight %", C_ENCODER,  0,100,  33, "enc_w"),
            ("Ultrasonic value", C_ULTRA,   10, 35, 230, "ult_val"),
            ("Ultrasonic weight%",C_ULTRA,   0,100,  33, "ult_w"),
            ("IR value",         C_IR,      10, 35, 245, "ir_val"),
            ("IR weight %",      C_IR,       0,100,  34, "ir_w"),
        ]

        self._sliders = {}
        self._val_lbls = {}
        for row_i, (name, color, lo, hi, init, key) in enumerate(sensors):
            r = row_i + 2
            sl_l.addWidget(lbl(name, 9, color), r, 0)
            sl = QSlider(Qt.Horizontal)
            sl.setRange(lo, hi)
            sl.setValue(init)
            sl.setStyleSheet(f"""
                QSlider::groove:horizontal{{background:{C_BORDER};height:6px;border-radius:3px;}}
                QSlider::handle:horizontal{{background:{color};width:14px;height:14px;
                    margin:-4px 0;border-radius:7px;}}
                QSlider::sub-page:horizontal{{background:{color};border-radius:3px;}}
            """)
            sl.valueChanged.connect(self._update)
            vl = lbl(str(init), 10, color, bold=True)
            vl.setFixedWidth(36)
            sl_l.addWidget(sl, r, 1)
            sl_l.addWidget(vl, r, 2)
            self._sliders[key]  = sl
            self._val_lbls[key] = vl

        sl_l.setColumnStretch(1, 1)
        l.addWidget(sl_card)

        # Insight
        self.insight2 = QLabel(
            "💡  Notice: when you increase a sensor's weight, the fused estimate moves toward it.  "
            "A sensor with smaller noise (σ) deserves a higher weight — that's the key idea behind fusion.")
        self.insight2.setWordWrap(True)
        self.insight2.setStyleSheet(
            f"color:{C_YELLOW};font-family:{FONT_MONO};font-size:12px;"
            f"background:{C_CARD};border:1px solid {C_BORDER};"
            f"border-radius:6px;padding:10px 14px;")
        l.addWidget(self.insight2)

        self._update()

    def set_hardware(self, robot):
        """Pass live RobotModel so sliders push weights into its weighted_fusion."""
        self._robot = robot

    def _update(self):
        enc_v = self._sliders["enc_val"].value() / 10.0
        ult_v = self._sliders["ult_val"].value() / 10.0
        ir_v  = self._sliders["ir_val"].value()  / 10.0
        enc_w = self._sliders["enc_w"].value()   / 100.0
        ult_w = self._sliders["ult_w"].value()   / 100.0
        ir_w  = self._sliders["ir_w"].value()    / 100.0

        for key, sl in self._sliders.items():
            v = sl.value()
            self._val_lbls[key].setText(
                f"{v/10:.1f}" if "val" in key else f"{v}%")

        total_w = enc_w + ult_w + ir_w
        if total_w < 0.01:
            return

        # ── Build synthetic Sensors object from slider values ─────────────
        # WeightedPositionFusion expects raw sensor readings, not cm positions.
        # Reverse-map slider cm → raw values using the same OFFSETS as snp_model.
        class _S: pass
        s = _S()
        s.encoder_left       = int(enc_v * 20)   # 20 counts/cm
        s.encoder_right      = s.encoder_left
        s.ultrasonic         = ARENA_CM - ult_v + 11.5   # OFFSETS["us_air"] = -11.5
        s.waterPultrasonic   = 0.0                        # not used here
        s.sharp_ir_distance  = ir_v + 11.5               # OFFSETS["ir"] = -11.5

        # Convert weight % → R values: higher weight = lower R (more trusted)
        # R = (1 - w) * 5 + 0.1  so 100% → R=0.1, 0% → R=5.1
        def w_to_R(w): return max(0.1, (1.0 - w) * 5.0 + 0.1)

        # Choose which fusion object to call: live or local sim instance
        wf = getattr(self._robot, "weighted_fusion", None)              if self._robot is not None else None
        if wf is None:
            wf = self._local_wfusion   # local instance created in __init__

        wf.set_weight("encoder",  w_to_R(enc_w))
        wf.set_weight("us_air",   w_to_R(ult_w))
        wf.set_weight("ir",       w_to_R(ir_w))
        # Seed encoder position directly so the display reflects slider value
        wf._enc_pos       = enc_v
        wf._prev_enc_left = wf._prev_enc_right = s.encoder_left

        fused = wf.step(s, s.encoder_left, s.encoder_right)
        actual_w = wf.weights   # normalised weights WeightedPositionFusion computed

        self.wcanvas.enc   = enc_v
        self.wcanvas.ultra = ult_v
        self.wcanvas.ir    = ir_v
        self.wcanvas.w_enc = actual_w.get("encoder", enc_w)
        self.wcanvas.w_ult = actual_w.get("us_air",  ult_w)
        self.wcanvas.w_ir  = actual_w.get("ir",      ir_w)
        self.wcanvas.fused = fused
        self.wcanvas.update()

        we = actual_w.get("encoder", enc_w)
        wu = actual_w.get("us_air",  ult_w)
        wi = actual_w.get("ir",      ir_w)
        self.eq_lbl.setText(
            f"  x̂  =  (w_enc × z_enc  +  w_ult × z_ult  +  w_ir × z_ir)\n"
            f"        ────────────────────────────────────────────────────\n"
            f"                        w_enc  +  w_ult  +  w_ir\n\n"
            f"     =  ({we:.3f} × {enc_v:.1f}  +  {wu:.3f} × {ult_v:.1f}  +  "
            f"{wi:.3f} × {ir_v:.1f})\n"
            f"        ────────────────────────────────────────────────────\n"
            f"                             {we+wu+wi:.3f}"
        )
        self.result_lbl.setText(
            f"  x̂  =  {fused:.2f} cm   [WeightedPositionFusion.step()]")


# ══════════════════════════════════════════════════════════════════════════════
# PAGE 3 — Kalman Filter
# ══════════════════════════════════════════════════════════════════════════════
class KalmanCanvas(QWidget):
    """Gaussian belief visualisation on a number line."""
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumHeight(160)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        # Prior (predict step)
        self.prior_mu    = 25.0
        self.prior_sigma = 4.0
        # Measurement
        self.z_mu        = 27.0
        self.z_sigma     = 2.0
        # Posterior (update step) — computed
        self.post_mu     = 26.0
        self.post_sigma  = 1.6

    def _px(self, cm):
        W = self.width(); PAD = 50
        return PAD + ((cm - 10) / 30.0) * (W - 2*PAD)

    def _draw_gaussian(self, p, mu, sigma, color, y_base, amp, label):
        W     = self.width()
        PAD   = 50
        scale = (W - 2*PAD) / 30.0   # px per cm
        pts   = []
        for i in range(200):
            cm = 10 + i * 30.0 / 200
            x  = self._px(cm)
            g  = math.exp(-0.5 * ((cm - mu) / sigma) ** 2)
            y  = y_base - int(amp * g)
            pts.append(QPointF(x, y))

        path = QPainterPath()
        if pts:
            path.moveTo(pts[0])
            for pt in pts[1:]:
                path.lineTo(pt)
        # Close area
        path.lineTo(pts[-1].x(), y_base)
        path.lineTo(pts[0].x(),  y_base)
        path.closeSubpath()

        c = QColor(color); c.setAlpha(50)
        p.setBrush(QBrush(c))
        p.setPen(QPen(QColor(color), 2))
        p.drawPath(path)

        # Mean line
        mx = int(self._px(mu))
        p.setPen(QPen(QColor(color), 2, Qt.DashLine))
        p.drawLine(mx, y_base - int(amp) - 4, mx, y_base + 8)

        # Label
        p.setPen(QColor(color))
        p.setFont(QFont(FONT_MONO, 11, QFont.Bold))
        p.drawText(mx - 35, y_base - int(amp) - 18, 70, 14,
                   Qt.AlignCenter, f"{label} μ={mu:.1f}")

    def paintEvent(self, _):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        W, H = self.width(), self.height()
        p.fillRect(0, 0, W, H, QColor(C_SURFACE))

        base = H - 28
        PAD  = 50

        # Axis
        p.setPen(QPen(QColor(C_BORDER), 2))
        p.drawLine(PAD, base, W - PAD, base)
        p.setPen(QColor(C_MUTED))
        p.setFont(QFont(FONT_MONO, 11))
        for v in range(10, 41, 5):
            x = int(self._px(v))
            p.drawLine(x, base - 3, x, base + 3)
            p.drawText(x - 12, base + 6, 24, 14, Qt.AlignCenter, str(v))

        AMP = (H - 60)
        self._draw_gaussian(p, self.prior_mu,  self.prior_sigma,
                            C_ENCODER, base, AMP * 0.55, "Prior")
        self._draw_gaussian(p, self.z_mu,      self.z_sigma,
                            C_ULTRA,   base, AMP * 0.65, "Meas.")
        self._draw_gaussian(p, self.post_mu,   self.post_sigma,
                            C_FUSION,  base, AMP * 0.85, "Posterior")
        p.end()


class Page3(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setStyleSheet(f"background:{C_BG};")
        self._step_idx = 0
        self._kf = ScalarPositionFusion()   # Kalman logic lives in snp_model
        self._build()

    def _build(self):
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setStyleSheet(
            f"QScrollArea{{background:{C_BG};border:none;}}"
            f"QScrollBar:vertical{{background:{C_BORDER};width:6px;border-radius:3px;}}"
            f"QScrollBar::handle:vertical{{background:{C_MUTED};border-radius:3px;}}")

        inner = QWidget(); inner.setStyleSheet(f"background:{C_BG};")
        il = QVBoxLayout(inner); il.setContentsMargins(20, 14, 20, 20); il.setSpacing(14)

        il.addWidget(title_lbl("PAGE 3 — KALMAN FILTER", 13))
        il.addWidget(lbl(
            "The Kalman Filter is an optimal weighted average that accounts for HOW UNCERTAIN "
            "each estimate is.  It models everything as a Gaussian (bell curve) and runs two steps "
            "every cycle: PREDICT and UPDATE.", 9, C_MUTED, wrap=True))

        # ── Gaussian canvas ──
        gc = card()
        gl = QVBoxLayout(gc); gl.setContentsMargins(6, 8, 6, 6); gl.setSpacing(6)
        gl.addWidget(lbl("BELIEF DISTRIBUTIONS  (wider = more uncertain)", 8, C_MUTED))
        self.kcanvas = KalmanCanvas()
        gl.addWidget(self.kcanvas)

        leg2 = QHBoxLayout()
        for col, txt in [(C_ENCODER, "Prior (predict step)"),
                         (C_ULTRA,   "Measurement"),
                         (C_FUSION,  "Posterior (after update)")]:
            leg2.addWidget(colored_dot(col)); leg2.addWidget(lbl(txt, 8, C_MUTED))
            leg2.addSpacing(12)
        leg2.addStretch()
        gl.addLayout(leg2)
        il.addWidget(gc)

        # ── Interactive sliders ──
        sc = card()
        sll = QGridLayout(sc); sll.setContentsMargins(16,12,16,12); sll.setSpacing(8)
        sll.addWidget(lbl("ADJUST PARAMETERS — see posterior shift in real time", 8, C_MUTED), 0,0,1,3)
        sll.addWidget(divider(), 1, 0, 1, 3)

        params = [
            ("Prior mean μ₀ (cm)",      C_ENCODER, 100, 350, 250, "pm"),
            ("Prior uncertainty σ₀",    C_ENCODER,  10, 100,  40, "ps"),
            ("Measurement z (cm)",      C_ULTRA,   100, 350, 270, "zm"),
            ("Measurement noise R",     C_ULTRA,    10, 100,  20, "zs"),
        ]
        self._ksliders = {}; self._kval_lbls = {}
        for i, (name, color, lo, hi, init, key) in enumerate(params):
            sll.addWidget(lbl(name, 9, color), i+2, 0)
            sl = QSlider(Qt.Horizontal)
            sl.setRange(lo, hi); sl.setValue(init)
            sl.setStyleSheet(f"""
                QSlider::groove:horizontal{{background:{C_BORDER};height:6px;border-radius:3px;}}
                QSlider::handle:horizontal{{background:{color};width:14px;height:14px;
                    margin:-4px 0;border-radius:7px;}}
                QSlider::sub-page:horizontal{{background:{color};border-radius:3px;}}
            """)
            sl.valueChanged.connect(self._kalman_update)
            vl = lbl(f"{init/10:.1f}", 10, color, bold=True); vl.setFixedWidth(40)
            sll.addWidget(sl, i+2, 1); sll.addWidget(vl, i+2, 2)
            self._ksliders[key] = sl; self._kval_lbls[key] = vl
        sll.setColumnStretch(1, 1)
        il.addWidget(sc)

        # ── Maths panel ──
        mc = card()
        ml2 = QVBoxLayout(mc); ml2.setContentsMargins(16,14,16,14); ml2.setSpacing(8)
        ml2.addWidget(lbl("THE MATHS — KALMAN UPDATE EQUATIONS", 8, C_MUTED))
        ml2.addWidget(divider())

        self.math_lbl = QLabel()
        self.math_lbl.setWordWrap(True)
        self.math_lbl.setTextFormat(Qt.PlainText)
        self.math_lbl.setStyleSheet(
            f"color:{C_TEXT};font-family:{FONT_MONO};font-size:14px;"
            f"background:transparent;border:none;line-height:1.8;")
        ml2.addWidget(self.math_lbl)
        il.addWidget(mc)

        # ── Step-through concept cards ──
        il.addWidget(lbl("STEP-THROUGH: TWO-PHASE CYCLE", 8, C_MUTED))

        steps = [
            ("PREDICT",  C_ENCODER,
             "1.  x̂⁻ₖ  =  F · x̂ₖ₋₁  +  B · uₖ\n"
             "2.  P⁻ₖ   =  F · Pₖ₋₁ · Fᵀ  +  Q\n\n"
             "The filter moves the belief forward using a motion model.\n"
             "Uncertainty P grows because motion adds process noise Q.\n"
             "Even without a sensor reading, we can make a prediction."),
            ("KALMAN GAIN",  C_YELLOW,
             "3.  K  =  P⁻ₖ · Hᵀ  ·  (H · P⁻ₖ · Hᵀ  +  R)⁻¹\n\n"
             "K balances trust between prediction and measurement.\n"
             "  K → 1  means trust the measurement (prediction was bad)\n"
             "  K → 0  means trust the prediction  (sensor is noisy)\n"
             "This is the 'smart weight' that ordinary averaging lacks."),
            ("UPDATE",  C_FUSION,
             "4.  x̂ₖ  =  x̂⁻ₖ  +  K · (zₖ  −  H · x̂⁻ₖ)\n"
             "5.  Pₖ   =  (I  −  K · H) · P⁻ₖ\n\n"
             "Innovation  (zₖ − H·x̂⁻ₖ)  is the surprise: how far off was prediction?\n"
             "The posterior is pulled toward the measurement by exactly K.\n"
             "Uncertainty P shrinks — we now know more than before."),
            ("WHY IT IS OPTIMAL",  C_KALMAN,
             "The Kalman Filter minimises Mean Squared Error.\n"
             "For linear Gaussian systems it is THE best possible estimator.\n\n"
             "Intuition:\n"
             "  • Encoder drifts → P grows → K rises → sensor trusted more\n"
             "  • Encoder reliable → P small → K small → prediction trusted\n"
             "  • Both noisy → posterior sits between them, weighted by σ²\n\n"
             "Real robots use Extended KF (nonlinear) or Unscented KF."),
        ]

        for title_s, color, body in steps:
            sc2 = QFrame()
            sc2.setStyleSheet(
                f"QFrame{{background:{C_CARD};border:1px solid {color}44;"
                f"border-left:3px solid {color};border-radius:8px;}}")
            scl = QVBoxLayout(sc2); scl.setContentsMargins(14,10,14,10); scl.setSpacing(6)
            scl.addWidget(lbl(title_s, 10, color, bold=True))
            body_lbl = QLabel(body)
            body_lbl.setWordWrap(True)
            body_lbl.setStyleSheet(
                f"color:{C_TEXT};font-family:{FONT_MONO};font-size:13px;"
                f"background:transparent;border:none;line-height:1.6;")
            scl.addWidget(body_lbl)
            il.addWidget(sc2)

        # Summary box
        summary = QLabel(
            "🔑  KEY TAKEAWAY\n\n"
            "Kalman Filter = Weighted Average  where the weights are computed automatically\n"
            "from the uncertainty of each source.  Certain sources get high weight; noisy\n"
            "sources get low weight.  The uncertainty itself is tracked and updated every step.")
        summary.setWordWrap(True)
        summary.setStyleSheet(
            f"color:{C_KALMAN};font-family:{FONT_MONO};font-size:13px;"
            f"background:{C_CARD};border:2px solid {C_KALMAN}44;"
            f"border-radius:8px;padding:14px;")
        il.addWidget(summary)

        scroll.setWidget(inner)
        outer = QVBoxLayout(self); outer.setContentsMargins(0,0,0,0)
        outer.addWidget(scroll)

        self._kalman_update()

    def _kalman_update(self):
        pm  = self._ksliders["pm"].value() / 10.0
        ps  = self._ksliders["ps"].value() / 10.0
        zm  = self._ksliders["zm"].value() / 10.0
        zs  = self._ksliders["zs"].value() / 10.0

        for key, sl in self._ksliders.items():
            v = sl.value() / 10.0
            self._kval_lbls[key].setText(f"{v:.1f}")

        # ── Use ScalarPositionFusion._update() from snp_model ─────────────
        # Inject prior state, call the real update function, read posterior back
        P  = ps ** 2
        R  = zs ** 2
        self._kf.Xk = pm
        self._kf.Pk = P
        self._kf._update(zm, R)   # ScalarPositionFusion._update(z, R)
        post_mu    = self._kf.Xk
        post_sigma = self._kf.uncertainty   # sqrt(Pk)
        K          = P / (P + R)            # recompute for equation display

        self.kcanvas.prior_mu    = pm
        self.kcanvas.prior_sigma = ps
        self.kcanvas.z_mu        = zm
        self.kcanvas.z_sigma     = zs
        self.kcanvas.post_mu     = post_mu
        self.kcanvas.post_sigma  = post_sigma
        self.kcanvas.update()

        self.math_lbl.setText(
            f"  Prior:        μ₀ = {pm:.1f} cm,    σ₀ = {ps:.1f}  →  P = σ₀² = {P:.2f}\n"
            f"  Measurement:  z  = {zm:.1f} cm,    R  = {zs:.1f}² = {R:.2f}\n\n"
            f"  Kalman Gain:  K  = P / (P + R)  =  {P:.2f} / ({P:.2f} + {R:.2f})  =  {K:.3f}\n\n"
            f"  Posterior mean:   μ  = μ₀ + K·(z − μ₀)\n"
            f"                       = {pm:.1f} + {K:.3f} × ({zm:.1f} − {pm:.1f})\n"
            f"                       = {post_mu:.2f} cm\n\n"
            f"  Posterior var:    P' = (1 − K) · P  =  {(1-K)*P:.3f}\n"
            f"  Posterior σ:      σ' = √P'          =  {post_sigma:.3f} cm\n\n"
            f"  → Posterior is {'CLOSER TO MEASUREMENT' if K > 0.5 else 'CLOSER TO PRIOR'} "
            f"  (K = {K:.3f}{'> 0.5' if K>0.5 else '≤ 0.5'})"
        )


# ══════════════════════════════════════════════════════════════════════════════
# Main Window with navigation
# ══════════════════════════════════════════════════════════════════════════════
class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Sensor Fusion Explorer")
        self.setMinimumSize(1100, 760)
        self.setStyleSheet(f"QMainWindow{{background:{C_BG};}}")
        self._build()

    def _build(self):
        root = QWidget(); root.setStyleSheet(f"background:{C_BG};")
        self.setCentralWidget(root)
        ml = QVBoxLayout(root); ml.setContentsMargins(0,0,0,0); ml.setSpacing(0)

        # ── Top nav bar ──
        nav = QWidget(); nav.setFixedHeight(56)
        nav.setStyleSheet(
            f"background:{C_SURFACE};border-bottom:2px solid {C_ACCENT};")
        nl = QHBoxLayout(nav); nl.setContentsMargins(24, 0, 24, 0); nl.setSpacing(10)

        title_w = QLabel("SENSOR FUSION EXPLORER")
        title_w.setStyleSheet(
            f"color:{C_ACCENT};font-family:{FONT_TITLE};font-size:20px;"
            f"font-weight:bold;background:transparent;letter-spacing:3px;")

        self.nav_btns = []
        pages = ["01 · World View", "02 · Weighted Avg", "03 · Kalman Filter"]
        for i, txt in enumerate(pages):
            b = nav_btn(txt, active=(i == 0))
            b.clicked.connect(lambda _, idx=i: self._go(idx))
            self.nav_btns.append(b)

        nl.addWidget(title_w)
        nl.addStretch()
        for b in self.nav_btns:
            nl.addWidget(b)
        ml.addWidget(nav)

        self.connect_btn = QPushButton("CONNECT ROBOT")
        self.connect_btn.clicked.connect(self._connect)

        self.disconnect_btn = QPushButton("DISCONNECT")
        self.disconnect_btn.clicked.connect(self._disconnect)

        nl.addWidget(self.connect_btn)
        nl.addWidget(self.disconnect_btn)
        # ── Stacked pages ──
        self.stack = QStackedWidget()
        self.stack.setStyleSheet(f"background:{C_BG};")
        self.p1 = Page1()
        self.p2 = Page2()
        self.p3 = Page3()
        self.stack.addWidget(self.p1)
        self.stack.addWidget(self.p2)
        self.stack.addWidget(self.p3)
        ml.addWidget(self.stack, stretch=1)

        # ── Bottom bar ──
        bot = QWidget(); bot.setFixedHeight(44)
        bot.setStyleSheet(
            f"background:{C_SURFACE};border-top:1px solid {C_BORDER};")
        bl = QHBoxLayout(bot); bl.setContentsMargins(24, 0, 24, 0)

        self.page_lbl = lbl("Page 1 of 3 — Robot World View", 9, C_MUTED)
        prev_btn = QPushButton("← PREV")
        next_btn = QPushButton("NEXT →")
        for b in (prev_btn, next_btn):
            b.setStyleSheet(f"""
                QPushButton{{background:transparent;color:{C_ACCENT};border:1px solid {C_BORDER};
                             border-radius:5px;padding:5px 14px;
                             font-family:{FONT_MONO};font-size:12px;font-weight:bold;}}
                QPushButton:hover{{background:{C_ACCENT}22;}}
            """)
        prev_btn.clicked.connect(lambda: self._go(max(0, self.stack.currentIndex()-1)))
        next_btn.clicked.connect(lambda: self._go(min(2, self.stack.currentIndex()+1)))

        bl.addWidget(self.page_lbl); bl.addStretch()
        bl.addWidget(prev_btn); bl.addWidget(next_btn)
        ml.addWidget(bot)

    def _connect(self):
        if not HW_AVAILABLE:
            from PyQt5.QtWidgets import QMessageBox
            QMessageBox.critical(self, "Import Error",
                "snp_model.py not found.\n"
                "Ensure snp_model.py is in the same directory as this script.")
            return
        try:
            self.real_robot   = RobotModel("COM7")
            self.hw_connected = True
            self.p1.set_hardware(self.real_robot)
            self.p2.set_hardware(self.real_robot)
        except Exception as e:
            from PyQt5.QtWidgets import QMessageBox
            QMessageBox.critical(self, "Connection Failed",
                f"Could not connect to COM7:\n{e}")

    def _disconnect(self):
        if getattr(self, "real_robot", None):
            try: self.real_robot.close()
            except: pass
        self.real_robot   = None
        self.hw_connected = False
        self.p1.set_hardware(None)
        self.p2.set_hardware(None)

    def _go(self, idx):
        self.stack.setCurrentIndex(idx)
        for i, b in enumerate(self.nav_btns):
            bg = C_ACCENT if i == idx else C_CARD
            fg = "#000"   if i == idx else C_MUTED
            b.setStyleSheet(f"""
                QPushButton{{background:{bg};color:{fg};border:1px solid {C_BORDER};
                             border-radius:6px;padding:7px 18px;
                             font-family:{FONT_MONO};font-size:13px;font-weight:bold;}}
                QPushButton:hover{{background:{C_ACCENT}44;color:{C_ACCENT};}}
            """)
        names = ["Robot World View", "Weighted Average", "Kalman Filter"]
        self.page_lbl.setText(f"Page {idx+1} of 3 — {names[idx]}")


# ══════════════════════════════════════════════════════════════════════════════
if __name__ == "__main__":
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    win = MainWindow()
    win.show()
    sys.exit(app.exec_())