"""
Robot Sensor Challenge — PyQt5 GUI  (v3)
=========================================
Educational robot challenge: move 25 cm using one sensor at a time.

Key improvements in this version
─────────────────────────────────
• Chart shows DISTANCE (cm) for both Current and Actual — always apples-to-apples
• Live conversion formula shown per sensor  (e.g. "ticks ÷ 41.2 = cm")
• Sensor Report Card appears after STOP:
    – Prediction accuracy
    – Noise / jitter rating (live variance of last 10 samples)
    – A one-liner limitation hint per sensor
• Session log panel: records each attempt across all sensors for comparison
• Layout uses the right column space fully

Run  :  python robot_sensor_gui.py
Needs:  pip install PyQt5
"""

import sys, random, collections, math, os
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QLabel, QPushButton,
    QComboBox, QLineEdit, QVBoxLayout, QHBoxLayout, QGridLayout,
    QFrame, QSizePolicy, QMessageBox, QScrollArea
)
from PyQt5.QtCore import Qt, QTimer, QRectF, QPointF
from PyQt5.QtGui  import QPainter, QColor, QPen, QBrush, QFont, QLinearGradient, QPolygonF

# ── Import fusion from snp_model (same directory) ─────────────────────────────
_HERE = os.path.dirname(os.path.abspath(__file__))

# go one level up (SNP folder)
_ROOT = os.path.dirname(_HERE)

# add SNP to python path
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)
    
try:
    from robot_core.snp_model import ScalarPositionFusion
    _FUSION_AVAILABLE = True
except ImportError:
    _FUSION_AVAILABLE = False
    ScalarPositionFusion = None

# ── Constants ──────────────────────────────────────────────────────────────────
ARENA_CM  = 90.0
TARGET_CM = 25.0
TOLERANCE = 0.5          # ±0.5 cm = ±5 mm

# Conversion: raw reading → cm distance travelled
# distance_cm = (reading - start) / scale  (scale can be negative for ultrasonic)
SENSORS = {
    "Encoder — Left Wheel": {
        "key":        "encoder_left",
        "unit":       "ticks",
        "hint":       "Count encoder ticks. How many ticks equal 25 cm of travel?",
        "sim_scale":  41.2,
        "sim_start":  0.0,
        "formula":    "ticks ÷ 41.2  =  cm travelled",
        "to_cm":      lambda r, start=0.0,  scale=41.2: (r - start) / scale,
        "limitation": "Encoders accumulate drift — small slip errors add up over distance.",
        "noise_char": "low",
    },
    "Ultrasonic — Front": {
        "key":        "ultrasonic",
        "unit":       "cm",
        "hint":       "Arena is 90 cm. Sensor reads distance to front wall. After 25 cm moved → what reading?",
        "sim_scale":  -1.0,
        "sim_start":  90.0,
        "formula":    "90 − reading  =  cm travelled",
        "to_cm":      lambda r, start=90.0, scale=-1.0: (start - r),
        "limitation": "Ultrasonic can't distinguish surfaces < 3 cm apart and bounces off angled walls.",
        "noise_char": "medium",
    },
    "Sharp IR — Back": {
        "key":        "sharp_ir_distance",
        "unit":       "cm",
        "hint":       "Measures distance behind the robot. Starts ~5 cm. What after 25 cm forward?",
        "sim_scale":  1.0,
        "sim_start":  5.0,
        "formula":    "reading − 5  =  cm travelled",
        "to_cm":      lambda r, start=5.0,  scale=1.0:  (r - start),
        "limitation": "Sharp IR is highly nonlinear beyond 30 cm and sensitive to surface colour.",
        "noise_char": "high",
    },
}

NOISE_COLORS = {"low": "#00e676", "medium": "#ffd740", "high": "#ff4d6d"}

# ── Palette ────────────────────────────────────────────────────────────────────
C_BG      = "#0b0e17"
C_SURFACE = "#111520"
C_CARD    = "#161b2e"
C_BORDER  = "#1e2540"
C_ACCENT  = "#00d4ff"
C_ACCENT2 = "#ff4d6d"
C_SUCCESS = "#00e676"
C_FAIL    = "#ff1744"
C_YELLOW  = "#ffd740"
C_TEXT    = "#dce6f5"
C_MUTED   = "#4a5a7a"
C_CURRENT = "#00d4ff"
C_ACTUAL  = "#ff4d6d"
FONT      = "Consolas"


# ══════════════════════════════════════════════════════════════════════════════
class SimRobot:
    def __init__(self):
        self.position_cm = 0.0

    def move(self, delta):
        self.position_cm = max(0.0, min(ARENA_CM, self.position_cm + delta))

    def get_reading(self, name):
        m = SENSORS[name]
        noise = {"Encoder — Left Wheel": random.uniform(-2, 2),
                 "Ultrasonic — Front":   random.uniform(-0.5, 0.5),
                 "Sharp IR — Back":      random.uniform(-0.8, 0.8)}[name]
        return round(m["sim_start"] + m["sim_scale"] * self.position_cm + noise, 1)

    def get_position(self):
        return self.position_cm

    def reset(self):
        self.position_cm = 0.0


# ══════════════════════════════════════════════════════════════════════════════
class SimSensors:
    """
    Minimal Sensors-compatible object for use with ScalarPositionFusion
    in simulation mode.  Mirrors the attribute names expected by step().
    """
    def __init__(self):
        self.sharp_ir_distance = 0.0
        self.waterPultrasonic  = 0.0
        self.ultrasonic        = 0.0
        self.encoder_left      = 0
        self.encoder_right     = 0

    def update(self, robot: "SimRobot", name: str):
        """Populate from SimRobot state so fusion gets realistic readings."""
        pos = robot.get_position()
        # Ultrasonic: reads distance to front wall
        self.ultrasonic        = round(ARENA_CM - pos + random.uniform(-0.3, 0.3), 1)
        # Waterproof ultrasonic: reads distance from back wall (same as pos)
        self.waterPultrasonic  = round(pos + random.uniform(-0.3, 0.3), 1)
        # Sharp IR: reads distance behind robot (approx pos + 5 cm offset)
        self.sharp_ir_distance = round(pos + 5.0 + random.uniform(-0.5, 0.5), 1)
        # Encoders: cumulative ticks
        self.encoder_left      = int(pos * 41.2 + random.uniform(-1, 1))
        self.encoder_right     = int(pos * 41.2 + random.uniform(-1, 1))


# ══════════════════════════════════════════════════════════════════════════════
class RobotTopView(QWidget):
    """
    Top-down (bird's eye) view of the robot inside its 90 cm arena.

    Layout (landscape orientation):
      ┌─────────────────────────────────────┐
      │  WALL          arena 90 cm         │
      │  ┌──────────────────────────────┐  │
      │  │  [US]  ╔══════╗  [US_W]     │  │
      │  │        ║ BODY ║             │  │
      │  │   [IR] ╚══════╝             │  │
      │  └──────────────────────────────┘  │
      │  HOME ──────────────→ 25 cm target │
      └─────────────────────────────────────┘

    The robot body (rectangle) slides right as position increases.
    A dashed target line shows the 25 cm goal.
    Sensor cone arcs are drawn to show what each sensor sees.
    """

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(300, 180)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self._pos_cm      = 0.0   # robot position (cm)
        self._fusion_cm   = 0.0   # fusion position (shown separately)
        self._active_sensor = ""  # currently selected sensor name

    def set_position(self, pos_cm: float, fusion_cm: float = None):
        self._pos_cm    = max(0.0, min(ARENA_CM, pos_cm))
        if fusion_cm is not None:
            self._fusion_cm = max(0.0, min(ARENA_CM, fusion_cm))
        self.update()

    def set_sensor(self, name: str):
        self._active_sensor = name
        self.update()

    def paintEvent(self, _):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)

        W, H  = self.width(), self.height()

        # ── Margins & arena geometry ──────────────────────────────────────
        PAD_L  = 10
        PAD_R  = 10
        PAD_T  = 32     # room for title + labels above
        PAD_B  = 26     # room for distance ruler below
        AW     = W - PAD_L - PAD_R   # arena pixel width
        AH     = H - PAD_T - PAD_B   # arena pixel height

        def cm_to_px(cm):
            return PAD_L + int(AW * cm / ARENA_CM)

        # ── Background ────────────────────────────────────────────────────
        p.fillRect(0, 0, W, H, QColor(C_CARD))

        # Title
        p.setPen(QColor(C_MUTED))
        p.setFont(QFont(FONT, 8))
        p.drawText(0, 6, W, 18, Qt.AlignCenter, "ARENA — TOP VIEW")

        # ── Arena floor ───────────────────────────────────────────────────
        p.setBrush(QColor("#0d1120"))
        p.setPen(QPen(QColor(C_BORDER), 1))
        p.drawRect(PAD_L, PAD_T, AW, AH)

        # Grid lines every 10 cm
        p.setPen(QPen(QColor(C_BORDER), 1, Qt.DotLine))
        for cm in range(10, int(ARENA_CM), 10):
            x = cm_to_px(cm)
            p.drawLine(x, PAD_T, x, PAD_T + AH)

        # ── Target line at 25 cm ──────────────────────────────────────────
        tx = cm_to_px(TARGET_CM)
        p.setPen(QPen(QColor(C_YELLOW), 2, Qt.DashLine))
        p.drawLine(tx, PAD_T, tx, PAD_T + AH)
        p.setPen(QColor(C_YELLOW))
        p.setFont(QFont(FONT, 7, QFont.Bold))
        p.drawText(tx - 20, PAD_T + 3, 40, 14, Qt.AlignCenter, "25 cm")

        # ── Robot geometry ────────────────────────────────────────────────
        # Robot body: 12 cm × 10 cm → scaled to pixels
        R_W_CM = 12.0
        R_H_CM = 10.0
        r_px_w = int(AW * R_W_CM / ARENA_CM)
        r_px_h = int(AH * R_H_CM / ARENA_CM) if AH > 0 else 30
        r_px_h = max(r_px_h, 28)   # minimum visible height

        robot_x = cm_to_px(self._pos_cm)             # left edge of robot
        robot_cy = PAD_T + AH // 2                   # vertical centre

        # ── Sensor cones ──────────────────────────────────────────────────
        sensor_key = SENSORS.get(self._active_sensor, {}).get("key", "")

        # Front ultrasonic — cyan cone pointing right
        if "ultrasonic" in sensor_key and "water" not in sensor_key:
            cone_x   = robot_x + r_px_w
            cone_len = int(AW * (ARENA_CM - self._pos_cm - R_W_CM) / ARENA_CM)
            cone_len = max(cone_len, 4)
            cone_half = r_px_h // 3
            pts = QPolygonF([
                QPointF(cone_x, robot_cy),
                QPointF(cone_x + cone_len, robot_cy - cone_half),
                QPointF(cone_x + cone_len, robot_cy + cone_half),
            ])
            cone_col = QColor(C_ACCENT); cone_col.setAlpha(55)
            p.setBrush(QBrush(cone_col))
            p.setPen(QPen(QColor(C_ACCENT), 1, Qt.DotLine))
            p.drawPolygon(pts)

        # Sharp IR — red cone pointing left (back sensor)
        elif sensor_key == "sharp_ir_distance":
            cone_x   = robot_x
            cone_len = int(AW * max(self._pos_cm - R_W_CM * 0.5, 2) / ARENA_CM)
            cone_len = max(cone_len, 4)
            cone_half = r_px_h // 4
            pts = QPolygonF([
                QPointF(cone_x, robot_cy),
                QPointF(cone_x - cone_len, robot_cy - cone_half),
                QPointF(cone_x - cone_len, robot_cy + cone_half),
            ])
            cone_col = QColor(C_ACCENT2); cone_col.setAlpha(55)
            p.setBrush(QBrush(cone_col))
            p.setPen(QPen(QColor(C_ACCENT2), 1, Qt.DotLine))
            p.drawPolygon(pts)

        # ── Draw wheels (4 corner rectangles) ────────────────────────────
        w_w = max(r_px_w // 7, 4)
        w_h = max(r_px_h // 4, 6)
        wheel_col = QColor("#2a3050")
        p.setBrush(QBrush(wheel_col)); p.setPen(Qt.NoPen)
        # Front wheels
        p.drawRect(robot_x + r_px_w - w_w,
                   robot_cy - r_px_h // 2 - w_h // 2, w_w, w_h)
        p.drawRect(robot_x + r_px_w - w_w,
                   robot_cy + r_px_h // 2 - w_h // 2, w_w, w_h)
        # Rear wheels
        p.drawRect(robot_x,
                   robot_cy - r_px_h // 2 - w_h // 2, w_w, w_h)
        p.drawRect(robot_x,
                   robot_cy + r_px_h // 2 - w_h // 2, w_w, w_h)

        # ── Robot body ────────────────────────────────────────────────────
        body_grad = QLinearGradient(robot_x, robot_cy - r_px_h // 2,
                                    robot_x, robot_cy + r_px_h // 2)
        body_grad.setColorAt(0.0, QColor("#1e2a45"))
        body_grad.setColorAt(1.0, QColor("#141d33"))
        p.setBrush(QBrush(body_grad))
        p.setPen(QPen(QColor(C_ACCENT), 1))
        p.drawRoundedRect(robot_x, robot_cy - r_px_h // 2, r_px_w, r_px_h, 3, 3)

        # Forward arrow on body
        arr_x = robot_x + r_px_w // 2
        p.setPen(QPen(QColor(C_ACCENT), 2))
        p.drawLine(arr_x - 4, robot_cy, arr_x + 5, robot_cy)
        p.drawLine(arr_x + 5, robot_cy, arr_x + 2, robot_cy - 3)
        p.drawLine(arr_x + 5, robot_cy, arr_x + 2, robot_cy + 3)

        # ── Sensor dot labels ─────────────────────────────────────────────
        # Front ultrasonic dot (right side)
        us_x = robot_x + r_px_w - 2
        p.setBrush(QColor(C_ACCENT)); p.setPen(Qt.NoPen)
        p.drawEllipse(us_x - 3, robot_cy - r_px_h // 2 + 3, 6, 6)
        p.setPen(QColor(C_ACCENT)); p.setFont(QFont(FONT, 6))
        p.drawText(us_x - 10, robot_cy - r_px_h // 2 + 10, 20, 10,
                   Qt.AlignCenter, "US")

        # Back IR dot (left side)
        ir_x = robot_x + 2
        p.setBrush(QColor(C_ACCENT2)); p.setPen(Qt.NoPen)
        p.drawEllipse(ir_x - 3, robot_cy + r_px_h // 2 - 9, 6, 6)
        p.setPen(QColor(C_ACCENT2)); p.setFont(QFont(FONT, 6))
        p.drawText(ir_x - 10, robot_cy + r_px_h // 2 - 14, 20, 10,
                   Qt.AlignCenter, "IR")

        # ── Encoder wheel highlight when encoder selected ─────────────────
        if sensor_key == "encoder_left":
            p.setBrush(Qt.NoBrush)
            p.setPen(QPen(QColor(C_YELLOW), 2))
            p.drawRect(robot_x,
                       robot_cy + r_px_h // 2 - w_h // 2, w_w, w_h)
            p.setPen(QColor(C_YELLOW)); p.setFont(QFont(FONT, 6))
            p.drawText(robot_x - 16, robot_cy + r_px_h // 2 + 2, 32, 10,
                       Qt.AlignCenter, "ENC")

        # ── Fusion ghost position ─────────────────────────────────────────
        if abs(self._fusion_cm - self._pos_cm) > 0.5:
            fx = cm_to_px(self._fusion_cm)
            p.setPen(QPen(QColor(C_ACTUAL), 1, Qt.DashLine))
            p.setBrush(Qt.NoBrush)
            p.drawRect(fx, robot_cy - r_px_h // 2, r_px_w, r_px_h)

        # ── Distance ruler below arena ────────────────────────────────────
        ruler_y = PAD_T + AH + 4
        p.setPen(QColor(C_MUTED))
        p.setFont(QFont(FONT, 7))
        for cm in range(0, int(ARENA_CM) + 1, 10):
            x = cm_to_px(cm)
            p.drawLine(x, ruler_y, x, ruler_y + 4)
            p.drawText(x - 12, ruler_y + 5, 24, 12, Qt.AlignCenter, str(cm))

        # Position readout
        p.setPen(QColor(C_ACCENT))
        p.setFont(QFont(FONT, 8, QFont.Bold))
        p.drawText(W - 80, PAD_T + 4, 76, 16, Qt.AlignRight | Qt.AlignVCenter,
                   f"{self._pos_cm:.1f} cm")

        p.end()


# ══════════════════════════════════════════════════════════════════════════════
class DistanceBarChart(QWidget):
    """
    Two vertical bars always in centimetres:
      • Current  – distance estimated from selected sensor reading
      • Actual   – sensor-fusion ground truth (hidden until STOP)
    A horizontal target line marks 25 cm.
    """
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(280, 300)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.current_cm   = 0.0
        self.actual_cm    = 0.0
        self._show_actual = False

    def set_values(self, current_cm, actual_cm):
        self.current_cm = current_cm
        self.actual_cm  = actual_cm
        self.update()

    def reveal(self, show: bool):
        self._show_actual = show
        self.update()

    def paintEvent(self, _):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        W, H   = self.width(), self.height()
        PAD_L  = 48
        PAD_R  = 16
        PAD_T  = 36
        PAD_B  = 50
        CW     = W - PAD_L - PAD_R
        CH     = H - PAD_T - PAD_B
        MAX_CM = 35.0   # chart tops out at 35 cm (target is 25)

        p.fillRect(0, 0, W, H, QColor(C_CARD))

        # Grid lines + Y labels
        for i in range(6):
            val = MAX_CM * i / 5
            y   = PAD_T + CH - int(CH * i / 5)
            p.setPen(QPen(QColor(C_BORDER), 1))
            p.drawLine(PAD_L, y, W - PAD_R, y)
            p.setPen(QColor(C_MUTED))
            p.setFont(QFont(FONT, 8))
            p.drawText(2, y - 8, PAD_L - 6, 18,
                       Qt.AlignRight | Qt.AlignVCenter, f"{val:.0f}")

        # Y-axis spine
        p.setPen(QPen(QColor(C_MUTED), 1))
        p.drawLine(PAD_L, PAD_T, PAD_L, PAD_T + CH)

        # TARGET LINE at 25 cm
        ty = PAD_T + CH - int(CH * TARGET_CM / MAX_CM)
        p.setPen(QPen(QColor(C_YELLOW), 2, Qt.DashLine))
        p.drawLine(PAD_L, ty, W - PAD_R, ty)
        p.setPen(QColor(C_YELLOW))
        p.setFont(QFont(FONT, 8, QFont.Bold))
        p.drawText(W - PAD_R - 48, ty - 14, 60, 14,
                   Qt.AlignLeft, "← 25 cm")

        # Bars
        GAP = 20
        BW  = (CW - GAP * 3) // 2

        bars = [
            ("Current\n(sensor)",  self.current_cm, C_CURRENT),
            ("Actual\n(fusion)",   self.actual_cm,  C_ACTUAL),
        ]

        for i, (lbl, val, col) in enumerate(bars):
            x = PAD_L + GAP + i * (BW + GAP)

            if lbl.startswith("Actual") and not self._show_actual:
                p.setPen(QColor(C_MUTED))
                p.setFont(QFont(FONT, 18, QFont.Bold))
                p.drawText(x, PAD_T, BW, CH, Qt.AlignCenter, "?")
                p.setFont(QFont(FONT, 8))
                p.drawText(x, PAD_T + CH + 8,  BW, 18, Qt.AlignCenter, "cm")
                for j, line in enumerate(lbl.split("\n")):
                    p.drawText(x, PAD_T + CH + 22 + j * 14, BW, 14,
                               Qt.AlignCenter, line)
                continue

            clipped = max(min(val, MAX_CM), 0.0)
            frac    = clipped / MAX_CM
            bar_h   = max(int(CH * frac), 3)
            y_top   = PAD_T + CH - bar_h

            grad = QLinearGradient(x, y_top, x, y_top + bar_h)
            c  = QColor(col); c2 = QColor(col); c2.setAlpha(120)
            grad.setColorAt(0.0, c); grad.setColorAt(1.0, c2)
            p.setBrush(QBrush(grad)); p.setPen(Qt.NoPen)
            p.drawRoundedRect(x, y_top, BW, bar_h, 5, 5)

            # Value label
            p.setPen(QColor(col))
            p.setFont(QFont(FONT, 10, QFont.Bold))
            p.drawText(x, max(y_top - 20, PAD_T), BW, 18,
                       Qt.AlignCenter, f"{val:.1f} cm")

            # Axis labels
            p.setPen(QColor(C_MUTED))
            p.setFont(QFont(FONT, 8))
            p.drawText(x, PAD_T + CH + 8,  BW, 18, Qt.AlignCenter, "cm")
            for j, line in enumerate(lbl.split("\n")):
                p.drawText(x, PAD_T + CH + 22 + j * 14, BW, 14,
                           Qt.AlignCenter, line)

        # Chart title
        p.setPen(QColor(C_MUTED))
        p.setFont(QFont(FONT, 8))
        p.drawText(0, 8, W, 18, Qt.AlignCenter, "DISTANCE TRAVELLED  (cm)")
        p.end()


# ══════════════════════════════════════════════════════════════════════════════
class NoiseBar(QWidget):
    """Compact horizontal bar showing live sensor jitter level."""
    def __init__(self, label="Noise", parent=None):
        super().__init__(parent)
        self.setFixedHeight(22)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        self._label  = label
        self._level  = 0.0   # 0–1

    def set_level(self, level: float):
        self._level = max(0.0, min(level, 1.0))
        self.update()

    def paintEvent(self, _):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        W, H = self.width(), self.height()
        LW   = 80
        BH   = 10
        by   = (H - BH) // 2

        p.fillRect(0, 0, W, H, QColor(C_CARD))

        # Label
        p.setPen(QColor(C_MUTED))
        p.setFont(QFont(FONT, 8))
        p.drawText(0, 0, LW, H, Qt.AlignVCenter | Qt.AlignLeft, self._label)

        bx = LW
        bw = W - LW - 4
        # Track
        p.setBrush(QColor(C_BORDER)); p.setPen(Qt.NoPen)
        p.drawRoundedRect(bx, by, bw, BH, 3, 3)
        # Fill
        fw = int(bw * self._level)
        if fw > 0:
            lvl = self._level
            col = QColor("#00e676") if lvl < 0.35 else \
                  QColor("#ffd740") if lvl < 0.65 else QColor("#ff4d6d")
            p.setBrush(col)
            p.drawRoundedRect(bx, by, fw, BH, 3, 3)
        p.end()


# ══════════════════════════════════════════════════════════════════════════════
class KeyWidget(QLabel):
    def __init__(self, text, parent=None):
        super().__init__(text, parent)
        self.setFixedSize(40, 40)
        self.setAlignment(Qt.AlignCenter)
        self.set_active(False)

    def set_active(self, v):
        if v:
            self.setStyleSheet(
                f"background:{C_ACCENT};color:#000;border-radius:5px;"
                f"font-family:{FONT};font-size:14px;font-weight:bold;")
        else:
            self.setStyleSheet(
                f"background:{C_BORDER};color:{C_TEXT};border-radius:5px;"
                f"font-family:{FONT};font-size:14px;font-weight:bold;"
                f"border:1px solid {C_MUTED};")


# ── Widget helpers ─────────────────────────────────────────────────────────────
def lbl(text, size=10, color=C_TEXT, bold=False):
    l = QLabel(text)
    l.setStyleSheet(
        f"color:{color};font-family:{FONT};font-size:{size}px;"
        f"font-weight:{'bold' if bold else 'normal'};"
        f"background:transparent;border:none;")
    return l

def btn(text, bg=C_ACCENT, fg="#000", size=11):
    b = QPushButton(text)
    b.setStyleSheet(f"""
        QPushButton{{background:{bg};color:{fg};border:none;border-radius:7px;
                     padding:8px 16px;font-family:{FONT};font-size:{size}px;font-weight:bold;}}
        QPushButton:hover   {{background:{bg}cc;}}
        QPushButton:pressed {{background:{bg}77;}}
        QPushButton:disabled{{background:{C_MUTED};color:{C_BORDER};}}
    """)
    return b

def card():
    f = QFrame()
    f.setStyleSheet(
        f"QFrame{{background:{C_CARD};border:1px solid {C_BORDER};border-radius:10px;}}")
    return f

def divider():
    d = QFrame(); d.setFrameShape(QFrame.HLine)
    d.setStyleSheet(f"background:{C_BORDER};border:none;max-height:1px;")
    return d


# ══════════════════════════════════════════════════════════════════════════════
class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Robot Sensor Challenge")
        self.setMinimumSize(1100, 740)
        self.setStyleSheet(f"QMainWindow{{background:{C_BG};}}")

        self.sim          = SimRobot()
        self.real_robot   = None
        self.hw_connected = False

        # ScalarPositionFusion used in both sim and live modes
        if _FUSION_AVAILABLE:
            self.fusion     = ScalarPositionFusion(arena_length_cm=ARENA_CM)
            self.sim_fusion = ScalarPositionFusion(arena_length_cm=ARENA_CM)
        else:
            self.fusion     = None
            self.sim_fusion = None
        self.sim_sensors = SimSensors()

        self.running      = False
        self.keys_held    = set()
        self.user_pred    = 0.0
        self.speed_cm     = 1.2
        self._noise_buf   = collections.deque(maxlen=12)  # last N raw readings
        self._session_log = []  # list of dicts per attempt

        # Pre-create widgets referenced by _sensor_changed before full UI build
        self.pred_unit_lbl = lbl("", 12, C_MUTED)
        self.unit_lbl      = lbl("", 11, C_MUTED)
        self.hint_lbl      = lbl("", 9, C_YELLOW)
        self.hint_lbl.setWordWrap(True)
        self.formula_lbl   = lbl("", 10, C_ACCENT)

        self._build_ui()

        self.drive_timer = QTimer(); self.drive_timer.setInterval(100)
        self.drive_timer.timeout.connect(self._drive_tick)

        self.poll_timer = QTimer(); self.poll_timer.setInterval(200)
        self.poll_timer.timeout.connect(self._poll)
        self.poll_timer.start()

    # ══════════════════════════════════════════════════════════════════════════
    def _build_ui(self):
        root = QWidget(); root.setStyleSheet(f"background:{C_BG};")
        self.setCentralWidget(root)
        ml = QVBoxLayout(root); ml.setContentsMargins(0, 0, 0, 0); ml.setSpacing(0)
        ml.addWidget(self._build_header())

        body = QWidget(); body.setStyleSheet(f"background:{C_BG};")
        bl = QHBoxLayout(body); bl.setContentsMargins(22, 16, 22, 16); bl.setSpacing(18)
        bl.addWidget(self._build_left(),  stretch=5)
        bl.addWidget(self._build_right(), stretch=5)
        ml.addWidget(body)

    # ── Header ─────────────────────────────────────────────────────────────────
    def _build_header(self):
        h = QWidget(); h.setFixedHeight(68)
        h.setStyleSheet(f"background:{C_SURFACE};border-bottom:2px solid {C_ACCENT};")
        hl = QHBoxLayout(h); hl.setContentsMargins(26, 0, 26, 0); hl.setSpacing(12)

        title = QLabel("ROBOT SENSOR CHALLENGE")
        title.setStyleSheet(
            f"color:{C_ACCENT};font-family:Courier New;font-size:20px;"
            f"font-weight:bold;background:transparent;")
        sub = QLabel("Use one sensor at a time to move the robot exactly 25 cm")
        sub.setStyleSheet(
            f"color:{C_MUTED};font-family:{FONT};font-size:9px;background:transparent;")
        tcol = QVBoxLayout(); tcol.setSpacing(2)
        tcol.addWidget(title); tcol.addWidget(sub)

        self.conn_badge = QLabel("  SIM MODE  ")
        self.conn_badge.setStyleSheet(
            f"background:{C_YELLOW};color:#000;font-family:{FONT};"
            f"font-size:9px;font-weight:bold;border-radius:4px;padding:3px 8px;")

        self.conn_btn = btn("⚡  Connect to COM7", C_SURFACE, C_ACCENT, 9)
        self.conn_btn.setStyleSheet(
            self.conn_btn.styleSheet() +
            f"QPushButton{{border:1px solid {C_ACCENT};}}")
        self.conn_btn.clicked.connect(self._toggle_connection)

        hl.addLayout(tcol)
        hl.addStretch()
        hl.addWidget(self.conn_btn)
        hl.addWidget(self.conn_badge)
        return h

    # ── Left column ────────────────────────────────────────────────────────────
    def _build_left(self):
        w = QWidget(); w.setStyleSheet("background:transparent;")
        l = QVBoxLayout(w); l.setContentsMargins(0, 0, 0, 0); l.setSpacing(12)
        l.addWidget(self._build_sensor_card())
        l.addWidget(self._build_prediction_card())
        l.addWidget(self._build_controls_card())
        l.addWidget(self._build_result_card())
        l.addStretch()
        return w

    def _build_sensor_card(self):
        f = card()
        l = QVBoxLayout(f); l.setContentsMargins(16, 12, 16, 12); l.setSpacing(7)
        l.addWidget(lbl("① SELECT SENSOR", 8, C_MUTED))
        l.addWidget(divider())

        self.combo = QComboBox()
        self.combo.addItems(list(SENSORS.keys()))
        self.combo.setStyleSheet(f"""
            QComboBox{{background:{C_BG};color:{C_TEXT};border:1px solid {C_BORDER};
                       border-radius:6px;padding:7px 12px;font-family:{FONT};font-size:11px;}}
            QComboBox::drop-down{{border:none;width:22px;}}
            QComboBox QAbstractItemView{{background:{C_CARD};color:{C_TEXT};
                selection-background-color:{C_ACCENT};selection-color:#000;
                border:1px solid {C_BORDER};font-family:{FONT};}}
        """)
        self.combo.currentTextChanged.connect(self._sensor_changed)
        l.addWidget(self.combo)

        # Formula row
        formula_row = QHBoxLayout()
        formula_row.addWidget(lbl("FORMULA:", 8, C_MUTED))
        formula_row.addWidget(self.formula_lbl)
        formula_row.addStretch()
        l.addLayout(formula_row)

        # Live reading row
        read_row = QHBoxLayout()
        read_row.addWidget(lbl("RAW READING:", 9, C_MUTED))
        self.reading_lbl = lbl("—", 20, C_ACCENT, bold=True)
        read_row.addWidget(self.reading_lbl)
        read_row.addWidget(self.unit_lbl)
        read_row.addStretch()
        l.addLayout(read_row)

        # Noise bar
        self.noise_bar = NoiseBar("Sensor noise:")
        l.addWidget(self.noise_bar)

        l.addWidget(self.hint_lbl)
        self._sensor_changed(self.combo.currentText())
        return f

    def _build_prediction_card(self):
        f = card()
        l = QVBoxLayout(f); l.setContentsMargins(16, 12, 16, 12); l.setSpacing(7)
        l.addWidget(lbl("② YOUR PREDICTION", 8, C_MUTED))
        l.addWidget(divider())
        l.addWidget(lbl("Enter the raw sensor reading you expect when the robot has moved 25 cm:", 10))

        row = QHBoxLayout()
        self.pred_entry = QLineEdit()
        self.pred_entry.setPlaceholderText("e.g.  65.0")
        self.pred_entry.setStyleSheet(f"""
            QLineEdit{{background:{C_BG};color:{C_ACCENT};border:1px solid {C_ACCENT};
                       border-radius:6px;padding:7px 14px;font-family:{FONT};
                       font-size:17px;font-weight:bold;}}
        """)
        row.addWidget(self.pred_entry, stretch=3)
        row.addWidget(self.pred_unit_lbl)
        row.addStretch()
        l.addLayout(row)
        return f

    def _build_controls_card(self):
        f = card()
        l = QVBoxLayout(f); l.setContentsMargins(16, 12, 16, 12); l.setSpacing(8)
        l.addWidget(lbl("③ CHALLENGE CONTROLS", 8, C_MUTED))
        l.addWidget(divider())

        row = QHBoxLayout()
        self.start_btn = btn("▶  START",     C_ACCENT,  "#000")
        self.stop_btn  = btn("■  STOP",      C_ACCENT2, "#fff")
        self.retry_btn = btn("↺  TRY AGAIN", C_BORDER,  C_TEXT)
        self.stop_btn.setEnabled(False)
        self.retry_btn.setEnabled(False)

        self.start_btn.clicked.connect(self._start)
        self.stop_btn.clicked.connect(self._stop)
        self.retry_btn.clicked.connect(self._retry)

        row.addWidget(self.start_btn)
        row.addWidget(self.stop_btn)
        row.addWidget(self.retry_btn)
        l.addLayout(row)

        self.status_lbl = lbl("● WAITING FOR START", 10, C_MUTED, bold=True)
        l.addWidget(self.status_lbl)
        return f

    def _build_result_card(self):
        self.res_card = card(); self.res_card.hide()
        l = QVBoxLayout(self.res_card); l.setContentsMargins(18, 12, 18, 12)
        self.res_lbl = QLabel("")
        self.res_lbl.setAlignment(Qt.AlignCenter)
        self.res_lbl.setWordWrap(True)
        self.res_lbl.setStyleSheet(
            f"color:{C_TEXT};font-family:{FONT};font-size:13px;"
            f"font-weight:bold;background:transparent;")
        l.addWidget(self.res_lbl)
        return self.res_card

    # ── Right column ───────────────────────────────────────────────────────────
    def _build_right(self):
        w = QWidget(); w.setStyleSheet("background:transparent;")
        l = QVBoxLayout(w); l.setContentsMargins(0, 0, 0, 0); l.setSpacing(12)

        # ── TOP PANEL: Robot top-view ─────────────────────────────────────
        tv_card = card()
        tvl = QVBoxLayout(tv_card); tvl.setContentsMargins(10, 8, 10, 8); tvl.setSpacing(4)
        tvl.addWidget(lbl("ARENA  —  TOP VIEW", 8, C_MUTED))
        tvl.addWidget(divider())

        self.top_view = RobotTopView()
        self.top_view.setMinimumHeight(160)
        tvl.addWidget(self.top_view, stretch=1)

        # Legend row for top view
        tv_leg = QHBoxLayout()
        for col, txt in [(C_ACCENT,  "Robot (sim)"),
                         (C_ACTUAL,  "Fusion estimate"),
                         (C_YELLOW,  "25 cm target")]:
            dot = QLabel("●")
            dot.setStyleSheet(
                f"color:{col};background:transparent;border:none;font-size:11px;")
            tv_leg.addWidget(dot)
            tv_leg.addWidget(lbl(txt, 8, C_MUTED))
            tv_leg.addSpacing(6)
        tv_leg.addStretch()
        tvl.addLayout(tv_leg)
        l.addWidget(tv_card, stretch=3)

        # Chart card — tall
        cc = card()
        cl = QVBoxLayout(cc); cl.setContentsMargins(12, 10, 12, 10); cl.setSpacing(6)
        cl.addWidget(lbl("DISTANCE COMPARISON", 8, C_MUTED))
        cl.addWidget(divider())

        self.chart = DistanceBarChart()
        cl.addWidget(self.chart, stretch=1)

        # Legend
        leg = QHBoxLayout()
        for col, txt in [(C_CURRENT, "Current (from sensor)"),
                         (C_ACTUAL,  "Actual (fusion) — revealed on STOP"),
                         (C_YELLOW,  "25 cm target")]:
            dot = QLabel("●")
            dot.setStyleSheet(
                f"color:{col};background:transparent;border:none;font-size:12px;")
            leg.addWidget(dot)
            leg.addWidget(lbl(txt, 8, C_MUTED))
            leg.addSpacing(6)
        leg.addStretch()
        cl.addLayout(leg)
        l.addWidget(cc, stretch=3)

        # Sensor report card — hidden until STOP
        l.addWidget(self._build_report_card(), stretch=0)

        # Teleop card
        l.addWidget(self._build_teleop_card(), stretch=0)

        # Session log
        l.addWidget(self._build_log_card(), stretch=2)

        return w

    def _build_report_card(self):
        self.report_card = card()
        self.report_card.hide()
        l = QVBoxLayout(self.report_card); l.setContentsMargins(16, 12, 16, 12); l.setSpacing(6)

        hrow = QHBoxLayout()
        hrow.addWidget(lbl("SENSOR REPORT", 8, C_MUTED))
        hrow.addStretch()
        self.report_sensor_lbl = lbl("", 9, C_ACCENT, bold=True)
        hrow.addWidget(self.report_sensor_lbl)
        l.addLayout(hrow)
        l.addWidget(divider())

        # Stats row
        stats = QHBoxLayout(); stats.setSpacing(20)

        acc_col = QVBoxLayout()
        acc_col.addWidget(lbl("Prediction accuracy", 8, C_MUTED))
        self.report_acc_lbl = lbl("—", 16, C_TEXT, bold=True)
        acc_col.addWidget(self.report_acc_lbl)
        stats.addLayout(acc_col)

        err_col = QVBoxLayout()
        err_col.addWidget(lbl("Distance error", 8, C_MUTED))
        self.report_err_lbl = lbl("—", 16, C_TEXT, bold=True)
        err_col.addWidget(self.report_err_lbl)
        stats.addLayout(err_col)

        noise_col = QVBoxLayout()
        noise_col.addWidget(lbl("Signal noise", 8, C_MUTED))
        self.report_noise_lbl = lbl("—", 12, C_TEXT, bold=True)
        noise_col.addWidget(self.report_noise_lbl)
        stats.addLayout(noise_col)

        stats.addStretch()
        l.addLayout(stats)

        # Limitation hint
        self.report_limit_lbl = lbl("", 9, C_YELLOW)
        self.report_limit_lbl.setWordWrap(True)
        l.addWidget(self.report_limit_lbl)

        return self.report_card

    def _build_teleop_card(self):
        f = card()
        l = QVBoxLayout(f); l.setContentsMargins(16, 10, 16, 10); l.setSpacing(6)
        l.addWidget(lbl("④ KEYBOARD TELEOP", 8, C_MUTED))
        l.addWidget(divider())

        self.kw = KeyWidget("W"); self.ka = KeyWidget("A")
        self.ks = KeyWidget("S"); self.kd = KeyWidget("D")

        kg = QGridLayout(); kg.setSpacing(4)
        kg.addWidget(self.kw, 0, 1)
        kg.addWidget(self.ka, 1, 0)
        kg.addWidget(self.ks, 1, 1)
        kg.addWidget(self.kd, 1, 2)

        row = QHBoxLayout(); row.addStretch()
        row.addLayout(kg); row.addSpacing(14)

        hints = QVBoxLayout()
        for t in ["W / ↑  Forward", "S / ↓  Backward",
                  "A / ←  Left", "D / →  Right"]:
            hints.addWidget(lbl(t, 9, C_MUTED))
        row.addLayout(hints); row.addStretch()
        l.addLayout(row)
        return f

    def _build_log_card(self):
        f = card()
        l = QVBoxLayout(f); l.setContentsMargins(16, 10, 16, 10); l.setSpacing(6)

        hrow = QHBoxLayout()
        hrow.addWidget(lbl("SESSION LOG", 8, C_MUTED))
        hrow.addStretch()
        clr = btn("clear", C_BORDER, C_MUTED, 8)
        clr.setFixedHeight(22)
        clr.clicked.connect(self._clear_log)
        hrow.addWidget(clr)
        l.addLayout(hrow)
        l.addWidget(divider())

        self.log_container = QWidget()
        self.log_container.setStyleSheet("background:transparent;")
        self.log_layout = QVBoxLayout(self.log_container)
        self.log_layout.setContentsMargins(0, 0, 0, 0)
        self.log_layout.setSpacing(3)
        self.log_layout.addStretch()

        scroll = QScrollArea()
        scroll.setWidget(self.log_container)
        scroll.setWidgetResizable(True)
        scroll.setStyleSheet(
            f"QScrollArea{{background:transparent;border:none;}}"
            f"QScrollBar:vertical{{background:{C_BORDER};width:6px;border-radius:3px;}}"
            f"QScrollBar::handle:vertical{{background:{C_MUTED};border-radius:3px;}}")
        scroll.setFixedHeight(100)
        l.addWidget(scroll)

        self.log_empty_lbl = lbl("No attempts yet — complete a challenge to see results here.",
                                 9, C_MUTED)
        self.log_empty_lbl.setAlignment(Qt.AlignCenter)
        l.addWidget(self.log_empty_lbl)

        return f

    # ── Hardware connection ────────────────────────────────────────────────────
    def _toggle_connection(self):
        self._disconnect() if self.hw_connected else self._connect()

    def _connect(self):
        try:
            from snp_model import ScalarPositionFusion, RobotModel
            self.real_robot   = RobotModel("COM7")
            if self.fusion is None and _FUSION_AVAILABLE:
                self.fusion   = ScalarPositionFusion(arena_length_cm=ARENA_CM)
            elif self.fusion is None:
                from snp_model import ScalarPositionFusion as SPF
                self.fusion   = SPF(arena_length_cm=ARENA_CM)
            self.hw_connected = True
            self.conn_badge.setText("  LIVE — COM7  ")
            self.conn_badge.setStyleSheet(
                f"background:{C_SUCCESS};color:#000;font-family:{FONT};"
                f"font-size:9px;font-weight:bold;border-radius:4px;padding:3px 8px;")
            self.conn_btn.setText("✕  Disconnect")
        except ImportError:
            QMessageBox.critical(self, "Import Error",
                "snp_model.py not found.\n"
                "Ensure snp_model.py is in the same directory.")
        except Exception as e:
            QMessageBox.critical(self, "Connection Failed",
                f"Could not connect to COM7:\n{e}")

    def _disconnect(self):
        if self.real_robot:
            try: self.real_robot.close()
            except: pass
        self.real_robot = None; self.fusion = None; self.hw_connected = False
        self.conn_badge.setText("  SIM MODE  ")
        self.conn_badge.setStyleSheet(
            f"background:{C_YELLOW};color:#000;font-family:{FONT};"
            f"font-size:9px;font-weight:bold;border-radius:4px;padding:3px 8px;")
        self.conn_btn.setText("⚡  Connect to COM7")

    # ── Sensor helpers ─────────────────────────────────────────────────────────
    def _sensor_changed(self, name):
        meta = SENSORS[name]
        self.hint_lbl.setText(f"💡  {meta['hint']}")
        self.formula_lbl.setText(meta["formula"])
        self.pred_unit_lbl.setText(meta["unit"])
        self.unit_lbl.setText(meta["unit"])
        self._noise_buf.clear()
        if hasattr(self, "top_view"):
            self.top_view.set_sensor(name)

    def _get_reading(self):
        name = self.combo.currentText()
        if self.hw_connected and self.real_robot:
            try:
                s = self.real_robot.get_sensors()
                return round(float(getattr(s, SENSORS[name]["key"], 0.0)), 2)
            except: return 0.0
        return self.sim.get_reading(name)

    def _reading_to_cm(self, reading):
        return SENSORS[self.combo.currentText()]["to_cm"](reading)

    def _get_position(self):
        if self.hw_connected and self.real_robot and self.fusion:
            try:
                s = self.real_robot.get_sensors()
                self.fusion.step(s, s.encoder_left, s.encoder_right)
                return self.fusion.position
            except: return 0.0
        # Sim mode: update sim_sensors then step ScalarPositionFusion
        self.sim_sensors.update(self.sim, self.combo.currentText())
        if self.sim_fusion is not None:
            self.sim_fusion.step(
                self.sim_sensors,
                self.sim_sensors.encoder_left,
                self.sim_sensors.encoder_right,
            )
            return self.sim_fusion.position
        return self.sim.get_position()

    # ── Poll ───────────────────────────────────────────────────────────────────
    def _poll(self):
        reading = self._get_reading()
        name    = self.combo.currentText()
        self.reading_lbl.setText(str(reading))
        self.unit_lbl.setText(SENSORS[name]["unit"])

        # Noise buffer
        self._noise_buf.append(reading)
        if len(self._noise_buf) >= 3:
            vals   = list(self._noise_buf)
            mean   = sum(vals) / len(vals)
            var    = sum((v - mean) ** 2 for v in vals) / len(vals)
            # Normalise roughly: encoder variance up to ~50, IR up to ~2, ultrasonic up to ~1
            maxvar = {"Encoder — Left Wheel": 50.0,
                      "Ultrasonic — Front":    1.0,
                      "Sharp IR — Back":       2.0}.get(name, 5.0)
            level = min(var / maxvar, 1.0)
            self.noise_bar.set_level(level)

        # Update top view regardless of running state
        sim_pos    = self.sim.get_position()
        fusion_pos = self._get_position()
        self.top_view.set_position(sim_pos, fusion_pos)

        if self.running:
            current_cm = max(self._reading_to_cm(reading), 0.0)
            actual_cm  = fusion_pos
            self.chart.set_values(current_cm, actual_cm)

    # ── Drive loop ─────────────────────────────────────────────────────────────
    def _drive_tick(self):
        fwd  = Qt.Key_W in self.keys_held or Qt.Key_Up    in self.keys_held
        back = Qt.Key_S in self.keys_held or Qt.Key_Down  in self.keys_held
        left = Qt.Key_A in self.keys_held or Qt.Key_Left  in self.keys_held
        rght = Qt.Key_D in self.keys_held or Qt.Key_Right in self.keys_held

        if fwd:    self._move( self.speed_cm)
        elif back: self._move(-self.speed_cm)
        else:
            # send zero velocity so robot actually stops
            if self.hw_connected and self.real_robot:
                try: self.real_robot.send_velocity(0.0, 0.0, 0.0)
                except: pass

        # turning
        if self.hw_connected and self.real_robot and (left or rght):
            try: self.real_robot.send_velocity(0.0, 0.0, 0.3 if left else -0.3)
            except: pass

        self.kw.set_active(fwd);  self.ks.set_active(back)
        self.ka.set_active(left); self.kd.set_active(rght)
        self._set_status("● DRIVING" if (fwd or back) else "● HOLDING",
                        C_SUCCESS   if (fwd or back) else C_YELLOW)

    def _move(self, d):
        if self.hw_connected and self.real_robot:
            try:
                vx = 0.15 if d > 0 else -0.15
                self.real_robot.send_velocity(vx, 0.0, 0.0)
            except: pass
        else:
            self.sim.move(d)

    # ── Challenge flow ─────────────────────────────────────────────────────────
    def _start(self):
        try:
            self.user_pred = float(self.pred_entry.text().strip())
        except ValueError:
            self._show_result("⚠  Please enter a valid number first.", C_YELLOW)
            return

        self.sim.reset()
        if self.fusion:
            try: self.fusion.reset()
            except: pass
        if self.sim_fusion:
            try: self.sim_fusion.reset()
            except: pass

        self.running = True
        self.chart.reveal(False)
        self.chart.set_values(0, 0)
        self.res_card.hide()
        self.report_card.hide()
        self._noise_buf.clear()

        self.start_btn.setEnabled(False)
        self.stop_btn.setEnabled(True)
        self.retry_btn.setEnabled(False)
        self.combo.setEnabled(False)
        self.pred_entry.setEnabled(False)
        self.conn_btn.setEnabled(False)

        self._set_status("● DRIVING", C_SUCCESS)
        self.drive_timer.start()
        self.setFocus()

    def _stop(self):
        if not self.running: return
        self.running = False
        self.drive_timer.stop()
        self.keys_held.clear()
        for k in (self.kw, self.ka, self.ks, self.kd): k.set_active(False)

        self.stop_btn.setEnabled(False)
        self.retry_btn.setEnabled(True)
        self.conn_btn.setEnabled(True)
        self._set_status("● STOPPED", C_MUTED)

        name      = self.combo.currentText()
        actual    = self._get_position()
        reading   = self._get_reading()
        current_cm = max(self._reading_to_cm(reading), 0.0)
        err_cm    = abs(actual - TARGET_CM)
        success   = err_cm <= TOLERANCE

        # Final chart
        self.chart.reveal(True)
        self.chart.set_values(current_cm, actual)

        # Prediction accuracy (how close was their predicted reading vs ideal)
        meta      = SENSORS[name]
        # Ideal reading at exactly 25 cm
        ideal_rdg = meta["sim_start"] + meta["sim_scale"] * TARGET_CM
        pred_err  = abs(self.user_pred - ideal_rdg)
        pred_pct  = max(0, 100 - (pred_err / max(abs(ideal_rdg), 1)) * 100)

        # Noise level from buffer
        noise_char = meta["noise_char"]
        noise_col  = NOISE_COLORS[noise_char]

        # Populate report card
        self.report_sensor_lbl.setText(name)
        self.report_acc_lbl.setText(f"{pred_pct:.0f}%")
        self.report_acc_lbl.setStyleSheet(
            f"color:{C_SUCCESS if pred_pct >= 80 else C_YELLOW if pred_pct >= 50 else C_FAIL};"
            f"font-family:{FONT};font-size:16px;font-weight:bold;background:transparent;")
        self.report_err_lbl.setText(f"{err_cm*10:.1f} mm")
        self.report_err_lbl.setStyleSheet(
            f"color:{C_SUCCESS if success else C_FAIL};"
            f"font-family:{FONT};font-size:16px;font-weight:bold;background:transparent;")
        self.report_noise_lbl.setText(noise_char.upper())
        self.report_noise_lbl.setStyleSheet(
            f"color:{noise_col};font-family:{FONT};"
            f"font-size:12px;font-weight:bold;background:transparent;")
        self.report_limit_lbl.setText(f"⚠  {meta['limitation']}")
        self.report_card.show()

        # Result banner
        if success:
            self._show_result(
                f"✓  SUCCESS!   Stopped at {actual:.2f} cm  ·  error = {err_cm*10:.1f} mm",
                C_SUCCESS)
        else:
            self._show_result(
                f"✗  NOT QUITE   Stopped at {actual:.2f} cm  ·  error = {err_cm*10:.1f} mm",
                C_FAIL)

        # Session log entry
        self._add_log_entry(name, actual, err_cm, success, pred_pct)

    def _retry(self):
        self.running = False
        self.drive_timer.stop()
        self.keys_held.clear()
        for k in (self.kw, self.ka, self.ks, self.kd): k.set_active(False)

        self.sim.reset()
        if self.fusion:
            try: self.fusion.reset()
            except: pass
        if self.sim_fusion:
            try: self.sim_fusion.reset()
            except: pass

        self.start_btn.setEnabled(True)
        self.stop_btn.setEnabled(False)
        self.retry_btn.setEnabled(False)
        self.combo.setEnabled(True)
        self.pred_entry.setEnabled(True)
        self.pred_entry.clear()
        self.conn_btn.setEnabled(True)

        self.chart.reveal(False)
        self.chart.set_values(0, 0)
        self.res_card.hide()
        self.report_card.hide()
        self._set_status("● WAITING FOR START", C_MUTED)

    # ── Session log ────────────────────────────────────────────────────────────
    def _add_log_entry(self, sensor_name, actual_cm, err_cm, success, pred_pct):
        attempt = len(self._session_log) + 1
        self._session_log.append({
            "sensor": sensor_name, "actual": actual_cm,
            "err": err_cm, "success": success, "pred_pct": pred_pct
        })

        short = sensor_name.split("—")[0].strip()
        col   = C_SUCCESS if success else C_FAIL
        mark  = "✓" if success else "✗"

        row_w = QWidget(); row_w.setStyleSheet("background:transparent;")
        row   = QHBoxLayout(row_w); row.setContentsMargins(0, 0, 0, 0); row.setSpacing(8)

        row.addWidget(lbl(f"#{attempt}", 8, C_MUTED))
        row.addWidget(lbl(short, 8, C_TEXT))
        row.addStretch()
        row.addWidget(lbl(f"{actual_cm:.1f} cm", 8, C_TEXT))
        row.addWidget(lbl(f"Δ {err_cm*10:.1f} mm", 8, C_MUTED))
        row.addWidget(lbl(f"{mark}", 11, col, bold=True))

        # Insert before the stretch at end
        self.log_layout.insertWidget(self.log_layout.count() - 1, row_w)
        self.log_empty_lbl.hide()

    def _clear_log(self):
        self._session_log.clear()
        while self.log_layout.count() > 1:
            item = self.log_layout.takeAt(0)
            if item.widget(): item.widget().deleteLater()
        self.log_empty_lbl.show()

    # ── Utilities ──────────────────────────────────────────────────────────────
    def _set_status(self, txt, color):
        self.status_lbl.setText(txt)
        self.status_lbl.setStyleSheet(
            f"color:{color};font-family:{FONT};font-size:10px;"
            f"font-weight:bold;background:transparent;border:none;")

    def _show_result(self, msg, color):
        self.res_lbl.setText(msg)
        self.res_lbl.setStyleSheet(
            f"color:{color};font-family:{FONT};font-size:13px;"
            f"font-weight:bold;background:transparent;")
        border = C_SUCCESS if color == C_SUCCESS else \
                 C_FAIL    if color == C_FAIL    else C_YELLOW
        self.res_card.setStyleSheet(
            f"QFrame{{background:{C_CARD};border:2px solid {border};border-radius:10px;}}")
        self.res_card.show()

    # ── Keyboard ───────────────────────────────────────────────────────────────
    def keyPressEvent(self, e):
        if self.running: self.keys_held.add(e.key())

    def keyReleaseEvent(self, e):
        self.keys_held.discard(e.key())

    def closeEvent(self, e):
        self.drive_timer.stop(); self.poll_timer.stop()
        self._disconnect(); e.accept()


# ══════════════════════════════════════════════════════════════════════════════
if __name__ == "__main__":
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    win = MainWindow()
    win.show()
    sys.exit(app.exec_())