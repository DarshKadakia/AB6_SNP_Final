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
# get directory of this file
_HERE = os.path.dirname(os.path.abspath(__file__))

# go one level up (SNP folder)
_ROOT = os.path.dirname(_HERE)
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

try:
    from robot_core.snp_model import (
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