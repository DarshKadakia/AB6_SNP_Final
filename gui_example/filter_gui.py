"""
Sensor Filter Explorer — PyQt6 + pyqtgraph
==========================================
Install deps:
    pip install PyQt6 pyqtgraph numpy

Run:
    python sensor_filter_explorer.py
"""

import sys, math, random, collections
import numpy as np
from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QPushButton, QComboBox, QSlider, QGroupBox, QScrollArea,
    QFrame, QGridLayout, QSizePolicy, QLineEdit, QMessageBox
)
from PyQt6.QtCore import Qt, QTimer, pyqtSignal
from PyQt6.QtGui import QFont, QColor, QPalette, QLinearGradient
import pyqtgraph as pg

# Try to import RobotModel — gracefully falls back to simulation if not found
try:
    from snp_model import RobotModel, Sensors
    HAS_ROBOT = True
    sensor = Sensors()
except ImportError:
    HAS_ROBOT = False

# Maps GUI sensor name → (use_raw, attribute_on_sensor_object)
# use_raw=True  → reads from get_sensorRaw()
# use_raw=False → reads from get_sensor()
SENSOR_ATTR_MAP = {
    "Sharp IR":"sharp_ir_distance",
    "Ultrasonic (Water)":"waterPultrasonic",
    "Ultrasonic (Air)":"ultrasonic",
    "Encoder (Left)":"encoder_left",
    "Encoder (Right)":"encoder_right",
    "IMU Gyro X":         "gyro_x",
    "IMU Accel Y":"accel_y",
    "IMU Roll":"roll",
    "IMU Pitch":"pitch",
    "IMU Yaw":"yaw",
}

# ─────────────────────────────────────────────────────────────────────────────
#  SENSOR PROFILES
# ─────────────────────────────────────────────────────────────────────────────
SENSORS = {
    "Sharp IR": dict(
        unit="cm", true_value=40.0, noise_type="spiky", noise_amp=8.0,
        color="#f59e0b",
        desc="Non-linear voltage-distance curve. Susceptible to spikes & reflections."
    ),
    "Ultrasonic (Water)": dict(
        unit="cm", true_value=55.0, noise_type="gaussian", noise_amp=5.0,
        color="#38bdf8",
        desc="Water surface causes multi-path echoes & moderate Gaussian noise."
    ),
    "Ultrasonic (Air)": dict(
        unit="cm", true_value=50.0, noise_type="gaussian", noise_amp=3.0,
        color="#818cf8",
        desc="Temperature-sensitive speed of sound; moderate Gaussian noise."
    ),
    "Encoder (Left)": dict(
        unit="ticks", true_value=100.0, noise_type="quantized", noise_amp=2.0,
        color="#34d399",
        desc="Quantization & mechanical vibration cause jitter in tick counts."
    ),
    "Encoder (Right)": dict(
        unit="ticks", true_value=100.0, noise_type="quantized", noise_amp=2.0,
        color="#a3e635",
        desc="Quantization & mechanical vibration cause jitter in tick counts."
    ),
    "IMU Gyro X": dict(
        unit="°/s", true_value=0.0, noise_type="drift", noise_amp=0.8,
        color="#f472b6",
        desc="Gyroscope suffers from integration drift & white noise bias."
    ),
    "IMU Accel Y": dict(
        unit="m/s²", true_value=9.81, noise_type="gaussian", noise_amp=0.4,
        color="#fb923c",
        desc="Accelerometer picks up vibration; high-frequency noise dominant."
    ),
    "IMU Roll": dict(
        unit="°", true_value=5.0, noise_type="drift", noise_amp=1.2,
        color="#e879f9",
        desc="Computed angle drifts over time; requires fusion with accelerometer."
    ),
    "IMU Pitch": dict(
        unit="°", true_value=2.0, noise_type="drift", noise_amp=1.0,
        color="#c084fc",
        desc="Same characteristics as Roll — complementary filter helps a lot."
    ),
    "IMU Yaw": dict(
        unit="°", true_value=90.0, noise_type="drift", noise_amp=2.0,
        color="#2dd4bf",
        desc="Yaw is hardest — no gravity reference. Magnetometer fusion needed."
    ),
}

# ─────────────────────────────────────────────────────────────────────────────
#  NOISE GENERATOR
# ─────────────────────────────────────────────────────────────────────────────
def generate_raw(sensor, t):
    v = sensor["true_value"]
    a = sensor["noise_amp"]
    nt = sensor["noise_type"]
    r = lambda: (random.random() - 0.5) * 2
    if nt == "spiky":
        return v + r()*a + (r()*a*5 if random.random() < 0.08 else 0)
    elif nt == "gaussian":
        return v + r()*a
    elif nt == "quantized":
        return v + round(r()*a)
    elif nt == "drift":
        return v + r()*a + math.sin(t * 0.05) * a * 1.5
    return v + r()*a

# ─────────────────────────────────────────────────────────────────────────────
#  FILTER DEFINITIONS
# ─────────────────────────────────────────────────────────────────────────────
class FilterBase:
    name = ""
    full_name = ""
    description = ""
    color = "#ffffff"
    param_defs = []  # list of dicts: key, label, min, max, step, default

    def reset(self): pass
    def apply(self, val, params): return val


class SMAFilter(FilterBase):
    name = "SMA"
    full_name = "Simple Moving Average"
    description = ("Averages the last N samples. Simple and effective for removing "
                   "random noise. Larger window = smoother but more lag.")
    color = "#22d3ee"
    param_defs = [dict(key="window", label="Window Size", min=2, max=50, step=1, default=10)]

    def reset(self):
        self._buf = collections.deque()

    def apply(self, val, params):
        return sensor.SMA("sma", val, int(params["window"]))


class EMAFilter(FilterBase):
    name = "EMA"
    full_name = "Exponential Moving Average"
    description = ("Weights recent samples more. Alpha closer to 1 = more responsive; "
                   "closer to 0 = smoother.")
    color = "#a78bfa"
    param_defs = [dict(key="alpha", label="Alpha (α)", min=0.01, max=1.0, step=0.01, default=0.2)]

    def reset(self):
        self._prev = None

    def apply(self, val, params):
        return sensor.EMA("ema", val, params["alpha"])

class MedianFilter(FilterBase):
    name = "Median"
    full_name = "Median Filter"
    description = ("Returns the median of the window. Excellent at eliminating "
                   "spikes (outliers) while preserving edges.")
    color = "#86efac"
    param_defs = [dict(key="window", label="Window Size", min=3, max=31, step=2, default=7)]

    def reset(self):
        self._buf = collections.deque()

    def apply(self, val, params):
        return sensor.MEDIAN("median", val, int(params["window"]))


class LowPassFilter(FilterBase):
    name = "Low-Pass"
    full_name = "RC Low-Pass Filter"
    description = ("Time-domain RC filter. Tau (τ) is the time constant — larger τ = "
                   "more smoothing. dt is your loop period.")
    color = "#fbbf24"
    param_defs = [
        dict(key="tau", label="Tau τ (s)", min=0.01, max=2.0, step=0.01, default=0.3),
        dict(key="dt",  label="dt (s)",    min=0.01, max=0.5, step=0.01, default=0.05),
    ]

    def reset(self):
        self._prev = None

    def apply(self, val, params):
        return sensor.low_pass("lowpass", val, params["tau"], params["dt"])


class KalmanFilter(FilterBase):
    name = "Kalman"
    full_name = "1D Kalman Filter"
    description = ("Optimal estimator that balances process uncertainty (Q) vs measurement "
                   "uncertainty (R). Q↑ = trust sensor more; R↑ = trust model more.")
    color = "#f472b6"
    param_defs = [
        dict(key="Q", label="Process Noise Q",  min=0.0001, max=1.0,  step=0.001, default=0.01),
        dict(key="R", label="Meas. Noise R",    min=0.01,   max=10.0, step=0.01,  default=0.5),
    ]

    def reset(self):
        self._x = None
        self._P = 1.0

    def apply(self, val, params):
        if self._x is None:
            self._x = val
            return val
        self._P += params["Q"]
        K = self._P / (self._P + params["R"])
        self._x += K * (val - self._x)
        self._P = (1 - K) * self._P
        return self._x


class ComplementaryFilter(FilterBase):
    name = "Complementary"
    full_name = "Complementary Filter"
    description = ("Fuses two signals: slow (accel/position) and fast (gyro/derivative). "
                   "Alpha weights the high-frequency source.")
    color = "#fb923c"
    param_defs = [
        dict(key="alpha", label="Alpha (α)", min=0.01, max=0.99, step=0.01, default=0.96),
        dict(key="dt",    label="dt (s)",    min=0.01, max=0.2,  step=0.01, default=0.05),
    ]

    def reset(self):
        self._angle = None

    def apply(self, val, params):
        gyro_rate = (random.random() - 0.5) * 2   # simulated gyro
        if self._angle is None:
            self._angle = val
            return val
        gyro_angle = self._angle + gyro_rate * params["dt"]
        self._angle = params["alpha"] * gyro_angle + (1 - params["alpha"]) * val
        return self._angle


FILTER_CLASSES = [SMAFilter, EMAFilter, MedianFilter, LowPassFilter, KalmanFilter, ComplementaryFilter]
FILTERS = {cls.name: cls for cls in FILTER_CLASSES}

# ─────────────────────────────────────────────────────────────────────────────
#  PARAM SLIDER ROW
# ─────────────────────────────────────────────────────────────────────────────
class ParamSlider(QWidget):
    value_changed = pyqtSignal(str, float)

    def __init__(self, pdef, accent_color, parent=None):
        super().__init__(parent)
        self.pdef = pdef
        self.key = pdef["key"]
        self._scale = 1000
        self._min = pdef["min"]
        self._max = pdef["max"]

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 4, 0, 4)
        layout.setSpacing(4)

        # Label row
        row = QHBoxLayout()
        self.name_lbl = QLabel(pdef["label"])
        self.name_lbl.setStyleSheet("color: #94a3b8; font-size: 11px;")
        self.val_lbl = QLabel(self._fmt(pdef["default"]))
        self.val_lbl.setStyleSheet(
            f"color: {accent_color}; font-weight: bold; font-size: 12px;"
            f"background: {accent_color}22; padding: 1px 8px; border-radius: 4px;"
        )
        row.addWidget(self.name_lbl)
        row.addStretch()
        row.addWidget(self.val_lbl)
        layout.addLayout(row)

        # Slider
        self.slider = QSlider(Qt.Orientation.Horizontal)
        self.slider.setMinimum(0)
        self.slider.setMaximum(self._scale)
        self.slider.setValue(self._to_slider(pdef["default"]))
        self.slider.setStyleSheet(f"""
            QSlider::groove:horizontal {{
                height: 4px; background: #1e2d4a; border-radius: 2px;
            }}
            QSlider::handle:horizontal {{
                background: {accent_color}; width: 14px; height: 14px;
                margin: -5px 0; border-radius: 7px;
            }}
            QSlider::sub-page:horizontal {{
                background: {accent_color}88; border-radius: 2px;
            }}
        """)
        self.slider.valueChanged.connect(self._on_change)
        layout.addWidget(self.slider)

        # Min/max labels
        mm = QHBoxLayout()
        lmin = QLabel(str(pdef["min"])); lmin.setStyleSheet("color: #334155; font-size: 9px;")
        lmax = QLabel(str(pdef["max"])); lmax.setStyleSheet("color: #334155; font-size: 9px;")
        mm.addWidget(lmin); mm.addStretch(); mm.addWidget(lmax)
        layout.addLayout(mm)

    def _to_slider(self, v):
        r = (v - self._min) / (self._max - self._min)
        return int(r * self._scale)

    def _from_slider(self, s):
        return self._min + (s / self._scale) * (self._max - self._min)

    def _fmt(self, v):
        step = self.pdef["step"]
        if step < 0.01: return f"{v:.4f}"
        if step < 0.1:  return f"{v:.3f}"
        if step < 1:    return f"{v:.2f}"
        return f"{int(v)}"

    def _on_change(self, s):
        v = self._from_slider(s)
        self.val_lbl.setText(self._fmt(v))
        self.value_changed.emit(self.key, v)

    def get_value(self):
        return self._from_slider(self.slider.value())

    def reset_to_default(self):
        self.slider.setValue(self._to_slider(self.pdef["default"]))

# ─────────────────────────────────────────────────────────────────────────────
#  MAIN WINDOW
# ─────────────────────────────────────────────────────────────────────────────
WINDOW = 200

class SensorFilterExplorer(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Sensor Filter Explorer")
        self.setMinimumSize(1200, 720)
        self._apply_dark_theme()

        # State
        self._tick = 0
        self._paused = False
        self._raw_buf = collections.deque(maxlen=WINDOW)
        self._filt_buf = collections.deque(maxlen=WINDOW)
        self._true_buf = collections.deque(maxlen=WINDOW)
        self._filter_instance = None
        self._params = {}

        # Robot / serial state
        self._robot = None          # RobotModel instance when connected
        self._live_mode = False     # True = read from robot, False = simulation

        self._build_ui()
        # Must be called AFTER _build_ui so all widgets exist
        self.sensor_combo.blockSignals(True)
        self.filter_combo.blockSignals(True)
        self.sensor_combo.setCurrentIndex(0)
        self.filter_combo.setCurrentIndex(0)
        self.sensor_combo.blockSignals(False)
        self.filter_combo.blockSignals(False)
        self._select_sensor(list(SENSORS.keys())[0])
        self._select_filter(list(FILTERS.keys())[0])

        self._timer = QTimer()
        self._timer.timeout.connect(self._tick_fn)
        self._timer.start(60)

    # ── Theme ────────────────────────────────────────────────────────────────
    def _apply_dark_theme(self):
        pg.setConfigOption("background", "#0a0e1a")
        pg.setConfigOption("foreground", "#475569")
        self.setStyleSheet("""
            QMainWindow, QWidget { background: #0a0e1a; color: #e2e8f0; }
            QLabel { color: #e2e8f0; }
            QComboBox {
                background: #0f172a; border: 1px solid #1e3a5f; color: #e2e8f0;
                padding: 6px 10px; border-radius: 6px; font-size: 12px;
            }
            QComboBox::drop-down { border: none; }
            QComboBox QAbstractItemView {
                background: #0f172a; border: 1px solid #1e3a5f; color: #e2e8f0;
                selection-background-color: #1e3a5f;
            }
            QGroupBox {
                border: 1px solid #1e2d4a; border-radius: 8px;
                margin-top: 12px; padding: 12px;
                font-size: 10px; color: #64748b; letter-spacing: 2px;
            }
            QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 6px; }
            QPushButton {
                border-radius: 6px; padding: 7px 18px;
                font-size: 11px; font-weight: bold;
            }
            QScrollBar:vertical { background: #0a0e1a; width: 6px; }
            QScrollBar::handle:vertical { background: #1e2d4a; border-radius: 3px; }
        """)

    # ── UI Layout ─────────────────────────────────────────────────────────────
    def _build_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(0)

        # ── Top bar
        topbar = QWidget()
        topbar.setFixedHeight(56)
        topbar.setStyleSheet("background: #0d1525; border-bottom: 1px solid #1e2d4a;")
        tb = QHBoxLayout(topbar)
        tb.setContentsMargins(20, 0, 20, 0)

        icon = QLabel("⚡")
        icon.setStyleSheet("font-size: 22px;")
        title1 = QLabel("SENSOR FILTER")
        title1.setStyleSheet("font-size: 14px; font-weight: 800; color: #93c5fd; letter-spacing: 3px;")
        title2 = QLabel("EXPLORER v1.0")
        title2.setStyleSheet("font-size: 9px; color: #334155; letter-spacing: 4px;")
        tstack = QVBoxLayout(); tstack.setSpacing(0)
        tstack.addWidget(title1); tstack.addWidget(title2)

        tb.addWidget(icon)
        tb.addLayout(tstack)
        tb.addStretch()

        self.pause_btn = QPushButton("⏸  PAUSE")
        self.pause_btn.setStyleSheet("background: rgba(239,68,68,0.12); border: 1px solid #ef4444; color: #ef4444;")
        self.pause_btn.clicked.connect(self._toggle_pause)

        self.reset_btn = QPushButton("↺  RESET")
        self.reset_btn.setStyleSheet("background: rgba(59,130,246,0.1); border: 1px solid #3b82f6; color: #93c5fd;")
        self.reset_btn.clicked.connect(self._reset)

        tb.addWidget(self.pause_btn)
        tb.addSpacing(8)
        tb.addWidget(self.reset_btn)

        # ── Serial connect bar ──────────────────────────────────────────────
        tb.addSpacing(20)
        sep2 = QFrame(); sep2.setFrameShape(QFrame.Shape.VLine)
        sep2.setStyleSheet("color: #1e2d4a;"); tb.addWidget(sep2)
        tb.addSpacing(12)

        port_lbl = QLabel("PORT:")
        port_lbl.setStyleSheet("color: #475569; font-size: 10px; letter-spacing: 1px;")
        tb.addWidget(port_lbl)

        self.port_input = QLineEdit("COM7")
        self.port_input.setFixedWidth(80)
        self.port_input.setStyleSheet("""
            background: #0f172a; border: 1px solid #1e3a5f; color: #e2e8f0;
            padding: 4px 8px; border-radius: 5px; font-size: 11px;
        """)
        tb.addWidget(self.port_input)

        self.connect_btn = QPushButton("⚡ CONNECT")
        self.connect_btn.setStyleSheet(
            "background: rgba(34,197,94,0.12); border: 1px solid #22c55e; color: #22c55e;"
        )
        self.connect_btn.clicked.connect(self._toggle_connect)
        tb.addWidget(self.connect_btn)

        self.conn_status = QLabel("● SIM")
        self.conn_status.setStyleSheet("color: #f59e0b; font-size: 11px; font-weight: bold;")
        tb.addWidget(self.conn_status)
        root.addWidget(topbar)

        # ── Body
        body = QHBoxLayout()
        body.setContentsMargins(0, 0, 0, 0)
        body.setSpacing(0)
        root.addLayout(body, stretch=1)

        # ── Left sidebar
        sidebar = QWidget()
        sidebar.setFixedWidth(220)
        sidebar.setStyleSheet("border-right: 1px solid #1e2d4a;")
        sb_layout = QVBoxLayout(sidebar)
        sb_layout.setContentsMargins(12, 16, 12, 16)
        sb_layout.setSpacing(20)

        # Sensor combo
        s_grp = QGroupBox("SELECT SENSOR")
        s_grp_l = QVBoxLayout(s_grp)
        self.sensor_combo = QComboBox()
        for name in SENSORS:
            self.sensor_combo.addItem(name)
        self.sensor_combo.currentTextChanged.connect(self._select_sensor)
        s_grp_l.addWidget(self.sensor_combo)
        sb_layout.addWidget(s_grp)

        # Filter combo
        f_grp = QGroupBox("SELECT FILTER")
        f_grp_l = QVBoxLayout(f_grp)
        self.filter_combo = QComboBox()
        for key, cls in FILTERS.items():
            self.filter_combo.addItem(f"{cls.name}  —  {cls.full_name}", key)
        self.filter_combo.currentIndexChanged.connect(
            lambda _: self._select_filter(self.filter_combo.currentData())
        )
        f_grp_l.addWidget(self.filter_combo)
        sb_layout.addWidget(f_grp)
        sb_layout.addStretch()
        body.addWidget(sidebar)

        # ── Right main panel
        main = QWidget()
        main_l = QVBoxLayout(main)
        main_l.setContentsMargins(20, 16, 20, 16)
        main_l.setSpacing(12)
        body.addWidget(main, stretch=1)

        # Info bar
        self.info_bar = self._make_info_bar()
        main_l.addWidget(self.info_bar)

        # Chart
        self.plot_widget = pg.PlotWidget()
        self.plot_widget.setMinimumHeight(300)
        self.plot_widget.showGrid(x=True, y=True, alpha=0.15)
        self.plot_widget.getAxis("left").setStyle(tickFont=QFont("monospace", 9))
        self.plot_widget.getAxis("bottom").setStyle(tickFont=QFont("monospace", 9))
        self.plot_widget.addLegend(offset=(10, 10))

        self.raw_curve  = self.plot_widget.plot(pen=pg.mkPen("#ef4444", width=1), name="Raw Signal")
        self.true_curve = self.plot_widget.plot(pen=pg.mkPen("#475569", width=1.5, style=Qt.PenStyle.DashLine), name="True Value")
        self.filt_curve = self.plot_widget.plot(pen=pg.mkPen("#22d3ee", width=2.5), name="Filtered")

        main_l.addWidget(self.plot_widget, stretch=1)

        # Bottom row: params + metrics
        bottom = QHBoxLayout()
        bottom.setSpacing(16)

        # Param panel
        self.param_group = QGroupBox("⚙  FILTER PARAMETERS")
        self._param_layout = QVBoxLayout(self.param_group)
        self._param_layout.setSpacing(2)
        self._slider_widgets = []
        bottom.addWidget(self.param_group, stretch=1)

        # Metrics panel
        self.metrics_group = QGroupBox("📊  PERFORMANCE METRICS")
        self.metrics_group.setFixedWidth(280)
        mg_l = QVBoxLayout(self.metrics_group)
        mg_l.setSpacing(8)

        def mk_metric(label):
            row = QHBoxLayout()
            lbl = QLabel(label); lbl.setStyleSheet("color: #64748b; font-size: 11px;")
            val = QLabel("—"); val.setStyleSheet("font-size: 12px; font-weight: bold; color: #e2e8f0;")
            row.addWidget(lbl); row.addStretch(); row.addWidget(val)
            mg_l.addLayout(row)
            return val

        self.raw_rmse_lbl  = mk_metric("Raw RMSE")
        self.filt_rmse_lbl = mk_metric("Filtered RMSE")

        sep = QFrame(); sep.setFrameShape(QFrame.Shape.HLine)
        sep.setStyleSheet("color: #1e2d4a;")
        mg_l.addWidget(sep)

        self.improve_lbl = QLabel("—")
        self.improve_lbl.setStyleSheet("font-size: 20px; font-weight: 800; color: #22c55e; qproperty-alignment: AlignCenter;")
        mg_l.addWidget(self.improve_lbl)

        self.progress_bar_frame = QFrame()
        self.progress_bar_frame.setFixedHeight(8)
        self.progress_bar_frame.setStyleSheet("background: #1e2d4a; border-radius: 4px;")
        pb_outer = QHBoxLayout(self.progress_bar_frame); pb_outer.setContentsMargins(0,0,0,0)
        self.progress_fill = QWidget()
        self.progress_fill.setFixedHeight(8)
        self.progress_fill.setStyleSheet("background: #22c55e; border-radius: 4px;")
        pb_outer.addWidget(self.progress_fill); pb_outer.addStretch()
        mg_l.addWidget(self.progress_bar_frame)

        self.true_val_lbl = QLabel("")
        self.true_val_lbl.setStyleSheet("font-size: 10px; color: #334155;")
        mg_l.addWidget(self.true_val_lbl)
        mg_l.addStretch()

        bottom.addWidget(self.metrics_group)
        main_l.addLayout(bottom)

        # Status bar
        self.status_lbl = QLabel("LIVE · 0 samples")
        self.status_lbl.setStyleSheet("color: #334155; font-size: 10px; letter-spacing: 2px; padding: 4px 0;")
        main_l.addWidget(self.status_lbl)

    def _make_info_bar(self):
        w = QWidget()
        w.setStyleSheet("background: #0d1525; border-radius: 8px; padding: 4px;")
        h = QHBoxLayout(w)
        h.setContentsMargins(16, 10, 16, 10)

        self.sensor_name_lbl = QLabel("")
        self.sensor_name_lbl.setStyleSheet("font-size: 15px; font-weight: 800; color: #f1f5f9;")
        self.sensor_unit_lbl = QLabel("")
        self.sensor_unit_lbl.setStyleSheet(
            "font-size: 10px; padding: 2px 8px; border-radius: 10px; background: #f59e0b22; color: #f59e0b;"
        )
        self.sensor_desc_lbl = QLabel("")
        self.sensor_desc_lbl.setStyleSheet("font-size: 11px; color: #64748b;")
        self.sensor_desc_lbl.setWordWrap(True)

        left = QVBoxLayout(); left.setSpacing(3)
        row1 = QHBoxLayout(); row1.setSpacing(8)
        row1.addWidget(self.sensor_name_lbl)
        row1.addWidget(self.sensor_unit_lbl)
        row1.addStretch()
        left.addLayout(row1)
        left.addWidget(self.sensor_desc_lbl)

        self.filter_name_lbl = QLabel("")
        self.filter_name_lbl.setStyleSheet("font-size: 13px; font-weight: 700; color: #f1f5f9;")
        self.filter_desc_lbl = QLabel("")
        self.filter_desc_lbl.setStyleSheet("font-size: 11px; color: #64748b;")
        self.filter_desc_lbl.setWordWrap(True)
        right = QVBoxLayout(); right.setSpacing(3)
        right.addWidget(self.filter_name_lbl)
        right.addWidget(self.filter_desc_lbl)

        h.addLayout(left, stretch=1)
        div = QFrame(); div.setFrameShape(QFrame.Shape.VLine)
        div.setStyleSheet("color: #1e2d4a;"); h.addWidget(div)
        h.addLayout(right, stretch=1)
        return w

    # ── Sensor / Filter selection ─────────────────────────────────────────────
    def _select_sensor(self, name):
        self._current_sensor_name = name
        s = SENSORS[name]
        self.sensor_name_lbl.setText(name)
        self.sensor_unit_lbl.setText(s["unit"])
        self.sensor_unit_lbl.setStyleSheet(
            f"font-size: 10px; padding: 2px 8px; border-radius: 10px;"
            f"background: {s['color']}22; color: {s['color']};"
        )
        self.sensor_desc_lbl.setText(s["desc"])
        self.true_val_lbl.setText(f"True value: {s['true_value']} {s['unit']}")
        self._reset()

    def _select_filter(self, key):
        if key is None:
            return
        self._current_filter_key = key
        cls = FILTERS[key]
        self.filter_name_lbl.setText(cls.full_name)
        self.filter_desc_lbl.setText(cls.description)

        # Rebuild filter instance
        self._filter_instance = cls()
        self._filter_instance.reset()

        # Rebuild param sliders
        for w in self._slider_widgets:
            w.setParent(None)
        self._slider_widgets.clear()

        self._params = {p["key"]: p["default"] for p in cls.param_defs}
        for pdef in cls.param_defs:
            slider = ParamSlider(pdef, cls.color)
            slider.value_changed.connect(self._on_param_change)
            self._param_layout.addWidget(slider)
            self._slider_widgets.append(slider)

        # Update filter curve color
        self.filt_curve.setPen(pg.mkPen(cls.color, width=2.5))
        self.filt_curve.setData(name=f"Filtered ({cls.name})")

        self._reset_buffers()

    def _on_param_change(self, key, value):
        self._params[key] = value

    # ── Connect / Disconnect ──────────────────────────────────────────────────
    def _toggle_connect(self):
        if self._live_mode:
            # Disconnect
            if self._robot:
                try:
                    self._robot.close()
                except: pass
                self._robot = None
            self._live_mode = False
            self.connect_btn.setText("⚡ CONNECT")
            self.connect_btn.setStyleSheet(
                "background: rgba(34,197,94,0.12); border: 1px solid #22c55e; color: #22c55e;"
            )
            self.conn_status.setText("● SIM")
            self.conn_status.setStyleSheet("color: #f59e0b; font-size: 11px; font-weight: bold;")
            self.port_input.setEnabled(True)
        else:
            # Connect
            if not HAS_ROBOT:
                QMessageBox.warning(self, "Import Error",
                    "snp_model.py not found.\nPlace it in the same folder as this script.")
                return
            port = self.port_input.text().strip()
            try:
                self._robot = RobotModel(port)
                self._live_mode = True
                self.connect_btn.setText("✕ DISCONNECT")
                self.connect_btn.setStyleSheet(
                    "background: rgba(239,68,68,0.12); border: 1px solid #ef4444; color: #ef4444;"
                )
                self.conn_status.setText(f"● LIVE {port}")
                self.conn_status.setStyleSheet("color: #22c55e; font-size: 11px; font-weight: bold;")
                self.port_input.setEnabled(False)
                self._reset()
            except Exception as e:
                QMessageBox.critical(self, "Connection Failed", str(e))

    def _read_live_sensor(self):
        sensor_name = self._current_sensor_name
        # The map now directly returns the attribute string
        attr = SENSOR_ATTR_MAP.get(sensor_name)

        if attr is None or self._robot is None:
            return None

        try:
            # Get the latest sensor snapshot from the RobotModel
            sensors = self._robot.get_sensors()
            # Dynamically get the value (e.g., sensors.sharp_ir_distance)
            val = getattr(sensors, attr)
            return float(val)
        except Exception as e:
            print(f"Error reading live sensor: {e}")
            return None
        
    def _toggle_pause(self):
        self._paused = not self._paused
        if self._paused:
            self.pause_btn.setText("▶  RESUME")
            self.pause_btn.setStyleSheet("background: rgba(34,197,94,0.12); border: 1px solid #22c55e; color: #22c55e;")
        else:
            self.pause_btn.setText("⏸  PAUSE")
            self.pause_btn.setStyleSheet("background: rgba(239,68,68,0.12); border: 1px solid #ef4444; color: #ef4444;")

    def _reset(self):
        if self._filter_instance:
            self._filter_instance.reset()
        self._reset_buffers()

    def _reset_buffers(self):
        self._raw_buf.clear()
        self._filt_buf.clear()
        self._true_buf.clear()
        self._tick = 0
        self.raw_curve.setData([])
        self.filt_curve.setData([])
        self.true_curve.setData([])

    # ── Tick ──────────────────────────────────────────────────────────────────
    def _tick_fn(self):
        if self._paused:
            return

        raw = None
        sensor = SENSORS[self._current_sensor_name]

        # ── Live robot data or simulation ─────────────────────────────────
        if self._live_mode:
            raw = self._read_live_sensor()
            if raw is None:
                return  # no data yet, skip tick
            # In live mode the "true value" line is hidden (unknown ground truth)
            # Show a rolling mean as a reference instead
            true_val = float(np.mean(self._raw_buf)) if self._raw_buf else raw
        # else:
        #     raw = generate_raw(sensor, self._tick)
        #     true_val = sensor["true_value"]

        if raw is None:
            return

        filtered = self._filter_instance.apply(raw, self._params) if self._filter_instance else raw

        self._raw_buf.append(raw)
        self._filt_buf.append(filtered)
        self._true_buf.append(true_val)
        self._tick += 1

        xs = list(range(len(self._raw_buf)))
        self.raw_curve.setData(xs, list(self._raw_buf))
        self.filt_curve.setData(xs, list(self._filt_buf))
        self.true_curve.setData(xs, list(self._true_buf))

        mode_tag = "LIVE" if self._live_mode else "SIM"
        self._update_metrics(sensor)
        self.status_lbl.setText(f"{mode_tag} · {self._tick} samples · {sensor['unit']}")

    # ── Metrics ───────────────────────────────────────────────────────────────
    def _update_metrics(self, sensor):
        if len(self._raw_buf) < 5:
            return
        tv = sensor["true_value"]
        raw_arr = np.array(self._raw_buf)
        filt_arr = np.array(self._filt_buf)

        raw_rmse  = math.sqrt(np.mean((raw_arr - tv) ** 2))
        filt_rmse = math.sqrt(np.mean((filt_arr - tv) ** 2))
        improvement = ((raw_rmse - filt_rmse) / raw_rmse * 100) if raw_rmse > 0 else 0

        unit = sensor["unit"]
        cls = FILTERS[self._current_filter_key]

        self.raw_rmse_lbl.setText(f"±{raw_rmse:.3f} {unit}")
        self.raw_rmse_lbl.setStyleSheet("font-size: 12px; font-weight: bold; color: #ef4444;")

        self.filt_rmse_lbl.setText(f"±{filt_rmse:.3f} {unit}")
        self.filt_rmse_lbl.setStyleSheet(f"font-size: 12px; font-weight: bold; color: {cls.color};")

        arrow = "▼" if improvement > 0 else "▲"
        color = "#22c55e" if improvement > 0 else "#ef4444"
        self.improve_lbl.setText(f"{arrow} {abs(improvement):.1f}% noise reduction")
        self.improve_lbl.setStyleSheet(
            f"font-size: 13px; font-weight: 800; color: {color}; qproperty-alignment: AlignCenter;"
        )

        pct = max(0, min(100, improvement))
        total_w = self.progress_bar_frame.width()
        self.progress_fill.setFixedWidth(int(total_w * pct / 100))
        self.progress_fill.setStyleSheet(f"background: {color}; border-radius: 4px;")


    def closeEvent(self, event):
        if self._robot:
            try: self._robot.close()
            except: pass
        event.accept()

# ─────────────────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    app = QApplication(sys.argv)
    app.setFont(QFont("JetBrains Mono, Fira Code, Courier New", 10))
    window = SensorFilterExplorer()
    window.show()
    sys.exit(app.exec())