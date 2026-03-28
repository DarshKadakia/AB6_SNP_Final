import { useState, useEffect, useRef, useCallback } from "react";
import { LineChart, Line, XAxis, YAxis, CartesianGrid, Tooltip, Legend, ResponsiveContainer } from "recharts";

// ─── Sensor noise profiles ───────────────────────────────────────────────────
const SENSORS = {
  "Sharp IR": {
    unit: "cm",
    trueValue: 40,
    noiseType: "spiky",
    noiseAmplitude: 8,
    description: "Non-linear voltage-distance curve. Susceptible to spikes & reflections.",
    color: "#f59e0b",
  },
  "Ultrasonic (Water)": {
    unit: "cm",
    trueValue: 55,
    noiseType: "gaussian",
    noiseAmplitude: 5,
    description: "Water surface causes multi-path echoes & moderate Gaussian noise.",
    color: "#38bdf8",
  },
  "Ultrasonic (Air)": {
    unit: "cm",
    trueValue: 50,
    noiseType: "gaussian",
    noiseAmplitude: 3,
    description: "Temperature-sensitive speed of sound; moderate Gaussian noise.",
    color: "#818cf8",
  },
  "Encoder (Left)": {
    unit: "ticks",
    trueValue: 100,
    noiseType: "quantized",
    noiseAmplitude: 2,
    description: "Quantization & mechanical vibration cause jitter in tick counts.",
    color: "#34d399",
  },
  "Encoder (Right)": {
    unit: "ticks",
    trueValue: 100,
    noiseType: "quantized",
    noiseAmplitude: 2,
    description: "Quantization & mechanical vibration cause jitter in tick counts.",
    color: "#a3e635",
  },
  "IMU Gyro X": {
    unit: "°/s",
    trueValue: 0,
    noiseType: "drift",
    noiseAmplitude: 0.8,
    description: "Gyroscope suffers from integration drift & white noise bias.",
    color: "#f472b6",
  },
  "IMU Accel Y": {
    unit: "m/s²",
    trueValue: 9.81,
    noiseType: "gaussian",
    noiseAmplitude: 0.4,
    description: "Accelerometer picks up vibration; high-frequency noise dominant.",
    color: "#fb923c",
  },
  "IMU Roll": {
    unit: "°",
    trueValue: 5,
    noiseType: "drift",
    noiseAmplitude: 1.2,
    description: "Computed angle drifts over time; requires fusion with accelerometer.",
    color: "#e879f9",
  },
  "IMU Pitch": {
    unit: "°",
    trueValue: 2,
    noiseType: "drift",
    noiseAmplitude: 1.0,
    description: "Same characteristics as Roll — complementary filter helps a lot.",
    color: "#c084fc",
  },
  "IMU Yaw": {
    unit: "°",
    trueValue: 90,
    noiseType: "drift",
    noiseAmplitude: 2.0,
    description: "Yaw is hardest — no gravity reference. Magnetometer fusion needed.",
    color: "#2dd4bf",
  },
};

// ─── Noise generators ────────────────────────────────────────────────────────
function generateRaw(sensor, t) {
  const { trueValue, noiseType, noiseAmplitude } = sensor;
  const r = () => (Math.random() - 0.5) * 2;
  switch (noiseType) {
    case "spiky":
      return trueValue + r() * noiseAmplitude + (Math.random() < 0.08 ? r() * noiseAmplitude * 5 : 0);
    case "gaussian":
      return trueValue + r() * noiseAmplitude;
    case "quantized":
      return trueValue + Math.round(r() * noiseAmplitude);
    case "drift":
      return trueValue + r() * noiseAmplitude + Math.sin(t * 0.05) * noiseAmplitude * 1.5;
    default:
      return trueValue + r() * noiseAmplitude;
  }
}

// ─── Filter implementations ──────────────────────────────────────────────────
const FILTERS = {
  SMA: {
    label: "SMA",
    fullName: "Simple Moving Average",
    description: "Averages the last N samples. Simple and effective for removing random noise. Larger window = smoother but more lag.",
    params: [{ key: "window", label: "Window Size", min: 2, max: 50, step: 1, default: 10 }],
    color: "#22d3ee",
    create() {
      const buf = [];
      return (val, p) => {
        buf.push(val);
        if (buf.length > p.window) buf.shift();
        return buf.reduce((a, b) => a + b, 0) / buf.length;
      };
    },
  },
  EMA: {
    label: "EMA",
    fullName: "Exponential Moving Average",
    description: "Weights recent samples more. Alpha closer to 1 = more responsive; closer to 0 = smoother.",
    params: [{ key: "alpha", label: "Alpha (α)", min: 0.01, max: 1.0, step: 0.01, default: 0.2 }],
    color: "#a78bfa",
    create() {
      let prev = null;
      return (val, p) => {
        if (prev === null) { prev = val; return val; }
        prev = p.alpha * val + (1 - p.alpha) * prev;
        return prev;
      };
    },
  },
  Median: {
    label: "Median",
    fullName: "Median Filter",
    description: "Returns the median of the window. Excellent at eliminating spikes (outliers) while preserving edges.",
    params: [{ key: "window", label: "Window Size", min: 3, max: 31, step: 2, default: 7 }],
    color: "#86efac",
    create() {
      const buf = [];
      return (val, p) => {
        buf.push(val);
        if (buf.length > p.window) buf.shift();
        const s = [...buf].sort((a, b) => a - b);
        const m = Math.floor(s.length / 2);
        return s.length % 2 === 1 ? s[m] : (s[m - 1] + s[m]) / 2;
      };
    },
  },
  LowPass: {
    label: "Low-Pass",
    fullName: "RC Low-Pass Filter",
    description: "Time-domain RC filter. Tau (τ) is the time constant — larger τ = more smoothing. dt is your loop period.",
    params: [
      { key: "tau", label: "Tau τ (s)", min: 0.01, max: 2.0, step: 0.01, default: 0.3 },
      { key: "dt", label: "dt (s)", min: 0.01, max: 0.5, step: 0.01, default: 0.05 },
    ],
    color: "#fbbf24",
    create() {
      let prev = null;
      return (val, p) => {
        const alpha = p.dt / (p.tau + p.dt);
        if (prev === null) { prev = val; return val; }
        prev = alpha * val + (1 - alpha) * prev;
        return prev;
      };
    },
  },
  Kalman: {
    label: "Kalman",
    fullName: "1D Kalman Filter",
    description: "Optimal estimator that balances process uncertainty (Q) vs measurement uncertainty (R). Q↑ = trust sensor more; R↑ = trust model more.",
    params: [
      { key: "Q", label: "Process Noise Q", min: 0.0001, max: 1.0, step: 0.001, default: 0.01 },
      { key: "R", label: "Meas. Noise R", min: 0.01, max: 10.0, step: 0.01, default: 0.5 },
    ],
    color: "#f472b6",
    create() {
      let x = null, P = 1.0;
      return (val, p) => {
        if (x === null) { x = val; return val; }
        P = P + p.Q;
        const K = P / (P + p.R);
        x = x + K * (val - x);
        P = (1 - K) * P;
        return x;
      };
    },
  },
  Complementary: {
    label: "Complementary",
    fullName: "Complementary Filter",
    description: "Fuses two signals: slow (accel/position) and fast (gyro/derivative). Alpha weights the high-frequency source.",
    params: [
      { key: "alpha", label: "Alpha (α)", min: 0.01, max: 0.99, step: 0.01, default: 0.96 },
      { key: "dt", label: "dt (s)", min: 0.01, max: 0.2, step: 0.01, default: 0.05 },
    ],
    color: "#fb923c",
    create() {
      let angle = null;
      return (val, p, extra) => {
        const gyroRate = extra?.gyroRate ?? 0;
        if (angle === null) { angle = val; return val; }
        const gyroAngle = angle + gyroRate * p.dt;
        angle = p.alpha * gyroAngle + (1 - p.alpha) * val;
        return angle;
      };
    },
  },
};

// ─── Metric helpers ───────────────────────────────────────────────────────────
function computeMetrics(data, trueValue) {
  if (!data.length) return null;
  const rawVals = data.map((d) => d.raw);
  const filtVals = data.map((d) => d.filtered);
  const rawRMSE = Math.sqrt(rawVals.reduce((s, v) => s + (v - trueValue) ** 2, 0) / rawVals.length);
  const filtRMSE = Math.sqrt(filtVals.reduce((s, v) => s + (v - trueValue) ** 2, 0) / filtVals.length);
  const improvement = ((rawRMSE - filtRMSE) / rawRMSE) * 100;
  return { rawRMSE: rawRMSE.toFixed(3), filtRMSE: filtRMSE.toFixed(3), improvement: improvement.toFixed(1) };
}

const WINDOW = 120;

export default function App() {
  const [selectedSensor, setSelectedSensor] = useState("Sharp IR");
  const [selectedFilter, setSelectedFilter] = useState("SMA");
  const [params, setParams] = useState(() => {
    const p = {};
    FILTERS["SMA"].params.forEach((pp) => (p[pp.key] = pp.default));
    return p;
  });
  const [data, setData] = useState([]);
  const [paused, setPaused] = useState(false);

  const filterFnRef = useRef(null);
  const tickRef = useRef(0);
  const pausedRef = useRef(false);
  pausedRef.current = paused;

  // Reset filter when sensor or filter type changes
  useEffect(() => {
    const filterDef = FILTERS[selectedFilter];
    filterFnRef.current = filterDef.create();
    const newParams = {};
    filterDef.params.forEach((p) => (newParams[p.key] = p.default));
    setParams(newParams);
    setData([]);
    tickRef.current = 0;
  }, [selectedSensor, selectedFilter]);

  // Rebuild filter fn when params change (don't reset data)
  const filterFnParamRef = useRef(params);
  filterFnParamRef.current = params;

  const sensorRef = useRef(selectedSensor);
  sensorRef.current = selectedSensor;

  useEffect(() => {
    const interval = setInterval(() => {
      if (pausedRef.current) return;
      const t = tickRef.current++;
      const sensor = SENSORS[sensorRef.current];
      const raw = generateRaw(sensor, t);
      const fn = filterFnRef.current;
      const filtered = fn ? fn(raw, filterFnParamRef.current, { gyroRate: (Math.random() - 0.5) * 2 }) : raw;

      setData((prev) => {
        const next = [...prev, { t, raw: +raw.toFixed(4), filtered: +filtered.toFixed(4), true: sensor.trueValue }];
        if (next.length > WINDOW) next.shift();
        return next;
      });
    }, 60);
    return () => clearInterval(interval);
  }, []);

  const sensor = SENSORS[selectedSensor];
  const filterDef = FILTERS[selectedFilter];
  const metrics = computeMetrics(data, sensor.trueValue);

  const handleParamChange = (key, val) => {
    setParams((prev) => ({ ...prev, [key]: parseFloat(val) }));
  };

  const resetFilter = () => {
    filterFnRef.current = FILTERS[selectedFilter].create();
    setData([]);
    tickRef.current = 0;
  };

  return (
    <div style={{
      minHeight: "100vh",
      background: "#0a0e1a",
      color: "#e2e8f0",
      fontFamily: "'JetBrains Mono', 'Fira Code', monospace",
      display: "flex",
      flexDirection: "column",
    }}>
      {/* Top Bar */}
      <div style={{
        borderBottom: "1px solid #1e2d4a",
        padding: "16px 28px",
        display: "flex",
        alignItems: "center",
        gap: 16,
        background: "rgba(10,14,26,0.95)",
        backdropFilter: "blur(12px)",
        position: "sticky",
        top: 0,
        zIndex: 100,
      }}>
        <div style={{ display: "flex", alignItems: "center", gap: 10 }}>
          <div style={{
            width: 32, height: 32, borderRadius: 8,
            background: "linear-gradient(135deg, #3b82f6, #8b5cf6)",
            display: "flex", alignItems: "center", justifyContent: "center",
            fontSize: 16,
          }}>⚡</div>
          <div>
            <div style={{ fontSize: 13, fontWeight: 700, letterSpacing: "0.12em", color: "#93c5fd" }}>SENSOR FILTER</div>
            <div style={{ fontSize: 10, color: "#475569", letterSpacing: "0.2em" }}>EXPLORER v1.0</div>
          </div>
        </div>
        <div style={{ flex: 1 }} />
        <button
          onClick={() => setPaused((p) => !p)}
          style={{
            background: paused ? "rgba(34,197,94,0.15)" : "rgba(239,68,68,0.15)",
            border: `1px solid ${paused ? "#22c55e" : "#ef4444"}`,
            color: paused ? "#22c55e" : "#ef4444",
            padding: "6px 16px", borderRadius: 6, cursor: "pointer",
            fontSize: 11, letterSpacing: "0.1em", fontFamily: "inherit",
          }}
        >
          {paused ? "▶ RESUME" : "⏸ PAUSE"}
        </button>
        <button
          onClick={resetFilter}
          style={{
            background: "rgba(59,130,246,0.1)",
            border: "1px solid #3b82f6",
            color: "#93c5fd",
            padding: "6px 16px", borderRadius: 6, cursor: "pointer",
            fontSize: 11, letterSpacing: "0.1em", fontFamily: "inherit",
          }}
        >
          ↺ RESET
        </button>
      </div>

      <div style={{ display: "flex", flex: 1, gap: 0 }}>
        {/* Sidebar */}
        <div style={{
          width: 260, flexShrink: 0,
          borderRight: "1px solid #1e2d4a",
          padding: "20px 16px",
          display: "flex", flexDirection: "column", gap: 24,
          overflowY: "auto",
        }}>
          {/* Sensor selector */}
          <div>
            <div style={{ fontSize: 10, color: "#64748b", letterSpacing: "0.2em", marginBottom: 10 }}>SELECT SENSOR</div>
            <div style={{ display: "flex", flexDirection: "column", gap: 4 }}>
              {Object.entries(SENSORS).map(([name, s]) => (
                <button
                  key={name}
                  onClick={() => setSelectedSensor(name)}
                  style={{
                    background: selectedSensor === name ? `${s.color}18` : "transparent",
                    border: `1px solid ${selectedSensor === name ? s.color : "#1e2d4a"}`,
                    color: selectedSensor === name ? s.color : "#64748b",
                    padding: "8px 12px",
                    borderRadius: 6,
                    cursor: "pointer",
                    textAlign: "left",
                    fontSize: 11,
                    fontFamily: "inherit",
                    letterSpacing: "0.05em",
                    transition: "all 0.15s",
                  }}
                >
                  {name}
                </button>
              ))}
            </div>
          </div>

          {/* Filter selector */}
          <div>
            <div style={{ fontSize: 10, color: "#64748b", letterSpacing: "0.2em", marginBottom: 10 }}>SELECT FILTER</div>
            <div style={{ display: "flex", flexDirection: "column", gap: 4 }}>
              {Object.entries(FILTERS).map(([key, f]) => (
                <button
                  key={key}
                  onClick={() => setSelectedFilter(key)}
                  style={{
                    background: selectedFilter === key ? `${f.color}18` : "transparent",
                    border: `1px solid ${selectedFilter === key ? f.color : "#1e2d4a"}`,
                    color: selectedFilter === key ? f.color : "#64748b",
                    padding: "8px 12px",
                    borderRadius: 6,
                    cursor: "pointer",
                    textAlign: "left",
                    fontSize: 11,
                    fontFamily: "inherit",
                    letterSpacing: "0.05em",
                    transition: "all 0.15s",
                  }}
                >
                  {f.label}
                  <div style={{ fontSize: 9, opacity: 0.6, marginTop: 1 }}>{f.fullName}</div>
                </button>
              ))}
            </div>
          </div>
        </div>

        {/* Main content */}
        <div style={{ flex: 1, display: "flex", flexDirection: "column", overflow: "hidden" }}>
          {/* Info row */}
          <div style={{
            padding: "16px 24px",
            borderBottom: "1px solid #1e2d4a",
            display: "flex", alignItems: "flex-start", gap: 24,
          }}>
            <div style={{ flex: 1 }}>
              <div style={{ display: "flex", alignItems: "center", gap: 12, marginBottom: 4 }}>
                <div style={{ width: 10, height: 10, borderRadius: "50%", background: sensor.color }} />
                <span style={{ fontSize: 16, fontWeight: 700, color: "#f1f5f9" }}>{selectedSensor}</span>
                <span style={{
                  fontSize: 10, background: `${sensor.color}22`, color: sensor.color,
                  padding: "2px 8px", borderRadius: 20, border: `1px solid ${sensor.color}44`,
                }}>{sensor.unit}</span>
              </div>
              <div style={{ fontSize: 11, color: "#64748b", maxWidth: 420 }}>{sensor.description}</div>
            </div>
            <div style={{ flex: 1 }}>
              <div style={{ display: "flex", alignItems: "center", gap: 8, marginBottom: 4 }}>
                <div style={{ width: 10, height: 10, borderRadius: "50%", background: filterDef.color }} />
                <span style={{ fontSize: 14, fontWeight: 700, color: "#f1f5f9" }}>{filterDef.fullName}</span>
              </div>
              <div style={{ fontSize: 11, color: "#64748b" }}>{filterDef.description}</div>
            </div>
          </div>

          {/* Chart */}
          <div style={{ flex: 1, padding: "20px 24px 0", minHeight: 0 }}>
            <ResponsiveContainer width="100%" height={320}>
              <LineChart data={data} margin={{ top: 10, right: 20, bottom: 10, left: 10 }}>
                <CartesianGrid stroke="#1e2d4a" strokeDasharray="4 4" />
                <XAxis dataKey="t" tick={{ fill: "#475569", fontSize: 10 }} tickLine={false} axisLine={{ stroke: "#1e2d4a" }} />
                <YAxis tick={{ fill: "#475569", fontSize: 10 }} tickLine={false} axisLine={{ stroke: "#1e2d4a" }}
                  tickFormatter={(v) => v.toFixed(1)} />
                <Tooltip
                  contentStyle={{ background: "#0f172a", border: "1px solid #1e3a5f", borderRadius: 8, fontSize: 11 }}
                  labelStyle={{ color: "#94a3b8" }}
                />
                <Legend wrapperStyle={{ fontSize: 11, paddingTop: 8 }} />
                <Line
                  type="monotone" dataKey="raw" name="Raw Signal"
                  stroke="#ef4444" strokeWidth={1} dot={false} opacity={0.6}
                  isAnimationActive={false}
                />
                <Line
                  type="monotone" dataKey="true" name="True Value"
                  stroke="#475569" strokeWidth={1.5} dot={false}
                  strokeDasharray="6 3" isAnimationActive={false}
                />
                <Line
                  type="monotone" dataKey="filtered" name={`Filtered (${filterDef.label})`}
                  stroke={filterDef.color} strokeWidth={2.5} dot={false}
                  isAnimationActive={false}
                />
              </LineChart>
            </ResponsiveContainer>
          </div>

          {/* Parameters + metrics row */}
          <div style={{
            padding: "16px 24px 20px",
            display: "flex", gap: 24, alignItems: "flex-start",
          }}>
            {/* Params */}
            <div style={{
              flex: 1,
              background: "#0f172a",
              border: "1px solid #1e2d4a",
              borderRadius: 10,
              padding: "16px 20px",
            }}>
              <div style={{ fontSize: 10, color: "#64748b", letterSpacing: "0.2em", marginBottom: 14 }}>
                ⚙ FILTER PARAMETERS
              </div>
              <div style={{ display: "flex", flexDirection: "column", gap: 16 }}>
                {filterDef.params.map((p) => (
                  <div key={p.key}>
                    <div style={{ display: "flex", justifyContent: "space-between", marginBottom: 6 }}>
                      <span style={{ fontSize: 11, color: "#94a3b8" }}>{p.label}</span>
                      <span style={{
                        fontSize: 12, color: filterDef.color, fontWeight: 700,
                        background: `${filterDef.color}18`, padding: "1px 8px", borderRadius: 4,
                      }}>
                        {(params[p.key] ?? p.default).toFixed(p.step < 0.1 ? 3 : p.step < 1 ? 2 : 0)}
                      </span>
                    </div>
                    <div style={{ position: "relative" }}>
                      <input
                        type="range"
                        min={p.min} max={p.max} step={p.step}
                        value={params[p.key] ?? p.default}
                        onChange={(e) => handleParamChange(p.key, e.target.value)}
                        style={{
                          width: "100%",
                          accentColor: filterDef.color,
                          cursor: "pointer",
                        }}
                      />
                      <div style={{ display: "flex", justifyContent: "space-between", fontSize: 9, color: "#334155", marginTop: 2 }}>
                        <span>{p.min}</span>
                        <span>{p.max}</span>
                      </div>
                    </div>
                  </div>
                ))}
              </div>
            </div>

            {/* Metrics */}
            <div style={{
              width: 280,
              background: "#0f172a",
              border: "1px solid #1e2d4a",
              borderRadius: 10,
              padding: "16px 20px",
            }}>
              <div style={{ fontSize: 10, color: "#64748b", letterSpacing: "0.2em", marginBottom: 14 }}>
                📊 PERFORMANCE METRICS
              </div>
              {metrics ? (
                <div style={{ display: "flex", flexDirection: "column", gap: 12 }}>
                  <MetricRow label="Raw RMSE" value={`±${metrics.rawRMSE}`} color="#ef4444" unit={sensor.unit} />
                  <MetricRow label="Filtered RMSE" value={`±${metrics.filtRMSE}`} color={filterDef.color} unit={sensor.unit} />
                  <div style={{ borderTop: "1px solid #1e2d4a", paddingTop: 12, marginTop: 4 }}>
                    <div style={{ display: "flex", justifyContent: "space-between", alignItems: "center" }}>
                      <span style={{ fontSize: 11, color: "#64748b" }}>Noise Reduction</span>
                      <span style={{
                        fontSize: 16, fontWeight: 800,
                        color: parseFloat(metrics.improvement) > 0 ? "#22c55e" : "#ef4444",
                      }}>
                        {metrics.improvement > 0 ? "▼" : "▲"} {Math.abs(metrics.improvement)}%
                      </span>
                    </div>
                    <div style={{
                      marginTop: 8,
                      height: 6, borderRadius: 3,
                      background: "#1e2d4a", overflow: "hidden",
                    }}>
                      <div style={{
                        height: "100%",
                        width: `${Math.min(100, Math.max(0, parseFloat(metrics.improvement)))}%`,
                        background: `linear-gradient(90deg, ${filterDef.color}88, ${filterDef.color})`,
                        borderRadius: 3,
                        transition: "width 0.3s",
                      }} />
                    </div>
                  </div>
                  <div style={{ fontSize: 10, color: "#334155", marginTop: 2 }}>
                    True value: <span style={{ color: "#64748b" }}>{sensor.trueValue} {sensor.unit}</span>
                  </div>
                </div>
              ) : (
                <div style={{ fontSize: 11, color: "#334155" }}>Collecting data…</div>
              )}
            </div>
          </div>

          {/* Legend / hint bar */}
          <div style={{
            padding: "10px 24px",
            borderTop: "1px solid #1e2d4a",
            display: "flex", gap: 20, alignItems: "center",
          }}>
            <LegendChip color="#ef4444" label="Raw sensor signal" />
            <LegendChip color={filterDef.color} label={`${filterDef.label} filtered output`} />
            <LegendChip color="#475569" label="True value" dashed />
            <div style={{ flex: 1 }} />
            <div style={{ fontSize: 10, color: "#334155", letterSpacing: "0.1em" }}>
              LIVE · {data.length} samples · {sensor.unit}
            </div>
          </div>
        </div>
      </div>
    </div>
  );
}

function MetricRow({ label, value, color, unit }) {
  return (
    <div style={{ display: "flex", justifyContent: "space-between", alignItems: "center" }}>
      <span style={{ fontSize: 11, color: "#64748b" }}>{label}</span>
      <span style={{ fontSize: 12, color, fontWeight: 700 }}>
        {value} <span style={{ fontSize: 9, opacity: 0.6 }}>{unit}</span>
      </span>
    </div>
  );
}

function LegendChip({ color, label, dashed }) {
  return (
    <div style={{ display: "flex", alignItems: "center", gap: 6 }}>
      <div style={{
        width: 20, height: 2,
        background: dashed ? "transparent" : color,
        borderTop: dashed ? `2px dashed ${color}` : "none",
        opacity: dashed ? 0.5 : 1,
      }} />
      <span style={{ fontSize: 10, color: "#475569" }}>{label}</span>
    </div>
  );
}