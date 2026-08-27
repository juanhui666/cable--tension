import { Activity, Cable, CheckCircle2, Server } from "lucide-react";
import { useEffect, useRef, useState } from "react";
import { advanceRealtimeSession, ApiError, createRealtimeSession, DEFAULT_API_BASE, getHealth } from "./api";
import { RealtimeParameterPanel } from "./RealtimeParameterPanel";
import { RealtimeResultView } from "./RealtimeResultView";
import { DEFAULT_REALTIME_FORM } from "./simulationConfig";
import { parseRealtimeCsv, type RealtimeCsvDataset } from "./realtimeCsv";
import { realtimePlaybackDeadlineMs, realtimeScheduleDelayMs } from "./realtimeSchedule";
import type {
  CreateRealtimeSessionRequest,
  HealthResponse,
  RealtimeFrameResponse,
  RealtimeSensorPacket,
  RealtimeStaticForm,
} from "./types";
import "./styles.css";

const API_BASE = import.meta.env.VITE_API_BASE_URL || DEFAULT_API_BASE;

export default function App() {
  const [health, setHealth] = useState<HealthResponse | null>(null);
  const [form, setForm] = useState(DEFAULT_REALTIME_FORM);
  const [dataset, setDataset] = useState<RealtimeCsvDataset | null>(null);
  const [history, setHistory] = useState<RealtimeFrameResponse[]>([]);
  const [index, setIndex] = useState(-1);
  const [sessionId, setSessionId] = useState<string | null>(null);
  const [playing, setPlaying] = useState(false);
  const [busy, setBusy] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const timerRef = useRef<number | null>(null);
  const playbackOriginRef = useRef<number | null>(null);
  const playbackGenerationRef = useRef(0);
  const datasetLoadGenerationRef = useRef(0);
  const playingRef = useRef(false);
  const requestInFlightRef = useRef(false);

  useEffect(() => {
    let active = true;
    getHealth(API_BASE)
      .then((payload) => { if (active) setHealth(payload); })
      .catch((caught) => { if (active) setError(messageFrom(caught)); });
    return () => {
      active = false;
      playbackGenerationRef.current += 1;
      datasetLoadGenerationRef.current += 1;
      playingRef.current = false;
      if (timerRef.current !== null) window.clearTimeout(timerRef.current);
    };
  }, []);

  function pausePlayback() {
    playingRef.current = false;
    playbackOriginRef.current = null;
    if (timerRef.current !== null) window.clearTimeout(timerRef.current);
    timerRef.current = null;
    setPlaying(false);
  }

  function schedulePacket(
    activeDataset: RealtimeCsvDataset,
    activeSessionId: string,
    packetIndex: number,
    generation: number,
    originMs: number,
  ) {
    if (!playingRef.current || generation !== playbackGenerationRef.current) return;
    if (packetIndex >= activeDataset.rows.length) {
      pausePlayback();
      return;
    }
    const deadlineMs = realtimePlaybackDeadlineMs(originMs, packetIndex);
    timerRef.current = window.setTimeout(async () => {
      timerRef.current = null;
      if (!playingRef.current || generation !== playbackGenerationRef.current) return;
      // 调度链只在前一个请求完成后安排下一个请求；该守卫同时防止
      // 手动动作或重复回调造成重叠提交。
      if (requestInFlightRef.current) return;
      requestInFlightRef.current = true;
      setBusy(true);
      try {
        const result = await advanceRealtimeSession(activeSessionId, activeDataset.rows[packetIndex], API_BASE);
        if (generation !== playbackGenerationRef.current) return;
        setHistory((current) => [...current, result].slice(-120));
        setIndex(packetIndex);
        if (playingRef.current) {
          schedulePacket(activeDataset, activeSessionId, packetIndex + 1, generation, originMs);
        }
      } catch (caught) {
        if (generation === playbackGenerationRef.current) {
          pausePlayback();
          setError(messageFrom(caught));
        }
      } finally {
        requestInFlightRef.current = false;
        if (generation === playbackGenerationRef.current) setBusy(false);
      }
    }, realtimeScheduleDelayMs(deadlineMs));
  }

  async function startOrPause() {
    if (playing) {
      pausePlayback();
      return;
    }
    if (!dataset) return;
    setError(null);
    if (!sessionId) {
      const generation = playbackGenerationRef.current;
      const originMs = Date.now();
      playbackOriginRef.current = originMs;
      playingRef.current = true;
      setPlaying(true);
      requestInFlightRef.current = true;
      setBusy(true);
      try {
        const first = await createRealtimeSession(buildRealtimeSessionRequest(form, dataset.rows[0]), API_BASE);
        if (generation !== playbackGenerationRef.current) return;
        setSessionId(first.session_id);
        setHistory([first]);
        setIndex(0);
        if (dataset.rows.length > 1 && playingRef.current) {
          schedulePacket(dataset, first.session_id, 1, generation, originMs);
        } else {
          pausePlayback();
        }
      } catch (caught) {
        pausePlayback();
        setError(messageFrom(caught));
        return;
      } finally {
        requestInFlightRef.current = false;
        setBusy(false);
      }
      return;
    }
    if (index < dataset.rows.length - 1) {
      // 暂停期间的墙钟时间不属于数据节拍。恢复时重建原点，
      // 使下一包从恢复动作起等待完整一个周期。
      const originMs = Date.now() - index * 1000;
      playbackOriginRef.current = originMs;
      playingRef.current = true;
      setPlaying(true);
      schedulePacket(dataset, sessionId, index + 1, playbackGenerationRef.current, originMs);
    }
  }

  async function step() {
    if (!dataset) return;
    if (!sessionId) {
      const generation = playbackGenerationRef.current;
      requestInFlightRef.current = true;
      setBusy(true);
      setError(null);
      try {
        const first = await createRealtimeSession(buildRealtimeSessionRequest(form, dataset.rows[0]), API_BASE);
        if (generation !== playbackGenerationRef.current) return;
        setSessionId(first.session_id);
        setHistory([first]);
        setIndex(0);
      } catch (caught) {
        setError(messageFrom(caught));
      } finally {
        requestInFlightRef.current = false;
        setBusy(false);
      }
      return;
    }
    if (requestInFlightRef.current) return;
    const nextIndex = index + 1;
    if (nextIndex >= dataset.rows.length) return;
    requestInFlightRef.current = true;
    setBusy(true);
    setError(null);
    try {
      const result = await advanceRealtimeSession(sessionId, dataset.rows[nextIndex], API_BASE);
      setHistory((current) => [...current, result].slice(-120));
      setIndex(nextIndex);
    } catch (caught) {
      setError(messageFrom(caught));
    } finally {
      requestInFlightRef.current = false;
      setBusy(false);
    }
  }

  function clearSessionState() {
    playbackGenerationRef.current += 1;
    pausePlayback();
    setSessionId(null);
    setHistory([]);
    setIndex(-1);
    setError(null);
  }

  function reset() {
    datasetLoadGenerationRef.current += 1;
    clearSessionState();
    setBusy(false);
  }

  async function loadFile(file: File) {
    const loadGeneration = datasetLoadGenerationRef.current + 1;
    datasetLoadGenerationRef.current = loadGeneration;
    setBusy(true);
    setError(null);
    try {
      const nextDataset = parseRealtimeCsv(await file.text(), file.name);
      if (loadGeneration !== datasetLoadGenerationRef.current) return;
      setDataset(nextDataset);
      clearSessionState();
    } catch (caught) {
      if (loadGeneration === datasetLoadGenerationRef.current) setError(messageFrom(caught));
    } finally {
      if (loadGeneration === datasetLoadGenerationRef.current) setBusy(false);
    }
  }

  const completed = Boolean(dataset && index >= dataset.rows.length - 1);
  return (
    <main className="app-shell">
      <header className="topbar">
        <div className="brand"><Cable aria-hidden="true" /><div><strong>海底缆线实时张力工作台</strong><span>三维缆形 · 实时张力</span></div></div>
        <div className={`backend-status ${health ? "online" : ""}`}>
          {health ? <CheckCircle2 aria-hidden="true" /> : <Server aria-hidden="true" />}
          {health ? "后端在线" : "等待后端"}
        </div>
      </header>
      <section className="status-ribbon">
        <Activity aria-hidden="true" />
        <span>+X 沿航迹 · +Y 右舷 · +Z 向下</span>
      </section>
      {error ? <div role="alert" className="error-banner">{error}</div> : null}
      <RealtimeParameterPanel
        active={sessionId !== null}
        busy={busy}
        currentIndex={index}
        dataset={dataset}
        form={form}
        onFile={loadFile}
        onFormChange={(field, value) => setForm((current) => ({ ...current, [field]: value }))}
        onReset={reset}
        onStartPause={startOrPause}
        onStep={step}
        onStop={pausePlayback}
        playing={playing}
      />
      <RealtimeResultView completed={completed} history={history} playing={playing} waterDepthM={form.water_depth_m} />
    </main>
  );
}

export function buildRealtimeSessionRequest(
  form: RealtimeStaticForm,
  initialPacket: RealtimeSensorPacket,
): CreateRealtimeSessionRequest {
  return {
    cable: {
      diameter_m: form.diameter_m,
      mass_air_kg_per_m: form.weight_air_n_per_m / 9.8,
      tangential_drag_coefficient: form.tangential_drag_coefficient,
      normal_drag_coefficient: form.normal_drag_coefficient,
      axial_stiffness_n: form.axial_stiffness_n,
      ...(form.min_bending_radius_m === null ? {} : { min_bending_radius_m: form.min_bending_radius_m }),
    },
    environment: { water_depth_m: form.water_depth_m },
    initial_geometry: {
      initial_suspended_length_m: form.initial_suspended_length_m,
      plough_layback_m: form.plough_layback_m,
      plough_depth_m: form.plough_depth_m,
    },
    initial_packet: initialPacket,
  };
}

function messageFrom(caught: unknown): string {
  if (caught instanceof ApiError) {
    const fields = typeof caught.details === "object" && caught.details !== null
      ? (caught.details as { fields?: Record<string, string> }).fields
      : undefined;
    const detail = fields ? Object.entries(fields).map(([field, reason]) => `${field}：${reason}`).join("；") : "";
    return detail || caught.message;
  }
  return caught instanceof Error ? caught.message : "发生未知错误。";
}
