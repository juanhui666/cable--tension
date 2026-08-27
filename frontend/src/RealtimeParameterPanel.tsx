import { FileUp, Pause, Play, RotateCcw, StepForward, Square } from "lucide-react";
import { useRef } from "react";
import type { ChangeEvent } from "react";
import type { RealtimeCsvDataset } from "./realtimeCsv";
import type { RealtimeStaticForm } from "./types";

type NumericField = keyof RealtimeStaticForm;

export function RealtimeParameterPanel({
  form, dataset, active, playing, busy, currentIndex,
  onFormChange, onFile, onStartPause, onStep, onStop, onReset,
}: {
  form: RealtimeStaticForm;
  dataset: RealtimeCsvDataset | null;
  active: boolean;
  playing: boolean;
  busy: boolean;
  currentIndex: number;
  onFormChange: (field: NumericField, value: number | null) => void;
  onFile: (file: File) => void;
  onStartPause: () => void;
  onStep: () => void;
  onStop: () => void;
  onReset: () => void;
}) {
  const fallbackFileInput = useRef<HTMLInputElement>(null);
  const atEnd = Boolean(dataset && currentIndex >= dataset.rows.length - 1);
  const previewIndex = dataset ? Math.min(Math.max(currentIndex, 0), dataset.rows.length - 1) : -1;
  const previewPacket = previewIndex >= 0 ? dataset?.rows[previewIndex] : undefined;
  const fileChange = (event: ChangeEvent<HTMLInputElement>) => {
    const file = event.target.files?.[0];
    if (file) onFile(file);
    event.target.value = "";
  };
  const chooseCsvFile = async () => {
    const picker = (window as Window & { showOpenFilePicker?: OpenFilePicker }).showOpenFilePicker;
    if (!picker) {
      fallbackFileInput.current?.click();
      return;
    }
    try {
      const [handle] = await picker({
        id: "realtime-csv-source",
        multiple: false,
        types: [{ description: "CSV 数据", accept: { "text/csv": [".csv"] } }],
      });
      if (handle) onFile(await handle.getFile());
    } catch (error) {
      if ((error as DOMException).name !== "AbortError") fallbackFileInput.current?.click();
    }
  };
  const field = (name: NumericField, label: string, optional = false) => (
    <label className="field"><span>{label}</span><input
      aria-label={label}
      disabled={active}
      onChange={(event) => onFormChange(name, event.target.value === "" && optional ? null : Number(event.target.value))}
      placeholder={optional ? "未设置" : undefined}
      type="number"
      value={form[name] ?? ""}
    /></label>
  );
  return (
    <section aria-label="实时数据参数" className="input-panel parameter-panel realtime-parameter-panel">
      <div className="panel-heading"><div><h2>实时张力计算</h2></div></div>
      <div className="realtime-input-grid">
        <section className="form-section realtime-static-section">
          <h3>材料与环境</h3>
          <div className="field-grid">
            {field("diameter_m", "外径 m")}
            {field("weight_air_n_per_m", "空气中单位长度重量 N/m")}
            {field("tangential_drag_coefficient", "切向阻力系数")}
            {field("normal_drag_coefficient", "法向阻力系数")}
            {field("axial_stiffness_n", "轴向刚度 EA N")}
            {field("min_bending_radius_m", "最小弯曲半径 m（可选）", true)}
            {field("water_depth_m", "作业水深 m")}
          </div>
          <h3>初始几何</h3>
          <div className="field-grid">
            {field("initial_suspended_length_m", "初始悬垂长度 m")}
            {field("plough_layback_m", "犁入口水平后拖距离 m")}
            {field("plough_depth_m", "犁入口绝对深度 m")}
          </div>
        </section>
        <section className="form-section realtime-data-section">
          <h3>1 Hz 边界数据</h3>
          <button className="csv-file-control" disabled={active || busy} onClick={chooseCsvFile} type="button"><FileUp aria-hidden="true" /><span>选择本地 CSV</span></button>
          <input accept=".csv,text/csv" aria-label="选择本地 CSV 文件" className="csv-fallback-input" disabled={active || busy} onChange={fileChange} ref={fallbackFileInput} type="file" />
          {dataset ? <div className="csv-summary"><strong>{dataset.fileName}</strong><span>{dataset.rows.length} 包 · 0-{dataset.rows.at(-1)?.time_s ?? 0} s</span></div> : <p className="csv-empty">请选择本地 CSV。</p>}
          {previewPacket ? (
            <div aria-label="当前帧船端导缆点输入" className="fairlead-frame-preview">
              <div className="fairlead-frame-heading"><span>船端导缆点 · 当前帧</span><strong>#{previewPacket.sequence} · t={previewPacket.time_s.toFixed(0)} s</strong></div>
              <dl>
                <div><dt>导缆点位置 m</dt><dd>X {formatBoundaryValue(previewPacket.vessel.x_m)} · Y {formatBoundaryValue(previewPacket.vessel.y_m)} · Z {formatBoundaryValue(previewPacket.vessel.z_m)}</dd></div>
                <div><dt>导缆点速度 m/s</dt><dd>Vx {formatBoundaryValue(previewPacket.vessel.velocity_x_mps)} · Vy {formatBoundaryValue(previewPacket.vessel.velocity_y_mps)} · Vz {formatBoundaryValue(previewPacket.vessel.velocity_z_mps)}</dd></div>
              </dl>
            </div>
          ) : null}
          <div className="realtime-playback-controls">
            <button className="primary-action" disabled={!dataset || busy || atEnd} onClick={onStartPause} type="button">{playing ? <Pause /> : <Play />}{playing ? "暂停" : active ? "继续" : "初始化并开始"}</button>
            <button disabled={!dataset || busy || playing || atEnd} onClick={onStep} type="button"><StepForward />单步</button>
            <button disabled={!playing} onClick={onStop} type="button"><Square />暂停计算</button>
            <button disabled={busy} onClick={onReset} type="button"><RotateCcw />重新初始化</button>
          </div>
        </section>
      </div>
    </section>
  );
}

function formatBoundaryValue(value: number): string {
  return value.toFixed(3);
}

type OpenFilePicker = (options: {
  id: string;
  multiple: boolean;
  types: Array<{ description: string; accept: Record<string, string[]> }>;
}) => Promise<Array<{ getFile: () => Promise<File> }>>;
