import { FileUp, Pause, Play, RotateCcw, StepForward, Square } from "lucide-react";
import type { ChangeEvent } from "react";
import type { RealtimeCsvDataset } from "./realtimeCsv";
import type { PloughPositionMode, RealtimeStaticForm } from "./types";

type NumericField = Exclude<keyof RealtimeStaticForm, "cable_name" | "plough_position_mode">;

export function RealtimeParameterPanel({
  form, dataset, active, playing, busy, currentIndex,
  onFormChange, onCableNameChange, onPloughPositionModeChange, onFile, onStartPause, onStep, onStop, onReset,
}: {
  form: RealtimeStaticForm;
  dataset: RealtimeCsvDataset | null;
  active: boolean;
  playing: boolean;
  busy: boolean;
  currentIndex: number;
  onFormChange: (field: NumericField, value: number | null) => void;
  onCableNameChange: (value: string) => void;
  onPloughPositionModeChange: (mode: PloughPositionMode) => void;
  onFile: (file: File) => void;
  onStartPause: () => void;
  onStep: () => void;
  onStop: () => void;
  onReset: () => void;
}) {
  const measuredPloughPosition = form.plough_position_mode === "measured";
  const boundaryFields = measuredPloughPosition
    ? [
        { symbol: "X", label: "犁入口纵向位置" },
        { symbol: "Y", label: "犁入口横向位置" },
        { symbol: "Z", label: "犁入口垂向位置" },
      ]
    : [
        { symbol: "L", label: "船到犁水平直线距离" },
        { symbol: "β", label: "船向犁水平角" },
        { symbol: "h", label: "犁入口距海床高度" },
      ];
  const boundaryColumns = measuredPloughPosition
    ? ["plough_x_m", "plough_y_m", "plough_z_m"]
    : ["plough_horizontal_distance_m", "plough_bearing_deg", "plough_inlet_height_above_seabed_m"];
  const atEnd = Boolean(dataset && currentIndex >= dataset.rows.length - 1);
  const previewIndex = dataset ? Math.min(Math.max(currentIndex, 0), dataset.rows.length - 1) : -1;
  const previewPacket = previewIndex >= 0 ? dataset?.rows[previewIndex] : undefined;
  const bendingPreset = [78.0e3, 783.6e3, 80.8e3, 788.3e3].includes(
    form.bending_stiffness_n_m2,
  ) ? String(form.bending_stiffness_n_m2) : "custom";
  const fileChange = (event: ChangeEvent<HTMLInputElement>) => {
    const file = event.target.files?.[0];
    if (file) onFile(file);
    event.target.value = "";
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
          <h3>计算输入</h3>
          <div className="field-grid">
            <label className="field"><span>缆线名称 Cable name</span><input
              aria-label="缆线名称 Cable name"
              disabled={active}
              maxLength={128}
              onChange={(event) => onCableNameChange(event.target.value)}
              type="text"
              value={form.cable_name}
            /></label>
            {field("diameter_m", "名义外径 Umbilical OD m（厂家 232.2±8 mm）")}
            {field("mass_air_kg_per_m", "空气中单位长度质量（无内部流体）Total mass of umbilical in air kg/m")}
            {field("submerged_weight_n_per_m", "水中单位重 Total weight of umbilical in water - flooded N/m")}
            {field("axial_stiffness_n", "轴向刚度 Axial stiffness N")}
            <label className="field"><span>弯曲刚度厂家工况</span><select
              aria-label="弯曲刚度厂家工况"
              disabled={active}
              onChange={(event) => {
                if (event.target.value !== "custom") {
                  onFormChange("bending_stiffness_n_m2", Number(event.target.value));
                }
              }}
              value={bendingPreset}
            >
              <option value="78000">20°C Full Slip · 78.0 kN·m²</option>
              <option value="783600">20°C Full Stick · 783.6 kN·m²</option>
              <option value="80800">−2°C Full Slip · 80.8 kN·m²</option>
              <option value="788300">−2°C Full Stick · 788.3 kN·m²</option>
              <option value="custom">自定义有效 EI</option>
            </select></label>
            {field("bending_stiffness_n_m2", "有效弯曲刚度 EI N·m²（参与求解）")}
            {field("tangential_drag_coefficient", "切向阻力系数 Ct（工程参数）")}
            {field("normal_drag_coefficient", "法向阻力系数 Cn（工程参数）")}
          </div>
          <h3>厂家参考值（不参与求解）</h3>
          <div className="field-grid manufacturer-reference-grid">
            {field("installation_lc_mbr_m", "安装态最小弯曲半径 MBR for Installation (LC, 355kN tension with 138Bar pressure) m", true)}
            {field("normal_operation_lc_mbr_m", "正常运行最小弯曲半径 MBR for normal operation (LC, 10kN tension with DWP) m", true)}
            {field("storage_dc_mbr_m", "储存最小弯曲半径 MBR for Storage (DC) m", true)}
            {field("installation_dc_mbr_m", "安装态最小弯曲半径 MBR for Installation (DC, 355kN tension with 138Bar pressure) m", true)}
            {field("maximum_working_load_n", "最大工作载荷 Maximum Working Load (Straight with DWP) N", true)}
            {field("maximum_abnormal_operation_load_n", "最大异常运行载荷 Maximum abnormal operation Load (Straight with DWP) N", true)}
            {field("dwp_breaking_load_n", "DWP破断载荷 Breaking Load at tubes' UTS (Straight with DWP) N", true)}
          </div>
          <h3>初始材料长度</h3>
          <div className="field-grid">
            {field("initial_suspended_length_m", "初始悬空材料长度 L0 m")}
          </div>
          <h3>犁入口边界</h3>
          <div className="field-grid">
            <label className="field"><span>犁入口位置来源</span><select
              aria-label="犁入口位置来源"
              disabled={active}
              onChange={(event) => onPloughPositionModeChange(event.target.value as PloughPositionMode)}
              value={form.plough_position_mode}
            >
              <option value="measured">传感器实测三维位置</option>
              <option value="reconstructed">无三维犁位，逐帧重建</option>
            </select></label>
          </div>
        </section>
        <section className="form-section realtime-data-section">
          <h3>1 Hz 边界数据</h3>
          <div
            aria-label="当前犁入口逐帧输入要求"
            className={`realtime-boundary-contract ${measuredPloughPosition ? "measured" : "reconstructed"}`}
          >
            <div className="boundary-contract-heading">
              <strong>{measuredPloughPosition ? "实测三维犁位" : "L / 水平角 / h 重建"}</strong>
              <span>每 1 秒随 CSV 输入</span>
            </div>
            <div className="boundary-contract-fields">
              {boundaryFields.map((item) => (
                <div key={item.symbol}>
                  <strong>{item.symbol}</strong>
                  <span>{item.label}</span>
                </div>
              ))}
            </div>
            <div className="boundary-contract-rule">
              {measuredPloughPosition
                ? "三维位置直接作为犁端边界"
                : "xp=xv+L cosβ · yp=yv+L sinβ · zp=水深−h"}
            </div>
            <div className="boundary-contract-columns">
              <strong>CSV 必填列</strong>
              <div>{boundaryColumns.map((column) => <code key={column}>{column}</code>)}</div>
            </div>
          </div>
          <label className={`csv-file-control ${active || busy ? "disabled" : ""}`}>
            <FileUp aria-hidden="true" />
            <span>选择本地 CSV</span>
            <input accept=".csv,text/csv" aria-label="选择本地 CSV 文件" className="csv-direct-input" disabled={active || busy} onChange={fileChange} type="file" />
          </label>
          {dataset ? <div className="csv-summary"><strong>{dataset.fileName}</strong><span>{dataset.rows.length} 包 · 0-{dataset.rows.at(-1)?.time_s ?? 0} s</span></div> : <p className="csv-empty">请选择本地 CSV。</p>}
          {previewPacket ? (
            <div aria-label="当前帧船端导缆点输入" className="fairlead-frame-preview">
              <div className="fairlead-frame-heading"><span>船端导缆点 · 当前帧</span><strong>#{previewPacket.sequence} · t={previewPacket.time_s.toFixed(0)} s</strong></div>
              <dl>
                <div><dt>水深 m</dt><dd>{formatBoundaryValue(previewPacket.water_depth_m)}</dd></div>
                <div><dt>导缆点位置 m</dt><dd>X {formatBoundaryValue(previewPacket.vessel.x_m)} · Y {formatBoundaryValue(previewPacket.vessel.y_m)} · Z {formatBoundaryValue(previewPacket.vessel.z_m)}</dd></div>
                <div><dt>导缆点速度 m/s</dt><dd>Vx {formatBoundaryValue(previewPacket.vessel.velocity_x_mps)} · Vy {formatBoundaryValue(previewPacket.vessel.velocity_y_mps)} · Vz {formatBoundaryValue(previewPacket.vessel.velocity_z_mps)}</dd></div>
                <div><dt>犁入口位置 m</dt><dd>{previewPacket.plough_position ? `X ${formatBoundaryValue(previewPacket.plough_position.x_m)} · Y ${formatBoundaryValue(previewPacket.plough_position.y_m)} · Z ${formatBoundaryValue(previewPacket.plough_position.z_m)}` : "由逐帧 L、角度和 h 重建"}</dd></div>
                {previewPacket.plough_horizontal_distance_m === undefined ? null : <div><dt>水平距离 L m</dt><dd>{formatBoundaryValue(previewPacket.plough_horizontal_distance_m)}</dd></div>}
                {previewPacket.plough_bearing_deg === undefined ? null : <div><dt>船向犁水平角 °</dt><dd>{formatBoundaryValue(previewPacket.plough_bearing_deg)}</dd></div>}
                {previewPacket.plough_inlet_height_above_seabed_m === undefined ? null : <div><dt>入口距海床 h m</dt><dd>{formatBoundaryValue(previewPacket.plough_inlet_height_above_seabed_m)}</dd></div>}
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
