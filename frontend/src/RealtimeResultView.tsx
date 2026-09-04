import { DynamicFrameViewer } from "./DynamicFrameViewer";
import { formatKiloNewton, formatNumber } from "./format";
import type { DynamicFrame, RealtimeFrameResponse } from "./types";

export function RealtimeResultView({ history, completed, playing, waterDepthM }: {
  history: RealtimeFrameResponse[];
  completed: boolean;
  playing: boolean;
  waterDepthM: number;
}) {
  const latest = history.at(-1);
  if (!latest) return null;
  const frames: DynamicFrame[] = history.map(toDynamicFrame);
  return (
    <section className="realtime-result" aria-label="1 Hz 实时计算结果">
      <div className="realtime-status-bar">
        <div><strong>{latest.cable.name}</strong><span>1 Hz 实时计算</span><span>{completed ? "计算完成" : playing ? "计算中" : "已暂停"}</span><span>帧 {latest.sequence}</span><span>数据时刻 {formatNumber(latest.time_s, 1)} s</span><span>单帧耗时 {formatNumber(latest.runtime.compute_wall_s * 1000, 0)} ms</span></div>
      </div>
      <div className="realtime-grid">
        <section className="result-block simulation-viewer realtime-viewer">
          <div className="block-heading stage-block-heading"><h3>最新三维状态</h3></div>
          <DynamicFrameViewer
            currentFrame={frames.length - 1}
            endpointLoads={{
              top_tension_n: latest.tensions.top_tension_n,
              plough_inlet_tension_n: latest.tensions.plough_inlet_tension_n,
            }}
            frames={{ source: "realtime_result_v1", label: "实时状态序列", items: frames }}
            timelineLocked
            waterDepthM={waterDepthM}
          />
        </section>
        <aside className="realtime-metrics" aria-label="实时张力指标">
          <section className="metric-group calculated-values" aria-label="计算值">
            <h3>计算值</h3>
            <Metric label="船端导缆点轴向端载荷" value={formatKiloNewton(latest.tensions.top_tension_n)} />
            <Metric label="犁前末段张力" value={formatKiloNewton(latest.tensions.plough_inlet_tension_n)} />
            <Metric label="船端实测轴向载荷" value={latest.tensions.measured_top_tension_n === null ? "未上传" : formatKiloNewton(latest.tensions.measured_top_tension_n)} />
            <Metric label="船端载荷残差（实测－计算）" value={latest.tensions.top_tension_residual_n === null ? "未提供" : formatKiloNewton(latest.tensions.top_tension_residual_n)} />
            <Metric label="实际最小曲率半径" value={latest.minimum_bend_radius.minimum_m === null ? "不可用" : `${formatNumber(latest.minimum_bend_radius.minimum_m, 3)} m`} />
            <Metric label="有效弯曲刚度 EI" value={`${formatNumber(latest.bending.effective_stiffness_n_m2 / 1000, 1)} kN·m²`} />
            <Metric label="最大离散曲率" value={latest.bending.maximum_curvature_per_m === null ? "不可用" : `${formatNumber(latest.bending.maximum_curvature_per_m, 5)} 1/m`} />
            <Metric label="最大弯矩 EIκ" value={latest.bending.maximum_moment_n_m === null ? "不可用" : `${formatNumber(latest.bending.maximum_moment_n_m / 1000, 2)} kN·m`} />
            <Metric label="船端出缆水平角（+X向+Y为正）" value={`${formatNumber(latest.vessel_departure_angles.horizontal_deg, 2)}°`} />
            <Metric label="船端出缆垂直角（向下为正）" value={`${formatNumber(latest.vessel_departure_angles.vertical_deg, 2)}°`} />
            <Metric label="实时倍率" value={latest.runtime.realtime_factor === null ? "初始化帧" : formatNumber(latest.runtime.realtime_factor, 2)} />
            <Metric label="节点 / 缆段" value={`${latest.cable_shape.points.length} / ${latest.cable_shape.segment_tensions_n.length}`} />
          </section>
          <section className="metric-group manufacturer-values" aria-label="厂家参考值">
            <h3>厂家参考值</h3>
            <Metric label="安装态最小弯曲半径 · MBR for Installation (LC, 355kN tension with 138Bar pressure)" value={formatReference(latest.manufacturer_limits.installation_lc_mbr_m, "m")} />
            <Metric label="正常运行最小弯曲半径 · MBR for normal operation (LC, 10kN tension with DWP)" value={formatReference(latest.manufacturer_limits.normal_operation_lc_mbr_m, "m")} />
            <Metric label="储存最小弯曲半径 · MBR for Storage (DC)" value={formatReference(latest.manufacturer_limits.storage_dc_mbr_m, "m")} />
            <Metric label="安装态最小弯曲半径 · MBR for Installation (DC, 355kN tension with 138Bar pressure)" value={formatReference(latest.manufacturer_limits.installation_dc_mbr_m, "m")} />
            <Metric label="最大工作载荷 · Maximum Working Load (Straight with DWP)" value={formatReferenceKn(latest.manufacturer_limits.maximum_working_load_n)} />
            <Metric label="最大异常运行载荷 · Maximum abnormal operation Load (Straight with DWP)" value={formatReferenceKn(latest.manufacturer_limits.maximum_abnormal_operation_load_n)} />
            <Metric label="DWP破断载荷 · Breaking Load at tubes' UTS (Straight with DWP)" value={formatReferenceKn(latest.manufacturer_limits.dwp_breaking_load_n)} />
          </section>
        </aside>
      </div>
      <TensionTrend history={history} />
    </section>
  );
}

function toDynamicFrame(result: RealtimeFrameResponse): DynamicFrame {
  const tensions = result.cable_shape.segment_tensions_n;
  return {
    time_s: result.time_s,
    boundary: "known_plough_trajectory",
    segment_tensions_n: tensions,
    points: result.cable_shape.points.map((point, index) => ({
      ...point,
      tension_n: tensions[Math.min(index, Math.max(tensions.length - 1, 0))] ?? 0,
    })),
    vessel_x_m: result.cable_shape.points[0]?.x_m,
    vessel_y_m: result.cable_shape.points[0]?.y_m,
    vessel_z_m: result.cable_shape.points[0]?.z_m,
    plough_x_m: result.cable_shape.points.at(-1)?.x_m,
    plough_y_m: result.cable_shape.points.at(-1)?.y_m,
    plough_z_m: result.cable_shape.points.at(-1)?.z_m,
  };
}

function Metric({ label, value }: { label: string; value: string }) {
  return <div><span>{label}</span><strong>{value}</strong></div>;
}

function formatReference(value: number | null, unit: string) {
  return value === null ? "未提供" : `${formatNumber(value, 2)} ${unit}`;
}

function formatReferenceKn(value: number | null) {
  return value === null ? "未提供" : formatKiloNewton(value);
}

function TensionTrend({ history }: { history: RealtimeFrameResponse[] }) {
  const width = 900; const height = 190; const pad = 34;
  const values = history.flatMap((item) => [item.tensions.top_tension_n, item.tensions.plough_inlet_tension_n]);
  const min = Math.min(...values); const max = Math.max(...values); const span = Math.max(max - min, 1);
  const line = (selector: (item: RealtimeFrameResponse) => number) => history.map((item, index) => {
    const x = pad + (index / Math.max(history.length - 1, 1)) * (width - 2 * pad);
    const y = height - pad - ((selector(item) - min) / span) * (height - 2 * pad);
    return `${x},${y}`;
  }).join(" ");
  return <section className="realtime-trend" aria-label="实时张力趋势"><div><h3>最近 {history.length} 帧张力趋势</h3></div><svg aria-label="张力随时间变化曲线" role="img" viewBox={`0 0 ${width} ${height}`}><polyline className="trend-top" fill="none" points={line((item) => item.tensions.top_tension_n)} /><polyline className="trend-plough" fill="none" points={line((item) => item.tensions.plough_inlet_tension_n)} /></svg></section>;
}
