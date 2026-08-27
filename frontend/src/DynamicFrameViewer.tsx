/**
 * 使用 Three.js 渲染后端帧。几何与张力值不从表单输入重建，
 * 视图变换只改变显示方式。
 */
import { useEffect, useMemo, useRef, useState } from "react";
import * as THREE from "three";
import { OrbitControls } from "three/examples/jsm/controls/OrbitControls.js";
import { currentStateAtTime } from "./motionSchedule";
import type { DynamicFrame, DynamicFramePlotData, DynamicFramePoint, FrameEndpointLoads, RunTimeHistorySummary } from "./types";

interface Bounds {
  centerX: number;
  centerY: number;
  centerZ: number;
  span: number;
}

interface TensionRange {
  min: number;
  max: number;
}

interface CableVisualPoint {
  position: THREE.Vector3;
  tension_n: number;
  segmentIndex: number;
  showNode?: boolean;
}

type ViewMode = "global" | "vessel";

type MotionSummary = Pick<
  RunTimeHistorySummary,
  | "vessel_initial_speed_mps"
  | "vessel_final_speed_mps"
  | "transition_duration_s"
  | "total_duration_s"
  | "current_speed_mps"
  | "current_direction_deg"
> & Pick<RunTimeHistorySummary, "current_velocity_segments">;

const ZERO_OFFSET = { x: 0, y: 0, z: 0 };
const VESSEL_STERN_EXIT_LOCAL_Z_RATIO = -0.185;
const VESSEL_STERN_EXIT_LOCAL_Y_RATIO = 0;
const ENGINEERING_CAMERA_FOV_DEG = 22;
const TENSION_COLOR_STOPS = [
  { at: 0, color: new THREE.Color(0x004dff) },
  { at: 0.25, color: new THREE.Color(0x00a8ff) },
  { at: 0.5, color: new THREE.Color(0x00d85a) },
  { at: 0.75, color: new THREE.Color(0xffd11a) },
  { at: 1, color: new THREE.Color(0xff2a2a) },
];

export function DynamicFrameViewer({
  currentFrame: controlledCurrentFrame,
  frames,
  endpointLoads,
  onCurrentFrameChange,
  waterDepthM,
  summary,
  timelineLocked = false,
}: {
  currentFrame?: number;
  frames?: DynamicFramePlotData;
  endpointLoads?: FrameEndpointLoads;
  onCurrentFrameChange?: (frameIndex: number) => void;
  waterDepthM?: number;
  summary?: MotionSummary;
  timelineLocked?: boolean;
}) {
  const [internalCurrentFrame, setInternalCurrentFrame] = useState(0);
  const [isPlaying, setIsPlaying] = useState(false);
  const [viewMode, setViewMode] = useState<ViewMode>(summary ? "global" : "vessel");
  const canvasRef = useRef<HTMLCanvasElement | null>(null);
  const cameraStateRef = useRef<{ mode: ViewMode; position: THREE.Vector3; target: THREE.Vector3 } | null>(null);
  const onCurrentFrameChangeRef = useRef(onCurrentFrameChange);
  const summaryAvailableForInitializationRef = useRef(Boolean(summary));
  onCurrentFrameChangeRef.current = onCurrentFrameChange;
  summaryAvailableForInitializationRef.current = Boolean(summary);
  // 后端帧是缆型、端点和分段张力的唯一显示真值；summary 输入仅提供
  // 海流及矢量上下文，并为兼容帧提供全局 X 偏移。
  const frameItems = frames?.items ?? [];
  const dataSourceKey = frames
    ? JSON.stringify([frames.source, frames.label])
    : "__no_dynamic_frame_source__";
  const knownPloughBoundary = frameItems.some((frame) => frame.boundary === "known_plough_trajectory");
  const firstNodeCount = frameItems[0]?.points.length ?? 0;
  const currentFrame = controlledCurrentFrame ?? internalCurrentFrame;
  const activeFrameIndex = Math.min(Math.max(currentFrame, 0), Math.max(frameItems.length - 1, 0));
  const activeFrame = frameItems[activeFrameIndex];
  const activeCurrent = useMemo(
    () => summary ? currentStateAtTime(summary, activeFrame?.time_s ?? 0) : null,
    [activeFrame?.time_s, summary],
  );
  const activeOffset = frameOffset(activeFrame, summary, viewMode);
  const displayFrames = useMemo(
    () => frameItems.map((frame) => toDisplayFrame(frame, frameOffset(frame, summary, viewMode))),
    [frameItems, summary, viewMode],
  );
  const bounds = useMemo(() => frameBounds(displayFrames.flatMap((frame) => frame.points)), [displayFrames]);
  const depthTicks = useMemo(() => buildDepthTicks(displayFrames), [displayFrames]);
  // 整段时程共用同一量程，使拖动时间轴时的颜色仍具有物理可比性。
  const tensionRange = useMemo(() => frameTensionRange(frameTensionValues(frameItems)), [frameItems]);
  const activeSegmentTensions = activeFrame ? activeFrameTensions(activeFrame) : null;

  useEffect(() => {
    setInternalCurrentFrame(0);
    onCurrentFrameChangeRef.current?.(0);
    setIsPlaying(false);
    setViewMode(summaryAvailableForInitializationRef.current || knownPloughBoundary ? "global" : "vessel");
    cameraStateRef.current = null;
  }, [dataSourceKey, knownPloughBoundary]);

  useEffect(() => {
    if (!isPlaying || frameItems.length < 2) {
      return;
    }
    const maxFrameIndex = frameItems.length - 1;
    const id = window.setInterval(() => {
      if (controlledCurrentFrame === undefined) {
        setInternalCurrentFrame((value) => {
          const nextFrame = value >= maxFrameIndex ? 0 : value + 1;
          onCurrentFrameChangeRef.current?.(nextFrame);
          return nextFrame;
        });
      } else {
        onCurrentFrameChangeRef.current?.(activeFrameIndex >= maxFrameIndex ? 0 : activeFrameIndex + 1);
      }
    }, 220);
    return () => window.clearInterval(id);
  }, [activeFrameIndex, controlledCurrentFrame, frameItems.length, isPlaying]);

  const changeFrame = (frameIndex: number) => {
    const nextFrame = Math.min(Math.max(Math.round(frameIndex), 0), Math.max(frameItems.length - 1, 0));
    setInternalCurrentFrame(nextFrame);
    onCurrentFrameChangeRef.current?.(nextFrame);
  };

  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas || displayFrames.length === 0 || typeof WebGLRenderingContext === "undefined") {
      return;
    }

    let renderer: THREE.WebGLRenderer;
    try {
      renderer = new THREE.WebGLRenderer({ antialias: true, canvas, preserveDrawingBuffer: true });
    } catch {
      return;
    }

    const scene = new THREE.Scene();
    const surfaceDepth = activeOffset.z;
    const seabedDepth = displaySeabedDepth(waterDepthM, activeOffset.z, displayFrames);
    const seaSurfaceY = depthToSceneY(surfaceDepth, bounds);
    const seabedY = depthToSceneY(seabedDepth, bounds);
    scene.background = new THREE.Color(0xcfeeff);
    scene.fog = new THREE.Fog(0xcfeeff, bounds.span * 1.15, bounds.span * 4.6);

    const camera = new THREE.PerspectiveCamera(defaultCameraFovDeg(), 1, 0.1, bounds.span * 10);
    const defaultTarget = defaultCameraTarget(seaSurfaceY, seabedY);
    const storedCameraState = cameraStateRef.current?.mode === viewMode ? cameraStateRef.current : null;
    if (storedCameraState) {
      camera.position.copy(storedCameraState.position);
    } else {
      camera.position.copy(defaultCameraPosition(bounds));
    }
    camera.lookAt(storedCameraState?.target ?? defaultTarget);

    const waterMaterial = new THREE.MeshBasicMaterial({
      color: 0x8bc7ee,
      transparent: true,
      opacity: 0.42,
      side: THREE.DoubleSide,
    });
    const waterPlane = new THREE.Mesh(new THREE.PlaneGeometry(bounds.span * 2.8, bounds.span * 2.35), waterMaterial);
    waterPlane.rotation.x = -Math.PI / 2;
    waterPlane.position.y = seaSurfaceY;
    scene.add(waterPlane);
    const waterRipples = createWaterRipples(bounds.span, seaSurfaceY);
    scene.add(waterRipples);

    const seabedMaterial = new THREE.MeshBasicMaterial({
      color: 0xc8d8df,
      transparent: true,
      opacity: 0.26,
      side: THREE.DoubleSide,
    });
    const seabedPlane = new THREE.Mesh(new THREE.PlaneGeometry(bounds.span * 2.35, bounds.span * 2.2), seabedMaterial);
    seabedPlane.rotation.x = -Math.PI / 2;
    seabedPlane.position.y = seabedY - Math.max(bounds.span * 0.001, 0.04);
    scene.add(seabedPlane);

    const grid = new THREE.GridHelper(bounds.span * 2.2, 12, 0x7f9cac, 0xc1d4dc);
    grid.position.y = seabedY;
    setObjectOpacity(grid, 0.52);
    scene.add(grid);

    const depthGuide = createDepthGuide(bounds, seaSurfaceY, seabedY);
    scene.add(depthGuide);
    const currentArrow = activeCurrent ? createCurrentDirectionArrow(bounds, seaSurfaceY, activeCurrent.direction_deg) : null;
    if (currentArrow) {
      scene.add(currentArrow);
    }

    const light = new THREE.DirectionalLight(0xffffff, 1.4);
    light.position.set(bounds.span, bounds.span, bounds.span);
    scene.add(light);
    scene.add(new THREE.HemisphereLight(0xf5fbff, 0x55707d, 0.82));
    scene.add(new THREE.AmbientLight(0xffffff, 0.46));

    const active = displayFrames[activeFrameIndex] ?? displayFrames[0];
    const activeForwardDirection = vesselForwardDirection(displayFrames, activeFrameIndex, bounds);
    const activePloughForwardDirection = ploughForwardDirection(
      displayFrames,
      activeFrameIndex,
      bounds,
      activeForwardDirection,
    );
    const cableVisualPoints = buildCableVisualPoints(active, bounds, activeForwardDirection);
    const cableScenePoints = cableVisualPoints.map((point) => point.position);
    const segmentRadius = Math.max(Math.min(bounds.span * 0.0038, 0.9), 0.08);
    const segmentGeometry = new THREE.CylinderGeometry(segmentRadius, segmentRadius, 1, 12, 1);
    const segmentMaterials: THREE.MeshLambertMaterial[] = [];
    const segmentMeshes = Array.from({ length: Math.max(cableVisualPoints.length - 1, 0) }, () => {
      const material = new THREE.MeshLambertMaterial({ color: 0x1f7a8c, emissive: 0x032b38, emissiveIntensity: 0.22 });
      const mesh = new THREE.Mesh(segmentGeometry, material);
      segmentMaterials.push(material);
      scene.add(mesh);
      return mesh;
    });
    placeCableSegments({
      meshes: segmentMeshes,
      materials: segmentMaterials,
      visualPoints: cableVisualPoints,
      segmentTensions: active.segment_tensions_n,
      tensionRange,
    });

    const nodeGeometry = new THREE.SphereGeometry(Math.max(Math.min(bounds.span * 0.0036, 0.72), 0.06), 10, 8);
    const nodeMaterials: THREE.MeshLambertMaterial[] = [];
    const nodeMeshes = cableVisualPoints.map(() => {
      const material = new THREE.MeshLambertMaterial({ color: 0x1f7a8c, emissive: 0x032b38, emissiveIntensity: 0.26 });
      const mesh = new THREE.Mesh(nodeGeometry, material);
      nodeMaterials.push(material);
      scene.add(mesh);
      return mesh;
    });
    placeCableNodes({
      meshes: nodeMeshes,
      materials: nodeMaterials,
      visualPoints: cableVisualPoints,
      tensionRange,
    });

    const laidCablePoints = laidCableScenePoints(displayFrames, bounds, activeFrameIndex);
    const laidCableGroup = createTubePath(laidCablePoints, Math.max(Math.min(bounds.span * 0.003, 0.7), 0.06), 0x7a6248, 0.78);
    scene.add(laidCableGroup);

    const laidTrailGeometry = new THREE.BufferGeometry();
    const laidTrailMaterial = new THREE.LineBasicMaterial({ color: 0xb87817, transparent: true, opacity: 0.64 });
    if (laidCablePoints.length >= 2) {
      laidTrailGeometry.setFromPoints(laidCablePoints);
      scene.add(new THREE.Line(laidTrailGeometry, laidTrailMaterial));
    }

    const topTrailGeometry = new THREE.BufferGeometry();
    const topTrailMaterial = new THREE.LineBasicMaterial({ color: 0x236a8d, transparent: true, opacity: 0.22 });
    const topTrailPoints = displayFrames
      .filter((frame) => frame.points.length > 0)
      .map((frame) => cableOriginScenePoint(frame, bounds));
    if (viewMode === "global" && topTrailPoints.length >= 2) {
      topTrailGeometry.setFromPoints(topTrailPoints);
      scene.add(new THREE.Line(topTrailGeometry, topTrailMaterial));
    }

    const ghostGeometries: THREE.BufferGeometry[] = [];
    const ghostMaterials: THREE.LineBasicMaterial[] = [];
    const ghostGroup = new THREE.Group();
    sampledFrameIndexes(displayFrames.length, 6).forEach((frameIndex) => {
      const frame = displayFrames[frameIndex];
      const ghostPoints = frame ? buildCableVisualPoints(frame, bounds, vesselForwardDirection(displayFrames, frameIndex, bounds)).map((point) => point.position) : [];
      if (ghostPoints.length < 2) {
        return;
      }
      const ghostGeometry = new THREE.BufferGeometry().setFromPoints(ghostPoints);
      const ghostMaterial = new THREE.LineBasicMaterial({
        color: 0x607078,
        transparent: true,
        opacity: 0.18,
      });
      ghostGeometries.push(ghostGeometry);
      ghostMaterials.push(ghostMaterial);
      ghostGroup.add(new THREE.Line(ghostGeometry, ghostMaterial));
    });
    scene.add(ghostGroup);

    const topMarkerMaterial = new THREE.MeshBasicMaterial({ color: 0x26353b });
    const ploughMarkerMaterial = new THREE.MeshBasicMaterial({ color: 0xb43b32 });
    const topMarker = new THREE.Mesh(new THREE.SphereGeometry(Math.max(Math.min(bounds.span * 0.006, 1.1), 0.08), 12, 10), topMarkerMaterial);
    const ploughMarker = new THREE.Mesh(new THREE.SphereGeometry(Math.max(Math.min(bounds.span * 0.008, 1.4), 0.1), 12, 10), ploughMarkerMaterial);
    if (active.points.length > 0) {
      topMarker.position.copy(cableScenePoints[0]);
      ploughMarker.position.copy(mapPoint(active.points[active.points.length - 1], bounds));
    }
    scene.add(topMarker, ploughMarker);

    const vesselModel = createVesselModel(bounds.span);
    const vesselPosition = vesselPoint(active);
    if (vesselPosition) {
      vesselModel.rotation.y = vesselYawFromForwardDirection(activeForwardDirection);
      vesselModel.position.copy(vesselModelOriginFromSternPoint(mapPoint(vesselPosition, bounds), activeForwardDirection, bounds.span));
      scene.add(vesselModel);
    }

    const ploughModel = createPloughModel(bounds.span);
    const ploughPosition = ploughPoint(active);
    if (ploughPosition) {
      ploughModel.rotation.y = ploughYawFromForwardDirection(activePloughForwardDirection);
      ploughModel.position.copy(mapPoint(ploughPosition, bounds));
      ploughModel.position.y += ploughModelBottomOffset(bounds.span);
      scene.add(ploughModel);
    }

    const width = Math.max(320, canvas.clientWidth || canvas.parentElement?.clientWidth || 640);
    const height = Math.max(280, canvas.clientHeight || 360);
    renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
    renderer.setSize(width, height, false);
    camera.aspect = width / height;
    camera.updateProjectionMatrix();
    const controls = new OrbitControls(camera, renderer.domElement);
    controls.target.copy(storedCameraState?.target ?? defaultTarget);
    controls.enableDamping = true;
    controls.dampingFactor = 0.08;
    controls.enablePan = true;
    controls.screenSpacePanning = false;
    controls.minDistance = bounds.span * 0.45;
    controls.maxDistance = bounds.span * 4.2;
    controls.mouseButtons = {
      LEFT: THREE.MOUSE.ROTATE,
      MIDDLE: THREE.MOUSE.DOLLY,
      RIGHT: THREE.MOUSE.PAN,
    };

    const persistCameraState = () => {
      cameraStateRef.current = {
        mode: viewMode,
        position: camera.position.clone(),
        target: controls.target.clone(),
      };
    };
    controls.addEventListener("change", persistCameraState);
    let animationFrame = 0;
    const renderScene = () => {
      controls.update();
      renderer.render(scene, camera);
      animationFrame = window.requestAnimationFrame(renderScene);
    };
    renderScene();

    return () => {
      // 帧或视图依赖变化时会重建场景；此处释放全部 GPU 资源和监听器，
      // 防止长时间播放保留失效的几何体、材质、控制器或渲染循环。
      persistCameraState();
      controls.removeEventListener("change", persistCameraState);
      controls.dispose();
      window.cancelAnimationFrame(animationFrame);
      segmentGeometry.dispose();
      segmentMaterials.forEach((material) => material.dispose());
      nodeGeometry.dispose();
      nodeMaterials.forEach((material) => material.dispose());
      disposeObject3D(laidCableGroup);
      disposeObject3D(waterPlane);
      disposeObject3D(waterRipples);
      disposeObject3D(seabedPlane);
      disposeObject3D(grid);
      disposeObject3D(depthGuide);
      if (currentArrow) {
        disposeObject3D(currentArrow);
      }
      laidTrailGeometry.dispose();
      laidTrailMaterial.dispose();
      topTrailGeometry.dispose();
      topTrailMaterial.dispose();
      ghostGeometries.forEach((geometry) => geometry.dispose());
      ghostMaterials.forEach((material) => material.dispose());
      topMarker.geometry.dispose();
      ploughMarker.geometry.dispose();
      topMarkerMaterial.dispose();
      ploughMarkerMaterial.dispose();
      disposeObject3D(vesselModel);
      disposeObject3D(ploughModel);
      renderer.dispose();
    };
  }, [activeCurrent, activeFrameIndex, activeOffset.z, bounds, displayFrames, summary, tensionRange, viewMode, waterDepthM]);

  if (!frames || frameItems.length === 0 || firstNodeCount < 2) {
    return null;
  }

  return (
    <section className="dynamic-3d" aria-label="dynamic 3d frame section">
      <div className="dynamic-3d-header">
        <div>
          <h3>{frames.label}</h3>
          <span>{frameItems.length} 帧 / {firstNodeCount} 节点</span>
        </div>
        <strong>{activeFrame ? `${activeFrame.time_s.toFixed(1)} s` : "0.0 s"}</strong>
      </div>
      <div className="dynamic-view-controls" aria-label="三维视图控制">
        <button
          className={viewMode === "global" ? "active" : ""}
          disabled={!summary && !knownPloughBoundary}
          onClick={() => setViewMode("global")}
          type="button"
        >
          全局铺设视图
        </button>
        <button
          className={viewMode === "vessel" ? "active" : ""}
          onClick={() => setViewMode("vessel")}
          type="button"
        >
          船端固定视图
        </button>
        <span className="view-control-hint">鼠标拖拽旋转，滚轮缩放，右键平移</span>
      </div>
      <div className="dynamic-3d-viewport">
        <canvas aria-label={frames.label} className="dynamic-3d-canvas" ref={canvasRef} role="img" />
        <div className="scene-surface-label">海面</div>
        <div className="scene-seabed-label">海床网格</div>
        <div className="scene-depth-scale" aria-label="水深刻度">
          {depthTicks.map((tick) => (
            <span key={tick} style={{ top: `${depthTickPosition(tick, depthTicks)}%` }}>
              {formatDepthTick(tick)}
            </span>
          ))}
        </div>
        <div className="scene-layer-panel" aria-label="三维图层">
          <strong>张力 (kN)</strong>
          <div className="scene-gradient-meter" aria-hidden="true">
            <span className="scene-gradient-bar" />
            <span>{formatKiloNewton(tensionRange.max)}</span>
            <span>{formatKiloNewton(tensionRange.min)}</span>
          </div>
          <span><i className="object-swatch vessel" />铺缆船</span>
          <span><i className="object-swatch stern" />船尾出缆</span>
          <span><i className="object-swatch bow" />船首向前</span>
          <span><i className="object-swatch cable" />缆线</span>
          <span><i className="object-swatch laid-cable" />犁入口运动轨迹</span>
          <span><i className="object-swatch nodes" />离散节点</span>
          <span><i className="object-swatch plough" />埋设犁</span>
          <span><i className="object-swatch seabed" />海床</span>
        </div>
        {activeCurrent ? (
          <div className="current-direction-panel" aria-label="海流去向">
            <strong>{activeCurrent.is_time_varying ? "当前帧海流去向" : "海流去向"}</strong>
            <span>{formatCurrentDirection(activeCurrent)}</span>
            <i
              aria-hidden="true"
              className="current-direction-arrow"
              style={{ transform: `rotate(${normalizeDegrees(activeCurrent.direction_deg)}deg)` }}
            />
          </div>
        ) : null}
      </div>
      {timelineLocked ? (
        <div className="time-scrubber"><span>实时视图锁定最新帧</span></div>
      ) : (
        <div className="time-scrubber">
          <button onClick={() => setIsPlaying((value) => !value)} type="button">
            {isPlaying ? "暂停" : "播放"}
          </button>
          <label>
            <span>时间轴</span>
            <input
              aria-label="时间轴"
              max={Math.max(frameItems.length - 1, 0)}
              min={0}
              onChange={(event) => {
                setIsPlaying(false);
                changeFrame(Number(event.currentTarget.value));
              }}
              step={1}
              type="range"
              value={activeFrameIndex}
            />
          </label>
        </div>
      )}
      <div className="dynamic-frame-readout" aria-label="当前帧数据">
        <span>
          {knownPloughBoundary
            ? `船位 X=${formatMeters(displayFrames[activeFrameIndex]?.vessel_x_m ?? 0)}`
            : viewMode === "global"
              ? `船位 X=${formatMeters(activeOffset.x)}`
              : "船位随船坐标固定"}
        </span>
        {knownPloughBoundary ? <span>犁入口 X={formatMeters(displayFrames[activeFrameIndex]?.plough_x_m ?? 0)}</span> : null}
        {endpointLoads || activeSegmentTensions ? (
          <>
            {endpointLoads ? <span>船端轴向端载荷 {formatKiloNewton(endpointLoads.top_tension_n)}</span> : null}
            {activeSegmentTensions ? <span>中段轴向约束张力 {formatKiloNewton(activeSegmentTensions.mid)}</span> : null}
            {endpointLoads?.plough_boundary_tension_n !== undefined
              ? <span>犁入口轴向端载荷 {formatKiloNewton(endpointLoads.plough_boundary_tension_n)}</span>
              : null}
            {endpointLoads?.plough_inlet_tension_n !== undefined
              ? <span>犁前末段张力 {formatKiloNewton(endpointLoads.plough_inlet_tension_n)}</span>
              : null}
          </>
        ) : null}
      </div>
      <div className="dynamic-3d-footer">
        <div className="tension-legend" aria-label="张力颜色">
          <strong>张力颜色</strong>
          <span>低 {formatKiloNewton(tensionRange.min)}</span>
          <span className="tension-gradient" aria-hidden="true" />
          <span>高 {formatKiloNewton(tensionRange.max)}</span>
        </div>
        <BoundaryTrajectory frames={displayFrames} label="犁入口运动轨迹" />
      </div>
    </section>
  );
}

function toDisplayFrame(frame: DynamicFrame, offset: typeof ZERO_OFFSET): DynamicFrame {
  // 通过偏移实现全局视图和跟船视图，不修改后端数据。
  return {
    ...frame,
    vessel_x_m: withOffset(frame.vessel_x_m, offset.x),
    vessel_y_m: withOffset(frame.vessel_y_m, offset.y),
    vessel_z_m: withOffset(frame.vessel_z_m, offset.z),
    plough_x_m: withOffset(frame.plough_x_m, offset.x),
    plough_y_m: withOffset(frame.plough_y_m, offset.y),
    plough_z_m: withOffset(frame.plough_z_m, offset.z),
    points: frame.points.map((point) => ({
      ...point,
      x_m: point.x_m + offset.x,
      y_m: point.y_m + offset.y,
      z_m: point.z_m + offset.z,
    })),
  };
}

function withOffset(value: number | undefined, offset: number): number | undefined {
  return value === undefined ? undefined : value + offset;
}

function frameOffset(frame: DynamicFrame | undefined, summary: MotionSummary | undefined, viewMode: ViewMode) {
  if (!frame) {
    return ZERO_OFFSET;
  }
  if (frame.boundary === "known_plough_trajectory") {
    if (viewMode !== "vessel") {
      return ZERO_OFFSET;
    }
    return {
      x: -(frame.vessel_x_m ?? 0),
      y: -(frame.vessel_y_m ?? 0),
      z: -(frame.vessel_z_m ?? 0),
    };
  }
  if (!summary || viewMode !== "global") {
    return ZERO_OFFSET;
  }
  return { x: vesselTravelM(summary, frame.time_s), y: 0, z: 0 };
}

function vesselTravelM(summary: MotionSummary, timeS: number): number {
  const duration = Math.max(summary.transition_duration_s, 0);
  if (duration <= 1.0e-12) {
    return summary.vessel_final_speed_mps * Math.max(timeS, 0);
  }
  const rampTime = Math.min(Math.max(timeS, 0), duration);
  const delta = summary.vessel_final_speed_mps - summary.vessel_initial_speed_mps;
  const rampDistance = summary.vessel_initial_speed_mps * rampTime + 0.5 * delta * rampTime * rampTime / duration;
  const afterRampDistance = Math.max(timeS - duration, 0) * summary.vessel_final_speed_mps;
  return rampDistance + afterRampDistance;
}

function placeCableSegments({
  meshes,
  materials,
  visualPoints,
  segmentTensions,
  tensionRange,
}: {
  meshes: THREE.Mesh[];
  materials: THREE.MeshLambertMaterial[];
  visualPoints: CableVisualPoint[];
  segmentTensions?: number[];
  tensionRange: TensionRange;
}) {
  const verticalAxis = new THREE.Vector3(0, 1, 0);
  meshes.forEach((mesh, index) => {
    const startPoint = visualPoints[index];
    const endPoint = visualPoints[index + 1];
    if (!startPoint || !endPoint) {
      mesh.visible = false;
      return;
    }
    const start = startPoint.position;
    const end = endPoint.position;
    const direction = end.clone().sub(start);
    const length = direction.length();
    if (length <= 1.0e-6) {
      mesh.visible = false;
      return;
    }
    const tension = segmentTensions?.[startPoint.segmentIndex] ?? (startPoint.tension_n + endPoint.tension_n) / 2;
    materials[index].color.copy(tensionColor(tension, tensionRange));
    mesh.visible = true;
    mesh.position.copy(start.clone().add(end).multiplyScalar(0.5));
    mesh.quaternion.setFromUnitVectors(verticalAxis, direction.normalize());
    mesh.scale.set(1, length, 1);
  });
}

function placeCableNodes({
  meshes,
  materials,
  visualPoints,
  tensionRange,
}: {
  meshes: THREE.Mesh[];
  materials: THREE.MeshLambertMaterial[];
  visualPoints: CableVisualPoint[];
  tensionRange: TensionRange;
}) {
  meshes.forEach((mesh, index) => {
    const point = visualPoints[index];
    if (!point) {
      mesh.visible = false;
      return;
    }
    if (point.showNode === false) {
      mesh.visible = false;
      return;
    }
    mesh.visible = true;
    mesh.position.copy(point.position);
    materials[index].color.copy(tensionColor(point.tension_n, tensionRange));
  });
}

function createTubePath(points: THREE.Vector3[], radius: number, color: number, opacity: number): THREE.Group {
  const group = new THREE.Group();
  if (points.length < 2) {
    return group;
  }
  const verticalAxis = new THREE.Vector3(0, 1, 0);
  points.slice(0, -1).forEach((start, index) => {
    const end = points[index + 1];
    const direction = end.clone().sub(start);
    const length = direction.length();
    if (length <= 1.0e-6) {
      return;
    }
    const material = new THREE.MeshLambertMaterial({
      color,
      transparent: true,
      opacity,
      emissive: 0x20160e,
      emissiveIntensity: 0.08,
    });
    const mesh = new THREE.Mesh(new THREE.CylinderGeometry(radius, radius, 1, 10, 1), material);
    mesh.position.copy(start.clone().add(end).multiplyScalar(0.5));
    mesh.quaternion.setFromUnitVectors(verticalAxis, direction.normalize());
    mesh.scale.set(1, length, 1);
    group.add(mesh);
  });
  return group;
}

function frameBounds(points: DynamicFramePoint[]): Bounds {
  if (points.length === 0) {
    return { centerX: 0, centerY: 0, centerZ: 0, span: 1 };
  }
  const xs = points.map((point) => point.x_m);
  const ys = points.map((point) => point.y_m);
  const zs = points.map((point) => point.z_m);
  const minX = Math.min(...xs);
  const maxX = Math.max(...xs);
  const minY = Math.min(...ys);
  const maxY = Math.max(...ys);
  const minZ = Math.min(...zs);
  const maxZ = Math.max(...zs);
  return {
    centerX: (minX + maxX) / 2,
    centerY: (minY + maxY) / 2,
    centerZ: (minZ + maxZ) / 2,
    span: Math.max(maxX - minX, maxY - minY, maxZ - minZ, 1),
  };
}

function frameMaxDepth(frames: DynamicFramePlotData["items"]): number {
  const depths = frames.flatMap((frame) => [
    ...frame.points.map((point) => point.z_m),
    frame.vessel_z_m ?? 0,
    frame.plough_z_m ?? 0,
  ]);
  return Math.max(1, ...depths.filter((depth) => Number.isFinite(depth)));
}

function displaySeabedDepth(
  waterDepthM: number | undefined,
  displayOffsetZ: number,
  frames: DynamicFramePlotData["items"],
): number {
  return waterDepthM !== undefined && Number.isFinite(waterDepthM)
    ? waterDepthM + displayOffsetZ
    : frameMaxDepth(frames);
}

function buildDepthTicks(frames: DynamicFramePlotData["items"]): number[] {
  const maxDepth = frameMaxDepth(frames);
  const step = maxDepth <= 120 ? 25 : maxDepth <= 320 ? 50 : 100;
  const maxTick = Math.max(step, Math.ceil(maxDepth / step) * step);
  const tickCount = Math.min(Math.floor(maxTick / step) + 1, 7);
  if (tickCount <= 1) {
    return [0, step];
  }
  return Array.from({ length: tickCount }, (_, index) => Math.round((index * maxTick) / (tickCount - 1)));
}

function depthTickPosition(tick: number, ticks: number[]): number {
  const maxTick = Math.max(...ticks, 1);
  return 8 + (tick / maxTick) * 84;
}

function formatDepthTick(tick: number): string {
  return `${tick} m`;
}

function mapPoint(point: DynamicFramePoint, bounds: Bounds): THREE.Vector3 {
  // 工程坐标为 +X 向前、+Y 右舷、+Z 向下；Three.js 场景采用
  // (X,Y,Z)=(x,-z,y)，使水深方向在画面中向下。
  return new THREE.Vector3(
    point.x_m - bounds.centerX,
    -(point.z_m - bounds.centerZ),
    point.y_m - bounds.centerY,
  );
}

function depthToSceneY(depthM: number, bounds: Bounds): number {
  return -(depthM - bounds.centerZ);
}

function defaultCameraTarget(surfaceY: number, seabedY: number): THREE.Vector3 {
  return new THREE.Vector3(0, surfaceY + (seabedY - surfaceY) * 0.46, 0);
}

function defaultCameraFovDeg(): number {
  return ENGINEERING_CAMERA_FOV_DEG;
}

function defaultCameraPosition(bounds: Bounds): THREE.Vector3 {
  return new THREE.Vector3(bounds.span * 0.18, bounds.span * 0.68, bounds.span * 2.55);
}

function currentDirectionSceneVector(directionDeg: number): THREE.Vector3 {
  const rad = (normalizeDegrees(directionDeg) * Math.PI) / 180;
  return new THREE.Vector3(Math.cos(rad), 0, Math.sin(rad)).normalize();
}

function createCurrentDirectionArrow(bounds: Bounds, surfaceY: number, directionDeg: number): THREE.ArrowHelper {
  const direction = currentDirectionSceneVector(directionDeg);
  const length = Math.max(bounds.span * 0.3, 12);
  const origin = new THREE.Vector3(-bounds.span * 0.68, surfaceY + Math.max(bounds.span * 0.018, 0.4), -bounds.span * 0.44);
  return new THREE.ArrowHelper(direction, origin, length, 0x0b7895, length * 0.22, length * 0.1);
}

function vesselModelScale(span: number): number {
  return Math.min(Math.max(span * 0.24, 28), 88);
}

function cableOriginScenePoint(frame: DynamicFrame, bounds: Bounds): THREE.Vector3 {
  const origin = vesselPoint(frame) ?? frame.points[0];
  return mapPoint(origin, bounds);
}

function buildCableVisualPoints(frame: DynamicFrame, bounds: Bounds, forwardDir: THREE.Vector3): CableVisualPoint[] {
  void forwardDir;
  const lastSegmentIndex = Math.max(frame.segment_tensions_n.length - 1, 0);
  return frame.points.map((point, index) => {
    const segmentIndex = Math.min(index, lastSegmentIndex);
    return {
      position: mapPoint(point, bounds),
      // 缆线颜色以分段张力为准；仅当帧缺少分段字段时，才兼容回退到节点张力。
      tension_n: frame.segment_tensions_n[segmentIndex] ?? point.tension_n,
      segmentIndex,
    };
  });
}

function laidCableScenePoints(frames: DynamicFrame[], bounds: Bounds, activeFrameIndex: number): THREE.Vector3[] {
  const lastFrameIndex = Math.min(Math.max(activeFrameIndex, 0), Math.max(frames.length - 1, 0));
  return frames
    .slice(0, lastFrameIndex + 1)
    .map((frame) => ploughPoint(frame))
    .filter((point): point is DynamicFramePoint => Boolean(point))
    .map((point) => mapPoint(point, bounds));
}

function sternScenePointFromForwardDirection(origin: THREE.Vector3, forwardDir: THREE.Vector3, span: number): THREE.Vector3 {
  const forward = normalizedHorizontal(forwardDir);
  const scenePoint = origin.clone();
  const scale = vesselModelScale(span);
  scenePoint.y += scale * VESSEL_STERN_EXIT_LOCAL_Y_RATIO;
  scenePoint.x -= forward.x * Math.abs(VESSEL_STERN_EXIT_LOCAL_Z_RATIO) * scale;
  scenePoint.z -= forward.z * Math.abs(VESSEL_STERN_EXIT_LOCAL_Z_RATIO) * scale;
  return scenePoint;
}

function vesselModelOriginFromSternPoint(sternPoint: THREE.Vector3, forwardDir: THREE.Vector3, span: number): THREE.Vector3 {
  const forward = normalizedHorizontal(forwardDir);
  const origin = sternPoint.clone();
  const scale = vesselModelScale(span);
  origin.y -= scale * VESSEL_STERN_EXIT_LOCAL_Y_RATIO;
  origin.x += forward.x * Math.abs(VESSEL_STERN_EXIT_LOCAL_Z_RATIO) * scale;
  origin.z += forward.z * Math.abs(VESSEL_STERN_EXIT_LOCAL_Z_RATIO) * scale;
  return origin;
}

function vesselForwardDirection(frames: DynamicFrame[], activeIndex: number, bounds: Bounds): THREE.Vector3 {
  return endpointForwardDirection(frames, activeIndex, bounds, vesselPoint, new THREE.Vector3(1, 0, 0));
}

function ploughForwardDirection(
  frames: DynamicFrame[],
  activeIndex: number,
  bounds: Bounds,
  fallback: THREE.Vector3,
): THREE.Vector3 {
  return endpointForwardDirection(frames, activeIndex, bounds, ploughPoint, fallback);
}

function endpointForwardDirection(
  frames: DynamicFrame[],
  activeIndex: number,
  bounds: Bounds,
  pointForFrame: (frame: DynamicFrame) => DynamicFramePoint | undefined,
  fallback: THREE.Vector3,
): THREE.Vector3 {
  const active = frames[activeIndex];
  if (!active) {
    return fallback.clone();
  }
  const origin = mapPoint(pointForFrame(active) ?? active.points[0], bounds);
  for (let offset = 1; offset < frames.length; offset += 1) {
    const nextFrame = frames[activeIndex + offset];
    if (nextFrame) {
      const nextDirection = mapPoint(pointForFrame(nextFrame) ?? nextFrame.points[0], bounds).sub(origin);
      nextDirection.y = 0;
      if (nextDirection.lengthSq() > 1.0e-10) {
        return nextDirection.normalize();
      }
    }
    const previousFrame = frames[activeIndex - offset];
    if (previousFrame) {
      const previousDirection = origin.clone().sub(mapPoint(pointForFrame(previousFrame) ?? previousFrame.points[0], bounds));
      previousDirection.y = 0;
      if (previousDirection.lengthSq() > 1.0e-10) {
        return previousDirection.normalize();
      }
    }
  }
  return fallback.clone();
}

function vesselYawFromForwardDirection(forwardDir: THREE.Vector3): number {
  const forward = normalizedHorizontal(forwardDir);
  return Math.atan2(forward.x, forward.z);
}

function ploughYawFromForwardDirection(forwardDir: THREE.Vector3): number {
  return vesselYawFromForwardDirection(forwardDir) - Math.PI / 2;
}

function normalizedHorizontal(direction: THREE.Vector3): THREE.Vector3 {
  const horizontal = new THREE.Vector3(direction.x, 0, direction.z);
  if (horizontal.lengthSq() <= 1.0e-12) {
    return new THREE.Vector3(0, 0, 1);
  }
  return horizontal.normalize();
}

function vesselPoint(frame: DynamicFrame): DynamicFramePoint | undefined {
  const fallback = frame.points[0];
  if (frame.vessel_x_m === undefined || frame.vessel_y_m === undefined) {
    return fallback;
  }
  return {
    index: -1,
    x_m: frame.vessel_x_m,
    y_m: frame.vessel_y_m,
    z_m: frame.vessel_z_m ?? 0,
    tension_n: fallback?.tension_n ?? 0,
  };
}

function ploughPoint(frame: DynamicFrame): DynamicFramePoint | undefined {
  const fallback = frame.points[frame.points.length - 1];
  if (frame.plough_x_m === undefined || frame.plough_y_m === undefined) {
    return fallback;
  }
  return {
    index: -2,
    x_m: frame.plough_x_m,
    y_m: frame.plough_y_m,
    z_m: frame.plough_z_m ?? fallback?.z_m ?? 0,
    tension_n: fallback?.tension_n ?? 0,
  };
}

function createWaterRipples(span: number, surfaceY: number): THREE.Group {
  const group = new THREE.Group();
  const rippleCount = 16;
  const width = span * 2.35;
  const depth = span * 1.9;
  for (let index = 0; index < rippleCount; index += 1) {
    const yOffset = Math.sin(index * 0.9) * span * 0.003;
    const z = -depth / 2 + (index / Math.max(rippleCount - 1, 1)) * depth;
    const geometry = new THREE.BufferGeometry().setFromPoints([
      new THREE.Vector3(-width / 2, surfaceY + yOffset, z),
      new THREE.Vector3(width / 2, surfaceY + yOffset, z + Math.sin(index) * span * 0.04),
    ]);
    const material = new THREE.LineBasicMaterial({
      color: 0xf2fbff,
      transparent: true,
      opacity: 0.24,
    });
    group.add(new THREE.Line(geometry, material));
  }
  return group;
}

function createDepthGuide(bounds: Bounds, surfaceY: number, seabedY: number): THREE.LineSegments {
  const span = bounds.span;
  const x = -span * 1.08;
  const z = -span * 0.92;
  const tickWidth = span * 0.14;
  const positions: number[] = [];
  const addLine = (startX: number, startY: number, startZ: number, endX: number, endY: number, endZ: number) => {
    positions.push(startX, startY, startZ, endX, endY, endZ);
  };
  addLine(x, surfaceY, z, x, seabedY, z);
  for (let index = 0; index <= 5; index += 1) {
    const y = surfaceY + ((seabedY - surfaceY) * index) / 5;
    addLine(x, y, z, x + tickWidth, y, z);
  }
  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute("position", new THREE.Float32BufferAttribute(positions, 3));
  const material = new THREE.LineBasicMaterial({
    color: 0x6c8ea3,
    transparent: true,
    opacity: 0.42,
  });
  return new THREE.LineSegments(geometry, material);
}

function createVesselModel(span: number): THREE.Group {
  const group = new THREE.Group();
  const scale = vesselModelScale(span);
  const hull = new THREE.Mesh(
    new THREE.BoxGeometry(scale * 0.082, scale * 0.045, scale * 0.32),
    new THREE.MeshLambertMaterial({ color: 0x0c7a68 }),
  );
  const bow = new THREE.Mesh(
    new THREE.ConeGeometry(scale * 0.043, scale * 0.08, 4),
    new THREE.MeshLambertMaterial({ color: 0x0a6c5f }),
  );
  const deck = new THREE.Mesh(
    new THREE.BoxGeometry(scale * 0.072, scale * 0.018, scale * 0.22),
    new THREE.MeshLambertMaterial({ color: 0xe8f1ef }),
  );
  const cabin = new THREE.Mesh(
    new THREE.BoxGeometry(scale * 0.06, scale * 0.075, scale * 0.09),
    new THREE.MeshLambertMaterial({ color: 0xf3f7f4 }),
  );
  const bridge = new THREE.Mesh(
    new THREE.BoxGeometry(scale * 0.048, scale * 0.044, scale * 0.065),
    new THREE.MeshLambertMaterial({ color: 0xd8e3e8 }),
  );
  const boom = new THREE.Mesh(
    new THREE.BoxGeometry(scale * 0.16, scale * 0.013, scale * 0.013),
    new THREE.MeshLambertMaterial({ color: 0xcfd9df }),
  );
  const mast = new THREE.Mesh(
    new THREE.BoxGeometry(scale * 0.012, scale * 0.1, scale * 0.012),
    new THREE.MeshLambertMaterial({ color: 0xb9c7cf }),
  );
  const sternRoller = new THREE.Mesh(
    new THREE.CylinderGeometry(scale * 0.015, scale * 0.015, scale * 0.085, 16),
    new THREE.MeshLambertMaterial({ color: 0xf07a28 }),
  );
  bow.rotation.x = Math.PI / 2;
  bow.position.set(0, 0, scale * 0.185);
  deck.position.set(0, scale * 0.038, 0);
  cabin.position.set(0, scale * 0.086, scale * -0.055);
  bridge.position.set(0, scale * 0.145, scale * -0.068);
  boom.rotation.x = -0.62;
  boom.position.set(0, scale * 0.125, scale * -0.075);
  mast.position.set(0, scale * 0.19, scale * -0.1);
  sternRoller.rotation.z = Math.PI / 2;
  sternRoller.position.set(0, scale * VESSEL_STERN_EXIT_LOCAL_Y_RATIO, scale * VESSEL_STERN_EXIT_LOCAL_Z_RATIO);
  group.add(hull, bow, deck, cabin, bridge, boom, mast, sternRoller);
  return group;
}

function createPloughModel(span: number): THREE.Group {
  const group = new THREE.Group();
  const scale = Math.max(span, 20);
  const body = new THREE.Mesh(
    new THREE.BoxGeometry(scale * 0.082, scale * 0.03, scale * 0.046),
    new THREE.MeshLambertMaterial({ color: 0x8b6d4e }),
  );
  const blade = new THREE.Mesh(
    new THREE.ConeGeometry(scale * 0.026, scale * 0.065, 4),
    new THREE.MeshLambertMaterial({ color: 0x5b6670 }),
  );
  const leftSkid = new THREE.Mesh(
    new THREE.BoxGeometry(scale * 0.095, scale * 0.008, scale * 0.007),
    new THREE.MeshLambertMaterial({ color: 0x4e5a62 }),
  );
  const rightSkid = leftSkid.clone();
  const towBar = new THREE.Mesh(
    new THREE.BoxGeometry(scale * 0.075, scale * 0.009, scale * 0.009),
    new THREE.MeshLambertMaterial({ color: 0x606b73 }),
  );
  blade.rotation.z = Math.PI / 2;
  blade.position.x = scale * 0.045;
  leftSkid.position.set(0, -scale * 0.022, scale * 0.03);
  rightSkid.position.set(0, -scale * 0.022, -scale * 0.03);
  towBar.rotation.z = 0.26;
  towBar.position.set(-scale * 0.06, scale * 0.012, 0);
  group.add(body, blade, leftSkid, rightSkid, towBar);
  return group;
}

function ploughModelBottomOffset(span: number): number {
  return Math.max(span, 20) * 0.026;
}

function setObjectOpacity(object: THREE.Object3D, opacity: number) {
  object.traverse((child) => {
    const materials = objectMaterials(child);
    materials.forEach((material) => {
      material.transparent = true;
      material.opacity = opacity;
    });
  });
}

function disposeObject3D(object: THREE.Object3D) {
  object.traverse((child) => {
    const renderable = child as THREE.Object3D & { geometry?: THREE.BufferGeometry };
    renderable.geometry?.dispose();
    objectMaterials(child).forEach((material) => material.dispose());
  });
}

function objectMaterials(object: THREE.Object3D): THREE.Material[] {
  const candidate = object as THREE.Object3D & { material?: THREE.Material | THREE.Material[] };
  if (!candidate.material) {
    return [];
  }
  return Array.isArray(candidate.material) ? candidate.material : [candidate.material];
}

function frameTensionValues(frames: DynamicFramePlotData["items"]): number[] {
  // 优先使用 XPBD 分段字段；端点节点值可能表示控制体边界支反力，
  // 与分段张力的物理含义不同。
  return frames.flatMap((frame) =>
    frame.segment_tensions_n && frame.segment_tensions_n.length > 0
      ? frame.segment_tensions_n
      : frame.points.map((point) => point.tension_n),
  );
}

function frameTensionRange(values: number[]): TensionRange {
  if (values.length === 0) {
    return { min: 0, max: 0 };
  }
  return {
    min: Math.min(...values),
    max: Math.max(...values),
  };
}

function activeFrameTensions(frame: DynamicFrame) {
  const tensions = frame.segment_tensions_n;
  if (tensions.length === 0) {
    return null;
  }
  return {
    mid: tensions[Math.floor((tensions.length - 1) / 2)],
  };
}

function tensionColor(value: number, range: TensionRange): THREE.Color {
  const span = Math.max(range.max - range.min, 1);
  const ratio = Math.min(1, Math.max(0, (value - range.min) / span));
  return colorFromStops(ratio, TENSION_COLOR_STOPS);
}

function colorFromStops(ratio: number, stops: typeof TENSION_COLOR_STOPS): THREE.Color {
  const clamped = Math.min(1, Math.max(0, ratio));
  const first = stops[0];
  if (!first || clamped <= first.at) {
    return first?.color.clone() ?? new THREE.Color(0x004dff);
  }
  for (let index = 1; index < stops.length; index += 1) {
    const previous = stops[index - 1];
    const next = stops[index];
    if (clamped <= next.at) {
      const localRatio = (clamped - previous.at) / Math.max(next.at - previous.at, 1.0e-6);
      return previous.color.clone().lerp(next.color, localRatio);
    }
  }
  return stops[stops.length - 1]?.color.clone() ?? new THREE.Color(0xff2a2a);
}

function formatKiloNewton(value: number): string {
  return `${(value / 1000).toFixed(2)} kN`;
}

function formatMeters(value: number): string {
  return `${value.toFixed(2)} m`;
}

function formatCurrentDirection(current: { direction_deg: number; speed_mps: number }): string {
  return `${normalizeDegrees(current.direction_deg).toFixed(0)}° / ${current.speed_mps.toFixed(2)} m/s`;
}

function normalizeDegrees(value: number): number {
  return ((value % 360) + 360) % 360;
}

function sampledFrameIndexes(frameCount: number, targetCount: number): number[] {
  if (frameCount <= 0) {
    return [];
  }
  if (frameCount <= targetCount) {
    return Array.from({ length: frameCount }, (_, index) => index);
  }
  const indexes = new Set<number>();
  for (let index = 0; index < targetCount; index += 1) {
    indexes.add(Math.round((index * (frameCount - 1)) / (targetCount - 1)));
  }
  return [...indexes].sort((left, right) => left - right);
}

function BoundaryTrajectory({ frames, label }: { frames: DynamicFramePlotData["items"]; label: string }) {
  const points = frames
    .map((frame) => ploughPoint(frame))
    .filter((point): point is DynamicFramePoint => Boolean(point));

  if (points.length < 2) {
    return (
      <div className="dynamic-trajectory" aria-label={label}>
        <span>{label}</span>
        <strong>点数不足</strong>
      </div>
    );
  }

  const xs = points.map((point) => point.x_m);
  const ys = points.map((point) => point.y_m);
  const minX = Math.min(...xs);
  const maxX = Math.max(...xs);
  const minY = Math.min(...ys);
  const maxY = Math.max(...ys);
  const spanX = Math.max(maxX - minX, 1);
  const spanY = Math.max(maxY - minY, 1);
  const mapX = (value: number) => 10 + ((value - minX) / spanX) * 110;
  const mapY = (value: number) => 46 - ((value - minY) / spanY) * 36;
  const polyline = points.map((point) => `${mapX(point.x_m).toFixed(1)},${mapY(point.y_m).toFixed(1)}`).join(" ");
  const start = points[0];
  const end = points[points.length - 1];

  return (
    <div className="dynamic-trajectory" aria-label={label}>
      <span>{label}</span>
      <svg viewBox="0 0 130 56" role="img" aria-label={`${label}投影`}>
        <polyline points={polyline} />
        <circle cx={mapX(start.x_m)} cy={mapY(start.y_m)} r="3" className="trajectory-start" />
        <circle cx={mapX(end.x_m)} cy={mapY(end.y_m)} r="3.6" className="trajectory-end" />
      </svg>
    </div>
  );
}

export const dynamicFrameGeometryForTest = {
  buildCableVisualPoints,
  laidCableScenePoints,
  vesselYawFromForwardDirection,
  ploughYawFromForwardDirection,
  sternScenePointFromForwardDirection,
  defaultCameraFovDeg,
  defaultCameraPosition,
  tensionColor,
  mapPoint,
  currentDirectionSceneVector,
  vesselModelOriginFromSternPoint,
  vesselForwardDirection,
  displaySeabedDepth,
  ploughModelBottomOffset,
};
