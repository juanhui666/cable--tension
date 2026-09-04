/**
 * 规范 /api/v1 JSON 合同的前端镜像。
 * 单位编码在字段后缀中；工程坐标为 +X 向前、+Y 右舷、+Z 向下。
 */
export interface TimeHistoryCaseInputs {
  case_name: string;
  diameter_m: number;
  weight_air_n_per_m: number;
  submerged_weight_n_per_m: number;
  tangential_drag_coefficient: number;
  normal_drag_coefficient: number;
  axial_stiffness_n: number;
  current_speed_mps: number;
  current_bottom_speed_mps?: number | null;
  current_profile_exponent?: number;
  integration_time_step_max_s?: number | null;
  current_direction_deg: number;
  speed_change: "steady" | "accel" | "decel";
  vessel_initial_speed_mps: number;
  vessel_final_speed_mps: number;
  payout_initial_speed_mps?: number | null;
  payout_final_speed_mps?: number | null;
  length_boundary_source: "known_plough_trajectory";
  transition_duration_s: number;
  total_duration_s: number;
  water_depth_m: number;
  element_count: number;
  vessel_initial_x_m?: number;
  vessel_initial_y_m?: number;
  vessel_heading_deg?: number;
  plough_initial_x_m: number | null;
  plough_initial_y_m: number | null;
  plough_initial_z_m: number | null;
  initial_suspended_length_m: number | null;
  min_bending_radius_m?: number | null;
  vessel_motion_segments?: MotionSegmentInput[];
  vessel_motion_samples?: MotionSampleInput[];
  payout_speed_segments?: SpeedSegmentInput[];
}

export interface MotionSegmentInput {
  duration_s: number;
  start_speed_mps: number;
  end_speed_mps: number;
  heading_deg: number;
  interpolation?: "linear" | "smootherstep" | "sampled_smootherstep";
  sample_interval_s?: number;
  start_velocity_x_mps?: number;
  start_velocity_y_mps?: number;
  end_velocity_x_mps?: number;
  end_velocity_y_mps?: number;
}

export interface SpeedSegmentInput {
  duration_s: number;
  start_speed_mps: number;
  end_speed_mps: number;
  interpolation?: "linear" | "smootherstep" | "sampled_smootherstep";
  sample_interval_s?: number;
}

export interface CurrentVelocitySegmentInput {
  duration_s: number;
  interpolation?: "cartesian_linear" | "polar_unwrapped";
  start_velocity_x_mps?: number;
  start_velocity_y_mps?: number;
  end_velocity_x_mps?: number;
  end_velocity_y_mps?: number;
  start_speed_mps?: number;
  end_speed_mps?: number;
  start_direction_unwrapped_deg?: number;
  end_direction_unwrapped_deg?: number;
}

export interface CasePreset {
  id: string;
  label: string;
  purpose: string;
  category: "engineering_accuracy" | "realtime_extreme";
  mode: "batch" | "realtime" | "realtime_only";
  caution?: string;
  suggested_output_dir: string;
  inputs: TimeHistoryCaseInputs;
}

export interface MotionSampleInput {
  time_s: number;
  x_m: number;
  y_m: number;
  z_m?: number | null;
  velocity_x_mps?: number | null;
  velocity_y_mps?: number | null;
  velocity_z_mps?: number | null;
}

export interface RealtimeMotionSampleInput {
  time_s: number;
  x_m: number;
  y_m: number;
  z_m?: number;
  velocity_x_mps?: number;
  velocity_y_mps?: number;
  velocity_z_mps?: number;
}

export interface TimeHistoryCase {
  name: string;
  label: string;
  description: string;
  group: string;
  example: boolean;
  display_order?: number;
  suggested_output_dir: string;
  inputs: TimeHistoryCaseInputs;
}

export interface TensionFieldMetadata {
  engineering_name: string;
  unit: "N";
  description: string;
  comparison_mouth?: string;
}

export interface ResultFieldMetadata {
  tensions: Record<string, TensionFieldMetadata>;
}

export interface TimeHistoryPlotPoint {
  time_s: number;
  top_tension_n: number;
  has_contact: boolean;
  contact_transition_x_m: number | null;
  contact_transition_y_m: number | null;
  suspended_length_m?: number;
  material_suspended_length_m?: number;
  geometric_length_deficit_m?: number;
  contact_transition_arc_length_m?: number | null;
  free_span_material_length_m?: number;
  seabed_contact_length_m?: number;
  seabed_normal_reaction_n?: number;
  iterations?: number;
  plough_x_m?: number;
  plough_y_m?: number;
  plough_z_m?: number;
  plough_inlet_tension_n?: number;
  contact_transition_tension_n: number | null;
  plough_boundary_tension_n?: number;
  plough_adjacent_segment_tension_n?: number;
  plough_entry_angle_deg?: number;
  minimum_bend_radius_m?: number;
  minimum_bend_radius_node_index?: number;
  minimum_bend_radius_left_segment_m?: number;
  minimum_bend_radius_right_segment_m?: number;
  minimum_bend_radius_turn_angle_deg?: number;
  minimum_bend_radius_node_depth_m?: number;
  minimum_bend_radius_near_seabed?: boolean;
  minimum_bend_radius_excluded_tail_nodes?: number;
  minimum_bend_radius_raw_m?: number;
  minimum_bend_radius_raw_node_index?: number;
  minimum_bend_radius_raw_left_segment_m?: number;
  minimum_bend_radius_raw_right_segment_m?: number;
  minimum_bend_radius_raw_turn_angle_deg?: number;
  minimum_bend_radius_raw_node_depth_m?: number;
  minimum_bend_radius_raw_near_seabed?: boolean;
}

export interface TimeHistoryPlotData {
  source: string;
  label: string;
  points: TimeHistoryPlotPoint[];
}

export interface DynamicFramePoint {
  index: number;
  x_m: number;
  y_m: number;
  z_m: number;
  tension_n: number;
}

export interface DynamicFrame {
  /** 节点、分段张力和端点共用的物理输出时刻。 */
  time_s: number;
  /** 求解器节点坐标；视图可以施加偏移，但不会重新计算坐标。 */
  points: DynamicFramePoint[];
  /**
   * 每个相邻节点区间对应一个轴向张力。t=0 帧可采用悬链线或载荷递推初值，
   * 动态步采用 XPBD 反力。
   */
  segment_tensions_n: number[];
  boundary?: string;
  vessel_x_m?: number;
  vessel_y_m?: number;
  vessel_z_m?: number;
  plough_x_m?: number;
  plough_y_m?: number;
  plough_z_m?: number;
  minimum_bend_radius_m?: number;
  minimum_bend_radius_node_index?: number;
  minimum_bend_radius_left_segment_m?: number;
  minimum_bend_radius_right_segment_m?: number;
  minimum_bend_radius_turn_angle_deg?: number;
  minimum_bend_radius_node_depth_m?: number;
  minimum_bend_radius_near_seabed?: boolean;
  minimum_bend_radius_excluded_tail_nodes?: number;
  minimum_bend_radius_raw_m?: number;
  minimum_bend_radius_raw_node_index?: number;
  minimum_bend_radius_raw_left_segment_m?: number;
  minimum_bend_radius_raw_right_segment_m?: number;
  minimum_bend_radius_raw_turn_angle_deg?: number;
  minimum_bend_radius_raw_node_depth_m?: number;
  minimum_bend_radius_raw_near_seabed?: boolean;
}

export interface FrameEndpointLoads {
  top_tension_n: number;
  /** 犁入口端节点轴向支持反力；只在后端明确返回该口径时显示。 */
  plough_boundary_tension_n?: number;
  /** 犁入口前最后一个活动缆段的缆内轴向张力。 */
  plough_inlet_tension_n?: number;
}

export interface DynamicFramePlotData {
  source: string;
  label: string;
  items: DynamicFrame[];
}

export interface OperatorTimeHistoryRequest extends TimeHistoryCaseInputs {
  points: number;
  output_dir?: string;
}

/** 离线时程的可编辑表单。 */
export interface DynamicTimeHistoryForm extends OperatorTimeHistoryRequest {
  current_bottom_speed_mps: number;
  current_profile_exponent: number;
}

export type RunTimeHistoryRequest = OperatorTimeHistoryRequest;

export interface RunTimeHistorySummary {
  diameter_m: number;
  weight_air_n_per_m: number;
  submerged_weight_n_per_m: number;
  tangential_drag_coefficient: number;
  normal_drag_coefficient: number;
  axial_stiffness_n: number;
  current_speed_mps: number;
  current_bottom_speed_mps?: number;
  current_profile_exponent?: number;
  current_direction_deg: number;
  speed_change: "steady" | "accel" | "decel";
  vessel_initial_speed_mps: number;
  vessel_final_speed_mps: number;
  payout_initial_speed_mps?: number | null;
  payout_final_speed_mps?: number | null;
  length_boundary_source: string;
  initial_suspended_length_m?: number | null;
  transition_duration_s: number;
  total_duration_s: number;
  water_depth_m: number;
  element_count: number;
  solver_id: "known_plough_ale_xpbd";
  initial_tension_n: number;
  extreme_tension_n: number;
  steady_tension_n: number;
  plough_speed_mps?: number | null;
  plough_exit_speed_mps?: number | null;
  plough_exit_speed_source?: "explicit" | "measured" | "vessel_longitudinal_inferred" | "not_applicable";
  plough_inlet_tension_final_n?: number | null;
  contact_transition_tension_final_n?: number | null;
  plough_boundary_tension_final_n?: number | null;
  plough_adjacent_segment_tension_final_n?: number | null;
  plough_tension_status?: string | null;
  minimum_bend_radius_min_m?: number | null;
  minimum_bend_radius_limit_m?: number | null;
  minimum_bend_radius_margin_m?: number | null;
  minimum_bend_radius_status?: "ok" | "below_limit" | "not_available" | "not_configured";
  minimum_bend_radius_time_s?: number | null;
  minimum_bend_radius_node_index?: number | null;
  minimum_bend_radius_left_segment_m?: number | null;
  minimum_bend_radius_right_segment_m?: number | null;
  minimum_bend_radius_turn_angle_deg?: number | null;
  minimum_bend_radius_node_depth_m?: number | null;
  minimum_bend_radius_near_seabed?: boolean | null;
  minimum_bend_radius_excluded_tail_nodes?: number | null;
  minimum_bend_radius_raw_m?: number | null;
  minimum_bend_radius_raw_time_s?: number | null;
  minimum_bend_radius_raw_node_index?: number | null;
  minimum_bend_radius_raw_left_segment_m?: number | null;
  minimum_bend_radius_raw_right_segment_m?: number | null;
  minimum_bend_radius_raw_turn_angle_deg?: number | null;
  minimum_bend_radius_raw_node_depth_m?: number | null;
  minimum_bend_radius_raw_near_seabed?: boolean | null;
  integration_time_step_max_s?: number | null;
  integration_time_step_min_s?: number | null;
  spatial_step_mean_m?: number | null;
  spatial_step_min_m?: number | null;
  xpbd_iterations_per_step?: number | null;
  xpbd_iterations_per_step_min?: number | null;
  xpbd_iterations_per_step_max?: number | null;
  xpbd_iteration_limit_per_solve?: number | null;
  axial_constraint_residual_max_m?: number | null;
  geometric_length_deficit_max_m?: number | null;
  geometric_length_deficit_final_m?: number | null;
  vessel_motion_segments?: MotionSegmentInput[];
  plough_motion_segments?: MotionSegmentInput[];
  vessel_motion_samples?: MotionSampleInput[];
  plough_motion_samples?: MotionSampleInput[];
  payout_speed_segments?: SpeedSegmentInput[];
  current_velocity_segments?: CurrentVelocitySegmentInput[];
}

export interface RunTimeHistoryResponse {
  case_name: string;
  field_metadata: ResultFieldMetadata;
  summary: RunTimeHistorySummary;
  artifacts: {
    time_summary_csv: string;
    time_history_csv: string;
    time_history_svg: string;
  };
  plot_data: {
    time_history: TimeHistoryPlotData;
    frames?: DynamicFramePlotData;
  };
}

export interface RealtimeEndpointSample {
  x_m: number;
  y_m: number;
  z_m: number;
  velocity_x_mps: number;
  velocity_y_mps: number;
  velocity_z_mps: number;
}

export interface RealtimeSensorPacket {
  sequence: number;
  time_s: number;
  water_depth_m: number;
  vessel: RealtimeEndpointSample;
  payout_speed_mps: number;
  surface_current: {
    velocity_x_mps: number;
    velocity_y_mps: number;
  };
  plough_position?: Pick<RealtimeEndpointSample, "x_m" | "y_m" | "z_m">;
  plough_horizontal_distance_m?: number;
  plough_bearing_deg?: number;
  plough_inlet_height_above_seabed_m?: number;
  measured_top_tension_n?: number;
}

export type RealtimeSensorPacketDraft = RealtimeSensorPacket;

export type PloughPositionMode = "measured" | "reconstructed";

export interface RealtimeStaticForm {
  cable_name: string;
  diameter_m: number;
  mass_air_kg_per_m: number;
  submerged_weight_n_per_m: number;
  tangential_drag_coefficient: number;
  normal_drag_coefficient: number;
  axial_stiffness_n: number;
  bending_stiffness_n_m2: number;
  initial_suspended_length_m: number;
  plough_position_mode: PloughPositionMode;
  installation_lc_mbr_m: number | null;
  normal_operation_lc_mbr_m: number | null;
  storage_dc_mbr_m: number | null;
  installation_dc_mbr_m: number | null;
  maximum_working_load_n: number | null;
  maximum_abnormal_operation_load_n: number | null;
  dwp_breaking_load_n: number | null;
}

export interface CreateRealtimeSessionRequest {
  cable: {
    name: string;
    diameter_m: number;
    mass_air_kg_per_m: number;
    submerged_weight_n_per_m: number;
    tangential_drag_coefficient: number;
    normal_drag_coefficient: number;
    axial_stiffness_n: number;
    bending_stiffness_n_m2?: number;
  };
  manufacturer_limits?: {
    installation_lc_mbr_m?: number;
    normal_operation_lc_mbr_m?: number;
    storage_dc_mbr_m?: number;
    installation_dc_mbr_m?: number;
    maximum_working_load_n?: number;
    maximum_abnormal_operation_load_n?: number;
    dwp_breaking_load_n?: number;
  };
  initial_geometry: {
    initial_suspended_length_m: number;
    plough_position_mode: PloughPositionMode;
  };
  initial_packet: RealtimeSensorPacket;
}

export interface RealtimeFrameResponse {
  session_id: string;
  sequence: number;
  time_s: number;
  cable: { name: string };
  cable_shape: {
    points: Array<{ index: number; x_m: number; y_m: number; z_m: number }>;
    segment_tensions_n: number[];
  };
  tensions: {
    top_tension_n: number;
    plough_inlet_tension_n: number;
    measured_top_tension_n: number | null;
    top_tension_residual_n: number | null;
  };
  vessel_departure_angles: {
    horizontal_deg: number;
    vertical_deg: number;
  };
  minimum_bend_radius: {
    minimum_m: number | null;
  };
  bending: {
    effective_stiffness_n_m2: number;
    maximum_curvature_per_m: number | null;
    minimum_curvature_radius_m: number | null;
    maximum_moment_n_m: number | null;
  };
  manufacturer_limits: {
    installation_lc_mbr_m: number | null;
    normal_operation_lc_mbr_m: number | null;
    storage_dc_mbr_m: number | null;
    installation_dc_mbr_m: number | null;
    maximum_working_load_n: number | null;
    maximum_abnormal_operation_load_n: number | null;
    dwp_breaking_load_n: number | null;
  };
  runtime: {
    compute_wall_s: number;
    realtime_factor: number | null;
  };
}

export interface HealthResponse {
  status: string;
  service: string;
  module_version: string;
}
