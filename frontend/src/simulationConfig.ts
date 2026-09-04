import type { DynamicTimeHistoryForm, RealtimeStaticForm, TimeHistoryCase } from "./types";

export type ConsoleModule = "parameters" | "simulation" | "tension";
export type SpeedChange = "steady" | "accel" | "decel";

// 默认连接本机后端；部署或联调其他地址时可通过 VITE_API_BASE_URL 覆盖。
export const DEFAULT_DYNAMIC_FORM: DynamicTimeHistoryForm = {
  case_name: "当前 500 kV 稳态基线",
  points: 61,
  output_dir: "time_histories/power-500kv-steady",
  diameter_m: 0.139,
  weight_air_n_per_m: 470.4,
  submerged_weight_n_per_m: 317.9703603438039,
  tangential_drag_coefficient: 0,
  normal_drag_coefficient: 1,
  axial_stiffness_n: 2.66e8,
  current_speed_mps: 1.5,
  current_bottom_speed_mps: 0,
  current_profile_exponent: 2,
  current_direction_deg: 90,
  speed_change: "steady",
  vessel_initial_speed_mps: 0.514,
  vessel_final_speed_mps: 0.514,
  payout_initial_speed_mps: 0.514,
  payout_final_speed_mps: 0.514,
  length_boundary_source: "known_plough_trajectory",
  transition_duration_s: 60,
  total_duration_s: 60,
  water_depth_m: 80,
  element_count: 48,
  integration_time_step_max_s: 0.01,
  vessel_initial_x_m: 0,
  vessel_initial_y_m: 0,
  vessel_heading_deg: 0,
  plough_initial_x_m: -20.74971026,
  plough_initial_y_m: 0,
  plough_initial_z_m: 79,
  initial_suspended_length_m: 85.057647044,
  vessel_motion_segments: [
    {
      duration_s: 60,
      start_speed_mps: 0.514,
      end_speed_mps: 0.514,
      heading_deg: 0,
      start_velocity_x_mps: 0.514,
      start_velocity_y_mps: 0,
      end_velocity_x_mps: 0.514,
      end_velocity_y_mps: 0,
    },
  ],
  payout_speed_segments: [
    { duration_s: 60, start_speed_mps: 0.514, end_speed_mps: 0.514 },
  ],
  min_bending_radius_m: null,
};

export const DEFAULT_REALTIME_FORM: RealtimeStaticForm = {
  cable_name: "Umbilical",
  diameter_m: 0.2322,
  mass_air_kg_per_m: 68.3,
  submerged_weight_n_per_m: 304.5,
  tangential_drag_coefficient: 0,
  normal_drag_coefficient: 1,
  axial_stiffness_n: 950.5e6,
  // Manufacturer sheet selection used by the editable workbench default.
  bending_stiffness_n_m2: 78.0e3,
  initial_suspended_length_m: 85.057647044,
  plough_position_mode: "measured",
  installation_lc_mbr_m: 8.3,
  normal_operation_lc_mbr_m: 13.1,
  storage_dc_mbr_m: 3.5,
  installation_dc_mbr_m: 4.65,
  maximum_working_load_n: 1535e3,
  maximum_abnormal_operation_load_n: 2025e3,
  dwp_breaking_load_n: 2640e3,
};

export const SPEED_CHANGE_OPTIONS: { value: SpeedChange; label: string }[] = [
  { value: "steady", label: "匀速" },
  { value: "accel", label: "加速" },
  { value: "decel", label: "减速" },
];

export function groupTimeHistoryCases(cases: TimeHistoryCase[]): { label: string; cases: TimeHistoryCase[] }[] {
  const groups = new Map<string, TimeHistoryCase[]>();
  cases.forEach((item) => {
    const existing = groups.get(item.group) ?? [];
    existing.push(item);
    groups.set(item.group, existing);
  });
  return Array.from(groups.entries()).map(([label, groupCases]) => ({ label, cases: groupCases }));
}

export function labelForTimeHistoryCase(caseName: string, timeHistoryCases: TimeHistoryCase[]): string {
  return timeHistoryCases.find((item) => item.name === caseName)?.label ?? caseName;
}
