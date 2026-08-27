/** 只读时程求值，用于标注已经求解的输出帧。 */
import type { CurrentVelocitySegmentInput, MotionSegmentInput, SpeedSegmentInput } from "./types";

type ScalarSegment = Pick<
  MotionSegmentInput | SpeedSegmentInput,
  "duration_s" | "start_speed_mps" | "end_speed_mps" | "interpolation" | "sample_interval_s"
>;

export function scheduledScalarAtTime(
  segments: ScalarSegment[] | undefined,
  initial: number,
  final: number,
  durationS: number,
  timeS: number,
): number {
  // 分段指令优先于兼容的起末值摘要，与后端输入合同一致。
  if (!segments?.length) {
    const fraction = clamp01(timeS / Math.max(durationS, 1.0e-9));
    return initial + (final - initial) * fraction;
  }
  let remaining = Math.max(timeS, 0);
  for (const segment of segments) {
    if (remaining <= segment.duration_s) {
      const fraction = interpolationFraction(
        segment.interpolation ?? "linear",
        remaining / Math.max(segment.duration_s, 1.0e-12),
        segment.duration_s,
        segment.sample_interval_s,
      );
      return segment.start_speed_mps + (segment.end_speed_mps - segment.start_speed_mps) * fraction;
    }
    remaining -= segment.duration_s;
  }
  return segments[segments.length - 1].end_speed_mps;
}

export function currentStateAtTime(
  input: {
    current_speed_mps: number;
    current_direction_deg: number;
    current_velocity_segments?: CurrentVelocitySegmentInput[];
  },
  timeS: number,
): { speed_mps: number; direction_deg: number; is_time_varying: boolean } {
  // 此函数只提供视图标注；求解器采用的海流已经反映在后端几何和张力字段中。
  const segments = input.current_velocity_segments;
  if (!segments?.length) {
    return {
      speed_mps: input.current_speed_mps,
      direction_deg: input.current_direction_deg,
      is_time_varying: false,
    };
  }
  let remaining = Math.max(timeS, 0);
  for (const segment of segments) {
    if (remaining <= segment.duration_s) {
      return currentSegmentState(segment, clamp01(remaining / Math.max(segment.duration_s, 1.0e-12)));
    }
    remaining -= segment.duration_s;
  }
  return currentSegmentState(segments[segments.length - 1], 1);
}

function currentSegmentState(
  segment: CurrentVelocitySegmentInput,
  fraction: number,
): { speed_mps: number; direction_deg: number; is_time_varying: true } {
  if (segment.interpolation === "polar_unwrapped") {
    // 显示归一化前先插值未解包角度，使跨越 360 度的转向沿指令方向连续变化，
    // 不会反向绕行。
    const startSpeed = segment.start_speed_mps ?? 0;
    const startDirection = segment.start_direction_unwrapped_deg ?? 0;
    return {
      speed_mps: startSpeed + ((segment.end_speed_mps ?? startSpeed) - startSpeed) * fraction,
      direction_deg: startDirection + ((segment.end_direction_unwrapped_deg ?? startDirection) - startDirection) * fraction,
      is_time_varying: true,
    };
  }
  const x = (segment.start_velocity_x_mps ?? 0)
    + ((segment.end_velocity_x_mps ?? 0) - (segment.start_velocity_x_mps ?? 0)) * fraction;
  const y = (segment.start_velocity_y_mps ?? 0)
    + ((segment.end_velocity_y_mps ?? 0) - (segment.start_velocity_y_mps ?? 0)) * fraction;
  return {
    speed_mps: Math.hypot(x, y),
    direction_deg: Math.atan2(y, x) * 180 / Math.PI,
    is_time_varying: true,
  };
}

function interpolationFraction(
  interpolation: "linear" | "smootherstep" | "sampled_smootherstep",
  fraction: number,
  durationS: number,
  sampleIntervalS?: number,
): number {
  const u = clamp01(fraction);
  if (interpolation === "linear") return u;
  if (interpolation === "smootherstep") return smootherstep(u);
  if (!sampleIntervalS || sampleIntervalS <= 0) throw new Error("sample_interval_s is required");
  const count = Math.max(1, Math.round(durationS / sampleIntervalS));
  const scaled = u * count;
  const lower = Math.min(Math.floor(scaled), count - 1);
  const local = clamp01(scaled - lower);
  const start = smootherstep(lower / count);
  return start + (smootherstep((lower + 1) / count) - start) * local;
}

function smootherstep(value: number): number {
  const u = clamp01(value);
  return 6 * u ** 5 - 15 * u ** 4 + 10 * u ** 3;
}

function clamp01(value: number): number {
  return Math.min(Math.max(value, 0), 1);
}
