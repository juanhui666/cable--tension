/**
 * 根据绝对截止时刻计算下一次数据包的等待时间。
 * 计算耗时会占用当前一秒周期，而不会叠加到周期之后。
 */
export function realtimeScheduleDelayMs(nextDeadlineMs: number, nowMs = Date.now()): number {
  return Math.max(0, nextDeadlineMs - nowMs);
}

export const REALTIME_PACKET_PERIOD_MS = 1000;

export function realtimeScheduleDeadlineMs(baseDeadlineMs: number): number {
  return baseDeadlineMs + REALTIME_PACKET_PERIOD_MS;
}

export function realtimePlaybackDeadlineMs(originMs: number, packetIndex: number): number {
  if (!Number.isInteger(packetIndex) || packetIndex < 0) {
    throw new Error("实时数据包序号必须是非负整数。");
  }
  return originMs + packetIndex * REALTIME_PACKET_PERIOD_MS;
}
