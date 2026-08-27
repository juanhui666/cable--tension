/** 对唯一规范后端命名空间 /api/v1 的类型化客户端。 */
import type {
  HealthResponse,
  CreateRealtimeSessionRequest,
  RealtimeFrameResponse,
  RealtimeSensorPacket,
} from "./types";

export const DEFAULT_API_BASE = "http://127.0.0.1:8765";

export class ApiError extends Error {
  readonly status: number;
  readonly code: string;
  readonly details?: unknown;

  constructor(status: number, code: string, message: string, details?: unknown) {
    super(message);
    this.name = "ApiError";
    this.status = status;
    this.code = code;
    this.details = details;
  }
}

export async function getHealth(apiBase = DEFAULT_API_BASE): Promise<HealthResponse> {
  return requestJson<HealthResponse>(`${trimSlash(apiBase)}/api/v1/health`);
}

/** 历史只读结果组件使用的纯 URL 格式化工具；主动工作台不调用文件接口。 */
export function buildFileUrl(relativePath: string, apiBase = DEFAULT_API_BASE): string {
  const encoded = relativePath.split("/").filter(Boolean).map(encodeURIComponent).join("/");
  return `${trimSlash(apiBase)}/api/v1/files/${encoded}`;
}

export async function createRealtimeSession(
  request: CreateRealtimeSessionRequest,
  apiBase = DEFAULT_API_BASE,
): Promise<RealtimeFrameResponse> {
  return requestJson<RealtimeFrameResponse>(`${trimSlash(apiBase)}/api/v1/realtime-sessions`, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(request),
  });
}

export async function advanceRealtimeSession(
  sessionId: string,
  packet: RealtimeSensorPacket,
  apiBase = DEFAULT_API_BASE,
): Promise<RealtimeFrameResponse> {
  return requestJson<RealtimeFrameResponse>(
    `${trimSlash(apiBase)}/api/v1/realtime-sessions/${encodeURIComponent(sessionId)}/samples`,
    {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(packet),
    },
  );
}

async function requestJson<T>(url: string, init?: RequestInit): Promise<T> {
  // HTTP 错误响应保留后端稳定的错误码和字段详情，
  // 工作台无需解析自然语言即可报告输入合同违规。
  const response = init === undefined ? await fetch(url) : await fetch(url, init);
  const payload = await response.json().catch(() => undefined);
  if (!response.ok) {
    const errorPayload = isRecord(payload) ? payload : {};
    throw new ApiError(
      response.status,
      stringValue(errorPayload.error, "request_failed"),
      stringValue(errorPayload.message, `Request failed with status ${response.status}`),
      errorPayload.details,
    );
  }
  return payload as T;
}

function trimSlash(value: string): string {
  return value.replace(/\/+$/, "");
}

function isRecord(value: unknown): value is Record<string, unknown> {
  return typeof value === "object" && value !== null;
}

function stringValue(value: unknown, fallback: string): string {
  return typeof value === "string" ? value : fallback;
}
