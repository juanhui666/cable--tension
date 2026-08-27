import type { RealtimeSensorPacketDraft } from "./types";

export const REALTIME_CSV_COLUMNS = [
  "sequence", "time_s",
  "vessel_x_m", "vessel_y_m", "vessel_z_m",
  "vessel_velocity_x_mps", "vessel_velocity_y_mps", "vessel_velocity_z_mps",
  "payout_speed_mps", "surface_current_velocity_x_mps", "surface_current_velocity_y_mps",
] as const;

const OPTIONAL_COLUMNS = [
  "plough_x_m", "plough_y_m", "plough_z_m", "measured_top_tension_n",
] as const;

export interface RealtimeCsvDataset {
  fileName: string;
  rows: RealtimeSensorPacketDraft[];
}

export function parseRealtimeCsv(text: string, fileName = "传感器数据.csv"): RealtimeCsvDataset {
  const matrix = parseCsv(text.replace(/^\uFEFF/, ""));
  if (matrix.length < 2) throw new Error("CSV 至少需要表头和一行数据。");
  const headers = matrix[0].map((item) => item.trim());
  const duplicate = headers.find((name, index) => headers.indexOf(name) !== index);
  if (duplicate) throw new Error(`CSV 表头重复：${duplicate}`);
  const missing = REALTIME_CSV_COLUMNS.filter((name) => !headers.includes(name));
  if (missing.length) throw new Error(`CSV 缺少必填列：${missing.join("、")}`);
  const supported = new Set<string>([...REALTIME_CSV_COLUMNS, ...OPTIONAL_COLUMNS]);
  const unknown = headers.filter((name) => !supported.has(name));
  if (unknown.length) throw new Error(`CSV 包含未知列：${unknown.join("、")}`);

  const rows = matrix.slice(1).filter((row) => row.some((cell) => cell.trim() !== "")).map((cells, rowIndex) => {
    if (cells.length !== headers.length) throw new Error(`第 ${rowIndex + 2} 行列数与表头不一致。`);
    return recordToPacket(
      Object.fromEntries(headers.map((header, index) => [header, cells[index].trim()])),
      rowIndex + 2,
    );
  });
  if (!rows.length) throw new Error("CSV 没有可用的数据行。");
  rows.forEach((row, index) => {
    if (row.sequence !== index) throw new Error(`第 ${index + 2} 行 sequence 应为 ${index}。`);
    if (Math.abs(row.time_s - index) > 1e-9) throw new Error(`第 ${index + 2} 行 time_s 应为 ${index}.0 s。`);
  });
  return { fileName, rows };
}

function recordToPacket(record: Record<string, string>, line: number): RealtimeSensorPacketDraft {
  const number = (name: string, minimum?: number) => numeric(record, name, line, minimum, false) as number;
  const optional = (name: string, minimum?: number) => numeric(record, name, line, minimum, true);
  const ploughValues = OPTIONAL_COLUMNS.slice(0, 3).map((name) => optional(name));
  const providedPloughValues = ploughValues.filter((value) => value !== undefined);
  if (providedPloughValues.length !== 0 && providedPloughValues.length !== 3) {
    throw new Error(`第 ${line} 行犁位必须同时提供 plough_x_m、plough_y_m、plough_z_m。`);
  }
  const measured = optional("measured_top_tension_n", 0);
  return {
    sequence: number("sequence", 0),
    time_s: number("time_s", 0),
    vessel: {
      x_m: number("vessel_x_m"),
      y_m: number("vessel_y_m"),
      z_m: number("vessel_z_m"),
      velocity_x_mps: number("vessel_velocity_x_mps"),
      velocity_y_mps: number("vessel_velocity_y_mps"),
      velocity_z_mps: number("vessel_velocity_z_mps"),
    },
    payout_speed_mps: number("payout_speed_mps", 0),
    surface_current: {
      velocity_x_mps: number("surface_current_velocity_x_mps"),
      velocity_y_mps: number("surface_current_velocity_y_mps"),
    },
    ...(providedPloughValues.length === 3 ? {
      plough_position: {
        x_m: ploughValues[0] as number,
        y_m: ploughValues[1] as number,
        z_m: number("plough_z_m", 0),
      },
    } : {}),
    ...(measured === undefined ? {} : { measured_top_tension_n: measured }),
  };
}

function numeric(record: Record<string, string>, name: string, line: number, minimum?: number, allowEmpty = false): number | undefined {
  const raw = record[name] ?? "";
  if (allowEmpty && raw === "") return undefined;
  const value = Number(raw);
  if (raw === "" || !Number.isFinite(value)) throw new Error(`第 ${line} 行 ${name} 必须是有限数。`);
  if (minimum !== undefined && value < minimum) throw new Error(`第 ${line} 行 ${name} 不得小于 ${minimum}。`);
  if (name === "sequence" && !Number.isInteger(value)) throw new Error(`第 ${line} 行 sequence 必须是整数。`);
  return value;
}

function parseCsv(text: string): string[][] {
  const rows: string[][] = [];
  let row: string[] = [];
  let cell = "";
  let quoted = false;
  for (let index = 0; index < text.length; index += 1) {
    const char = text[index];
    if (quoted) {
      if (char === '"' && text[index + 1] === '"') { cell += '"'; index += 1; }
      else if (char === '"') quoted = false;
      else cell += char;
    } else if (char === '"') quoted = true;
    else if (char === ",") { row.push(cell); cell = ""; }
    else if (char === "\n") { row.push(cell.replace(/\r$/, "")); rows.push(row); row = []; cell = ""; }
    else cell += char;
  }
  if (quoted) throw new Error("CSV 引号未闭合。");
  if (cell.length || row.length) { row.push(cell.replace(/\r$/, "")); rows.push(row); }
  return rows;
}
