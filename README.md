# 海底缆线实时张力计算工作台

本项目面向海底缆线铺设过程，根据船端导缆点运动、放缆速度、海流和犁入口边界，按 `1 Hz` 接收工程数据，计算当前悬空缆段的三维形态与动态张力。

项目包含：

- Python 实时张力后端；
- React + TypeScript 三维可视化工作台；
- 一键启动和停止脚本；
- 公开 API 的 JSON Schema。

## 当前可以计算什么

每个成功帧返回：

- 船端导缆点至犁入口之间的三维缆型节点；
- 从船端到犁端排列的逐段张力；
- 船端计算张力 `top_tension_n`；
- 犁入口张力 `plough_inlet_tension_n`；
- 最大离散曲率、对应最小曲率半径、按 `M=EIκ` 得到的最大弯矩，以及船端三维出缆方向；
- 厂家 MBR、MWL、异常载荷和破断载荷参考值并列回显；
- 单帧计算耗时和实时倍率。

船端实测张力 `measured_top_tension_n` 是可选监测输入。当前版本只返回“实测值－计算值”的残差，不用实测张力修正缆型或计算张力。

### 创建会话时输入一次的参数

这些参数通过 `POST /api/v1/realtime-sessions` 提交，并在当前会话内保持不变：

| 输入 | 单位 | 作用 |
| --- | --- | --- |
| 缆线名称 `cable.name` | — | 会话元数据，随结果原样回显，不进入求解；示例 `Umbilical` 现场必须替换为正式产品或项目名称 |
| 缆径 `diameter_m` | `m` | 计算拖曳迎流面积和附加质量所用排水体积；不用于反推或重复扣除浮力 |
| 空气中单位长度质量 `mass_air_kg_per_m` | `kg/m` | 结构惯性质量 |
| 水中有效单位重 `submerged_weight_n_per_m` | `N/m` | 厂家直接输入并形成水中重力载荷 |
| 切向、法向拖曳系数 | — | 计算 Morison 水动力 |
| 轴向刚度 `axial_stiffness_n` | `N` | 控制缆线轴向伸长和张力 |
| 等效弯曲刚度 `bending_stiffness_n_m2` | `N·m²` | 以离散弯曲能参与每个 XPBD 子步；省略或为0时兼容原轴向缆路径 |
| 厂家参考值 `manufacturer_limits` | `m` / `N` | 可选回显，不进入求解或自动安全判定 |
| 初始悬空材料长度 `initial_suspended_length_m` | `m` | 第0秒的已知未伸长活动参考长度 `L0` |
| 犁位模式 `plough_position_mode` | — | `measured` 使用逐帧实测三维位置；`reconstructed` 使用逐帧 `L`、水平角和 `h` 重建 |

`initial_packet` 是第0秒实时数据，也随创建会话请求一起提交。初始 `L0` 不是待求未知量；后续活动参考长度由放缆和犁端材料流动在求解器内部推进，不需要每秒重新提交 `L0`。
前端显示的数值只是可编辑的示例预填值；创建会话时提交的是操作者当前填写值，后端没有把这些初始值写死。

### 每秒实时输入的值

初始化成功后，每隔 `1.0 s` 提交一包：

| 实时输入 | 单位 | 必填 | 作用 |
| --- | --- | --- | --- |
| `sequence`、`time_s` | —、`s` | 是 | 标识连续帧；序号连续且时间每包增加 `1.0 s` |
| 水深 `water_depth_m` | `m` | 是 | 定义本帧静水面至海床的垂向距离，并更新接触边界和流速剖面 |
| 船端导缆点三维位置 | `m` | 是 | 作为缆线船端位置边界 |
| 船端导缆点三维速度 | `m/s` | 是 | 作为缆线船端速度边界并参与动态张力计算 |
| 放缆速度 `payout_speed_mps` | `m/s` | 是 | 控制船端进入水中缆材的速度和活动参考长度变化 |
| 表层海流二维速度 | `m/s` | 是 | 输入 `+X`、`+Y` 分量，后端生成随深度变化的流速剖面 |
| 犁入口三维位置 `plough_position` | `m` | 按模式 | 实测模式每帧必填，且作为最高优先级犁端位置边界 |
| 船到犁水平直线距离 `plough_horizontal_distance_m` | `m` | 重建模式 | 每帧实测或人工给定的水平面直线距离 `L`；不是缆长或 `L0` |
| 船向犁水平角 `plough_bearing_deg` | `°` | 重建模式 | 每帧输入；`+X=0°`，从 `+X` 向 `+Y` 旋转为正 |
| 犁入口距海床高度 `plough_inlet_height_above_seabed_m` | `m` | 重建模式 | 每帧实测的 `h`；入口向下坐标为 `water_depth_m-h` |
| 船端实测张力 `measured_top_tension_n` | `N` | 否 | 只计算实测与模型结果的残差，不参与状态修正 |

实测模式直接用每帧 `plough_position` 确定犁端边界，不再用 `L`、水平角或 `h` 反算位置；即使同时采到 `h`，它也只用于上游一致性检查，不覆盖三维位置。重建模式每帧按 `x_p=x_v+L cosβ`、`y_p=y_v+L sinβ`、`z_p=water_depth_m-h` 得到犁入口位置，相邻帧位置差再给出犁端运动速度。两种模式不能在同一会话中混用。水深在两种模式下都必须逐帧提交，因为它还定义整条缆线的海床接触面和海流沿深度的剖面。

### 后端当前固定的主要模型值

以下值没有开放为接口参数：

| 固定值 | 当前设置 | 含义 |
| --- | ---: | --- |
| 重力加速度 | `9.8 m/s²` | 将空气中单位长度质量换算为结构单位重；厂家水中单位重已是 `N/m`，施加载荷时不再乘 `g` |
| 海水密度 | `1025 kg/m³` | 阻力和附加质量计算；水中有效单位重已由厂家直接输入，不再重复扣除浮力 |
| 数据包物理间隔 | `1.0 s` | 实时接口固定为 `1 Hz` |
| 初始单元数量 | `48` | 第0秒初始离散规模 |
| 内部最大积分步长 | `0.01 s` | 每个1秒数据包内部继续细分推进 |
| 近海床流速 | `0 m/s` | 流速剖面的海床端值 |
| 流速剖面指数 | `2.0` | 表层流速向海床二次衰减 |
| 轴向附加质量系数 | `0.0` | 缆轴方向不增加流体附加质量 |
| 法向附加质量系数 | `1.0` | 缆线法向质量包含排开海水质量 |

## 坐标与单位

- `+X`：沿铺设航迹前进；
- `+Y`：指向航迹右侧；
- `+Z`：竖直向下；
- 位置、长度和弯曲半径：`m`；
- 速度：`m/s`；
- 单位长度质量：`kg/m`；
- 轴向刚度和张力：`N`；等效弯曲刚度：`N·m²`；曲率：`1/m`；弯矩：`N·m`；
- 时间：`s`。

接口中的 `vessel` 表示已经转换到上述坐标系的船端导缆点位置和速度，不是船舶重心或 GNSS 天线位置。

## 运行环境

- Windows PowerShell 5.1 或更高版本；
- Python 3.11 或更高版本；
- Node.js 与 npm。

后端只使用 Python 标准库。前端依赖由 `frontend/package-lock.json` 锁定，并通过 `npm ci` 安装。

## 快速启动

克隆仓库：

```powershell
git clone https://github.com/juanhui666/cable--tension.git
cd cable--tension
```

在仓库根目录启动后端和前端：

```powershell
.\start-workbench.ps1
```

如果 PowerShell 阻止执行脚本，可以使用：

```powershell
powershell -ExecutionPolicy Bypass -File .\start-workbench.ps1
```

默认地址：

- 前端工作台：`http://127.0.0.1:5173/`
- 后端 API：`http://127.0.0.1:8765/api/v1`

指定其他前端端口：

```powershell
.\start-workbench.ps1 -FrontendPort 5174
```

停止由脚本启动的服务：

```powershell
.\stop-workbench.ps1
```

运行日志保存在本地 `.run/` 目录中。

## 分别启动后端和前端

只启动后端：

```powershell
python backend/api/app.py --host 127.0.0.1 --port 8765
```

检查后端：

```powershell
Invoke-RestMethod http://127.0.0.1:8765/api/v1/health
```

只启动前端：

```powershell
cd frontend
npm ci
npm run dev -- --host 127.0.0.1 --port 5173
```

前端默认连接 `http://127.0.0.1:8765`。需要连接其他后端时，在启动前设置：

```powershell
$env:VITE_API_BASE_URL = "http://127.0.0.1:8765"
```

## API 使用流程

对外交付的人工接口合同见 [backend/API.md](backend/API.md)。机器可读合同由[会话初始化 Schema](backend/api/contracts/realtime_session_create_v1.schema.json)、[每秒数据包 Schema](backend/api/contracts/realtime_sensor_packet_v1.schema.json)和[统一结果 Schema](backend/api/contracts/realtime_result_v1.schema.json)三份文件共同定义。

公开路由只有三条：

| 方法 | 路径 | 作用 |
| --- | --- | --- |
| `GET` | `/api/v1/health` | 检查服务是否存活 |
| `POST` | `/api/v1/realtime-sessions` | 使用静态参数和第0秒数据创建会话 |
| `POST` | `/api/v1/realtime-sessions/{session_id}/samples` | 每秒提交下一包并返回最新结果 |

**兼容性说明：**当前开发版已将 `cable.name` 定为创建会话的必填字段；旧请求缺少该字段时返回 `400 invalid_input`，旧客户端必须补充正式产品或项目名称。成功响应也新增必填的顶层 `cable.name`，客户端解析模型需同步迁移。本次属于发布前合同定版，API 路径仍保持 `/api/v1`。

### 1. 创建实时会话

```powershell
$initialBody = @'
{
  "cable": {
    "name": "Umbilical",
    "diameter_m": 0.2322,
    "mass_air_kg_per_m": 68.3,
    "submerged_weight_n_per_m": 304.5,
    "tangential_drag_coefficient": 0.0,
    "normal_drag_coefficient": 1.0,
    "axial_stiffness_n": 950500000.0,
    "bending_stiffness_n_m2": 78000.0
  },
  "manufacturer_limits": {
    "installation_lc_mbr_m": 8.3,
    "normal_operation_lc_mbr_m": 13.1,
    "storage_dc_mbr_m": 3.5,
    "installation_dc_mbr_m": 4.65,
    "maximum_working_load_n": 1535000.0,
    "maximum_abnormal_operation_load_n": 2025000.0,
    "dwp_breaking_load_n": 2640000.0
  },
  "initial_geometry": {
    "initial_suspended_length_m": 85.057647044,
    "plough_position_mode": "measured"
  },
  "initial_packet": {
    "sequence": 0,
    "time_s": 0.0,
    "water_depth_m": 80.0,
    "vessel": {
      "x_m": 0.0,
      "y_m": 0.0,
      "z_m": 0.0,
      "velocity_x_mps": 0.514,
      "velocity_y_mps": 0.0,
      "velocity_z_mps": 0.0
    },
    "plough_position": {
      "x_m": -20.74971026,
      "y_m": 0.0,
      "z_m": 79.0
    },
    "payout_speed_mps": 0.514,
    "surface_current": {
      "velocity_x_mps": 0.0,
      "velocity_y_mps": 1.5
    }
  }
}
'@

$initialResult = Invoke-RestMethod `
  -Method Post `
  -Uri "http://127.0.0.1:8765/api/v1/realtime-sessions" `
  -ContentType "application/json" `
  -Body $initialBody

$sessionId = $initialResult.session_id
$initialResult.tensions
```

### 2. 提交第1秒数据

```powershell
$nextBody = @'
{
  "sequence": 1,
  "time_s": 1.0,
  "water_depth_m": 80.0,
  "vessel": {
    "x_m": 0.514,
    "y_m": 0.0,
    "z_m": 0.0,
    "velocity_x_mps": 0.514,
    "velocity_y_mps": 0.0,
    "velocity_z_mps": 0.0
  },
  "plough_position": {
    "x_m": -20.23571026,
    "y_m": 0.0,
    "z_m": 79.0
  },
  "payout_speed_mps": 0.514,
  "surface_current": {
    "velocity_x_mps": 0.0,
    "velocity_y_mps": 1.5
  }
}
'@

$frame = Invoke-RestMethod `
  -Method Post `
  -Uri "http://127.0.0.1:8765/api/v1/realtime-sessions/$sessionId/samples" `
  -ContentType "application/json" `
  -Body $nextBody

$frame.tensions
$frame.runtime
```

后续数据包要求：

- `sequence` 按 `0, 1, 2...` 连续增加；
- `time_s` 每包严格增加 `1.0 s`；
- `water_depth_m` 每包必填，作为该帧采用的海床深度；
- 失败的数据包不会提交状态，修正后应使用原序号重试；
- `measured` 模式每包必须提供 `plough_position`，并且不得同时提供 `L` 或水平角；
- `reconstructed` 模式每包不得提供 `plough_position`，必须提供 `plough_horizontal_distance_m`、`plough_bearing_deg` 和 `plough_inlet_height_above_seabed_m`；
- `measured_top_tension_n` 可选，仅用于计算监测残差。

机器可读接口合同：

- [会话初始化 Schema](backend/api/contracts/realtime_session_create_v1.schema.json)
- [每秒数据包 Schema](backend/api/contracts/realtime_sensor_packet_v1.schema.json)
- [统一结果 Schema](backend/api/contracts/realtime_result_v1.schema.json)

## 前端 CSV 格式

前端可以读取本地 CSV 并按 `1 Hz` 顺序提交。必填列为：

```text
sequence,time_s,water_depth_m,vessel_x_m,vessel_y_m,vessel_z_m,vessel_velocity_x_mps,vessel_velocity_y_mps,vessel_velocity_z_mps,payout_speed_mps,surface_current_velocity_x_mps,surface_current_velocity_y_mps
```

可选列为：

```text
plough_x_m,plough_y_m,plough_z_m,plough_horizontal_distance_m,plough_bearing_deg,plough_inlet_height_above_seabed_m,measured_top_tension_n
```

三项犁位置必须同时提供或同时留空。实测模式要求每行都有三项犁位置，且 `L` 与水平角留空；重建模式要求三项犁位置留空，而 `L`、水平角与 `h` 每行都有值。CSV 第一行是列名，数据从 `sequence=0,time_s=0` 开始。

## 项目结构

```text
.
├─ backend/
│  ├─ api/                         HTTP接口、请求校验和JSON Schema
│  └─ src/cable_tension/           ALE/XPBD实时缆线求解器
├─ frontend/
│  └─ src/                         React工作台、CSV解析与结果可视化
├─ start-workbench.ps1             一键启动前后端
└─ stop-workbench.ps1              停止脚本启动的服务
```

## 当前模型边界

- 船端和犁端采用已知位置、速度边界，内部缆线节点进行动力学计算；
- 铺缆绞车拉力、转矩及控制动力学尚未作为独立载荷或控制边界进入模型；当前 `top_tension_n` 是在给定端点运动与 `payout_speed_mps` 下反算得到的船端轴向支反力，`measured_top_tension_n` 只用于计算监测残差；
- 当前接口不接收船端出缆方向输入。首帧方向优先来自纯自重悬链线或接触悬链线初始化；两者不适用时，采用端点几何回退初值。后续方向随已计算缆型的船端第一段切线更新；返回的出缆角不是实测角，也不是机械导向约束；
- 初始悬空长度是材料参考长度，不是两端直线距离；
- 当前介质按海水处理，海水密度固定为 `1025 kg/m³`，只用于阻力和附加质量；水中有效单位重直接采用厂家输入；
- 表层海流按深度二次衰减，近海床流速固定为零；
- 犁土耦合、犁内导缆槽摩擦和设备原始异步数据融合不在当前模型中；
- 会话保存在单个后端进程内存中，服务重启后必须重新初始化；
- 当前版本没有鉴权、TLS、限流和持久化，生产部署应由外层平台提供；
- OrcaFlex、MoorPy 和 MoorDyn 不是生产运行依赖，只能作为离线验证工具。

## 常见问题

### 前端打不开

检查 `.run/frontend.err.log`，确认 Node.js、npm 和前端依赖安装正常。也可以手动执行 `npm ci` 后重新启动。

### 后端健康检查失败

检查 `.run/backend.err.log`，或者直接运行：

```powershell
python backend/api/app.py --host 127.0.0.1 --port 8765
```

### 提交下一帧返回序号或时间错误

确认 `sequence` 连续，并且 `time_s` 相比上一成功帧增加且严格等于 `1.0 s`。服务重启后需要重新创建会话。
