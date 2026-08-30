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
- 最小弯曲半径及其限值状态；
- 单帧计算耗时和实时倍率。

船端实测张力 `measured_top_tension_n` 是可选监测输入。当前版本只返回“实测值－计算值”的残差，不用实测张力修正缆型或计算张力。

### 创建会话时输入一次的参数

这些参数通过 `POST /api/v1/realtime-sessions` 提交，并在当前会话内保持不变：

| 输入 | 单位 | 作用 |
| --- | --- | --- |
| 缆径 `diameter_m` | `m` | 计算排水体积、浮力和水动力受力面积 |
| 空气中单位长度质量 `mass_air_kg_per_m` | `kg/m` | 计算缆线自重和水中有效重量 |
| 切向、法向拖曳系数 | — | 计算 Morison 水动力 |
| 轴向刚度 `axial_stiffness_n` | `N` | 控制缆线轴向伸长和张力 |
| 最小弯曲半径限值 `min_bending_radius_m` | `m` | 可选，用于结果中的限值判断 |
| 水深 `water_depth_m` | `m` | 定义静水面至海床的垂向距离 |
| 初始悬空材料长度 `initial_suspended_length_m` | `m` | 第0秒的已知未伸长活动参考长度 `L0` |
| 初始水平后拖距离 `plough_layback_m` | `m` | 给出第0秒船端与犁入口的水平关系 |
| 初始犁入口深度 `plough_depth_m` | `m` | 给出第0秒犁入口相对静水面的绝对深度 |

`initial_packet` 是第0秒实时数据，也随创建会话请求一起提交。初始 `L0` 不是待求未知量；后续活动参考长度由放缆和犁端材料流动在求解器内部推进，不需要每秒重新提交 `L0`。

### 每秒实时输入的值

初始化成功后，每隔 `1.0 s` 提交一包：

| 实时输入 | 单位 | 必填 | 作用 |
| --- | --- | --- | --- |
| `sequence`、`time_s` | —、`s` | 是 | 标识连续帧；序号连续且时间每包增加 `1.0 s` |
| 船端导缆点三维位置 | `m` | 是 | 作为缆线船端位置边界 |
| 船端导缆点三维速度 | `m/s` | 是 | 作为缆线船端速度边界并参与动态张力计算 |
| 放缆速度 `payout_speed_mps` | `m/s` | 是 | 控制船端进入水中缆材的速度和活动参考长度变化 |
| 表层海流二维速度 | `m/s` | 是 | 输入 `+X`、`+Y` 分量，后端生成随深度变化的流速剖面 |
| 犁入口三维位置 `plough_position` | `m` | 否 | 提供时作为实测犁端边界，并由相邻帧位置差计算犁速 |
| 船端实测张力 `measured_top_tension_n` | `N` | 否 | 只计算实测与模型结果的残差，不参与状态修正 |

未提供 `plough_position` 时，后端使用初始后拖距离和入口深度，按被动拖曳运动学估算犁入口运动。当前版本中，水深是会话固定参数，不能随每秒数据包更新；犁入口三维位置则可以每秒更新。

### 后端当前固定的主要模型值

以下值没有开放为接口参数：

| 固定值 | 当前设置 | 含义 |
| --- | ---: | --- |
| 重力加速度 | `9.8 m/s²` | 自重和浮力换算 |
| 海水密度 | `1025 kg/m³` | 浮力、阻力和附加质量计算 |
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
- 轴向刚度和张力：`N`；
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

公开路由只有三条：

| 方法 | 路径 | 作用 |
| --- | --- | --- |
| `GET` | `/api/v1/health` | 检查服务是否存活 |
| `POST` | `/api/v1/realtime-sessions` | 使用静态参数和第0秒数据创建会话 |
| `POST` | `/api/v1/realtime-sessions/{session_id}/samples` | 每秒提交下一包并返回最新结果 |

### 1. 创建实时会话

```powershell
$initialBody = @'
{
  "cable": {
    "diameter_m": 0.139,
    "mass_air_kg_per_m": 48.0,
    "tangential_drag_coefficient": 0.0,
    "normal_drag_coefficient": 1.0,
    "axial_stiffness_n": 266000000.0,
    "min_bending_radius_m": 5.0
  },
  "environment": {
    "water_depth_m": 80.0
  },
  "initial_geometry": {
    "initial_suspended_length_m": 85.057647044,
    "plough_layback_m": 20.74971026,
    "plough_depth_m": 79.0
  },
  "initial_packet": {
    "sequence": 0,
    "time_s": 0.0,
    "vessel": {
      "x_m": 0.0,
      "y_m": 0.0,
      "z_m": 0.0,
      "velocity_x_mps": 0.514,
      "velocity_y_mps": 0.0,
      "velocity_z_mps": 0.0
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
  "vessel": {
    "x_m": 0.514,
    "y_m": 0.0,
    "z_m": 0.0,
    "velocity_x_mps": 0.514,
    "velocity_y_mps": 0.0,
    "velocity_z_mps": 0.0
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
- 失败的数据包不会提交状态，修正后应使用原序号重试；
- `plough_position` 可选；不提供时由后端按被动拖曳边界估算；
- `measured_top_tension_n` 可选，仅用于计算监测残差。

机器可读接口合同：

- [会话初始化 Schema](backend/api/contracts/realtime_session_create_v1.schema.json)
- [每秒数据包 Schema](backend/api/contracts/realtime_sensor_packet_v1.schema.json)
- [统一结果 Schema](backend/api/contracts/realtime_result_v1.schema.json)

## 前端 CSV 格式

前端可以读取本地 CSV 并按 `1 Hz` 顺序提交。必填列为：

```text
sequence,time_s,vessel_x_m,vessel_y_m,vessel_z_m,vessel_velocity_x_mps,vessel_velocity_y_mps,vessel_velocity_z_mps,payout_speed_mps,surface_current_velocity_x_mps,surface_current_velocity_y_mps
```

可选列为：

```text
plough_x_m,plough_y_m,plough_z_m,measured_top_tension_n
```

三项犁位置必须同时提供或同时留空。CSV 第一行是列名，数据从 `sequence=0,time_s=0` 开始。

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
- 初始悬空长度是材料参考长度，不是两端直线距离；
- 当前介质按海水处理，海水密度固定为 `1025 kg/m³`；
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
