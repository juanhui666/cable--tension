# 实时张力 API v1

基础 URL：`http://127.0.0.1:8765/api/v1`

机器可读合同由以下三份 JSON Schema 共同定义：

- [会话初始化](api/contracts/realtime_session_create_v1.schema.json)
- [每秒数据包](api/contracts/realtime_sensor_packet_v1.schema.json)
- [统一成功结果](api/contracts/realtime_result_v1.schema.json)

公开路由只有：

| 方法 | 路径 | 作用 |
| --- | --- | --- |
| `GET` | `/health` | 进程存活探针 |
| `POST` | `/realtime-sessions` | 用静态参数和第 0 包初始化一次会话 |
| `POST` | `/realtime-sessions/{session_id}/samples` | 提交下一秒工程测量包并直接返回结果 |

POST 请求使用 `Content-Type: application/json`。所有 `/api/v1` 成功和错误响应均包含 `X-Cable-Tension-API-Version: v1`。`OPTIONS` 预检返回 `204`，HTTP 服务同时返回允许 `GET, POST, OPTIONS` 及 `Content-Type` 的 CORS 头。
除 `OPTIONS` 预检外，在请求体可正常解析的情况下，未列出的 HTTP 方法或路径返回 `404 not_found`。HTTP Handler 会先解析 POST 请求体，因此畸形 POST JSON 会在路由判断前返回 `400 invalid_json`。

兼容性说明：当前开发版已将 `cable.name` 定为创建会话的必填字段；旧请求缺少该字段时返回 `400 invalid_input`，旧客户端必须补充正式产品或项目名称。成功响应也新增必填的顶层 `cable.name`，客户端响应模型需同步迁移。该变化属于发布前合同定版，不改变现有 `/api/v1` 路径。

## 坐标与单位

- 作业坐标：`+X` 沿铺设航迹前进，`+Y` 指向航迹右侧，`+Z` 竖直向下。
- 位置和长度：`m`；速度：`m/s`；单位长度质量：`kg/m`；轴向刚度和张力：`N`；等效弯曲刚度：`N·m²`；曲率：`1/m`；弯矩：`N·m`；时间：`s`。
- 上游平台负责设备坐标转换、杆臂修正、单位换算、共同测量时刻组包、采集时刻新鲜度判断和传感器质量门禁。本 API 接收已经完成这些处理的工程量。

## GET /health

成功返回 `200`，包含 `status`、`service` 和 `module_version`。该路由不创建或推进会话。

## POST /realtime-sessions

请求由 `cable`、`initial_geometry` 和 `initial_packet` 三组构成。完整机器约束见 `api/contracts/realtime_session_create_v1.schema.json`。

```json
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
    "plough_position": {"x_m": -20.74971026, "y_m": 0.0, "z_m": 79.0},
    "payout_speed_mps": 0.514,
    "surface_current": {"velocity_x_mps": 0.0, "velocity_y_mps": 1.5}
  }
}
```

上例是 `measured` 模式。若第0包采用 `reconstructed` 模式，`cable`、`manufacturer_limits` 和材料长度保持相同，把犁位模式及首包改为：

```json
{
  "initial_geometry": {
    "initial_suspended_length_m": 85.057647044,
    "plough_position_mode": "reconstructed"
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
    "plough_horizontal_distance_m": 20.74971026,
    "plough_bearing_deg": 180.0,
    "plough_inlet_height_above_seabed_m": 1.0,
    "payout_speed_mps": 0.514,
    "surface_current": {"velocity_x_mps": 0.0, "velocity_y_mps": 1.5}
  }
}
```

这是对完整创建请求中 `initial_geometry` 和 `initial_packet` 两组的替换片段，不是可单独提交的请求。两种模式在同一会话内互斥且不能切换。

### 初始化字段

| 字段 | 必填 | 作用 |
| --- | --- | --- |
| `cable.name` | 是 | 非空缆线产品/项目名称，最长128字符；仅作为会话元数据并在每次结果中原样回显，不进入求解。`Umbilical` 是通用示例名，现场必须替换为正式名称，不代表具体型号 |
| `cable.diameter_m` | 是 | 缆外径，`m`；用于拖曳迎流面积和附加质量排水体积，不用于反推水中重量 |
| `cable.mass_air_kg_per_m` | 是 | 空气中单位长度质量，`kg/m` |
| `cable.submerged_weight_n_per_m` | 是 | 厂家给定水中有效单位重，`N/m`，直接进入重力载荷 |
| `cable.tangential_drag_coefficient` | 是 | 切向拖曳系数 |
| `cable.normal_drag_coefficient` | 是 | 法向拖曳系数 |
| `cable.axial_stiffness_n` | 是 | 轴向刚度，`N` |
| `cable.bending_stiffness_n_m2` | 否 | 当前会话采用的单一等效弯曲刚度 `EI`，`N·m²`；进入离散弯曲能。省略或传 `0` 时严格保留原轴向缆路径 |
| `initial_geometry.initial_suspended_length_m` | 是 | 初始悬空活动参考长度，`m` |
| `initial_geometry.plough_position_mode` | 是 | `measured` 表示逐帧实测三维犁位；`reconstructed` 表示逐帧使用 `L`、水平角和 `h` 重建 |
| `initial_packet` | 是 | 与后续包同结构；首包必须为 `sequence=0,time_s=0` |

前端的材料量和 `L0` 均为操作者可编辑值；界面预填值只是演示数据，不是后端常量。犁端几何不再写入静态初始化：实测模式要求第0包和后续每包都包含 `plough_position`；重建模式要求每包都提交 `plough_horizontal_distance_m`、`plough_bearing_deg` 和 `plough_inlet_height_above_seabed_m`。

`manufacturer_limits` 可选，包含安装 LC、正常运行 LC、储存 DC、安装 DC 四个 MBR，以及 MWL、异常载荷和 DWP 破断载荷。这些厂家值仅原样回显供界面对照，不进入初值、受力、约束、投影或状态判定。后端只用 `mass_air_kg_per_m` 形成惯性质量，并直接采用 `submerged_weight_n_per_m` 形成水中重力载荷；直径和海水密度仅用于拖曳与附加质量，不再反推或重复扣除浮力。近海床流速固定为 0，表层流速按每帧水深进行二次剖面衰减。会话初始采用 48 个单元和 `0.01 s` 最大积分步长，ALE 推进时节点和缆段数量可以变化。

弯曲项以内部节点两侧单位切向的转角 `θ` 和相邻参考材料段平均长度 `l̄` 离散：`U=0.5(EI/l̄)θ²`。在 XPBD 中使用 `α~=l̄/(EI Δt²)`，与轴向、端点和海床约束在同一投影循环内耦合。每轮全部投影和端点回写后检查 `max|θ+α~λ|`；该量是弧度，当前内部闭合容差为 `1e-6 rad`，达到迭代上限仍超限时求解失败，不返回未闭合缆型。当前采用零自然曲率、各向同性、常数等效 `EI`，未表示复合脐带缆层间 stick/slip 状态切换或弯曲滞回。

初始化成功返回 `201` 和“唯一结果结构”。

## POST /realtime-sessions/{session_id}/samples

每个包采用 `api/contracts/realtime_sensor_packet_v1.schema.json`。`sequence` 从 0 连续加一，`time_s` 从 0 开始每包严格增加 `1.0 s`。第 1 包示例：

```json
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
  "plough_position": {"x_m": -20.23571026, "y_m": 0.0, "z_m": 79.0},
  "payout_speed_mps": 0.514,
  "surface_current": {"velocity_x_mps": 0.0, "velocity_y_mps": 1.5}
}
```

同一时刻的 `reconstructed` 模式更新包为：

```json
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
  "plough_horizontal_distance_m": 20.74971026,
  "plough_bearing_deg": 180.0,
  "plough_inlet_height_above_seabed_m": 1.0,
  "payout_speed_mps": 0.514,
  "surface_current": {"velocity_x_mps": 0.0, "velocity_y_mps": 1.5}
}
```

`water_depth_m` 每包必填，作为该帧的海床接触深度和海流剖面尺度。实测模式每包必须提供 `plough_position={x_m,y_m,z_m}`，该三维位置具有最高优先级，三维犁速由相邻两次成功位置按时间差计算；`L` 和水平角不得同时提交。重建模式每包不得提供 `plough_position`，后端用 `x_p=x_v+L cosβ`、`y_p=y_v+L sinβ`、`z_p=water_depth_m-h` 重建三维位置，其中 `L` 是船端到犁入口的水平直线距离，`β` 以 `+X=0°`、从 `+X` 向 `+Y` 为正，`h` 是入口距海床高度。相邻帧重建位置差给出犁端速度；船上放缆速度仍由 `payout_speed_mps` 独立给出。

犁入口三维位置不能代替水深。犁坐标只给出端点边界；水深还决定整条缆线的海床接触平面、接触反力以及表层流到海床流的深度归一化。当前求解在每个1秒推进区间采用区间终点数据包的 `water_depth_m`，即按包更新的分段常值海床边界，不在包间推测连续海床坡面。

重建模式不把 `L`、水平角或 `h` 当作期望控制指令，也不做隐藏的被动犁估计或平滑。操作者或传感器对这些量的逐帧改变会直接成为犁端位置变化；若输入跳变，相邻帧差分速度和动态张力也会出现相应瞬态。

更新包不包含采集绝对时间或传感器质量字段。后端不判断采集前传输延迟和传感器质量，只校验包序号连续、物理时刻严格每包增加 `1.0 s` 以及传感器间隔上限。

实测船端张力仅用于返回“实测减计算”的残差，不修正求解结果。更新成功返回 `200` 和“唯一结果结构”。失败包不提交会话状态，可修正后用同一序号重试。

## 唯一结果结构

机器约束见 `api/contracts/realtime_result_v1.schema.json`。初始化和更新都返回：

| 字段 | 单位/含义 |
| --- | --- |
| `session_id` | 进程内会话标识 |
| `cable.name` | 创建会话时提交的缆线产品/项目名称，原样回显 |
| `sequence`, `time_s` | 已接受包序号和物理时刻 |
| `cable_shape.points[]` | 当前三维缆型节点，`m` |
| `cable_shape.segment_tensions_n[]` | 从船端到犁入口的逐段张力，`N` |
| `tensions.top_tension_n` | 船端导缆点轴向端载荷，`N` |
| `tensions.plough_inlet_tension_n` | 犁入口前末段缆内张力，`N` |
| `tensions.measured_top_tension_n` | 可选实测船端张力原值，未上传为 `null` |
| `tensions.top_tension_residual_n` | 实测减计算，未上传实测值时为 `null` |
| `vessel_departure_angles.horizontal_deg` | 船端第一段在水平面的出缆方向，`atan2(Δy,Δx)`，`+X=0°`、向 `+Y` 为正 |
| `vessel_departure_angles.vertical_deg` | 船端第一段垂直出缆角，`atan2(Δz,sqrt(Δx²+Δy²))`，向下为正 |
| `minimum_bend_radius.minimum_m` | 对当前求解缆型全部内部节点、按转角与相邻参考材料段平均长度计算的离散最小曲率半径；退化段或无有限弯曲时为 `null` |
| `bending.effective_stiffness_n_m2` | 当前会话实际采用的等效 `EI`，`N·m²` |
| `bending.maximum_curvature_per_m` | 全部内部节点的最大离散曲率 `κ=θ/l̄`，`1/m`；退化段为 `null` |
| `bending.minimum_curvature_radius_m` | 与最大曲率对应的 `1/κmax`，`m`；直线为 `null` |
| `bending.maximum_moment_n_m` | 按 `Mmax=EI·κmax` 得到的最大弯矩幅值，`N·m`；退化段为 `null` |
| `manufacturer_limits` | 创建会话时提供的厂家参考值原样回显；不含状态、裕量或合格判定 |
| `runtime.compute_wall_s` | 本包求解墙钟耗时，`s` |
| `runtime.realtime_factor` | 物理步长与求解耗时之比；初始化帧可为 `null` |

首帧缆型优先使用纯自重悬链线或接触悬链线初始化；两者不适用时使用端点几何回退初值。后续出缆方向由每帧已计算缆型的船端第一段切线更新，因此该输出既不是实测角，也不是机械导向约束。

始终满足 `len(segment_tensions_n)=len(points)-1`。调用方必须按每帧实际数组长度渲染，不能假定节点数固定。

## 错误

错误响应结构为：

```json
{"error": "invalid_input", "message": "...", "details": {"fields": {"field": "reason"}}}
```

| HTTP | `error` | 处理 |
| ---: | --- | --- |
| 400 | `invalid_json`, `invalid_input`, `invalid_packet`, `invalid_time_step` | 修正当前请求 |
| 404 | `unknown_session`, `not_found` | 检查 URL；会话丢失时重新初始化 |
| 409 | `sequence_conflict`, `non_monotonic_time`, `session_busy` | 保留当前包，按原序号稍后重试 |
| 422 | `sensor_gap`, `solver_infeasible` | 停止自动推进并保留最后有效结果 |
| 500 | `solver_failure` | 保留输入和最后有效结果，记录 `details.reason` 并检查服务日志 |

几何不可行和内部数值失败的详情位于 `details.reason`；字段结构错误位于 `details.fields`。
任何被拒绝的更新包都不会提交求解状态、犁端边界或已接受序号；调用方修正后应使用同一 `sequence` 重试。初始化失败不会创建可用会话。

## 模型边界

- 船端和犁端采用已知位置、速度边界，内部缆线节点进行动力学计算。
- 铺缆绞车拉力、转矩和控制动力学未作为独立载荷或控制边界；`top_tension_n` 是给定端点运动和放缆速度下反算的船端轴向支反力。
- 接口没有船端出缆方向输入；首帧使用前述悬链线或端点几何初始化，后续输出角来自计算缆型首段切线，不是实测角或机械导向约束。
- `measured_top_tension_n` 只形成监测残差，不修正缆型或计算张力。
- 厂家 MBR 和载荷值仅回显；后端不据此改形、钳制、计算裕量或自动判断合格。等效 `EI` 是独立材料输入，不能用 MBR 替代。
- 当前弯曲模型使用单一、常数、各向同性的等效 `EI` 和零自然曲率；未实现 Full Stick/Full Slip 随载荷切换、滞回、扭弯耦合或多层截面应力恢复。
- 当前介质按海水处理；海水密度固定为 `1025 kg/m³`，仅用于阻力和附加质量。水中有效单位重由 `submerged_weight_n_per_m` 直接输入。
- 当前不包含犁土耦合、犁内导缆槽摩擦或铺缆设备控制系统。

## 部署边界

- 会话驻留在单个 Python 进程内存中，无持久化和跨进程共享；重启后重新初始化。
- 调用方每秒形成一个已对时包并主动提交；后端不主动拉取传感器数据。
- 服务要求 `sequence` 连续且 `time_s` 每包严格增加 `1.0 s`；超过固定传感器间隔上限的包被拒绝，并保留上一有效状态。
- 采集前传输延迟和传感器质量由上游平台判断，本后端没有相应输入字段或判断逻辑。
- 本发布包没有鉴权、TLS、限流或持久化，生产部署应由外层平台提供。
- 公开 HTTP 面只包含本文三条路由；请求体可正常解析时，其他方法或路径按 `404 not_found` 处理。畸形 POST JSON 会先于路由判断返回 `400 invalid_json`。
