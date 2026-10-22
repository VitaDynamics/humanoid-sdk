# 手柄策略 demo

对应机器人 action 17 的模型/合同，SDK 本地推理并请求 **EXTERNAL**；模型必须显式传入。本分支0.2.1直播输入固定为 Aorta `locomotion/joy`，通过与velocity相同的JoyReader；不提供pygame、ROS2或固定中立输入回退。完整公共31槽配置、策略关节安装状态与关节范围须通过代码检查。

```bash
python -m pip install --no-index --find-links ./wheelhouse 'locomotion-aorta[teleop-demo]==0.2.1'
python examples/teleop_policy/teleop_policy.py --help
python examples/teleop_policy/teleop_policy.py --model /path/to/action17.onnx --dry-run --body-state-jsonl /path/to/body-state.jsonl
```

离线JSONL格式见下面的包装说明；离线不连接/不发布。直播命令为上面脚本加 `--model`、匹配 `--group`、`--confirm-motion`（不带dry-run）；先确认现场条件再执行。`action17_contract.json` 随目录提供，`--contract` 可显式指定匹配合同。`--duration 0`保持直到中断，正值限时；可用 `--log /new/path/teleop.jsonl` 记录S0–S6、publish与applied诊断。固定50Hz与状态频率无关；停收/过期、TIMEOUT_HOLD或所有权变化按既有检查退出。

## 离线 JSONL 包装

`main()` 读取首行的 `body`，后续每行读取 `state` 和 `axes`；递归转为属性对象后
送入同一个 Policy。不能直接把裸 BodyInfo 或原始 BodyState JSON 逐行传给此入口。
`joints[].error_code` 必须等于 `JointError.NONE`，策略会拒绝其他错误码。

| 行 | 必需结构与策略读取字段 |
|---|---|
| 首行 | `{"body": ...}`；body 含 `slot_count: 31`、31个唯一 `joint_names`、31个 `joints` 能力条目（含 `populated`）。名字集合必须是模型30个关节加占位 `waist_roll_joint`；策略驱动关节必须已安装。 |
| 后续行 | `{"state": ..., "axes": [...]}`；state 含 `state_id`、31个 `joints`（每项 `q`、`dq`、`error_code`）、`attitude_valid` 与 `imu_states`。axes按下面action17轴映射解释。 |
| HIP样本 | 离线路径传入符号 `'HIP'` 查找 `imu_states[].location`，因此该条目的 location 必须是字符串 `"HIP"`，不是线上枚举数字；需唯一、`status: 1`、state非空，state含长度4的 `quaternion` 与长度3的 `gyroscope`，并有 `attitude_valid: true`。 |

首行与帧的包装键分别是 `body`、`state`/`axes`；不要外加 get_robot_info 响应层。
保留真实关节顺序与数值，按实际模型元数据和合同提供完整31槽；不要为了生成示例输入
伪造缺失关节或反馈。此入口不会自动把 MCAP、ROS lowstate 或别的采集 JSONL 转为
上述包装，也不会把数字 IMU location 转成符号。采集/导出层需先完成这些转换；
输出仅是离线推理结果，不是实时控制或硬件验收。

## 环境与安全

以下命令从 SDK 仓库或离线发布包根目录运行。安装命令面向含完整 wheelhouse 的离线发布包（含所需 extra 依赖）；缺包时先获取匹配产物，不安装别的消息版本或绕过依赖检查。使用独立 CPython 3.10 venv；依赖版本以根目录 `compatibility.json` 为准。机器人上的 SDK 主机先 source `/app/script/env.sh`，再激活 venv；PC 使用部署方提供的 peer 配置。设置 `ZENOH_SESSION_CONFIG_URI` 和与机器人一致的 `--group`，不复制认证内容。安装与 `--help` 不代表已获得运动许可。

现场运动须单独授权并确认吊装/支撑、活动空间、夹点与可用急停；由现场人员操作。`--confirm-motion` 仅为程序确认参数。异常先停测试，由现场人员使用物理急停，不自动重试、清错或重启。运动例程正常退出走 CANCEL→PASSIVE；PASSIVE 可能使负载下落，客户端崩溃不能保证释放控制权。只读与离线模式不进入 EXTERNAL，不需要或发送 CANCEL。

## Aorta Joy 映射依据

# Action 17 teleop policy input

Live input uses Aorta `locomotion/joy`, through the same SDK JoyReader as
velocity mode. It drains pending Joy samples each policy
tick, uses the latest axes, and records button-edge/gap diagnostics. A sample
is fresh for 1 s using its original local REALTIME dequeue stamp; reading a
cached sample does not refresh that stamp. Missing or stale local receipt
supplies zero velocity axes, followed by the profile's normal deceleration.

The source contract is robot `a0ecd6e208a092e7eb55aa6bf96336e5f40c133e`:

- `humanoid/io/humanoid_aorta_command_ingress.cpp:51` copies Joy axes/buttons
  unchanged; the ingress subscribes to `locomotion/joy` at line 291.
- `humanoid/actions/rl_business_logic/humanoid_business_logic.cpp:5398`
  maps the configured Joy axes; the final range clamp is at line 5589.
- `humanoid/actions/policy_profiles/vt_human_v2_teleop_asample*.yaml` has the
  same command block for the base, none, BrainCo and gripper variants.
  The exact profile path/SHA checked on the robot is recorded in
  [action17_contract.json](action17_contract.json).

| Command | Joy axis | Range | Units |
|---|---:|---|---|
| vx | 1 | -0.4 to 0.6 | m/s |
| vy | 0 | -0.5 to 0.5 | m/s |
| wz | 3 | -0.8 to 0.8 | rad/s |
| height | disabled (-1) | default 0.855; clamp 0.375 to 0.855 | m |

For each velocity axis, `abs(v)<0.05` gives zero; otherwise positive values
scale by the upper range and negative values by the magnitude of the lower
range, then clamp. The raw arrays are not reordered or sign-flipped. Buttons
are logged as input/edge counters; this demo assigns them no motion actions.
The robot profile also supports Twist; this SDK demo selects only the Joy
branch as specified by the input contract.

Velocity smoothing uses LPF alpha 0.4 followed by a slew cap: xy acceleration/
deceleration 1.0/2.0 m/s², wz 1.3/2.6 rad/s². Height retains its configured
0.25/0.15 m/s down/up rates and alpha 1.0. These steps run at the configured
50 Hz policy frequency, independent of BodyState or Joy publication frequency.
Neither a local HID input option nor a fixed neutral-input option is exposed.

The action 17 model is separately supplied and must match its 492-input/
12-output metadata. A neutral Joy input retains the height/training pose; it
does not mean a zero BodyCmd. Live execution still requires authorization for
the specific supported robot, an attendant and physical emergency stop.
