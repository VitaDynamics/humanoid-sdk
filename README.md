# 人形 SDK

基于 Aorta 的 Python SDK 与人形机器人示例，Python 导入名为
`locomotion_aorta`。客户端不需要 ROS 2、机器人主仓库或私有消息源码。
通常在用户自己的 Linux PC 或计算设备上运行 SDK，通过有线网络连接机器人的 S100；
不要求把 SDK 安装到 S100。

> **当前分支 HIL only**：BrainCo 手部使用独立 topic，精确依赖 schemas-internal
> `489488e52b5fa3d808db8538badd537c1e78c70f` 的临时 `vita-msgs` wheel；它尚未发布。
> 由交付方提供完整离线 bundle 和摘要，不能用官方 `vita-msgs==0.1.0` 替代。
> 机器人必须先完成手域启动修复并由现场确认可用。本分支没有设备连接或运动验收。
> SDK Python 包版本仍为 `0.1.0`；正式发布信息见本仓库 Releases。

## 使用 agent 开始

克隆仓库后，在仓库根目录启动 Codex 或 Claude Code。两者共用
[AGENTS.md](AGENTS.md)；`CLAUDE.md` 是指向它的相对软链接，无需额外插件。

可以直接告诉 agent：

> 帮我在本机安装与机器人版本匹配的人形 SDK，完成依赖、消息绑定和只读订阅
> 检查。先确认缺失的目标信息，不申请控制权、不发送运动命令。

或：

> 帮我把 SDK 部署到指定 S100 的独立目录，保留现有安装以便回退，并完成
> 只读 quick start。不要替换 OTA、重启服务或改变电机配置。

需要运动示例时再单独提出。Agent 应先说明目标动作与退出行为，并等待现场
支撑、看护、无干涉及急停确认。CANCEL 回退到 PASSIVE，负载肢体可能自然下落。

## 文档与兼容性

用户文档入口：[VitaDynamics 人形机器人文档](https://vitadynamics.feishu.cn/wiki/Mu1hw8wcSiKw3ykDGtAcvgXpnAg)。

SDK 专页：[人形 SDK](https://vitadynamics.feishu.cn/wiki/OG6KwVrf8i0fNdk2oGTc3nDKnFh)，
完整教程由该文档入口维护；网络接入须区分用户设备与 S100，不能共用本机回环端点。

文件名按工具约定使用大写：[Codex 的 AGENTS.md](https://developers.openai.com/codex/guides/agents-md)
与 [Claude Code 的 CLAUDE.md](https://code.claude.com/docs/en/memory)。
推荐在 Linux 原生 Git checkout 中使用。若解压工具把软链接变成了包含
`AGENTS.md` 字样的普通文件，先修复链接再启动 agent；该普通文件不等于指引正文。

## 安装与只读 quick start

兼容版本见 [compatibility.json](compatibility.json)：Linux x86_64/aarch64、
Python 3.10+、Aorta `2026.10.12`（ABI 13）、
`vita-msgs==0.1.0+hil.brainco.20261014`（HIL only，schemas-internal `489488e`）与 FlatBuffers `25.9.23`。
Aorta 使用官方 release；临时消息 wheel 必须来自交付方核验的完整 HIL bundle。以下在用户自己的 Linux 设备上，
从 bundle 根目录执行，不覆盖旧环境，也不需要 `/app/script/env.sh`：

仓库提供 [PC peer 配置](config/pc_session_peer.json5)，新构建的 offline bundle
也在同一路径包含该文件并记录 SHA-256。它保留已测试 PC 的网络与传输设置，
namespace、账号、密码和认证字典路径使用占位符，不能原样连接。
已发布的 `v0.1.0-rc.1` bundle 不含此文件，不会被原地更新；使用该旧包时，
可从配套的已审核 SDK 提交获取此配置，记录来源提交和摘要。仅安装 wheel 不会安装此文件。
填写步骤见 [PC 网络与 Zenoh 配置](https://vitadynamics.feishu.cn/wiki/M1vEw22vziSNAjkHVCncFlu1nyh)。

```bash
python3 -m venv venv
source venv/bin/activate
python -m pip install --no-index --find-links wheelhouse locomotion-aorta==0.1.0
python -m pip check
python -c 'import aorta, flatbuffers, locomotion_aorta; import lowlevel.LowCmd, locomotion_sdk.ControlStatus'
# 复制到仓库外，保留已有文件；仅修改私有副本，不提交认证材料。
mkdir -p "$HOME/.config/vita"
PC_PROFILE="$HOME/.config/vita/pc_session_peer.json5"
test ! -e "$PC_PROFILE" && install -m 600 config/pc_session_peer.json5 "$PC_PROFILE"
# 按网络文档填写实际 IP、namespace、认证信息和字典绝对路径，消除全部占位符。
${EDITOR:-vi} "$PC_PROFILE"
export ZENOH_SESSION_CONFIG_URI="$PC_PROFILE"
test -r "$ZENOH_SESSION_CONFIG_URI"
python examples/lowstate_subscriber.py --group default --timeout 5
```

venv 已存在时仅激活。最后一条需要连接授权，只读一帧，不申请控制权。
PC 与 S100 须有可达的有线 IP；PC profile 的 `connect.endpoints` 指向 S100
的可达地址和交付端口（例如 `tcp/192.168.125.2:7447`），不能写 `127.0.0.1`。
SDK 的 `peer` 可以连接机器人上的 router；还须保留配套的 gossip/自动连接、
namespace 和认证配置。示例地址不是每台机器的默认值；部署方提供 namespace、
授权账号/密码和配套认证字典，通用 bundle 不分发设备凭据。
`--group default` 也须与机器人实际 group 匹配。缺少认证材料时停止，不删除 auth。

仅当明确选择在 S100 本机运行 SDK 时，才在激活 venv **之前**执行
`source /app/script/env.sh`，并改用部署方核验的
`/app_param/zenoh/s100_session_peer.json5`。SDK 安装位置与机器人 OTA 是两回事。

`examples/external_control.py` 是 42 槽全身归零往返 demo，含硬件调试增益和双肘
回程向零偏移 10°；必须逐机审查并单独授权，不由安装命令自动运行。
当前源码的 `ExternalController.enter_external(timeout=5.0)` 统一请求无限期控制；
`timeout` 只限制进入过程的等待，不限制控制时长。旧调用须删除 `lease_duration_s`
参数，示例不再计算剩余控制时长。结束时调用 `exit_external(timeout=5.0)` 发送
CANCEL；命令断流保护、控制权校验和安全检查保持不变。客户端异常退出不能依赖
自动到期释放控制权。协议中的 `lease_id` 仍是控制权标识，历史时长字段发送 0。
需要支持零时长无限期的机器人软件；旧服务端拒绝时会报错，不回退到有限期请求。
这些是源码变更，尚未进入已发布的 `v0.1.0-rc.1` 包；安装仍须核对匹配的发布组合。

## Revo2 手部 EXTERNAL 测试（HIL only）

获得本次只读连接授权、配置 PC peer profile 后，先读取左右手 info/state：

```bash
export AORTA_GROUP=default
python examples/brainco_hand_external.py --dry-run --side both --group default --timeout 5
```

默认入口同样只读，不申请控制权，不发布 cmd，不调用 set_touch；`get_info` 是只读 RPC。
`--side left|right|both` 使用 wire Side：left=1、right=2，不使用设备寄存器的左右编号。
实际运动须有本轮吊装/可靠支撑、看护、清空夹点与急停确认，再由现场执行：

```bash
python examples/brainco_hand_external.py --side both --group default --timeout 5 --body-gains zero --confirm-motion
```

本体收到 50 Hz 的完整 42 槽 LowCmd：q 为进入前静止 LowState 的姿态，mode=10、
dq=tau=0。`--body-gains zero` 令所有槽位 kp=kd=0，q 仍填测得姿态；
零增益不提供姿态保持。本次吊装手部测试使用 zero。默认 `--body-gains hold`
保留原有行为，kp/kd **直接复用** `external_control.py` 的 `DEMO_KP/FIXED_STAND_KD`。
手指槽位为 RESERVED（原有手指增益为 0），手命令只走
`/sdk/foot_humanoid/hand/<side>/cmd`。不执行本体归零或回程轨迹；“保持”仍依赖设备、
通信及增益适配，不能替代吊装。姿态漂移和反馈陈旧检查沿用现有示例容差/时间窗。
进入 EXTERNAL 的 measured-hold/增益插值、断流 safe-hold 和故障域保护仍由机器人侧
执行；zero 参数只选择持续发送帧的目标增益，不保证进入及故障阶段的物理失能。

先核对 `BRAINCO_REVO2`、6 DOF、关节名和 info 限位，随后各关节慢速移到限位区间
25%→75%→25%（部分张合，避开端点）。默认 `--pattern simultaneous`、
`--speed-deg-s 1`、`--low-frac 0.25 --high-frac 0.75`，保留原来的一次往返。
速度转换为 float32 rad/s，必须为有限正数且不超过每个关节 info 声明的 dq_max；
比例必须满足 0 ≤ low < high ≤ 1。默认速度是原配置占位值，不是已测性能/电流/安全结论。
`--pattern sequential` 先把所有关节移到低位，然后按 info.joint_names 顺序逐个
低位→高位→低位，其余关节的命令保持低位；双手模式先左后右，一次只动一个关节。
`--loop` 持续重复，没有轮数或运行时长上限，直到 SIGINT/SIGTERM 或安全检查失败。
每个关节往返和每轮各打印一行摘要；周期性反馈由 `--feedback-interval` 控制，
默认 1 秒，不能小于 1 秒。`--touch on|off` 可在租约进入前调用 set_touch，默认 keep。
左右手有各自随机非零会话与递增序号；手会话和本体 EXTERNAL generation 相互独立。

右手逐个循环的命令示例（仍需本轮现场授权和安全确认）：

```bash
python examples/brainco_hand_external.py --side right --group default --timeout 5 --body-gains zero --pattern sequential --loop --speed-deg-s 20 --confirm-motion
```

正常结束、异常、Ctrl-C 与 `kill -INT <demo_pid>` / `kill -TERM <demo_pid>` 均停止
发送新目标并尝试 CANCEL，要求确认 PASSIVE；不发手部归零/回程命令，
手按服务端契约保持最后目标。PASSIVE 可使本体负载肢体下落；进程被杀死不能保证
CANCEL，租约无限期、不会自动到期。超时/故障退出后不自动重试运动，退出未确认须
现场处理。旧 `brainco_hand_control.py` 现在转到此入口，不再支持旧 `--spec`/LowCmd 手槽路径。
长时间运行须由现场看护电量和本体姿态；zero 不保持本体，偏离起始姿态超过
既有 0.05 rad 容差仍立即退出，不能为了持续循环放宽检查或自动重试。

打印 state 的 q/q_cmd/dq、会话/序号回执及七阶段 µs 时间链。三个入口时间
`middleware_received/received/dequeued` 相同，代表回调交付；worker 等待包含在
accepted→first_output。0 为未测阶段，只计算机器人 VitaNow 时钟内的差值；PC 时间与
CAN 硬件时间不参与跨域延迟计算。逐命令 `cmd_trace` 默认关闭；需要时由设备操作方
另行授权给 locomotion 进程设置 `LOCOMOTION_HAND_CMD_TRACE=1` 并按部署流程重启，
PC 环境变量不能打开机器人 trace。话题为 `/sdk/foot_humanoid/hand/<side>/cmd_trace`。

`HandClient` 支持 state/touch/info 订阅及 get_info/set_touch。只读采集入口
`examples/brainco_capture.py` 同步改为左右手 cmd/state/touch/info 独立话题，保留本体
LowCmd/LowState/ControlStatus 和离线 MCAP 导出（需另装 `hil` extra）。采集格式 v2 中
缺失/非有限手反馈写 JSON null 并标记，保留 raw words 和硬件时间，不生成假零。

## 部署避坑

- `env.sh` 不安装 FlatBuffers；完整 wheelhouse 已将它列为依赖，不用 `--no-deps`。
- 仅 S100 先 source 环境再激活 venv；PC 直接使用自己的 venv，检查 `command -v python`。
- 全部 SDK/demo 使用核验过的 peer profile；不要回退 client 或未配置的默认 session。
- 清理未核验的 PYTHONPATH 覆盖，确认导入当前 SDK，不能把 HIL 临时源码当正式版本。
- OTA 装到 B 不代表当前已运行 B；SDK 安装不包含切槽、重启或电机配置修改。
- 非手指关节被意外配置为 MOCK 会与 safe_hold 冲突，应修正配置而非关闭校验。
- `auto_record_enabled=false` 不是录制常开，需明确 START 并验证覆盖整个测试。

## 开发与离线打包

PR 的 `SDK latency report` 检查在 x86 上运行 Python SDK ↔ C++ Mock 通信测试，
50～1000 Hz 每档至少 60 秒，保存 HTML、原始 CSV、JSON 与双向频率/延迟统计。
复现方式和测量边界见 [latency CI](tests/latency/README.md)；这不是实机/HIL 验收。

维护者安装匹配的 Aorta wheels 后，在独立 venv 内执行：

```bash
python -m pip install --find-links wheelhouse -e . build
make check
make build
python tools/package_release.py --wheelhouse wheelhouse --output dist/offline-bundle
python tests/offline_install.py dist/offline-bundle
```

wheelhouse 须包含清单中的两种 Linux Aorta runtime、`aorta-msgs`、`vita-msgs`
与 FlatBuffers；本 HIL 分支的临时 vita_msgs 由交付方提供，`dependency_release_tag=null` 表示尚无发布 tag，现有 release 下载 CI 不能据此获得 HIL wheel。打包器
检查固定依赖 SHA-256，生成含每文件摘要和源码状态的 MANIFEST.json，保留 agent
软链接。只打包指定的公共 PC profile，拒绝已填入部署认证值的模板；不要把私有配置
放进源码。`make build` 只构建 wheel，源码、demo 和 PC profile 随 offline bundle 交付，不发布
会将软链接展开为普通文件的 setuptools sdist。输出目录存在时拒绝覆盖。
正式交付必须来自审核后的干净提交；本机测试、
原生库导入和离线安装不替代 aarch64 实机与现场运动验收。
