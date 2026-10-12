# 人形 SDK

基于 Aorta 的 Python SDK 与人形机器人示例，Python 导入名为
`locomotion_aorta`。客户端不需要 ROS 2、机器人主仓库或私有消息源码。
通常在用户自己的 Linux PC 或计算设备上运行 SDK，通过有线网络连接机器人的 S100；
不要求把 SDK 安装到 S100。

> 安装包从本仓库的 [Releases](https://github.com/VitaDynamics/humanoid-sdk/releases)
> 获取。当前源码迁移到 Aorta main 与 schemas-internal 的消息，拟使用
> `v0.1.0-rc.4` 承载候选依赖；该 tag/资产须由 owner 批准发布。
> Python 包版本仍为 `0.1.0`。旧 rc.1–rc.3 不包含本次迁移组合；仅在匹配的完整
> bundle 和校验清单均可下载时安装，不用其他版本的 wheel 替代缺失依赖。
> 本地离线安装与消息检查不替代 aarch64 设备上的连接与运动验收。

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
Python 3.10+、Aorta main `2026.9.23+main.8ff30fe`（ABI 13）、
`vita-msgs==0.1.0`（schemas-internal `b697157a`）与 FlatBuffers `25.9.23`。
这是临时精确 pin，待 release A/V 后统一更新；不要换用旧 fork 的消息包。
正式部署须使用经交付方核验的完整 bundle。以下在用户自己的 Linux 设备上，
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

## Revo2 手部专用测试入口（开发中，未发布/实机验收）

`examples/brainco_hand_control.py` 不使用全身归零 demo 的目标和增益。
先由现场审核人准备完整的 JSON 测试规格，再离线校验：

```bash
python examples/brainco_hand_control.py --spec reviewed-test.json
```

默认 `--mode validate` 不创建通信节点、不申请控制权。获得只读连接授权后可用
`--mode observe` 检查 200 ms 持续更新的全身反馈与批准基线。
只有显式 `--mode move --confirm-motion` 才申请 EXTERNAL；命令行确认不能替代本轮
现场授权、支撑、看护、独立停机手段和设备单位/限值/固件核验。不要把测试中的
synthetic fixture 作为真机规格。源码入口不等于已发布 bundle 已包含此功能。

规格必须明确提供以下字段，拒绝缺项、额外字段、重复 JSON key 和非有限值：

| 字段 | 约定 |
| --- | --- |
| `format_version` / `robot_profile` | `1` / `foot_humanoid_asample_brainco`，全身 A-sample |
| `reviewed_by` / `robot_config_sha256` / `device_preflight_evidence` / `stop_method` | 审核人、有效配置摘要、设备预检证据、已验证的现场停止方式；程序只记录规格摘要，不能远程核验这些声明 |
| `baseline` | 完整 42 项，每项显式 `mode/q/dq/tau/kp/kd`；静态基线 `mode=10,dq=0`，由现场审定，不能自动复制全身反馈或归零 |
| `hand_limits` | 以槽位字符串为 key，覆盖左 25–30、右 34–39，每项有 `lower_rad/upper_rad/max_target_rate_rad_s` |
| `target_positions_rad` | 仅选上述手槽的绝对目标，可单指、单手或双手；其他槽及字段保持批准基线不变 |
| `position_tolerance_rad` | 42 项明确的预检/跟踪/非目标漂移容差 |
| `move_duration_s` / `settle_duration_s` | 单向插值时长、末端目标继续发布的时长；结束时检查一帧新鲜且静止的反馈，不证明手端物理停止/释力 |
| `max_feedback_gap_s` / `max_status_gap_s` / `stationary_dq_rad_s` | 反馈/控制状态更新预算和静止速度阈值，须按设备验证后填写 |

所有手槽的 `dq/tau/kp/kd` 必须为零。50 Hz 是目标发布频率上限，不追赶补发；
`max_target_rate_rad_s` 仅限制软件目标斜率，不能替代手端速度、电流设置。
最多执行向目标的一段运动，不自动回程或重试；超出量化后的运动与末端保持时长
加 `max_feedback_gap_s` 的总预算即取消，不无限延长。SDK 的控制权仍无自动到期。
预算在程序恢复执行时检查，不能抢占阻塞的 I/O 或失去调度的进程，不构成硬实时停机保证。

程序输出 JSONL，区分 LowState、完整 42 槽命令尝试、SDK 调用返回及 CANCEL 结果。
命令返回不代表电机 ACK；PASSIVE 确认不代表手端停止，也可能使负载肢体下落。
正常、异常和 Ctrl-C 均尝试退出已发起的会话；退出失败保留 `UNCONFIRMED`，由现场
执行独立安全处置。当前 SDK 只有整帧时间/序号，没有逐电机新鲜度，且不能据此证明
跨时钟绝对反馈延迟；该入口不订阅 HandTouch、不替代触觉绑定与 MCAP 验收。

### 只读采集与 MCAP 导出（候选能力）

新增 `examples/brainco_capture.py`：`live` 有时限地订阅 HandTouch、44 槽 LowState、
42 槽 LowCmd 和 ControlStatus；只创建订阅者，不申请控制权。`mcap-export` 完全离线。
两者都要求版本/设备元数据、生成的 `aorta.topic.lowlevel.HandTouch` 绑定；缺失即拒绝，
不使用手写解码器。当前固定 `aorta_msgs` 不含该模块，需由交付方提供与候选机器人匹配的
附加 HIL wheel；它不属于已发布 SDK 的默认安装。MCAP 功能另需 `hil` extra（mcap 1.3.1）。

```bash
python examples/brainco_capture.py --help
python examples/brainco_capture.py --metadata reviewed-metadata.json --output new-trace.jsonl \
  live --duration-s 60 --group default --stdin-markers
```

第二条需要新的只读连接授权和已核验的 peer profile。按行输入的标记属于观察端时钟，
不与机器人时间混用。输出拒绝覆盖，保留部分失败文件，记录本地队列丢样；此计数不等于
传输/recorder 无丢包。MCAP 导出要求显式 topic 映射及每种 BFBS 的 SHA256，校验容器 CRC
和末尾，不猜 topic 后缀。`--help` 列出离线参数。取得候选依赖后用 `make check-hil` 验证；
普通 `make check` 在缺少可选 HIL 依赖时明确跳过该测试类，不代表 HIL 通过。

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
与 FlatBuffers；候选依赖从清单的 `dependency_release_tag` 获取。打包器
检查固定依赖 SHA-256，生成含每文件摘要和源码状态的 MANIFEST.json，保留 agent
软链接。只打包指定的公共 PC profile，拒绝已填入部署认证值的模板；不要把私有配置
放进源码。`make build` 只构建 wheel，源码、demo 和 PC profile 随 offline bundle 交付，不发布
会将软链接展开为普通文件的 setuptools sdist。输出目录存在时拒绝覆盖。
正式交付必须来自审核后的干净提交；本机测试、
原生库导入和离线安装不替代 aarch64 实机与现场运动验收。
