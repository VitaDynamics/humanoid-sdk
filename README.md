# 人形 SDK

基于 Aorta 的 Python SDK 与人形机器人示例，Python 导入名为
`locomotion_aorta`。客户端不需要 ROS 2、机器人主仓库或私有消息源码。
通常在用户自己的 Linux PC 或计算设备上运行 SDK，通过有线网络连接机器人的 S100；
不要求把 SDK 安装到 S100。

> 安装包从本仓库的 [Releases](https://github.com/VitaDynamics/humanoid-sdk/releases)
> 获取。`v0.1.0-rc.1` 是候选交付，Python 包版本为 `0.1.0`；仅在该版本的完整
> bundle 和校验清单均可下载时安装，不用其他版本的 wheel 替代缺失依赖。
> 候选包已通过 Linux x86_64 离线安装和真实消息绑定检查；这不替代该包在
> aarch64 设备上的连接与运动验收，运动示例仍须逐机审查、单独授权。

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
Python 3.10+、Aorta `2026.9.10+humanoid.7100871`。不要换用旧版消息包。
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
`ExternalController.enter_external(..., lease_duration_s=0)` 可向支持该语义的
服务端请求无限期租约；它不会关闭命令断流保护，结束时仍须 CANCEL。旧服务端可能
拒绝此请求，不能把拒绝当成获得控制权。现有 demo 仍使用有限的 90 秒租约。

## 部署避坑

- `env.sh` 不安装 FlatBuffers；完整 wheelhouse 已将它列为依赖，不用 `--no-deps`。
- 仅 S100 先 source 环境再激活 venv；PC 直接使用自己的 venv，检查 `command -v python`。
- 全部 SDK/demo 使用核验过的 peer profile；不要回退 client 或未配置的默认 session。
- 清理未核验的 PYTHONPATH 覆盖，确认导入当前 SDK，不能把 HIL 临时源码当正式版本。
- OTA 装到 B 不代表当前已运行 B；SDK 安装不包含切槽、重启或电机配置修改。
- 非手指关节被意外配置为 MOCK 会与 safe_hold 冲突，应修正配置而非关闭校验。
- `auto_record_enabled=false` 不是录制常开，需明确 START 并验证覆盖整个测试。

## 开发与离线打包

维护者安装匹配的 Aorta wheels 后，在独立 venv 内执行：

```bash
python -m pip install --find-links wheelhouse -e . build
make check
make build
python tools/package_release.py --wheelhouse wheelhouse --output dist/offline-bundle
python tests/offline_install.py dist/offline-bundle
```

wheelhouse 须包含清单中的两种 Linux Aorta runtime、消息包与 FlatBuffers；打包器
检查固定依赖 SHA-256，生成含每文件摘要和源码状态的 MANIFEST.json，保留 agent
软链接。只打包指定的公共 PC profile，拒绝已填入部署认证值的模板；不要把私有配置
放进源码。`make build` 只构建 wheel，源码、demo 和 PC profile 随 offline bundle 交付，不发布
会将软链接展开为普通文件的 setuptools sdist。输出目录存在时拒绝覆盖。
正式交付必须来自审核后的干净提交；本机测试、
原生库导入和离线安装不替代 aarch64 实机与现场运动验收。
