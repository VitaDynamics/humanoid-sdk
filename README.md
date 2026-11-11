# Humanoid SDK

人形 Python SDK：本体 31 槽控制／状态、统一末端与手指交互，以及按需加载的本地 ONNX 示例。Python 导入名为 `locomotion_aorta`。

- [对外文档首页](https://vitadynamics.feishu.cn/wiki/Mu1hw8wcSiKw3ykDGtAcvgXpnAg)
- [SDK 手册稳定入口](https://vitadynamics.feishu.cn/wiki/OG6KwVrf8i0fNdk2oGTc3nDKnFh)
- [Releases 与实际交付包](https://github.com/VitaDynamics/humanoid-sdk/releases)

本次当前接口手册及归档正在更新；新页是否上线以实际发布回读为准，不提供未发布页面链接。客户不需要机器人主仓库、Bazel 或私有 schema 源码。

## 当前版本

源码版本 **0.3.0**，不是 1.0 发布公告。安装必须有匹配机器人版本的实际 SDK wheel 和完整 bundle，不能把依赖预发布当 SDK 正式版，也不能由源码版本推断产物已可下载。

| 项目 | 支持范围 |
| --- | --- |
| Python / 主机 | CPython 3.10.x；Linux x86_64 或 aarch64；发布基线 manylinux_2_35 |
| Body | BodyCmd3 / BodyState3；31 槽；公开踝坐标 PR |
| Control | 请求、ACK、状态协议 4 |
| EndEffector | Info1 / Cmd2 / State2 / Touch1；SetTouch2 |
| 依赖 | 固定组合见 `compatibility.json`；不跨代混装 |
| native | SDK wheel 内置编解码和按需加载的 ONNX 组件，无纯 Python 降级包 |

## 安装与只读检查

以下从已交付、确含 0.3.0 SDK wheel 的完整 bundle 根目录运行。SDK 默认在用户 PC；只有明确在 S100 本机运行时，先 source `/app/script/env.sh`，再激活 venv。使用新环境，保留旧安装。

```bash
python3.10 -m venv venv
source venv/bin/activate
python -m pip install --no-index --find-links ./wheelhouse 'locomotion-aorta==0.3.0'
python -m pip check
python examples/body_state_subscriber/body_state_subscriber.py --help
```

随后按配套网络手册在仓库外准备私有 `config/pc_session_peer.json5` 副本，填写部署方给出的网络、namespace 和认证。不要直接使用仍含占位符的模板。Node 创建前设置真实绝对路径及实际 group：

```bash
export ZENOH_SESSION_CONFIG_URI="$HOME/.config/vita/pc_session_peer.json5"
export AORTA_GROUP=default  # 仅在部署方确认此组时使用
python examples/body_state_subscriber/body_state_subscriber.py --group "$AORTA_GROUP" --timeout 5
```

当前 Aorta Core 的有效 group 由 `AORTA_GROUP`／默认值决定；仅设置 `--group` 或 `SDK_GROUP` 不够。不能只改 session mode、删除认证或复制机器人回环地址给 PC。

## 示例

| 需求 | 入口 |
| --- | --- |
| 本体只读 | `examples/body_state_subscriber/body_state_subscriber.py` |
| 末端只读 | `examples/end_effector_state_subscriber/end_effector_state_subscriber.py` |
| 本体控制 | `examples/external_control/external_control.py` |
| 手指／夹爪／OP3 | `examples/end_effector/`，各设备入口及 `--help` |
| 本地模型 | `examples/teleop_policy/`、`examples/velocity_mode/`、`examples/wbc_tracking_mode/` |
| 只读 MCAP | `examples/brainco_hand/` |

`--help`、安装或只读通过不授予运动许可。动作须单独授权并确认可靠支撑、活动／夹持区域无人、有人看护和急停可用。进入 ACK 不等于实际 RUNNING；正常退出走 CANCEL→PASSIVE，可能使负载肢体下落。客户端异常退出不保证释放所有权，不自动重试动作。

## 开发与维护

参考手册在独立文档仓维护，本仓只保留真实入口和开发说明。历史说明已标记 ARCHIVE，保存在 `docs/archive/`，不作为当前合同。SDK 结构／有限数值／身份和能力校验仍保留；本体设备范围由服务器 EXTERNAL 执行，普通 q/tau/kp/kd 裁剪、dq 越界拒绝。

本地检查入口为 `make check`；native 安装测试、生成绑定测试和设备验收分别记录。ONNX 构建与审计见 `demo_onnx/README.md`。`AGENTS.md` 为 agent 规则唯一正文，`CLAUDE.md` 保持相对软链接。不要在安装步骤启动运动示例。
