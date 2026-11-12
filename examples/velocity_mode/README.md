# 速度策略示例

根据匹配的 locomotion 模型／profile，在 SDK 主机推理并通过 EXTERNAL 输出，不直接请求机器人 action 13。输入由固定 Aorta JoyReader 提供，不使用 ROS2、pygame 或无输入时的伪中立命令。

```bash
python examples/velocity_mode/velocity_mode.py --help
```

--shadow 只读控制面，不创建控制输出；--live 申请控制。两者需要实际 --locomotion-dir 和新的 --log 路径。duration 为本地运行时长，0／省略不限时。velocity-demo extra 及匹配模型须预先交付。

方向、动作缩放和观测取实际 profile／合同，不在文档中补默认值。日志保留输入、推理、发布阶段与实际 session/sequence；applied 键和软件时间戳不是硬件到位证据，跨主机 monotonic 时钟不相减。

## 公共前提

使用匹配 bundle 的 CPython 3.10 venv；依赖以根 `compatibility.json` 为准。PC 在创建 Node 前设置私有 `ZENOH_SESSION_CONFIG_URI`，并让 `AORTA_GROUP` 与 `--group` 一致。S100 本机方式先 source `/app/script/env.sh` 再激活 venv。

完整协议、单位、控制时序与图示见[SDK 手册](https://vitadynamics.feishu.cn/wiki/OG6KwVrf8i0fNdk2oGTc3nDKnFh)；新版页面以正式发布回读为准。

运动须单独授权并确认当前现场条件。ACK accepted 不是 RUNNING，publish 返回不是到位。正常退出 CANCEL→PASSIVE 可能使负载下落；异常、超时或退出不完整时不自动重试。
