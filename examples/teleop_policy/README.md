# 本地遥操作策略示例

SDK 主机运行本地模型，再通过 EXTERNAL 提交 BodyCmd；不是给机器人内部策略直接上传 ONNX。使用匹配 --model 与 --contract，模型另行交付。公开踝坐标为 PR，不从内置 motor_theta 策略借用坐标合同。

```bash
python examples/teleop_policy/teleop_policy.py --help
```

--dry-run 使用离线输入检查模型和合同，不授予控制。实机入口会申请控制并要求 --confirm-motion。teleop-demo extra 只提供对应依赖；推理由 wheel 内按需加载的 C++ ORT 后端执行，无 Python ORT 降级。duration 省略或 0 可持续运行，按当前 --help 明确退出方式。

## 公共前提

使用匹配 bundle 的 CPython 3.10 venv；依赖以根 `compatibility.json` 为准。PC 在创建 Node 前设置私有 `ZENOH_SESSION_CONFIG_URI`，并让 `AORTA_GROUP` 与 `--group` 一致。S100 本机方式先 source `/app/script/env.sh` 再激活 venv。

完整协议、单位、控制时序与图示见[SDK 手册](https://vitadynamics.feishu.cn/wiki/OG6KwVrf8i0fNdk2oGTc3nDKnFh)；新版页面以正式发布回读为准。

运动须单独授权并确认当前现场条件。ACK accepted 不是 RUNNING，publish 返回不是到位。正常退出 CANCEL→PASSIVE 可能使负载下落；异常、超时或退出不完整时不自动重试。
