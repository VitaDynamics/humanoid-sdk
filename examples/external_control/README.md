# 本体 EXTERNAL 示例

```bash
python examples/external_control/external_control.py --help
```

此例程会请求 EXTERNAL 并移动本体，运行需要 --confirm-motion 和另行现场授权。先读取真实 BodyInfo／状态；使用公共 31 槽，踝命令和反馈为 PR，不是 A/B 电机角。目标／增益以当前脚本和配置为准，不复制旧 42 槽归零命令。

控制进入为发现→PREPARE→COMMIT→实际拥有的 RUNNING。SDK 检查结构、有限 float32、目标／字段、MOCK、能力和身份；设备范围留给服务器，普通 q/tau/kp/kd 裁剪、dq 越界拒绝。不会因为去掉 SDK 设备范围检查就绕过服务器安全门。

## 公共前提

使用匹配 bundle 的 CPython 3.10 venv；依赖以根 `compatibility.json` 为准。PC 在创建 Node 前设置私有 `ZENOH_SESSION_CONFIG_URI`，并让 `AORTA_GROUP` 与 `--group` 一致。S100 本机方式先 source `/app/script/env.sh` 再激活 venv。

完整协议、单位、控制时序与图示见[SDK 手册](https://vitadynamics.feishu.cn/wiki/OG6KwVrf8i0fNdk2oGTc3nDKnFh)；新版页面以正式发布回读为准。

运动须单独授权并确认当前现场条件。ACK accepted 不是 RUNNING，publish 返回不是到位。正常退出 CANCEL→PASSIVE 可能使负载下落；异常、超时或退出不完整时不自动重试。
