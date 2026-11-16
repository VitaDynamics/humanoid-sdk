# 本体 EXTERNAL 示例

```bash
python examples/external_control/external_control.py --help
```

此例程会请求 EXTERNAL 并移动本体，运行需要 --confirm-motion 和另行现场授权。先读取真实 BodyInfo／状态；使用公共 31 槽，踝命令和反馈为 PR，不是 A/B 电机角。目标／增益以当前脚本和配置为准，不复制旧 42 槽归零命令。

控制进入为发现→PREPARE→COMMIT→实际拥有的 RUNNING。SDK 检查结构、有限 float32、目标／字段、MOCK、能力和身份；设备范围留给服务器，普通 q/tau/kp/kd 裁剪、dq 越界拒绝。不会因为去掉 SDK 设备范围检查就绕过服务器安全门。

## Trace（开发中）

本体与统一末端 demo 使用相同参数。在原来已获授权的运动命令后增加：

```bash
--trace=fbs --trace-output=./body-trial.ftrace
```

默认 `--trace=off`；`--trace-compression=raw` 可关闭默认的 Zstd 压缩。
需要含 Trace native 模块的本 PR 候选 wheel，不代表既有 RC1 包已包含新功能。

本体例程只把 Trace 配置传入 PullController，其余仍调用原 SDK 接口。
SDK 记录原来那次 GetRobotInfo 回复，Info 仍只展示一次。一个文件记录
原静止预检、运动中状态读取、原完整归零／返回轨迹、实际本体命令键、
输入样本，以及 EXTERNAL 入场和原 CANCEL 结果。目标关节集合和所需字段
单独记录，不把未选槽位的 padding 当作运动目标。不会额外发现设备、读取
状态或创建控制线程；轨迹、限速、限位／MOCK／新鲜度门和退出路径保持。

这些是应用调用与已有反馈的记录，不是物理发送或到位证明。取件／解码细分、
模型／WTT、MCAP/HTML 和性能验收仍分别待接入；不要将文件完整性
当作设备测试通过。读取方式见末端 demo 的 Trace 说明。

## 公共前提

使用匹配 bundle 的 CPython 3.10 venv；依赖以根 `compatibility.json` 为准。PC 在创建 Node 前设置私有 `ZENOH_SESSION_CONFIG_URI`，并让 `AORTA_GROUP` 与 `--group` 一致。S100 本机方式先 source `/app/script/env.sh` 再激活 venv。

完整协议、单位、控制时序与图示见[SDK 手册](https://vitadynamics.feishu.cn/wiki/OG6KwVrf8i0fNdk2oGTc3nDKnFh)；新版页面以正式发布回读为准。

运动须单独授权并确认当前现场条件。ACK accepted 不是 RUNNING，publish 返回不是到位。正常退出 CANCEL→PASSIVE 可能使负载下落；异常、超时或退出不完整时不自动重试。
