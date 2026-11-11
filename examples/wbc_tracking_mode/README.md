# WBC 跟踪示例

当前源码仍提供此入口；历史“以后移除”不是已删除。模型、profile、motion 和 schema 须匹配；wbc-tracking-demo extra 不包含用户运动文件。

```bash
python examples/wbc_tracking_mode/wbc_tracking_mode.py --help
```

--dry-run 离线，--replay-mcap 读取匹配公共 BodyState 的录包；--live --confirm-motion 才进入实机控制。--model 与新 --log 路径必填，其余实际参数以 --help 为准。模型输出、观测维度、state age、计算预算和末端作用域都须按当前实现检查，不用旧 42／44 槽适配掩盖不匹配。

内置 ORT 后端与机器人采用的 Python ORT 版本不是同一承诺；独立 oracle、native 测试、ARM 编译和现场动作分别验收，不用历史跨版本对比作当前结果。

## 公共前提

使用匹配 bundle 的 CPython 3.10 venv；依赖以根 `compatibility.json` 为准。PC 在创建 Node 前设置私有 `ZENOH_SESSION_CONFIG_URI`，并让 `AORTA_GROUP` 与 `--group` 一致。S100 本机方式先 source `/app/script/env.sh` 再激活 venv。

完整协议、单位、控制时序与图示见[SDK 手册](https://vitadynamics.feishu.cn/wiki/OG6KwVrf8i0fNdk2oGTc3nDKnFh)；新版页面以正式发布回读为准。

运动须单独授权并确认当前现场条件。ACK accepted 不是 RUNNING，publish 返回不是到位。正常退出 CANCEL→PASSIVE 可能使负载下落；异常、超时或退出不完整时不自动重试。
