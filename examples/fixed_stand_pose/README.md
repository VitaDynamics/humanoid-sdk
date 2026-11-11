# FIXED_STAND 姿态辅助模块

`fixed_stand_pose.py` 是末端例程复用的本体姿态与前置检查辅助模块，不是独立可执行的通用归零工具。调用方从当前 BodyInfo／BodyState 和已确认配置构造完整 31 槽保持目标；MOCK 和不支持字段不伪装可控。

```bash
python examples/end_effector/end_effector_external.py --help
```

--ee-only 会省略 Body 输出；其他调用方式是否保持或改变本体由实际调用链决定。仅导入模块不产生运动，运行控制例程仍需独立授权。

## 公共前提

使用匹配 bundle 的 CPython 3.10 venv；依赖以根 `compatibility.json` 为准。PC 在创建 Node 前设置私有 `ZENOH_SESSION_CONFIG_URI`，并让 `AORTA_GROUP` 与 `--group` 一致。S100 本机方式先 source `/app/script/env.sh` 再激活 venv。

完整协议、单位、控制时序与图示见[SDK 手册](https://vitadynamics.feishu.cn/wiki/OG6KwVrf8i0fNdk2oGTc3nDKnFh)；新版页面以正式发布回读为准。

运动须单独授权并确认当前现场条件。ACK accepted 不是 RUNNING，publish 返回不是到位。正常退出 CANCEL→PASSIVE 可能使负载下落；异常、超时或退出不完整时不自动重试。
