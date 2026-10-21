# 一次 BodyState 只读订阅

读取公共31槽状态，不请求EXTERNAL、不发BodyCmd，与任何运动动作号无关。需要部署方授权连接、匹配peer/group且机器人持续发布BodyState；不要求进入运动状态。没有额外demo依赖。

```bash
python examples/body_state_subscriber/body_state_subscriber.py --help
python examples/body_state_subscriber/body_state_subscriber.py --group default --timeout 5
```

输出一帧 `state_id/ts_state_real_us`，逐关节打印 `error_code/q/dq`，随后关闭状态源。`--timeout`默认为5秒，超时要如实报告，不代表机器人没有故障。只读无需CANCEL；终端停止不会为别的控制方释放租约，物理急停仍由现场人员负责。

## 环境与安全

以下命令从 SDK 仓库或离线发布包根目录运行。使用已安装当前兼容 wheel 的独立 CPython 3.10 venv；依赖版本以根目录 `compatibility.json` 为准。机器人上的 SDK 主机先 source `/app/script/env.sh`，再激活 venv；PC 使用部署方提供的 peer 配置。设置 `ZENOH_SESSION_CONFIG_URI` 和与机器人一致的 `--group`，不复制认证内容。安装与 `--help` 不代表已获得运动许可。

现场运动须单独授权并确认吊装/支撑、活动空间、夹点与可用急停；由现场人员操作。`--confirm-motion` 仅为程序确认参数。异常先停测试，由现场人员使用物理急停，不自动重试、清错或重启。运动例程正常退出走 CANCEL→PASSIVE；PASSIVE 可能使负载下落，客户端崩溃不能保证释放控制权。只读与离线模式不进入 EXTERNAL，不需要或发送 CANCEL。
