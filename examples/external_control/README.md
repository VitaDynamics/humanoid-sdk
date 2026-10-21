# 本体 EXTERNAL 轨迹 demo

仅吊装：从当前测得姿态过渡到零姿态，再回到保留肘部间隙的返回目标；会控制公共31槽中已安装的30关节。它不是普通站立/行走按钮，现有速度/增益/稳定性检查沿用代码。需配置和Info一致、反馈静止且有效、现场授权与急停。

```bash
python examples/external_control/external_control.py --help
```

完成现场确认后才执行：

```bash
python examples/external_control/external_control.py --group default --timeout 5 --confirm-motion
```

先输出preflight/轨迹时长和肘部返回目标，再请求EXTERNAL；丢失所有权、拒绝、反馈异常会中断。原CLI只有group/timeout/confirm-motion，不新增速度或轨迹参数。正常/异常收尾按原finally CANCEL并核PASSIVE；没有新增JSONL参数。

## 环境与安全

以下命令从 SDK 仓库或离线发布包根目录运行。使用已安装当前兼容 wheel 的独立 CPython 3.10 venv；依赖版本以根目录 `compatibility.json` 为准。机器人上的 SDK 主机先 source `/app/script/env.sh`，再激活 venv；PC 使用部署方提供的 peer 配置。设置 `ZENOH_SESSION_CONFIG_URI` 和与机器人一致的 `--group`，不复制认证内容。安装与 `--help` 不代表已获得运动许可。

现场运动须单独授权并确认吊装/支撑、活动空间、夹点与可用急停；由现场人员操作。`--confirm-motion` 仅为程序确认参数。异常先停测试，由现场人员使用物理急停，不自动重试、清错或重启。运动例程正常退出走 CANCEL→PASSIVE；PASSIVE 可能使负载下落，客户端崩溃不能保证释放控制权。只读与离线模式不进入 EXTERNAL，不需要或发送 CANCEL。
