# FIXED_STAND 姿态辅助模块

这是供夹爪demo调用的 `FixedStandPose`，**没有独立CLI，也不会自己连接/进入EXTERNAL**。`fixed_stand_profile.json` 保存已有机器人零姿态增益/过渡合同，不修改设备或动作号；不能用它替代机器人现有FIXED_STAND动作。

```bash
python -c 'from examples.fixed_stand_pose.fixed_stand_pose import FixedStandPose; print(FixedStandPose.__name__)'
python examples/gripper/gripper_external.py --help
```

使用方必须传实际BodyInfo，核公共31个唯一名字、populated关节有效q范围且零姿态在范围内。`measured/check/duration/position/command` 辅助测量姿态与零姿态之间的既有插值；不新增范围或标定。它没有独立日志/退出命令；实际运动入口、日志、急停与CANCEL→PASSIVE由gripper demo管理。

## 环境与安全

以下命令从 SDK 仓库或离线发布包根目录运行。使用已安装当前兼容 wheel 的独立 CPython 3.10 venv；依赖版本以根目录 `compatibility.json` 为准。机器人上的 SDK 主机先 source `/app/script/env.sh`，再激活 venv；PC 使用部署方提供的 peer 配置。设置 `ZENOH_SESSION_CONFIG_URI` 和与机器人一致的 `--group`，不复制认证内容。安装与 `--help` 不代表已获得运动许可。

现场运动须单独授权并确认吊装/支撑、活动空间、夹点与可用急停；由现场人员操作。`--confirm-motion` 仅为程序确认参数。异常先停测试，由现场人员使用物理急停，不自动重试、清错或重启。运动例程正常退出走 CANCEL→PASSIVE；PASSIVE 可能使负载下落，客户端崩溃不能保证释放控制权。只读与离线模式不进入 EXTERNAL，不需要或发送 CANCEL。
