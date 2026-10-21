# AGX 夹爪 demo

独立GripperCmd/state/info入口，与本体共用EXTERNAL生命周期；默认只读。需要所选侧真实AGX、有效Info范围/ready及公共31槽本体反馈。动作不是旧本体槽位直控；不能用none末端验收。

```bash
python examples/gripper/gripper_external.py --help
python examples/gripper/gripper_external.py --side right --dry-run --group default --timeout 5
```

目标模式现场授权后使用 `--side right --target-m <实际Info内的总开口米值> --confirm-motion`；尖括号为必须替换的值，不能直接粘贴执行。`--speed-m-s 0`为机器人配置默认，`--torque-cmd-nm` 未填使用配置hold、显式0透传；它是电机侧软件力矩N·m，不是夹持力N或设备上限。`--pattern sine --side both` 使用代码的正弦流，范围必须落在实际Info；`--body-pose measured/zero`、`--body-gains hold/zero`、`--return-to-start` 与duration见help。程序输出命令/反馈和收尾信息，不把主机到时当设备到位；退出CANCEL→PASSIVE。零姿态/正弦均有运动风险。

## 环境与安全

以下命令从 SDK 仓库或离线发布包根目录运行。使用已安装当前兼容 wheel 的独立 CPython 3.10 venv；依赖版本以根目录 `compatibility.json` 为准。机器人上的 SDK 主机先 source `/app/script/env.sh`，再激活 venv；PC 使用部署方提供的 peer 配置。设置 `ZENOH_SESSION_CONFIG_URI` 和与机器人一致的 `--group`，不复制认证内容。安装与 `--help` 不代表已获得运动许可。

现场运动须单独授权并确认吊装/支撑、活动空间、夹点与可用急停；由现场人员操作。`--confirm-motion` 仅为程序确认参数。异常先停测试，由现场人员使用物理急停，不自动重试、清错或重启。运动例程正常退出走 CANCEL→PASSIVE；PASSIVE 可能使负载下落，客户端崩溃不能保证释放控制权。只读与离线模式不进入 EXTERNAL，不需要或发送 CANCEL。
