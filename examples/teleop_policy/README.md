# 手柄策略 demo

对应机器人 action 17 的模型/合同，SDK 本地推理并请求 **EXTERNAL**；模型必须显式传入。本分支0.2.0直播输入为 pygame 手柄（`--joystick` 默认0），不能把后续0.2.1 Aorta Joy功能当成当前行为。完整公共31槽配置、策略关节安装状态与关节范围须通过代码检查。

```bash
python -m pip install --no-index --find-links ./wheelhouse 'locomotion-aorta[teleop-demo]==0.2.0'
python examples/teleop_policy/teleop_policy.py --help
python examples/teleop_policy/teleop_policy.py --model /path/to/action17.onnx --dry-run --body-state-jsonl /path/to/body-state.jsonl
```

离线JSONL首行为 BodyInfo，后续为BodyState和axes，实际格式见根README；离线不连接/不发布。直播命令为上面脚本加 `--model`、匹配 `--group`、`--confirm-motion`（不带dry-run）；先确认现场条件再执行。`action17_contract.json` 随目录提供，`--contract` 可显式指定匹配合同。输出为检查/异常信息，没有新增日志文件参数；周期与安全检查完全沿用现有代码。

## 环境与安全

以下命令从 SDK 仓库或离线发布包根目录运行。安装命令面向含完整 wheelhouse 的离线发布包（含所需 extra 依赖）；缺包时先获取匹配产物，不安装别的消息版本或绕过依赖检查。使用独立 CPython 3.10 venv；依赖版本以根目录 `compatibility.json` 为准。机器人上的 SDK 主机先 source `/app/script/env.sh`，再激活 venv；PC 使用部署方提供的 peer 配置。设置 `ZENOH_SESSION_CONFIG_URI` 和与机器人一致的 `--group`，不复制认证内容。安装与 `--help` 不代表已获得运动许可。

现场运动须单独授权并确认吊装/支撑、活动空间、夹点与可用急停；由现场人员操作。`--confirm-motion` 仅为程序确认参数。异常先停测试，由现场人员使用物理急停，不自动重试、清错或重启。运动例程正常退出走 CANCEL→PASSIVE；PASSIVE 可能使负载下落，客户端崩溃不能保证释放控制权。只读与离线模式不进入 EXTERNAL，不需要或发送 CANCEL。
