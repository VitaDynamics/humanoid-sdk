# 强脑手 demo 与只读采集

`brainco_hand_external.py` 控制独立 HandCmd topic，并与本体共用一轮EXTERNAL；`brainco_capture.py` 只读采集/导出，不获取租约。需要真实 BrainCo 末端、匹配 HandInfo/state/touch 与公共31槽本体反馈；不能用 none 设备证明手部动作通过。

```bash
python -m pip install --no-index --find-links ./wheelhouse 'locomotion-aorta[hil]==0.2.1'
python examples/brainco_hand/brainco_hand_external.py --help
python examples/brainco_hand/brainco_hand_external.py --dry-run --side both --group default --timeout 5
python examples/brainco_hand/brainco_capture.py --help
python examples/brainco_hand/brainco_capture.py --metadata /path/to/capture-metadata.json --output /new/path/hand-capture.jsonl live --duration-s 10 --group default
```

metadata模板/捕获字段见根README；输出路径必须新建。手运动须移除dry-run并显式 `--confirm-motion`；`--side left/right/both`、`--body-gains zero/hold`、`--touch keep/on/off`，`--pattern simultaneous/sequential`、`--loop`、`--speed-deg-s`与`--low-frac/--high-frac`详见help，范围来自实际Info，不自定力矩/限值。打印反馈与退出信息；capture JSONL保留采样/来源时间、body/hand/touch及观察者标记，不推断触觉或设备确认。Ctrl-C/TERM走既有取消/收尾路径。

## 环境与安全

以下命令从 SDK 仓库或离线发布包根目录运行。安装命令面向含完整 wheelhouse 的离线发布包（含所需 extra 依赖）；缺包时先获取匹配产物，不安装别的消息版本或绕过依赖检查。使用独立 CPython 3.10 venv；依赖版本以根目录 `compatibility.json` 为准。机器人上的 SDK 主机先 source `/app/script/env.sh`，再激活 venv；PC 使用部署方提供的 peer 配置。设置 `ZENOH_SESSION_CONFIG_URI` 和与机器人一致的 `--group`，不复制认证内容。安装与 `--help` 不代表已获得运动许可。

现场运动须单独授权并确认吊装/支撑、活动空间、夹点与可用急停；由现场人员操作。`--confirm-motion` 仅为程序确认参数。异常先停测试，由现场人员使用物理急停，不自动重试、清错或重启。运动例程正常退出走 CANCEL→PASSIVE；PASSIVE 可能使负载下落，客户端崩溃不能保证释放控制权。只读与离线模式不进入 EXTERNAL，不需要或发送 CANCEL。

## 0.2.1 拉取式末端反馈

此demo选择 `pull_feedback=True`，状态/Info（及手touch）在调用方拉取解码，无Python接收回调线程；仍保留Python末端编解码。运动循环保持固定节拍，反馈高频不会提高命令频率。BrainCo capture 使用有界回调队列采集状态与命令，显式报告队列丢弃；传输丢失未测，不能宣无损。
