# 只读末端采集与离线导出

`brainco_capture.py` 保留历史文件名，只订阅新 EE 和当前 Body/ControlStatus。
它不申请控制权、不发布运动或触觉写入。独立运动示例为
`examples/end_effector/brainco_revo2.py`，复用同一
[末端 CLI 与控制循环](../end_effector/README.md)；旧 `brainco_hand_external.py` 已移除。

本源码迁移到 Schema73 直接 runtime_id 合同；旧 S91220 wheel 并非匹配依赖。
最终专用消息 wheel、native 头、完整 SDK/IPC 与机器人组合仍待验证。取得经验证的完整交付后，在已授权连接且配置正确
peer profile/group 的主机上查看：

```bash
python examples/brainco_hand/brainco_capture.py --help
python examples/brainco_hand/brainco_capture.py --metadata /path/to/capture-metadata.json --output /new/path/ee-capture.jsonl live --duration-s 10 --group default
```

metadata 与字段约束见根 README；输出路径必须新建。JSONL 使用
`end_effector_capture_v4`、`left_ee_*`/`right_ee_*`，触觉 payload 是精确
uint8 列表。直接 runtime_id、SampleRef、时间/有效位和控制关联键保留；
缺失/非有限值不伪造健康零值，queue drop 与 transport loss 分开。

MCAP 导出核真实 root/BFBS、编码、CRC 和源/payload 哈希。历史 LowCmd/LowState
只在显式离线归档路径使用，无旧 Hand/Gripper 在线 fallback。观察者 marker
不映射成硬件时间，采集/导出不证明设备执行或 HIL 通过。Ctrl-C/TERM 保留部分
采集与结束状态；不进行重试、设备改参或服务重启。
