# 本体状态只读例程

```bash
python examples/body_state_subscriber/body_state_subscriber.py --group "$AORTA_GROUP" --timeout 5
```

先通过 GetRobotInfo 绑定 runtime，再读取一帧公开 31 槽 BodyState。不创建控制 publisher，不申请租约、不发运动或 CANCEL。核对 runtime、状态身份／时间、有限数值与错误状态；一帧返回不代表长期稳定或新硬件采样。独立快照 API 为 BodyStateSource.get_robot_info → snapshot → BodyStateSnapshot.body_state；PullStateSource 使用 wait_state。

## 公共前提

使用匹配 bundle 的 CPython 3.10 venv；依赖以根 `compatibility.json` 为准。PC 在创建 Node 前设置私有 `ZENOH_SESSION_CONFIG_URI`，并让 `AORTA_GROUP` 与 `--group` 一致。S100 本机方式先 source `/app/script/env.sh` 再激活 venv。

完整协议、单位、控制时序与图示见[SDK 手册](https://vitadynamics.feishu.cn/wiki/OG6KwVrf8i0fNdk2oGTc3nDKnFh)；新版页面以正式发布回读为准。
