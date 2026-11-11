# 只读末端采集与离线分析

此入口采集公共消息和运行元数据，不申请控制、不发送手指目标或触觉开关。名称沿用 BrainCo，数据由明确 topic/metadata 合同描述，不能把缓存发布率当硬件采样率。

```bash
python examples/brainco_hand/brainco_capture.py --help
```

全局 --metadata、--output 必填；live 子命令另要求 --duration-s，mcap-export 子命令使用 input、--topic-map 和 --schema-hashes。先查看子命令 --help，使用匹配绑定和 hil extra。output 是新的 JSONL 文件路径，不覆盖既有采集；不公开原始身份、认证或完整负载。

sample_id／采集时间、消息发布、SDK 接收和操作者标记分别保留。离线导出或录包成功不代表触觉已开启、设备新鲜采样、关节到位或动作安全。

## 公共前提

使用匹配 bundle 的 CPython 3.10 venv；依赖以根 `compatibility.json` 为准。PC 在创建 Node 前设置私有 `ZENOH_SESSION_CONFIG_URI`，并让 `AORTA_GROUP` 与 `--group` 一致。S100 本机方式先 source `/app/script/env.sh` 再激活 venv。

完整协议、单位、控制时序与图示见[SDK 手册](https://vitadynamics.feishu.cn/wiki/OG6KwVrf8i0fNdk2oGTc3nDKnFh)；新版页面以正式发布回读为准。
