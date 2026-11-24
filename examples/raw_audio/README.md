# 原始语音订阅示例

`raw_audio_subscriber.py` 只订阅 `/raw_audio_dump`，将收到的 PCM 块连续写入 WAV，
不发布音频、不控制机器人。依赖 SDK 配套的 Aorta 运行时、`aorta-msgs` 和 FlatBuffers；
使用已导出的 `foxglove.RawAudio`，无需额外显示消息包、NumPy 或音频处理库。
先按根 README 安装匹配的交付包并准备部署方提供的连接配置。
此例程从 SDK 1.0.2 RC1 bundle 起提供；既有 RC3 bundle 不包含本例程。

```bash
export ZENOH_SESSION_CONFIG_URI="$HOME/.config/vita/pc_session_peer.json5"
export AORTA_GROUP=default  # 使用部署方确认的实际组
python examples/raw_audio/raw_audio_subscriber.py --output raw_audio.wav --duration 10
```

默认保存所有通道；提取第一个通道：

```bash
python examples/raw_audio/raw_audio_subscriber.py --channel 0 --output channel0.wav
```

`--config` 和 `--group` 可显式覆盖对应环境变量。`AORTA_GROUP` 未设置或为空时
回退到 `default`；显式 `--group ""` 会报参数错误。`--timeout` 默认 5 秒，
首次收不到数据或接收中断均报错退出；Ctrl+C 会结束录制并收尾 WAV 文件。
建连前先检查输出文件是否已存在、父目录是否可用；实际打开时仍禁止覆盖，防止检查后的竞态。
出错前已收到的数据仍保留。短超时和录制末尾的单次接收等待不超过剩余时间。

消息不是整句语音或语音识别文本。`format=pcm-s16` 表示小端有符号 16 位 PCM，
`data` 按采样帧交错排列多个通道，采样率和通道数取自 `sample_rate`、
`number_of_channels`。每块时长为 `len(data) / (2 * channels * sample_rate)`；
例如 16 kHz、6 通道、6144 字节对应每通道 512 个采样、32 ms。
脚本不假定固定块大小，也不将多通道误当成单声道。PCM 按整块字节复制，
避免逐字节调用生成访问器导致接收处理跟不上实时音频；不增加 NumPy 依赖。

使用与发布端一致的 SensorData QoS（尽力交付），本地接收队列深度为 256。
丢包或队列溢出可能使 WAV 比录制墙钟时间短；脚本仅拼接收到的块，不补静音，
不保证无损录音。误订阅不兼容消息或 WAV 格式错误会以 `error: ...` 报错退出。
`--duration` 从收到首块开始计墙钟时间。
无数据时，应确认语音服务和该 topic 的导出已启用，以及网络、认证和 group 匹配；
脚本不会自动启动服务或修改机器人配置。

脚本也可在 S100 本机运行：先 source `/app/script/env.sh`，再激活匹配的虚拟环境，
使用部署方确认的本机 peer 会话配置。若环境脚本带入系统 ROS 的 `PYTHONPATH`，
可仅对本次 SDK 命令使用 `env -u PYTHONPATH python ...`，让虚拟环境加载自身依赖；
不要因此改动全局系统配置。WAV 始终保存在运行脚本的主机上。

本地检查覆盖真实生成消息解码、通道提取和 WAV 收尾；另以本机 Aorta 进程间通信发送
合成 PCM，校验六通道及单通道 WAV 的头和全部采样字节。
另在 S100 实机验证了六通道和单通道各 10 秒录制：16 kHz、每块 256 帧（16 ms），
分别保存 10.048 秒和 10.032 秒音频，观察到的消息序号连续。此结果仅代表该设备与版本，
不代表其他采集配置固定为 16 ms，也不保证尽力交付链路永不丢包；未进行人工试听验收。
