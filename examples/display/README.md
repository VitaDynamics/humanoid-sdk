# 屏幕显示例程（预发布）

独立的 Aorta Python 示例，支持外部接管、图片、GIF/动态 WebP、key/value 或全屏文字，
以及结束外部画面并切回 RCP。只调用显示接口，不申请本体运动 EXTERNAL 会话。

此例程从 SDK 1.0.2 RC1 bundle 起提供，不属于既有 1.0.0 RC1 或 1.0.1 RC3 bundle 的已交付能力。
服务端需包含显示接管开关，运行依赖需由部署方提供匹配的 Aorta Python SDK、
显示产品消息绑定、FlatBuffers 与 Pillow。1.0.2 RC1 引用的
`vbot-humanoid-msgs==0.1.7` 已包含本例所需绑定，配套 Aorta
为正式 `2026.11.23`（ABI 15）。既有 RC3 bundle 的原消息包不包含这些绑定；
不要跨代混装。1.0.2 RC1 完整 bundle 附带 Pillow 12.0.0 wheel，可离线安装：

```bash
python -m pip install --no-index --find-links ./wheelhouse Pillow==12.0.0
```

```bash
python examples/display/display_custom.py --help
```

在客户 PC 上，使用部署方提供的私有 peer 会话配置和显示通道权限；在创建 Node 前
设置实际 ZENOH_SESSION_CONFIG_URI 和 AORTA_GROUP，或使用脚本的 --config、--group。
group 优先级为显式 --group / Screen(group=...) 参数、AORTA_GROUP 环境变量、default；
省略参数时会保留部署方配置的组。
不要把本机 loopback 地址、内部凭证或机器人上的配置路径当作 PC 接入配置。

下例在依赖、会话配置与显示服务已就绪后运行：

```bash
python examples/display/display_custom.py takeover
python examples/display/display_custom.py image examples/display/assets/sample-image.png --landscape
python examples/display/display_custom.py animation examples/display/assets/basketball-dance.gif --landscape
python examples/display/display_custom.py text --item NAME=MY_APP --item STATUS=READY
python examples/display/display_custom.py rcp
```

图片和动图默认不注册资源，客户端一次发送 JPEG 帧，设备持续显示或循环。
`text --message` 配合调用方的字体文件可显示中文；`text --item` 使用设备 key/value 信息页。
`image --name` 才会持久化注册。`rcp` 结束外部画面、恢复默认显示，再释放接管；
正常退出脚本不会自动释放接管。`state` 查询开关，`release` 是 `rcp` 的别名。

素材：`sample-image.png` 为 854×480 横屏输入；`basketball-dance.gif` 为 8 帧循环；
`basketball-dance.mp4` 用于文档预览，不作为 animation 子命令输入。

此前已在配套设备的受信本机会话验证五项操作、中文图片文字及注册资源占用不增加；
最新服务端组合单独验证了接管、图片持续显示、拒绝 RCP 请求和释放恢复。
信息页当前从横屏逻辑坐标左、上各 16 像素处起排，面板偏移下的排版适配待后续实现，
不将早期本地排版适配结果作为当前服务端能力。
外部 PC 网络/权限和正式离线依赖包未随本例程交付，不把这项结果当作外部接入
或已发布 SDK 兼容性验收。正式手册入口见根 README 的已发布飞书链接。
