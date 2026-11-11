# 公共 topic Mock 集成工具

仅用于隔离环境的软件握手和消息验证，不是设备模拟安全证明，也不是机器人部署工具。使用匹配 Body31、Control4、统一 EE 的生成绑定和 native 组件。

```bash
python tools/production_topic_mock/run.py --help
```

工具创建私有 session/profile 和专用 group；有效 AORTA_GROUP 须与请求值一致。不要连接真实机器人 peer、复用其认证或发布到其控制域。构建、运行参数及输出目录按 --help 指定，保留真实失败和清理结果。

组契约单测可由 make check 执行。Mock 反馈不是电机采样，成功不表示设备支持、OTA 已部署、Stop／Hold 已验收或物理运动完成。
