# SDK 与独立 C++ Mock 延迟基准

这是本机软件基准，不是机器人 HIL 或硬件延迟保证。当前测量必须固定 SDK、native、schema、Core、机器、调度和运行参数；旧 rc4/42/44 槽记录只保留在文档 ARCHIVE，不与当前 31 槽结果混用。

该目录的历史 Mock／测量合同须逐项核对后才用于当前接口；本次文档工作没有实际构建或运行它，不将其列为已经通过的 Body31／Control4 基准。

```bash
python benchmarks/latency/build_mock.py --help
python benchmarks/latency/run.py --help
```

在专用测试环境构建后运行，输出使用新目录，报告实际执行、缓存、跳过和错误。Mock 与真实设备命名空间／group 隔离，不把它接入机器人。状态生产、Python 接收、计算、native 编码、Core 调用和 applied 关联各自统计；嵌套阶段不重复求和。

不同主机 monotonic 时钟不能相减。设定频率不等于实测新采样率；先检查独立 cadence／计数，再解释分位数和最大值。make check、独立进程 integration 和 benchmark 是不同检查，不自动启动或重跑耗时测试。
