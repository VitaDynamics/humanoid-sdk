# Native 编解码与 ONNX 后端维护

0.3.0 的唯一 distribution 是 locomotion-aorta，内含 Body/EE native 编解码及按需加载的 Microsoft ORT 1.20.1 CPU 推理后端。导入 SDK 不加载模型；没有独立 backend 包或 Python ORT 降级。

## 构建和审计

使用 CMake 3.20+、C++17、目标 CPython 3.10 头文件与正确 SOABI。tools/fetch_ort.py 根据 ort-archives.json 获取官方对应架构归档并核 SHA；不借机器人 /app 动态库。

下面为独立开发环境的构建形状，所有产物路径须换成自己的新路径；这不是客户安装步骤：

```bash
python tools/fetch_ort.py --output /absolute/path/to/official-archive.tgz
python -m pip install build==1.2.2.post1 scikit-build-core==0.11.6 pybind11==2.13.6 ninja==1.11.1.3
CMAKE_ARGS='-DORT_ARCHIVE=/absolute/path/to/official-archive.tgz' \
  python -m build --wheel --no-isolation --outdir dist/raw .
python -m pip install -r demo_onnx/requirements-release.txt
python demo_onnx/tools/wheels.py --help
```

构建时通过 CMAKE_ARGS 的 ORT_ARCHIVE 指定已核归档，用 pyproject 固定构建依赖生成 raw wheel，再通过 wheels.py repair 按 manylinux_2_35_<arch> 审计。raw linux wheel 是中间物，不是可通过改名发布的产物。保留 raw wheel、调试符号、上游许可证和审计输出；不 strip、不放宽架构／ABI 检查。

发布收集两架构实际 repaired SDK wheel 后运行 wheels.py check-release。它核机器、ABI、许可证和布局，不执行外架构二进制，不代替设备验收。跨编译不能误用主机 Python 头文件。

## 检查

SDK 的 make check 与 demo_onnx/tests 是不同义务。native 完整 buffer 对比、真实生成绑定、独立 ORT oracle 和 ARM 安装分开记录；没有 native 时的 source-only skip 不能称通过。使用各 requirements-test/release/oracle 固定环境。native Body 编码和 DTO/NumPy 解码保留 GIL；publish_bytes 墙钟不是纯 Core 或网络时间。

历史构建合同与旧跨版本比较已移入 ARCHIVE；它们不构成当前验收。
