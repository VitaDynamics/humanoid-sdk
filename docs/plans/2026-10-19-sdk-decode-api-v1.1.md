# W1019/080 API v1.1 — 协调者批准冻结

基线 origin/main `b697f8bbdb76b3da4a8625e00aca212b93d5cd31`。
仅 body31 / public schema 0.1.2，按协调者批准进入 U1→U2→U5。

```python
@dataclass(frozen=True, slots=True)
class FrameToken:
    source_id: str       # 每个source实例独有UUID，代表其本地代次
    sequence: int        # 本实例接收序号；不是机器人state_id

class StateSourceClosedError(RuntimeError): ...
class StateSourceTimeoutError(TimeoutError): ...

class BodyStateSource:
    def __init__(self, *, group: str,
                 conversion: Literal['eager', 'on_demand'],
                 node_name: str | None = None) -> None: ...
    def snapshot(self, timeout: float, *,
                 after_token: FrameToken | None = None) -> BodyStateSnapshot: ...
    def close(self) -> None: ...

@dataclass(frozen=True, slots=True)
class BodyStateMetadata:
    state_id: int
    protocol_version: int
    roster_version: int
    ts_state_real_us: int
    ts_state_pub_us: int
    ts_consumed_us: int
    attitude_valid: bool

class BodyStateSnapshot:
    token: FrameToken
    received_monotonic_ns: int
    received_realtime_ns: int      # 回调中直接读取time.time_ns()
    metadata: BodyStateMetadata  # 只读，按需构造并帧内缓存
    def body_state(self) -> BodyState: ...
    # 以下U2实现，U1不提供空实现/伪结果。
    def select(self, *, joint_indices: tuple[int, ...],
               fields: tuple[str, ...],
               imu_location: int | None = None) -> JointSelection: ...
    def applied_cmd(self) -> BodyAppliedCmd | None: ...

class JointSelection:
    def columns(self) -> JointColumns: ...
    def imu_sample(self) -> BodyImuSample | None: ...

@dataclass(frozen=True, slots=True)
class JointColumns:
    token: FrameToken
    received_monotonic_ns: int
    received_realtime_ns: int      # 回调中直接读取time.time_ns()
    metadata: BodyStateMetadata
    joint_indices: tuple[int, ...]  # 显式请求顺序，不自动过滤slot14
    values: Mapping[str, memoryview]  # U2只支持q/dq，只读float32列
    body_index: memoryview         # uint16
    populated: memoryview          # uint8 0/1
    is_mock: memoryview            # uint8 0/1
    status: memoryview             # uint8
    valid_fields: memoryview       # uint16
    ts_acquired_hw_us: memoryview  # uint64
    ts_ingested_us: memoryview     # uint64
    ts_assembled_us: memoryview    # uint64
    lost: memoryview              # uint32

# 新增keyword-only参数；其余参数、默认分支与方法签名保持。
ExternalController(..., state_source: BodyStateSource | None = None)
```

1. source只创建一个BodyState subscriber；不创建控制publisher/client或租约。
   controller显式借入时不再订阅BodyState；ACK/status/command原路径保持。
   同group且source仍open才可借入，配置错误在创建控制transport前报告。
2. source创建者负责close；controller不关闭借入source。两者自身close均须唤醒
   各自等待者；source close不是CANCEL。controller.wait_body_state仍返回DTO，
   保持“可返回当前缓存”的行为，source超时映射为原ExternalControlTimeoutError。
3. payload bytes独立拥有；非bytes先copy。旧快照/列不被后帧覆盖；列以immutable
   bytes为所有者，不暴露可写array底层。Mapping只读，不共享全局可写输出池。
4. after_token只要求本source后来收到一帧，跨source token/非法选择参数报ValueError。
   不替代state_id、producer时间、新鲜度、唯一HIP、valid/populated/finite等旧guard。
5. snapshot(timeout)允许0：有符合after_token条件的缓存立即返回；无符合条件帧
   立即抛StateSourceTimeoutError。负数/NaN抛ValueError；正inf表示显式不限时等待。
   close优先于缓存，close后调用/等待均抛StateSourceClosedError。
6. 同一快照同请求以锁保护缓存，成功值只构造一次；解码失败保留原异常类型，
   不回旧帧或补零。未请求的IMU/applied/数值区域可延迟验证，成功selected不表示
   已验证整包；body_state仍走原完整eager decoder。
7. fields须非空、无重复、仅q/dq；joint_indices显式给出、无重复且在31槽范围。
   metadata/masks/时钟原样保留，Info role/valid_limits不混入状态有效性。
   imu_sample只在请求时按location选择，保留缺失/状态null/无效位；同location重复
   报歧义。applied_cmd单独按需解码，保持其独立来源key。
8. U1/U2不公开numpy/copy_into；不新增NumPy依赖。ddq/tau/temp扩展、U3、native
   实现、示例默认切换、客户程序迁移均不在本轮。版本仍由owner另定。

U0测量先锁现有eager全DTO：真实generated synthetic fixture（含合法slot14 NaN），
显式warmup50/sample1000，CLOCK_MONOTONIC wall + thread CPU，输入SHA，输出oracle。
U5在上述口径上扩展同合同A/B和signed delta；保留原11×60s矩阵门禁。
订阅回调入口连续直接读取time.monotonic_ns()与time.time_ns()，不做时钟换算。
U5所有跨进程分段只使用CLOCK_REALTIME；同进程耗时可用monotonic，并明确标注。
原U0矩阵的历史monotonic字段仅作为原门禁基线，不冒称REALTIME结果。

协调者2026-10-19评审：除上述REALTIME和timeout边界修订外同意冻结；
v1.1写回后直接推进U1→U2→U5，无需再次等待批准。v1保留供审计。
