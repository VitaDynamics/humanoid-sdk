# 080 / U0 baseline

Measured source: clean origin/main `b697f8bbdb76b3da4a8625e00aca212b93d5cd31`.
API v1.1 is approved for U1→U2→U5. No SDK implementation changes in this checkpoint.

Evidence: `/root/workspace/sdk-decode-w1019-evidence/U0-r2`.
The independently extracted archive/venv and pinned dependencies are in sibling `U0-r1`.
Archive SHA256: `062e7f1143d9d3caf547a90085f8c914a3d24c1fda19016ec72bcad1584812d9`.

- Original `make check`: 274 tests PASS, pip check, six example helps, CLAUDE.md symlink,
  git diff check PASS. Private index + archive worktree; no editable SDK install.
- `check-hil`: 19 tests PASS (subset of 274, not additional cases).
- Real x86 generated bindings: 17 BFBS hashes match compatibility.json; Aorta ABI 13.
- Original native Mock matrix: 50, 100, 200…1000 Hz, each active window ≥60 s;
  333007 sent = Python received = Mock received, missing state/reply = 0.
  All session/sequence/cmd_id/content flags and original eight-stage statistics
  independently recomputed from raw CSV. Private loopback topics, no robot connection.
- Full eager decode fixture: real generated public body31 bindings, 31 joints including
  legal unpopulated slot14 NaN, TRUNK/HIP; warmup50 + sample1000. Independent accessor oracle.
  Same-process MONOTONIC wall mean/P50/P95/P99/max:
  0.330630 / 0.285980 / 0.570448 / 0.586918 / 0.673643 ms.
  Thread CPU mean: 0.329580 ms. BASELINE_ONLY, no optimization benefit asserted.

U0 original matrix uses historical MONOTONIC cross-process fields; it is not a REALTIME
result. U5 adds directly sampled CLOCK_REALTIME points for all cross-process segments
while retaining the original matrix gate and historical fields.

Preserved failed evidence: U0-r1 check passed tests but failed final git diff check because
the archive lacked Git context. U0-r2 adds private Git context without changing source or
Makefile. First independent verifier assumed a nonexistent sent.csv session column; its
failed script/log remain preserved. Correct verifier binds each window to commands.json.

aarch64 packaging/runtime, device, live, customer migration, U3 NumPy and U4 native
implementation: NOT_MEASURED / not in this unit. Schema stays 0.1.2; SDK version/release
requires owner decision.
