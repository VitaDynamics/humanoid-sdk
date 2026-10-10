"""Exact-ID join and offline, self-contained latency report (stdlib only)."""
import csv
import html
import json
import math
from pathlib import Path
import statistics

STAGES = (
    ("publish_ns", "decode_ns", "状态发布 → Python 开始解析（通信与调度）"),
    ("decode_ns", "decoded_ns", "Python 解析并转换 44 槽状态"),
    ("decoded_ns", "built_ns", "Python 生成 42 槽回复对象"),
    ("built_ns", "fill_ns", "SDK 发布入口与消息头构造"),
    ("fill_ns", "filled_ns", "SDK 命令填充与序列化"),
    ("filled_ns", "bytes_ns", "FlatBuffer 收尾 → Aorta 字节发布入口"),
    ("bytes_ns", "callback_ns", "Aorta 发布入口 → Mock 回调（通信与调度）"),
    ("callback_ns", "copied_ns", "Mock 校验 FlatBuffer 并复制 42 槽命令"),
)


def read_rows(path):
    with Path(path).open(newline="") as stream:
        return [{k: int(v) for k, v in row.items()} for row in csv.DictReader(stream)]


def indexed(rows):
    result = {}
    for row in rows:
        seq = row["sequence"]
        if not 0 < seq <= 65000 or seq in result:
            raise ValueError(f"invalid/duplicate sequence {seq}")
        result[seq] = row
    if len(result) < 2:
        raise ValueError("fewer than two samples; not measured")
    return result


def stats(values):
    values = sorted(values)
    if not values or any(not math.isfinite(v) or v < 0 for v in values):
        raise ValueError("empty/negative/nonfinite measurement")
    def quantile(p):
        x = (len(values) - 1) * p
        lo = int(x)
        hi = min(lo + 1, len(values) - 1)
        return values[lo] + (values[hi] - values[lo]) * (x - lo)
    return {"n": len(values), "mean_ms": statistics.fmean(values),
            "p50_ms": quantile(.5), "p95_ms": quantile(.95),
            "p99_ms": quantile(.99), "max_ms": values[-1]}


def frequency(rows, stamp):
    times = sorted(r[stamp] for r in rows)
    if len(times) < 2 or times[-1] <= times[0]:
        raise ValueError("frequency requires a nonzero observation interval")
    return (len(times) - 1) * 1e9 / (times[-1] - times[0])


def analyze(directory, session, seconds, rate):
    sent = indexed(read_rows(directory / "sent.csv"))
    client = indexed(read_rows(directory / "client.csv"))
    received = indexed(read_rows(directory / "received.csv"))
    if set(sent) != set(range(1, len(sent) + 1)):
        raise ValueError("sender sequence is not contiguous")
    if not client.keys() <= sent.keys() or not received.keys() <= client.keys():
        raise ValueError("orphan response or client sample")
    for row in (*client.values(), *received.values()):
        if row["session"] != session:
            raise ValueError("wrong session")
    for row in received.values():
        if row["valid"] != 1 or row["cmd_id"] != row["sequence"]:
            raise ValueError("invalid command content/ID")
    active_ns = sent[max(sent)]["publish_ns"] - sent[1]["publish_ns"]
    if active_ns < seconds * 1e9:
        raise ValueError("send window shorter than requested")
    for row in sent.values():
        if row["return_ns"] < row["publish_ns"] or row["bytes"] <= 0:
            raise ValueError("invalid sender publication")
    for row in client.values():
        stamps = [row[k] for k in ("decode_ns", "decoded_ns", "built_ns", "fill_ns",
                                   "filled_ns", "bytes_ns", "returned_ns")]
        if stamps[0] <= 0 or stamps != sorted(stamps):
            raise ValueError("invalid Python stage order")
    joined = []
    for seq in sorted(received):
        row = {**sent[seq], **client[seq], **received[seq]}
        for start, end, _ in STAGES:
            if row[end] < row[start]:
                raise ValueError(f"non-monotonic stage {start}/{end}: {seq}")
        joined.append(row)
    with (directory / "joined.csv").open("w", newline="") as out:
        writer = csv.DictWriter(out, fieldnames=list(joined[0]))
        writer.writeheader()
        writer.writerows(joined)
    stages = [{"label": label, **stats([(r[end] - r[start]) / 1e6 for r in joined])}
              for start, end, label in STAGES]
    return {
        "rate_hz": rate, "requested_seconds": seconds, "active_seconds": active_ns / 1e9,
        "sent": len(sent), "python_received": len(client), "mock_received": len(received),
        "missing_state": len(sent) - len(client), "missing_reply": len(client) - len(received),
        "mock_send_hz": frequency(sent.values(), "publish_ns"),
        "python_receive_hz": frequency(client.values(), "decode_ns"),
        "python_send_hz": frequency(client.values(), "bytes_ns"),
        "mock_receive_hz": frequency(received.values(), "callback_ns"),
        "rtt": stats([(r["copied_ns"] - r["publish_ns"]) / 1e6 for r in joined]),
        "sdk_publish_call": stats([(r["returned_ns"] - r["built_ns"]) / 1e6 for r in client.values()]),
        "stages": stages,
    }


def render(directory, manifest):
    """Render failures too. A partial matrix must never acquire a PASS label."""
    directory = Path(directory)
    (directory / "results.json").write_text(json.dumps(manifest, indent=2, ensure_ascii=False) + "\n")
    results = manifest["results"]
    summary = ["# Python SDK ↔ C++ Mock latency", "", f"Status: **{manifest['status']}**", "",
               "|Hz|Mock send / Python receive Hz|Python send / Mock receive Hz|RTT P50 / P95 / P99 ms|Missing state / reply|",
               "|---:|---:|---:|---:|---:|"]
    body = []
    scale = max((r["rtt"]["mean_ms"] for r in results), default=1)
    colors = ["#3182bd", "#31a354", "#74c476", "#a1d99b", "#e6550d", "#fd8d3c", "#756bb1", "#9e9ac8"]
    for r in results:
        p = r["rtt"]
        summary.append(f"|{r['rate_hz']}|{r['mock_send_hz']:.2f} / {r['python_receive_hz']:.2f}|"
                       f"{r['python_send_hz']:.2f} / {r['mock_receive_hz']:.2f}|"
                       f"{p['p50_ms']:.3f} / {p['p95_ms']:.3f} / {p['p99_ms']:.3f}|"
                       f"{r['missing_state']} / {r['missing_reply']}|")
        bars = "".join(f'<span style="width:{s["mean_ms"] / scale * 100:.6f}%;background:{color}" '
                       f'title="{html.escape(s["label"])}: {s["mean_ms"]:.6f} ms"></span>'
                       for s, color in zip(r["stages"], colors))
        rows = "".join(f'<tr><td>{html.escape(s["label"])}</td>' +
                       "".join(f'<td>{s[key]:.6f}</td>' for key in ("mean_ms", "p50_ms", "p95_ms", "p99_ms", "max_ms")) +
                       '</tr>' for s in r["stages"])
        body.append(f'<section><h2>{r["rate_hz"]} Hz</h2>'
                    f'<p>实际发送窗口 {r["active_seconds"]:.3f} s；完整往返 {p["n"]} 帧；'
                    f'RTT 均值 {p["mean_ms"]:.6f} ms，P95 {p["p95_ms"]:.6f} ms。</p>'
                    f'<div class="timeline">{bars}</div><small>统一比例时间轴：全宽 {scale:.6f} ms；各段为同一批配对帧的均值，之和等于 RTT 均值。</small>'
                    f'<table><tr><th>阶段</th><th>均值 ms</th><th>P50</th><th>P95</th><th>P99</th><th>最大</th></tr>{rows}</table></section>')
    summary.extend(["", "Hardware/model/CAN/HIL: NOT_MEASURED. No profiler attached.",
                    "Loss is reported, not hidden; this is a report-integrity gate, not an absolute latency SLA."])
    if manifest.get("error"):
        summary.extend(["", "Failure: " + manifest["error"]])
    markdown = "\n".join(summary) + "\n"
    (directory / "summary.md").write_text(markdown)
    legend = "".join(f'<li style="color:{color}">{html.escape(label)}</li>' for (_, _, label), color in zip(STAGES, colors))
    charts = ""
    if results:
        charts = chart("往返延迟随发送频率变化", "ms", results,
                       [("RTT P50", [r["rtt"]["p50_ms"] for r in results]),
                        ("RTT P95", [r["rtt"]["p95_ms"] for r in results]),
                        ("RTT P99", [r["rtt"]["p99_ms"] for r in results])])
        charts += chart("实际发送与回复频率", "Hz", results,
                        [(name, [r[key] for r in results]) for name, key in
                         (("Mock 发送状态", "mock_send_hz"), ("Python 收到状态", "python_receive_hz"),
                          ("Python 发送回复", "python_send_hz"), ("Mock 收到回复", "mock_receive_hz"))])
    (directory / "report.html").write_text(
        '<!doctype html><html lang="zh-CN"><meta charset="utf-8"><title>SDK 通信延迟</title>'
        '<style>body{font:15px system-ui;max-width:1200px;margin:32px auto;color:#25334a;padding:16px}'
        'table{border-collapse:collapse;width:100%;margin:20px 0}td,th{padding:8px;border-bottom:1px solid #ddd;text-align:right}'
        'td:first-child,th:first-child{text-align:left}.timeline{height:32px;display:flex;background:#f3f4f6}'
        '.timeline span{display:block;min-width:0}section{margin:36px 0}pre{white-space:pre-wrap}</style>'
        '<h1>Python SDK ↔ C++ Mock 通信延迟</h1>'
        '<p>x86 同主机、两个进程、TCP loopback、SHM 关闭；44 槽状态 → 42 槽回复。'
        '时钟统一 CLOCK_MONOTONIC。通信段包含 Aorta/Zenoh 排队、调度及回调，不等于纯 TCP 传输。'
        'Mock 使用官方生成的 FlatBuffer 校验/访问器与内存复制，不运行 locomotion 控制循环。'
        '共享 CI runner 抖动不能外推 S100/HIL；分位数不可相加。SDK publish 调用耗时与往返部分重叠，不能重复相加。</p>'
        '<pre>' + html.escape(markdown) + '</pre>' + charts + '<ul>' + legend + '</ul>' + ''.join(body) + '</html>')


def chart(title, unit, results, series):
    colors = ["#3182bd", "#e6550d", "#31a354", "#756bb1"]
    xmax = max(r["rate_hz"] for r in results)
    ymax = max(v for _, values in series for v in values) * 1.05 or 1
    out = [f'<h2>{html.escape(title)}</h2><svg viewBox="0 0 950 290" role="img" aria-label="{html.escape(title)}">']
    for i in range(5):
        y = 240 - i * 55
        out.append(f'<path d="M70,{y}H910" stroke="#e5e7eb"/>'
                   f'<text x="5" y="{y}" font-size="12">{ymax * i / 4:.3f}</text>')
    for row in results:
        x = 70 + row["rate_hz"] / xmax * 840
        out.append(f'<text x="{x}" y="260" text-anchor="middle" font-size="12">{row["rate_hz"]}</text>')
    for (label, values), color in zip(series, colors):
        points = " ".join(f'{70 + r["rate_hz"] / xmax * 840},{240 - v / ymax * 220}' for r, v in zip(results, values))
        out.append(f'<polyline points="{points}" fill="none" stroke="{color}" stroke-width="2"/>')
        for row, value in zip(results, values):
            out.append(f'<circle cx="{70 + row["rate_hz"] / xmax * 840}" cy="{240 - value / ymax * 220}" r="3" fill="{color}">'
                       f'<title>{html.escape(label)}: {value:.6f} {unit} @ {row["rate_hz"]} Hz</title></circle>')
    out.append(f'<text x="15" y="12">{unit}</text><text x="750" y="285">目标发送频率 Hz</text></svg><p>')
    out.extend(f'<span style="color:{color};margin-right:24px">● {html.escape(label)}</span>' for (label, _), color in zip(series, colors))
    return "".join(out) + "</p>"
