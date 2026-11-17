"""Create a self-contained offline HTML/CSV report from an existing recording.

Never connects to a robot. Source recordings remain byte-for-byte unchanged.
"""
import argparse
import csv
import html
import json
import math
from pathlib import Path

from .index import TraceIndex, file_identity, uint_text
from .session import Kind, Outcome
from .summary import summarize_index

PAGE_SIZE = 128
CHART_BUCKETS = 128
KEY_FIELDS = ('runtime_id', 'control_id', 'session_id', 'command_sequence')


def _safe(value):
    """JSON for browsers: exact identities, masked nonfinite values, no HTML."""
    if isinstance(value, dict):
        return {key: _safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_safe(item) for item in value]
    if isinstance(value, int) and abs(value) > 2**53 - 1:
        return str(value)
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


def _json(path, value):
    path.write_text(json.dumps(_safe(value), ensure_ascii=False, indent=2,
                               allow_nan=False) + '\n', encoding='utf-8')


def _cell(value):
    return html.escape('未知' if value is None else str(value))


def _table(headers, rows):
    return '<table><thead><tr>' + ''.join('<th>' + _cell(v) + '</th>' for v in headers) + \
        '</tr></thead><tbody>' + ''.join('<tr>' + ''.join('<td>' + _cell(v) + '</td>'
                                                       for v in row) + '</tr>' for row in rows) + '</tbody></table>'


def _page(path, title, body, *, root='.'):
    path.write_text('<!doctype html><html lang="zh-CN"><meta charset="utf-8">'
        '<meta name="viewport" content="width=device-width,initial-scale=1">'
        '<meta http-equiv="Content-Security-Policy" content="default-src \'none\'; '
        'style-src \'self\'; script-src \'self\'; img-src \'self\' data:">'
        '<title>' + _cell(title) + '</title><link rel="stylesheet" href="' + root + '/assets/report.css">'
        '<body><header><a href="' + root + '/index.html">FBS TRACE / 离线分析</a></header><main><h1>' +
        _cell(title) + '</h1>' + body + '</main><script src="' + root +
        '/assets/report.js"></script></body></html>', encoding='utf-8')


def _commands(index):
    db = index.db
    db.execute('''CREATE TEMP TABLE commands (
        id INTEGER PRIMARY KEY, producer INTEGER, attempt TEXT, stream INTEGER,
        runtime TEXT, control TEXT, session TEXT, sequence TEXT, begin_us TEXT,
        end_us TEXT, outcome TEXT, duration_us TEXT, status TEXT, body TEXT)''')
    # An attempt is local to a producer. A command key is assigned later. Entry
    # without return, failure before key reservation, and duplicate events stay
    # visible rather than being silently classified as successful sends.
    for producer, attempt, stream in db.execute('''SELECT DISTINCT producer,attempt,stream
            FROM events WHERE kind IN (?,?) AND stage IN ('body_publish','endpoint_publish')
            ORDER BY producer,attempt''', (Kind.BODY_COMMAND, Kind.EFFECTOR_COMMAND)):
        entries, ends, reserved = [], [], []
        count = 0
        detail = []
        for event_id, row in index.rows('producer=? AND attempt=?', (producer, attempt)):
            count += 1
            if len(detail) < 64:
                detail.append(dict(event_id=event_id, **row))
            stage = index.header['stages'][row['stage']-1] if row['stage'] else ''
            if stage == 'key_reserved':
                if len(reserved) < 2:
                    reserved.append(row)
            elif row['kind'] in (Kind.BODY_COMMAND, Kind.EFFECTOR_COMMAND) and stage in (
                    'body_publish', 'endpoint_publish'):
                selected = entries if row['outcome'] == Outcome.UNKNOWN else ends
                if len(selected) < 2:
                    selected.append(row)
        unique = len(entries) == len(ends) == 1
        entry = entries[0] if len(entries) == 1 else None
        end = ends[0] if len(ends) == 1 else None
        key_row = end if end and all(end[k] for k in KEY_FIELDS) else (
            reserved[0] if len(reserved) == 1 else {})
        status = 'COMPLETE_CALL' if unique else 'AMBIGUOUS' if len(entries) > 1 or len(ends) > 1 else 'INCOMPLETE_CALL'
        if unique and (entry['clock'], entry['epoch'], entry['stream']) != (end['clock'], end['epoch'], end['stream']):
            status = 'AMBIGUOUS'
        if unique and end['timestamp_us'] < entry['timestamp_us']:
            status = 'INVALID_CLOCK_ORDER'
        if end and end['duration_us'] is not None and unique and status == 'COMPLETE_CALL':
            if end['duration_us'] != end['timestamp_us'] - entry['timestamp_us'] + 1:
                status = 'INVALID_DURATION'
        body = dict(producer=producer, attempt_id=int(attempt), stream=stream,
                    association=status, events=detail, detail_events=count,
                    detail_truncated=count > len(detail),
                    physical_completion='NOT_AVAILABLE', hardware_latency_us=None,
                    missing_evidence=['matched new hardware sample', 'verified command-to-sample association'],
                    note='SDK return/key reservation do not prove robot admission or physical output.')
        db.execute('INSERT INTO commands VALUES(NULL,?,?,?,?,?,?,?,?,?,?,?,?,?)', (
            producer, attempt, stream, *(uint_text(key_row.get(k, 0)) for k in KEY_FIELDS),
            uint_text(entry['timestamp_us']) if entry else None,
            uint_text(end['timestamp_us']) if end else None,
            Outcome(end['outcome']).name if end else 'UNKNOWN',
            uint_text(end['duration_us']) if end else None, status, json.dumps(body)))
    db.execute('CREATE INDEX command_key ON commands(runtime,control,session,sequence,stream)')


def _export_commands(index, destination):
    db = index.db
    count = db.execute('SELECT count(*) FROM commands').fetchone()[0]
    pages = (count + PAGE_SIZE-1) // PAGE_SIZE
    with (destination / 'commands.csv').open('x', newline='') as out, \
            (destination / 'unmatched.csv').open('x', newline='') as unmatched:
        writer, unknown = csv.writer(out), csv.writer(unmatched)
        writer.writerow(('command_row', 'producer', 'attempt_id', 'stream', *KEY_FIELDS,
                         'entry_us', 'return_us', 'outcome', 'sdk_duration_us', 'call_integrity',
                         'hardware_latency_us', 'hardware_association'))
        unknown.writerow(('command_row', 'reason', 'hardware_latency_us'))
        for page in range(pages):
            cards = []
            for row in db.execute('SELECT * FROM commands ORDER BY id LIMIT ? OFFSET ?',
                                  (PAGE_SIZE, page*PAGE_SIZE)):
                command_id, producer, attempt, stream, *rest = row
                runtime, control, session, sequence, begin, end, outcome, duration, status, raw = rest
                numeric = [int(v) if v is not None else None for v in (
                    attempt, runtime, control, session, sequence, begin, end)]
                writer.writerow((command_id, producer, numeric[0], stream, *numeric[1:], outcome,
                                 int(duration) if duration else None, status, '', 'NOT_AVAILABLE'))
                unknown.writerow((command_id, 'No proven command-linked new hardware sample', ''))
                body = json.loads(raw)
                key = ' / '.join(str(int(v)) for v in (runtime, control, session, sequence))
                cards.append('<article data-filter="' + _cell(f'{stream} {outcome} {status} {int(attempt)}') +
                    '" id="c' + str(command_id) + '"><h2>#' + str(command_id) +
                    ' · stream ' + str(stream) + ' · ' + _cell(outcome) + '</h2><p>R / control / session / sequence: <code>' +
                    _cell(key) + '</code></p><p>SDK 耗时 ' + _cell(int(duration) if duration else None) +
                    ' µs · ' + _cell(status) + ' · 设备延迟：缺关联证据</p>' +
                    '<details><summary>入口、预留键、编码与返回的原始记录</summary><pre>' +
                    html.escape(json.dumps(_safe(body), ensure_ascii=False, indent=2, allow_nan=False)) +
                    '</pre></details></article>')
            nav = '<nav>' + ('<a href="commands-' + str(page-1) + '.html">上一页</a> ' if page else '') + \
                  f'第 {page+1} / {pages} 页（每页最多 {PAGE_SIZE} 条） ' + \
                  ('<a href="commands-' + str(page+1) + '.html">下一页</a>' if page+1 < pages else '') + '</nav>'
            _page(destination / 'report/details' / f'commands-{page}.html', '命令逐条核对',
                  nav + '<label>筛选本页（stream / outcome / attempt）<input id="filter"></label>' +
                  ''.join(cards) + nav, root='..')
    return count, pages


def _histogram(index, category, stage, stream, outcome):
    where = 'category=? AND stage=? AND stream=? AND outcome=? AND duration IS NOT NULL'
    params = (category, stage, stream, outcome)
    db = index.db
    count, low, high = db.execute('SELECT count(*),min(duration),max(duration) FROM timings WHERE ' + where, params).fetchone()
    if not count:
        return '<p>没有有效耗时；不以 0 填充。</p>'
    low, high = int(low), int(high)
    bins = [0] * 32
    # One ordered scan over this group; memory is fixed regardless of run length.
    for value, in db.execute('SELECT duration FROM timings WHERE ' + where, params):
        bins[min(31, (int(value)-low)*32//max(1, high-low+1))] += 1
    top = max(bins)
    bars, points, cumulative = [], [], 0
    for i, value in enumerate(bins):
        x, height = 40+i*18, value/top*130
        bars.append(f'<rect x="{x}" y="{160-height}" width="16" height="{height}"/>')
        cumulative += value
        points.append(f'{x+8},{160-cumulative/count*130}')
    return '<svg viewBox="0 0 650 200" role="img" aria-label="耗时直方图和累计分布">' + \
        ''.join(bars) + '<polyline class="cdf" points="' + ' '.join(points) + '"/>' + \
        f'<text x="40" y="190">{low} µs</text><text x="500" y="190">{high} µs</text>' + \
        '</svg><p class="muted">柱：32 桶频数；线：累计分布。分位数仍按原始样本精确计算。</p>'


def _feedback(index, destination):
    """Observed position only, using recorded masks. No nearest-command join."""
    db = index.db
    db.execute('CREATE TEMP TABLE positions(stream INTEGER,axis INTEGER,stamp TEXT,value REAL,state TEXT,runtime TEXT)')
    for _, row in index.rows('kind IN (?,?)', (Kind.BODY_FEEDBACK, Kind.EFFECTOR_FEEDBACK)):
        if not row['values'] or not row['integers']:
            continue  # Linked cache views are not new payload samples.
        axes = row['integers'][0]
        if not 0 < axes <= 31 or len(row['values']) < axes or len(row['validity']) < axes:
            continue
        for axis in range(axes):
            value = row['values'][axis]
            if row['validity'][axis] and value is not None and math.isfinite(value):
                db.execute('INSERT INTO positions VALUES(?,?,?,?,?,?)', (row['stream'], axis,
                    uint_text(row['timestamp_us']), value, uint_text(row['state_id']), uint_text(row['runtime_id'])))
    charts = []
    for stream, axis, runtime, low, high, count in db.execute(
            'SELECT stream,axis,runtime,min(stamp),max(stamp),count(*) FROM positions GROUP BY stream,axis,runtime'):
        start, end = int(low), int(high)
        buckets = {}
        for stamp, value in db.execute('SELECT stamp,value FROM positions WHERE stream=? AND axis=? AND runtime=? ORDER BY stamp',
                                      (stream, axis, runtime)):
            stamp = int(stamp)
            bucket = min(CHART_BUCKETS-1, (stamp-start)*CHART_BUCKETS//max(1, end-start+1))
            point = (stamp-start, value)
            if bucket not in buckets:
                buckets[bucket] = [point, point]
            else:
                bounds = buckets[bucket]
                if value < bounds[0][1]: bounds[0] = point
                if value > bounds[1][1]: bounds[1] = point
        points = sorted(set(p for bounds in buckets.values() for p in bounds))
        vlow, vhigh = min(p[1] for p in points), max(p[1] for p in points)
        poly = ' '.join(f'{40+t/max(1,end-start)*570:.2f},{160-(v-vlow)/max(1e-12,vhigh-vlow)*130:.2f}'
                        for t, v in points)
        charts.append('<article><h2>stream ' + str(stream) + ' / axis ' + str(axis) + '</h2><p>runtime ' + str(int(runtime)) + '</p><p>' +
            f'{count} 个有效位置观测。仅展示原始 q / pos；单位须按该流 Info 解读。</p>' +
            '<svg viewBox="0 0 650 210"><polyline class="series" points="' + poly + '"/>' +
            f'<text x="40" y="185">0 µs</text><text x="450" y="185">{end-start} µs</text>' +
            f'<text x="40" y="205">值域 [{vlow:.7g}, {vhigh:.7g}]</text></svg></article>')
    _page(destination / 'report/feedback.html', '反馈位置观测',
          '<p>横轴：本 SDK 单调时钟相对时间（µs）；不是设备采样时间。每个时间桶保留最小和最大值。'
          '重复采样身份未冒充新硬件样本；不同流不按时间就近配对。</p>' + ''.join(charts))


def _event_exports(index, destination):
    """Full streamed data for independent inspection, bounded HTML timelines."""
    db = index.db
    with (destination / 'stages.csv').open('x', newline='') as out:
        writer = csv.writer(out)
        writer.writerow(('event_id', 'category', 'stage', 'stream', 'outcome', 'producer', 'duration_us'))
        for category, stage, stream, outcome, producer, duration, event in db.execute('SELECT * FROM timings ORDER BY event_id'):
            writer.writerow((event, category, stage, stream, Outcome(outcome).name, producer,
                             int(duration) if duration is not None else ''))
    with (destination / 'inferences.csv').open('x', newline='') as out:
        writer = csv.writer(out)
        writer.writerow(('event_id', 'kind', 'stage', 'producer', 'inference_id', 'iteration_id',
                         'span_id', 'parent_span_id', 'reference_id', 'timestamp_us', 'duration_us', 'outcome'))
        for event, row in index.rows('kind IN (?,?,?,?)', (
                Kind.USER_SPAN, Kind.INFERENCE_LIFECYCLE, Kind.TRACE_LINK, Kind.USER_MARK)):
            name = index.header['stages'][row['stage']-1] if row['stage'] else ''
            writer.writerow((event, Kind(row['kind']).name, name, row['producer'],
                             *(row[k] for k in ('inference_id', 'iteration_id', 'span_id', 'parent_span_id',
                                               'reference_id', 'timestamp_us', 'duration_us')), Outcome(row['outcome']).name))
    with (destination / 'events.jsonl').open('x', encoding='utf-8') as out:
        for event, row in index.rows():
            out.write(json.dumps(_safe(dict(event_id=event, **row)), allow_nan=False) + '\n')
    with (destination / 'definitions.jsonl').open('x', encoding='utf-8') as out:
        for definition in index.definitions():
            out.write(json.dumps(_safe(definition), allow_nan=False) + '\n')
    # Pagination by record identity keeps overlap and raw time coordinates; it
    # deliberately does not flatten worker spans into a serial control timeline.
    count = db.execute('SELECT count(*) FROM timings').fetchone()[0]
    pages = (count+PAGE_SIZE-1)//PAGE_SIZE
    for page in range(pages):
        rows, intervals = [], []
        for category, stage, stream, outcome, producer, duration, event in db.execute(
                'SELECT * FROM timings ORDER BY event_id LIMIT ? OFFSET ?', (PAGE_SIZE, page*PAGE_SIZE)):
            raw, = db.execute('SELECT row_json FROM events WHERE id=?', (event,)).fetchone()
            row = json.loads(raw)
            if duration is not None:
                elapsed = int(duration)
                begin = row['timestamp_us']
                if category == 'sdk' and stage not in ('body_encode', 'endpoint_encode'):
                    begin -= max(0, elapsed-1)
                if begin >= 0:
                    intervals.append((stage, producer, begin, begin+max(0, elapsed-1), row['clock'], row['epoch']))
            rows.append((event, stage, stream, producer, row['timestamp_us'],
                         int(duration) if duration else None, Outcome(outcome).name,
                         row['inference_id'], row['span_id'], row['parent_span_id']))
        nav = (f'<a href="timeline-{page-1}.html">上一页</a> ' if page else '') + \
              f'第 {page+1}/{pages} 页 ' + (f'<a href="timeline-{page+1}.html">下一页</a>' if page+1<pages else '')
        chart = ''
        for clock, epoch in sorted({(r[4], r[5]) for r in intervals}):
            selected = [r for r in intervals if (r[4], r[5]) == (clock, epoch)]
            start, end = min(r[2] for r in selected), max(r[3] for r in selected)
            bars = []
            for i, (name, producer, begin, finish, _, _) in enumerate(selected):
                x = 240+(begin-start)/max(1,end-start)*560
                width = max(1, (finish-begin)/max(1,end-start)*560)
                bars.append(f'<text x="0" y="{i*20+14}">{_cell(name)} / P{producer}</text>' +
                    f'<rect x="{x}" y="{i*20+2}" width="{width}" height="12">' +
                    f'<title>{_cell(name)}: {begin}–{finish} µs</title></rect>')
            chart += f'<p>clock {clock} / epoch {epoch}: {start}–{end} µs。P 是记录通道；子进程执行归属见原始 provenance。</p>' + \
                f'<svg class="timeline" viewBox="0 0 820 {len(selected)*20}">' + ''.join(bars) + '</svg>'
        _page(destination / 'report/details' / f'timeline-{page}.html', '阶段与异步跨度',
              '<p>所有时间 µs。USER_SPAN/native 编码时间戳是开始；SDK 调用完成记录时间戳是返回。'
              '原始点位并列展示，不将并行区间相加。</p>' + nav + chart + _table(
                  ('event', 'stage', 'stream', 'producer', 'timestamp_us', 'duration_us', 'outcome',
                   'inference', 'span', 'parent'), rows) + nav, root='..')
    return pages


def _mcap_export(index, destination, sources):
    """Exact key sightings only. Recorder time is never a client receive time."""
    db = index.db
    with (destination / 'mcap.csv').open('x', newline='') as out, \
            (destination / 'mcap-events.jsonl').open('x', encoding='utf-8') as decoded:
        writer = csv.writer(out)
        writer.writerow(('message', 'source_sha256', 'topic', 'root', 'log_time_us', 'publish_time_us',
                         'decode_status', 'association', 'command_row', 'reason'))
        for message, source, topic, root, log, publish, status, reason, raw in db.execute('SELECT * FROM mcap_messages'):
            data = json.loads(raw)
            decoded.write(json.dumps(_safe(dict(message=message, source_sha256=source, topic=topic,
                root=root, log_time_us=int(log), publish_time_us=int(publish), decode_status=status,
                reason=reason, payload=data)), allow_nan=False) + '\n')
            association, command = 'NOT_AVAILABLE', ''
            if status == 'DECODED' and data:
                fields = None
                if root in ('lowlevel.BodyCmd', 'lowlevel.ee.EndEffectorCmd'):
                    stream = 1 if root == 'lowlevel.BodyCmd' else 1+data['side']
                    fields = (data.get(k, 0) for k in KEY_FIELDS)
                    if root == 'lowlevel.ee.EndEffectorCmd' and not topic.endswith(
                            '/end_effector/' + ('left' if data['side'] == 1 else 'right') + '/cmd'):
                        fields, reason = None, 'topic/side mismatch'
                    if fields is not None:
                        key = tuple(uint_text(value) for value in fields)
                        candidates = list(db.execute('SELECT id FROM commands WHERE runtime=? AND control=? '
                            'AND session=? AND sequence=? AND stream=? LIMIT 2', (*key, stream)))
                        if len(candidates) == 1 and all(int(v) for v in key):
                            association, command = 'COMMAND_KEY_OBSERVED_IN_MCAP', candidates[0][0]
                            reason = 'key matched; payload/layout equivalence and robot admission are not inferred'
                        else:
                            association = 'AMBIGUOUS' if candidates else 'UNMATCHED'
                elif root == 'lowlevel.BodyState' and data.get('applied_key_valid') and data.get('association_source_type') == 1:
                    key = tuple(uint_text(data[k]) for k in ('runtime_id', 'applied_session_id', 'applied_sequence'))
                    candidates = list(db.execute('SELECT id FROM commands WHERE runtime=? AND session=? '
                        'AND sequence=? AND stream=1 LIMIT 2', key))
                    # Public BodyState has no control_id. This is a unique
                    # same-run application-key observation, not a full wire ACK.
                    if len(candidates) == 1 and all(int(v) for v in key):
                        association, command = 'APPLICATION_KEY_OBSERVED', candidates[0][0]
                        reason = 'control_id absent in state; new hardware sample not proved'
                    else:
                        association = 'AMBIGUOUS' if candidates else 'UNMATCHED'
                elif root == 'lowlevel.ee.EndEffectorState':
                    reason = 'public EE state has no applied command key; no nearest-time matching'
            writer.writerow((message, source, topic, root, int(log), int(publish), status, association, command, reason))
    return dict(status='PROVIDED', sources=sources,
                scope='Exact command/application-key sightings; recorder timestamps are not client receive timestamps. '
                      'No hardware E2E or one-way clock inference.')


def generate_report(path, output, *, mcap_paths=()):
    """Exclusive output directory; input and all unknown measurements preserved."""
    destination = Path(output)
    destination.mkdir(parents=True, exist_ok=False)
    (destination / 'report/assets').mkdir(parents=True)
    (destination / 'report/details').mkdir()
    assets = Path(__file__).with_name('report_assets')
    for name in ('report.css', 'report.js'):
        (destination / 'report/assets' / name).write_bytes((assets / name).read_bytes())
    with TraceIndex(path, directory=destination) as index:
        summary = summarize_index(index)
        _commands(index)
        mcap = dict(status='NOT_PROVIDED')
        if mcap_paths:
            from .mcap_input import ingest
            mcap = _mcap_export(index, destination, ingest(index, mcap_paths))
        _json(destination / 'mcap-integrity.json', mcap)
        command_count, command_pages = _export_commands(index, destination)
        timeline_pages = _event_exports(index, destination)
        _feedback(index, destination)
        _json(destination / 'summary.json', summary)
        _json(destination / 'integrity.json', index.integrity)
        _json(destination / 'clock_alignment.json', dict(status='NOT_AVAILABLE', mappings=[],
            reason='No verified cross-clock mapping supplied. No cross-domain subtraction performed.'))
        _json(destination / 'manifest.json', dict(format='ftrace-offline-report-1', source=index.source,
            run=index.header, command_attempts=command_count, command_detail_pages=command_pages,
            timeline_pages=timeline_pages, physical_completion='NOT_MEASURED',
            mcap=mcap,
            analysis=dict(index='SQLite, 4 MiB page cache, temporary disk sort',
                          percentiles='exact nearest rank', page_size=PAGE_SIZE,
                          feedback_downsampling='128 time buckets; minimum and maximum retained')))
        groups = []
        for category, collection in (('sdk', summary['sdk_stages']), ('model', summary['stages'])):
            for group in collection:
                stage, stream, outcome = group['stage'], group.get('stream', 0), Outcome[group['outcome']]
                groups.append('<article><h2>' + _cell(stage) + ' / ' + _cell(group.get('stream_name') or stream) +
                    ' / ' + _cell(group['outcome']) + '</h2>' + _table(
                        ('有效样本', 'min µs', 'p50 µs', 'p95 µs', 'p99 µs', 'max µs'),
                        [tuple(group.get(k) for k in ('duration_samples', 'min_us', 'p50_us', 'p95_us', 'p99_us', 'max_us'))]) +
                    _histogram(index, category, stage, stream, outcome) + '</article>')
        links = '<nav><a href="feedback.html">反馈位置曲线</a> '
        if command_pages: links += '<a href="details/commands-0.html">逐条命令</a> '
        if timeline_pages: links += '<a href="details/timeline-0.html">阶段与异步跨度</a> '
        links += '<a href="../commands.csv">命令 CSV</a> <a href="../stages.csv">阶段 CSV</a> '
        links += '<a href="../inferences.csv">推理及关联 CSV</a> <a href="../integrity.json">完整性</a></nav>'
        if mcap_paths:
            links += '<nav><a href="../mcap.csv">MCAP 实际观测与键关联</a><a href="../mcap-integrity.json">MCAP Schema / 完整性</a></nav>'
        _page(destination / 'report/index.html', 'SDK 调用与反馈分析',
            '<p class="eyebrow">FBS TRACE 1.3 · OFFLINE · µs</p><p>录制完整性：<strong>' +
            ('完整' if summary['capture_complete'] else '不完整') + '</strong> · ' +
            str(index.integrity['events']) + ' 条记录 · ' + str(command_count) + ' 次发布尝试</p>' +
            '<aside>SDK 耗时已有观测；设备闭环延迟仍不可测。发布成功不表示机器人接纳或机械到位。'
            '编码位于 publish 内部，不与 publish 耗时相加；未知值不记为 0。</aside>' +
            '<p class="muted">源 SHA256 <code>' + index.source['sha256'] + '</code></p>' + links +
            ''.join(groups) + '<h2>SDK 点位覆盖</h2>' + _table(('阶段', '本文件证据'),
                summary['sdk_stage_coverage'].items()) + '<h2>完整性与未满足条件</h2><pre>' +
            _cell(json.dumps(dict(issues=summary['issues'], producer_issues=summary['producer_issues'],
                                 mcap=mcap, hardware_association='未证明', clock_alignment='未提供'), ensure_ascii=False, indent=2)) + '</pre>')
        if file_identity(path) != index.source:
            raise ValueError('source recording changed during report generation')
    return destination / 'report/index.html'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('path', type=Path)
    parser.add_argument('--output', type=Path, required=True, help='new output directory')
    parser.add_argument('--mcap', action='append', type=Path, default=[], help='existing MCAP; repeat for shards')
    args = parser.parse_args()
    print(generate_report(args.path, args.output, mcap_paths=args.mcap))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
