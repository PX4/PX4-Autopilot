#!/usr/bin/env python3
"""Render a supporting HTML artifact for the GitHub bot's memory comparison."""

import argparse
from html import escape
import json
from pathlib import Path

from firmware_size import indicator


STYLE = """
:root { color-scheme: light; --ink:#15283b; --muted:#566577; --line:#dce3e9;
  --canvas:#f3f6f8; --blue:#205f96; --green:#176a4a; --amber:#8c5105; }
* { box-sizing:border-box; }
html { scroll-behavior:smooth; }
body { margin:0; background:var(--canvas); color:var(--ink);
  font:15px/1.55 system-ui,-apple-system,BlinkMacSystemFont,"Segoe UI",sans-serif; }
a { color:var(--blue); }
button,input,select { font:inherit; }
button,input,select,a { outline-offset:4px; }
button { cursor:pointer; }
main { max-width:1280px; margin:0 auto; padding:34px 40px 56px; }
.masthead { background:#142c43; color:#fff; }
.masthead-inner { max-width:1280px; margin:auto; padding:36px 40px 32px; }
.eyebrow { text-transform:uppercase; font-weight:700; font-size:12px;
  letter-spacing:.16em; color:#b9d3e9; }
.title-row { display:flex; gap:24px; justify-content:space-between; align-items:center; }
h1 { font-size:38px; line-height:1.15; letter-spacing:-.035em; margin:12px 0; }
.subtitle { color:#d3deea; margin:0; max-width:800px; }
.print-button { white-space:nowrap; border:1px solid #647b90; border-radius:8px;
  background:transparent; color:#fff; padding:9px 15px; }
.provenance { display:flex; gap:10px 24px; flex-wrap:wrap; margin-top:26px; }
.provenance div { display:flex; gap:8px; align-items:baseline; }
.provenance dt { color:#b9c9d8; font-size:12px; }
.provenance dd { margin:0; font-size:13px; overflow-wrap:anywhere; }
code,.number { font-family:ui-monospace,SFMono-Regular,Consolas,monospace;
  font-variant-numeric:tabular-nums; }
.scope-note { background:#e7f0f7; border:1px solid #c9dae8; border-radius:10px;
  padding:14px 18px; margin:0 0 22px; color:#234a6a; }
.scope-note strong { display:block; }
.cards { display:grid; grid-template-columns:repeat(4,minmax(0,1fr)); gap:16px; }
.card { display:flex; flex-direction:column; background:#fff; border:1px solid var(--line);
  border-radius:12px; padding:21px 22px; text-decoration:none; color:inherit;
  box-shadow:0 3px 8px #172a3b04; }
.card:hover { border-color:#8eacc5; }
.card-label { color:var(--muted); font-size:13px; font-weight:600; }
.card-value { font-size:30px; line-height:1.25; letter-spacing:-.035em; margin:11px 0 8px;
  font-weight:750; }
.card-detail { font-size:12px; color:var(--muted); margin-top:auto; }
.card-value.warning { font-size:25px; }
.up { color:var(--amber); }
.down { color:var(--green); }
.warning { color:var(--amber); }
.danger { color:#b42318; }
.neutral { color:var(--muted); }
.jump-links { display:flex; flex-wrap:wrap; gap:22px; margin:26px 0 10px;
  font-size:13px; }
.jump-links a { text-decoration:none; }
.panel { background:#fff; border:1px solid var(--line); border-radius:12px;
  margin-top:22px; padding:26px; scroll-margin-top:16px; }
.section-heading { display:flex; justify-content:space-between; gap:20px; align-items:start; }
h2 { font-size:21px; letter-spacing:-.02em; line-height:1.3; margin:0 0 7px; }
h3 { font-size:15px; margin:0 0 8px; }
p { margin:0 0 12px; }
.section-description { color:var(--muted); font-size:13px; max-width:850px; }
.badge { display:inline-block; flex-shrink:0; border-radius:5px; background:#eef2f5;
  color:#42556b; font-size:11px; padding:4px 8px; font-weight:650; white-space:nowrap; }
.badge.warning { background:#fff0d9; color:var(--amber); }
.badge.danger { background:#feeceb; color:#b42318; }
.badge.up { background:#fff0d9; color:var(--amber); }
.badge.down { background:#e7f4ed; color:var(--green); }
.table-wrap { overflow:auto; margin-top:14px; }
table { width:100%; border-collapse:collapse; font-size:13px; }
caption { text-align:left; font-size:12px; color:var(--muted); padding:0 0 10px; }
th { text-align:left; color:var(--muted); font-weight:600; font-size:11px;
  text-transform:uppercase; letter-spacing:.045em; border-bottom:1px solid var(--line);
  padding:11px 12px; }
td { padding:16px 12px; border-bottom:1px solid #e8edf1; vertical-align:top; }
tr:last-child td { border-bottom:0; }
th:first-child,td:first-child { padding-left:0; }
th:last-child,td:last-child { padding-right:0; }
.row-name { font-weight:650; overflow-wrap:anywhere; }
.function { font-size:12px; font-weight:550; overflow-wrap:anywhere; }
.source,.subtext { display:block; color:var(--muted); font-size:11px;
  line-height:1.55; margin-top:5px; overflow-wrap:anywhere; }
.measurement { min-width:125px; white-space:nowrap; }
.measurement .number { font-size:14px; }
.delta-cell { min-width:145px; }
.delta { font-weight:650; white-space:nowrap; }
.bar { height:5px; width:120px; background:#edf1f4; border-radius:4px; margin-top:8px;
  overflow:hidden; }
.bar span { display:block; height:100%; background:#a1b0be; border-radius:4px; }
.bar.after span { background:#367cac; }
.bar.after.warning span { background:#b2782d; }
.bar.after.danger span { background:#b42318; }
.bar.after.down span { background:#318368; }
.row-note { font-size:12px; max-width:280px; }
.table-note { color:var(--muted); font-size:12px; margin:10px 0 0; }
.notice { background:#fff5e5; border-left:3px solid #c08632; padding:12px 15px;
  color:#704712; border-radius:0 6px 6px 0; font-size:13px; margin:16px 0; }
.toolbar { display:flex; align-items:end; flex-wrap:wrap; gap:14px; margin:20px 0 6px; }
.toolbar label { display:flex; flex-direction:column; gap:5px; font-size:12px; color:var(--muted); }
.toolbar input,.toolbar select { border:1px solid #b8c7d3; border-radius:6px;
  padding:8px 10px; color:var(--ink); background:#fff; }
.toolbar input { width:290px; max-width:100%; }
.filter-count { font-size:12px; color:var(--muted); padding-bottom:9px; }
.empty { color:var(--muted); padding:20px 0; }
.memory-map { display:grid; grid-template-columns:repeat(3,minmax(0,1fr));
  gap:16px; margin:22px 0; }
.memory-zone { border:1px solid var(--line); border-radius:8px; padding:17px;
  background:#f8fafb; font-size:12px; }
.memory-zone h3 { color:var(--blue); }
.memory-zone p { color:var(--muted); margin:0; }
.memory-zone small { display:block; color:var(--muted); margin-top:12px; }
.methods { display:grid; grid-template-columns:1fr 1fr; gap:26px; }
.methods p,.methods li { font-size:13px; color:var(--muted); }
ul { margin:8px 0 0; padding-left:19px; }
li { margin-bottom:7px; }
details { margin-top:18px; border-top:1px solid var(--line); padding-top:15px; }
summary { cursor:pointer; font-size:13px; font-weight:650; }
pre { white-space:pre-wrap; overflow-wrap:anywhere; background:#f5f7f9; padding:14px;
  font-size:11px; color:var(--muted); border-radius:6px; }
footer { margin-top:25px; font-size:12px; color:var(--muted); }
@media (max-width:950px) { .cards { grid-template-columns:repeat(2,minmax(0,1fr)); }
  main,.masthead-inner { padding-left:24px; padding-right:24px; }
  .memory-map { grid-template-columns:1fr; } }
@media (max-width:600px) { main,.masthead-inner { padding-left:16px; padding-right:16px; }
  .cards { gap:10px; } .card { padding:16px; } .card-value { font-size:25px; }
  .panel { padding:18px; } .title-row { align-items:start; } h1 { font-size:32px; }
  .print-button { display:none; } .methods { grid-template-columns:1fr; }
  .section-heading { display:block; } .section-heading > .badge { margin:4px 0 8px; }
  .toolbar input { width:230px; } }
@media print { @page { margin:12mm; } body { background:#fff; font-size:10pt; }
  .masthead { background:#fff; color:#15283b; border-bottom:2px solid #15283b; }
  .masthead-inner,main { padding:18px 0; max-width:none; }
  .eyebrow,.subtitle,.provenance dt { color:#566577; }
  .print-button,.toolbar label,.jump-links { display:none; }
  .cards { gap:10px; } .card { padding:12px; box-shadow:none; }
  .card-value { font-size:21px; } .card-value.warning { font-size:19px; }
  .panel { padding:16px; margin-top:16px; } .table-wrap { overflow:visible; }
  tr,.card,.memory-zone { break-inside:avoid; }
  h2,h3,.section-heading { break-after:avoid; }
  .measurement { min-width:90px; } .bar { width:85px; }
  .delta-cell { min-width:100px; } th,td { padding:10px 7px; }
  .function { font-size:9px; } .source,.subtext { font-size:9px; }
  .memory-map { grid-template-columns:repeat(3,minmax(0,1fr)); }
  .methods { grid-template-columns:1fr 1fr; } a { text-decoration:none; color:inherit; }
  * { -webkit-print-color-adjust:exact; print-color-adjust:exact; } }
"""

SCRIPT = """
document.querySelectorAll('[data-filter-table]').forEach(function (toolbar) {
  const table = document.getElementById(toolbar.dataset.filterTable);
  const search = toolbar.querySelector('input');
  const select = toolbar.querySelector('select');
  const count = toolbar.querySelector('[role="status"]');
  const rows = Array.from(table.querySelectorAll('tbody tr'));
  function filter() {
    const query = search.value.toLocaleLowerCase();
    let visible = 0;
    rows.forEach(function (row) {
      const matchesText = row.dataset.search.toLocaleLowerCase().includes(query);
      const matchesStatus = select.value === 'changes' || row.dataset.status === select.value;
      row.hidden = !(matchesText && matchesStatus);
      if (!row.hidden) visible += 1;
    });
    count.textContent = visible + ' of ' + rows.length + ' rows · '
      + select.options[select.selectedIndex].text + (search.value ? ' · Search: ' + search.value : '');
  }
  search.addEventListener('input', filter);
  select.addEventListener('change', filter);
  filter();
});
document.getElementById('print-report').addEventListener('click', function () { window.print(); });
"""


def _text(value):
    return escape(str(value), quote=True) if value is not None else ""


def _integer(value):
    return isinstance(value, int) and not isinstance(value, bool)


def _bytes(value):
    return f"{value:,} B" if _integer(value) else "Unavailable"


def _change(value):
    return f"{value:+,} B" if _integer(value) else "Not comparable"


def _tone(value):
    if not _integer(value):
        return "warning"
    return {"🔴 ": "danger", "🟡 ": "warning", "🟢 ": "down"}.get(indicator(value), "neutral")


def _direction(value):
    if not _integer(value):
        return "warning"
    return "up" if value > 0 else "down" if value < 0 else "neutral"


def _notice(reason):
    return f'<p class="notice">{_text(reason or "Measurement unavailable for this comparison.")}</p>'


def _measurement(value, maximum, *, after=False, delta=None, note=None, bounded=True):
    bar = ""
    if bounded and _integer(value) and maximum > 0:
        width = min(100, max(0, 100 * value / maximum))
        variant = f"after {_tone(delta)}" if after else ""
        bar = f'<div class="bar {variant}" aria-hidden="true"><span style="width:{width:.2f}%"></span></div>'
    annotation = f'<span class="subtext">{_text(note)}</span>' if note else ""
    return f'<td class="measurement"><span class="number">{_bytes(value)}</span>{bar}{annotation}</td>'


def _card(label, value, detail, anchor, tone="neutral"):
    return (f'<a class="card" href="#{anchor}"><span class="card-label">{_text(label)}</span>'
            f'<span class="card-value {tone}">{_text(value)}</span>'
            f'<span class="card-detail">{_text(detail)}</span></a>')


def _static_section(static):
    rows = []
    if static.get("status") == "ok":
        for key, label, note in (
                ("flash", "Flash image", "Programmed image, including initialized data"),
                ("ram", "Static RAM", "Linked RAM sections, including .data and .bss")):
            item = static.get(key) or {}
            before, after = item.get("before"), item.get("after")
            delta = item.get("delta")
            maximum = max((value for value in (before, after) if _integer(value)), default=0)
            rows.append(f'<tr><td><span class="row-name">{label}</span><span class="subtext">{note}</span></td>'
                        + _measurement(before, maximum)
                        + _measurement(after, maximum, after=True, delta=delta)
                        + f'<td class="delta-cell"><span class="delta number {_tone(delta)}">{_change(delta)}</span></td></tr>')
    content = ('<div class="table-wrap"><table><caption>Bars compare the two builds within each row; they do not show board capacity.</caption>'
               '<thead><tr><th>Memory region</th><th>Baseline</th><th>With change</th><th>Difference</th></tr></thead>'
               f'<tbody>{"".join(rows)}</tbody></table></div>') if rows else _notice(static.get("reason"))
    return ('<section class="panel" id="static"><div class="section-heading"><div>'
            '<h2>Flash &amp; static RAM</h2><p class="section-description">The existing ELF view: storage present in the linked firmware image and its RAM sections.</p>'
            '</div><span class="badge">Measured from ELF</span></div>' + content + '</section>')


def _toolbar(table, label):
    return (f'<div class="toolbar" data-filter-table="{table}"><label>Search {label}'
            f'<input type="search" aria-controls="{table}" placeholder="Name or source file"></label>'
            f'<label>Show<select aria-controls="{table}"><option value="changes">Changed functions</option>'
            '<option value="up">Increases</option><option value="down">Decreases</option>'
            '<option value="warning">Unknown difference</option><option value="neutral">Same byte count</option>'
            '</select></label><span class="filter-count" role="status" aria-live="polite"></span></div>')


def _frame_note(frame):
    kind = (frame or {}).get("kind")
    return {"static": "Fixed frame", "dynamic_bounded": "Bounded dynamic frame",
            "dynamic_unbounded": "Recorded portion; dynamic upper bound unknown"}.get(kind, "No comparable record")


def _frame_order(row):
    unbounded = any((row.get(side) or {}).get("kind") == "dynamic_unbounded" for side in ("before", "after"))
    changed = row.get("status") in ("changed", "added", "removed")
    group = 0 if changed else 1 if row.get("status") == "ambiguous" or unbounded or row.get("delta") is None else 2
    delta = row.get("delta")
    return (group, 0 if unbounded else 1, -abs(delta) if _integer(delta) else 0,
            str(row.get("source", "")), str(row.get("function", "")))


def _changed_frames(stack):
    # Do not reintroduce unchanged unknown records from an older full-report
    # JSON. Ambiguous rows need explicit evidence that their observations changed.
    return [row for row in stack.get("rows") or []
            if row.get("status") in ("changed", "added", "removed")
            or (row.get("status") == "ambiguous" and row.get("measurement_changed") is True)]


def _stack_section(stack):
    rendered = []
    for row in sorted(_changed_frames(stack), key=_frame_order):
        before, after = row.get("before") or {}, row.get("after") or {}
        bounded = all(frame.get("kind") in ("static", "dynamic_bounded") and _integer(frame.get("bytes"))
                      for frame in (before, after))
        delta = row.get("delta") if bounded else None
        if row.get("status") == "ambiguous":
            delta = None
            bounded = False
        dynamic = any(frame.get("kind") == "dynamic_unbounded" for frame in (before, after))
        status = _direction(delta)
        change = _change(delta) if bounded else "Unknown upper bound" if dynamic else "Not comparable"
        values = [frame.get("bytes") for frame in (before, after)]
        maximum = max((value for value in values if _integer(value)), default=0)
        reason = row.get("reason")
        annotation = f'<span class="subtext">{_text(reason)}</span>' if reason else ""
        if row.get("status") in ("added", "removed", "ambiguous"):
            annotation += f'<span class="subtext">{_text(row["status"].capitalize())} compiler record</span>'
        rendered.append(f'<tr data-status="{status}" data-search="{_text(str(row.get("function", "")) + " " + str(row.get("source", "")))}">'
                        f'<td><code class="function">{_text(row.get("function", "Unnamed function"))}</code>'
                        f'<span class="source">{_text(row.get("source"))}</span></td>'
                        + _measurement(values[0], maximum, note=_frame_note(before), bounded=bounded)
                        + _measurement(values[1], maximum, after=True, delta=delta, note=_frame_note(after), bounded=bounded)
                        + f'<td class="delta-cell"><span class="badge {_tone(delta)}">{_text(change)}</span>{annotation}</td></tr>')
    content = _notice(stack.get("reason")) if stack.get("status") != "ok" else ""
    if rendered:
        content += (_toolbar("stack-table", "functions")
                    + '<div class="table-wrap"><table id="stack-table"><caption>Only records changed between the baseline and PR build. Each row is one function’s own frame.</caption>'
                    '<thead><tr><th>Function / source</th><th>Baseline</th><th>With change</th><th>Difference</th></tr></thead>'
                    f'<tbody>{"".join(rendered)}</tbody></table></div>'
                    '<p class="table-note">Unchanged functions are omitted. Bars compare only bounded, comparable records; print includes the selected rows.</p>')
    elif stack.get("status") == "ok":
        content += '<p class="empty">No function stack-frame changes in this comparison.</p>'
    coverage = stack.get("coverage")
    if coverage:
        content += ('<details><summary>Compiler record coverage</summary><pre>'
                    + _text(json.dumps(coverage, indent=2, sort_keys=True)) + '</pre></details>')
    return ('<section class="panel" id="stack"><div class="section-heading"><div>'
            '<h2>Function stack frames</h2><p class="section-description">Compiler stack-usage records expose local frame changes. '
            'A smaller frame does not establish the peak stack of a task.</p></div>'
            '<span class="badge warning">Not whole-task stack</span></div>' + content + '</section>')


def _object_section(objects):
    rendered = []
    for row in objects.get("rows") or []:
        before, after = row.get("before"), row.get("after")
        delta = (row.get("delta") if row.get("status") != "unavailable"
                 and all(_integer(value) for value in (before, after)) else None)
        maximum = max((value for value in (before, after) if _integer(value)), default=0)
        owner = row.get("included_in")
        ownership = (f'Included in {_text(owner)}; do not add again.' if owner
                     else _text(row.get("allocation") or "Allocation lifetime must be checked in source."))
        reason = row.get("reason")
        if reason:
            ownership += f'<span class="subtext">{_text(reason)}</span>'
        rendered.append(f'<tr><td><span class="row-name">{_text(row.get("label") or row.get("name"))}</span>'
                        f'<code class="source">{_text(row.get("name"))}</code></td>'
                        + _measurement(before, maximum)
                        + _measurement(after, maximum, after=True, delta=delta)
                        + f'<td class="delta-cell"><span class="delta number {_tone(delta)}">{_change(delta)}</span></td>'
                        f'<td class="row-note">{ownership}</td></tr>')
    content = _notice(objects.get("reason")) if objects.get("status") != "ok" else ""
    if rendered:
        content += ('<div class="table-wrap"><table><caption>Target object size per instance, from DWARF. Embedded members are already included in their owner.</caption>'
                    '<thead><tr><th>Tracked type</th><th>Baseline</th><th>With change</th><th>Difference</th><th>Ownership / allocation</th></tr></thead>'
                    f'<tbody>{"".join(rendered)}</tbody></table></div>')
    elif objects.get("status") == "ok":
        content += '<p class="empty">No tracked object types in this comparison.</p>'
    content += ('<p class="table-note">Object size includes inline members and padding. It excludes separate allocations reached through pointers, '
                'allocator overhead, and the number of live instances. These rows are not summed.</p>')
    return ('<section class="panel" id="objects"><div class="section-heading"><div>'
            '<h2>Module object sizes</h2><p class="section-description">An inline buffer can grow a module allocated at startup '
            'while the static RAM report stays unchanged.</p></div><span class="badge warning">Not runtime heap peak</span></div>'
            + content + '</section>')


def _methods(limitations):
    extra = ''.join(f'<li>{_text(item)}</li>' for item in limitations)
    return ('<section class="panel" id="interpretation"><h2>How to read the measurements</h2>'
            '<p class="section-description">Different measurements answer different questions about the same firmware.</p>'
            '<div class="memory-map" aria-label="Memory categories">'
            '<div class="memory-zone"><h3>Flash</h3><p>Code, constants and the initial values copied into RAM.</p>'
            '<small>Measured: linked load image.</small></div>'
            '<div class="memory-zone"><h3>Static RAM</h3><p>Global and static storage, including .data and .bss.</p>'
            '<small>Measured: linked RAM sections.</small></div>'
            '<div class="memory-zone"><h3>Runtime RAM</h3><p>Live module objects, other heap allocations and task stacks.</p>'
            '<small>Partial views: tracked object sizes and individual function frames.</small></div></div>'
            '<div class="methods"><div><h3>What this comparison establishes</h3><ul>'
            '<li>Flash and static RAM changes for this target build.</li>'
            '<li>Compiler-reported frame changes for functions with comparable records.</li>'
            '<li>Changes in the target size of explicitly tracked types, per instance.</li></ul></div>'
            '<div><h3>What still needs runtime validation</h3><ul>'
            '<li>Peak task stack along complete call paths, including recursion and indirect calls.</li>'
            '<li>Heap high-water mark, allocation failures and the largest free contiguous block.</li>'
            '<li>Configuration, mission and vehicle paths exercised in simulation or hardware tests.</li></ul></div></div>'
            '<p class="notice">Do not add the four categories into a RAM total. Embedded objects are already part of their owner. '
            'On NuttX, task-stack storage may also be counted by heap accounting.</p>'
            + (f'<h3>Comparison limitations</h3><ul>{extra}</ul>' if extra else '') + '</section>')


def render_html(report):
    """Return a self-contained report; missing measurements remain visibly unknown."""
    provenance = report.get("provenance") or {}
    static = report.get("static") or {}
    stack = report.get("stack") or {}
    objects = report.get("objects") or {}
    target = report.get("target") or "Unspecified target"
    label = provenance.get("label") or "Build comparison"
    cards = []
    for key, name in (("flash", "Flash image"), ("ram", "Static RAM")):
        item = static.get(key) or {}
        known = static.get("status") == "ok" and all(_integer(item.get(key)) for key in ("before", "after", "delta"))
        cards.append(_card(name, _change(item.get("delta")) if known else "Unavailable",
                           f'{_bytes(item.get("before"))} → {_bytes(item.get("after"))}' if known else "No comparable ELF measurement",
                           "static", _tone(item.get("delta")) if known else "warning"))
    frames = _changed_frames(stack)
    changed_frames = len(frames)
    unbounded = sum(any((row.get(side) or {}).get("kind") == "dynamic_unbounded" for side in ("before", "after")) for row in frames)
    ambiguous = sum(row.get("status") == "ambiguous" for row in frames)
    unknown_frames = sum(row.get("delta") is None or row.get("status") == "ambiguous" for row in frames)
    stack_known = stack.get("status") == "ok"
    stack_detail = "PR changes only · individual frames"
    warnings = []
    if ambiguous:
        warnings.append(f"{ambiguous:,} ambiguous {'record' if ambiguous == 1 else 'records'}")
    if unbounded:
        warnings.append(f"{unbounded:,} unknown dynamic {'bound' if unbounded == 1 else 'bounds'}")
    if warnings:
        stack_detail = " · ".join(warnings)
    elif unknown_frames:
        stack_detail = f"{unknown_frames:,} comparisons unavailable or ambiguous"
    cards.append(_card("Function stack frames", f"{changed_frames:,} changed" if stack_known else "Unavailable",
                       stack_detail if stack_known else "Compiler records unavailable", "stack",
                       "warning" if unbounded or unknown_frames or not stack_known else "neutral"))
    object_rows = objects.get("rows") or []
    changed_objects = sum(row.get("status") == "changed" and not row.get("included_in") for row in object_rows)
    missing_objects = sum(row.get("status") == "unavailable" for row in object_rows)
    objects_known = objects.get("status") == "ok" and (not missing_objects or missing_objects < len(object_rows))
    object_detail = (f"{missing_objects:,} tracked types unavailable" if missing_objects
                     else "Per instance · embedded types not counted twice")
    cards.append(_card("Module object sizes", f"{changed_objects:,} changed" if objects_known else "Unavailable",
                       object_detail if objects_known else "Target type sizes unavailable", "objects",
                       "warning" if missing_objects or not objects_known else "neutral"))
    fields = [("Target", target), ("Baseline", provenance.get("before") or "Not recorded"),
              ("With change", provenance.get("after") or "Not recorded"),
              ("Toolchain", provenance.get("toolchain") or "Not recorded")]
    metadata = ''.join(f'<div><dt>{name}</dt><dd><code>{_text(value)}</code></dd></div>' for name, value in fields)
    return ('<!doctype html><html lang="en"><head><meta charset="utf-8">'
            '<meta name="viewport" content="width=device-width,initial-scale=1">'
            f'<title>PX4 memory impact — {_text(target)}</title><style>{STYLE}</style></head><body>'
            '<header class="masthead"><div class="masthead-inner"><div class="eyebrow">PX4 / GitHub bot supporting report</div>'
            '<div class="title-row"><div><h1>Memory impact</h1>'
            f'<p class="subtitle">{_text(label)}</p></div><button class="print-button" id="print-report" type="button">Print / save PDF</button></div>'
            f'<dl class="provenance">{metadata}</dl></div></header><main>'
            '<p class="scope-note"><strong>Four separate views. No combined RAM total.</strong>'
            'Flash and static RAM are linked sizes. Function frames and module objects expose additional changes; '
            'they do not measure runtime peaks or prove a safe memory margin.</p>'
            f'<div class="cards">{"".join(cards)}</div>'
            '<nav class="jump-links" aria-label="Report sections"><a href="#static">Flash &amp; static RAM</a>'
            '<a href="#objects">Module objects</a><a href="#stack">Function frames</a>'
            '<a href="#interpretation">Reading this report</a></nav>'
            + _static_section(static) + _object_section(objects) + _stack_section(stack)
            + _methods(report.get("limitations") or [])
            + '<footer>The GitHub PR bot comment is the primary report; this artifact provides supporting detail. '
            'All sizes are bytes for the selected target. Bars are scaled independently within each row; '
            'they are comparisons, not capacity gauges. This file has no external assets or network dependencies.</footer>'
            f'</main><script>{SCRIPT}</script></body></html>')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, required=True, help="Normalized memory comparison JSON")
    parser.add_argument("--output", type=Path, required=True, help="Standalone HTML output")
    args = parser.parse_args()
    args.output.write_text(render_html(json.loads(args.input.read_text(encoding="utf-8"))), encoding="utf-8")


if __name__ == "__main__":
    main()
