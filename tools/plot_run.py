#!/usr/bin/env python3
"""Renders one RunLog run directory as a self-contained interactive HTML page.

The output embeds the data and a vendored copy of uPlot, so it opens with no network access: usable
on a competition laptop that is joined to the robot's wifi.

    python3 tools/plot_run.py --latest
    python3 tools/plot_run.py logs/20260901-101500-TeleOp_Red --only drive,lift

Standard library only.
"""

from __future__ import annotations

import argparse
import html
import json
import math
import sys
import webbrowser
from pathlib import Path

import runlog

VENDOR = Path(__file__).resolve().parent / "vendor"
UPLOT_JS = VENDOR / "uPlot.iife.min.js"
UPLOT_CSS = VENDOR / "uPlot.min.css"

PALETTE = (
    "#1f77b4",
    "#d62728",
    "#2ca02c",
    "#9467bd",
    "#ff7f0e",
    "#17becf",
    "#8c564b",
    "#7f7f7f",
)

EVENT_COLORS = {
    "run/start": "#666666",
    "run/end": "#666666",
    "run/truncated": "#000000",
    "command/start": "#2ca02c",
    "command/interrupt": "#d62728",
    "command/finish": "#1f77b4",
}
EVENT_FALLBACK_COLOR = "#e69138"

CHART_HEIGHT = 220
GROUP_INDEX_THRESHOLD = 3

CURL_HINT = """vendored uPlot is missing; fetch it with:
  curl -fsSL -o tools/vendor/uPlot.iife.min.js https://cdn.jsdelivr.net/npm/uplot@1.6.32/dist/uPlot.iife.min.js
  curl -fsSL -o tools/vendor/uPlot.min.css    https://cdn.jsdelivr.net/npm/uplot@1.6.32/dist/uPlot.min.css
  curl -fsSL -o tools/vendor/uPlot.LICENSE    https://cdn.jsdelivr.net/npm/uplot@1.6.32/LICENSE"""


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        prog="plot_run.py", description="Plot a RunLog run as a self-contained HTML page."
    )
    parser.add_argument("run", nargs="?", help="run directory, or a run name inside --logs-dir")
    parser.add_argument("--latest", action="store_true", help="use the newest run in --logs-dir")
    parser.add_argument("--logs-dir", type=Path, default=runlog.repo_root() / "logs")
    parser.add_argument("--out", type=Path, help="output file (default <run>/plot.html)")
    parser.add_argument("--no-open", action="store_true", help="do not open a browser")
    parser.add_argument(
        "--max-points",
        type=int,
        default=20000,
        help="decimate to at most N rows per series (0 disables)",
    )
    parser.add_argument("--only", help="comma-separated groups to plot")
    parser.add_argument("--exclude", help="comma-separated groups to skip")
    parser.add_argument(
        "--list-groups", action="store_true", help="print groups with column counts and exit"
    )
    args = parser.parse_args(argv)

    if not args.run and not args.latest:
        parser.print_help(sys.stderr)
        print("\ngive a run directory or --latest", file=sys.stderr)
        return 2

    try:
        run_dir = runlog.resolve_run(args.run, args.logs_dir, args.latest)
        run = runlog.load_run(run_dir)
    except runlog.RunLogError as exc:
        print(str(exc), file=sys.stderr)
        return 2

    groups = runlog.group_columns(run)

    if args.list_groups:
        for name, columns in groups.items():
            print(f"{name} {len(columns)}")
        return 0

    try:
        groups = filter_groups(groups, args.only, args.exclude)
    except ValueError as exc:
        print(str(exc), file=sys.stderr)
        return 2

    if not UPLOT_JS.exists() or not UPLOT_CSS.exists():
        print(CURL_HINT, file=sys.stderr)
        return 3

    summary = runlog.format_summary(run)
    print(summary)

    stride, kept = decimation(len(run.t), args.max_points)
    if stride > 1:
        print(f"decimated {len(run.t)} -> {kept} rows (stride {stride})")
    if not run.t:
        print("no rows in signals.csv; writing summary and events only", file=sys.stderr)

    out = args.out or (run_dir / "plot.html")
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(render(run, groups, summary, stride), encoding="utf-8")
    print(f"wrote {out}")

    if not args.no_open:
        webbrowser.open(out.resolve().as_uri())
    return 0


def filter_groups(
    groups: dict[str, list[str]], only: str | None, exclude: str | None
) -> dict[str, list[str]]:
    """--only wins over --exclude; an unknown name is an error, never a silently empty page."""
    available = ", ".join(groups) or "(none)"
    for raw in (only, exclude):
        for name in split_names(raw):
            if name not in groups:
                raise ValueError(f'unknown group "{name}"; available: {available}')
    only_names = split_names(only)
    if only_names:
        return {name: cols for name, cols in groups.items() if name in only_names}
    exclude_names = split_names(exclude)
    return {name: cols for name, cols in groups.items() if name not in exclude_names}


def split_names(raw: str | None) -> list[str]:
    return [part.strip() for part in raw.split(",") if part.strip()] if raw else []


def decimation(rows: int, max_points: int) -> tuple[int, int]:
    if max_points <= 0 or rows <= max_points:
        return 1, rows
    stride = math.ceil(rows / max_points)
    return stride, len(range(0, rows, stride))


def is_flag(values: list[float | None]) -> bool:
    seen = [v for v in values if v is not None]
    return bool(seen) and all(v in (0.0, 1.0) for v in seen)


def pad_edges(
    xs: list[float], values: dict[str, list[float | None]], event_times: list[float]
) -> None:
    """Extends the x data to cover events outside the sampled window, in place.

    An event can land before the first logged loop or after the last one, and uPlot ranges x from
    the data. A boundary sample carrying null for every series widens the view without drawing
    anything, and unlike forcing scales.x.range it leaves drag-zoom working: uPlot calls a range
    function on every setScale, so a widening range function would undo every zoom.
    """
    if not xs or not event_times:
        return
    first, last = min(event_times), max(event_times)
    if first < xs[0]:
        xs.insert(0, first)
        for column in values:
            values[column].insert(0, None)
    if last > xs[-1]:
        xs.append(last)
        for column in values:
            values[column].append(None)


def render(run: runlog.Run, groups: dict[str, list[str]], summary: str, stride: int) -> str:
    events = [
        {
            "t": float(event.get("t", 0.0)),
            "kind": runlog.event_kind(event),
            "label": runlog.event_label(event),
        }
        for event in run.events
        if isinstance(event.get("t", None), (int, float))
    ]
    xs = run.t[::stride]
    values = {column: run.series[column][::stride] for column in run.series}
    flags = {column: is_flag(run.series[column]) for column in run.series}
    pad_edges(xs, values, [e["t"] for e in events])

    payload = {
        "x": xs,
        "groups": [
            {
                "name": name,
                "title": f"{name} ({len(columns)} signal{'s' if len(columns) != 1 else ''})",
                "series": [
                    {
                        "label": column,
                        "stroke": PALETTE[index % len(PALETTE)],
                        "flag": flags[column],
                        "values": values[column],
                    }
                    for index, column in enumerate(columns)
                ],
            }
            for name, columns in groups.items()
        ],
        "events": events,
    }
    data_json = json.dumps(payload, allow_nan=False).replace("</", "<\\/")

    return TEMPLATE.format(
        title=html.escape(f"{run.name} - {run.op_mode}"),
        vendor_css=UPLOT_CSS.read_text(encoding="utf-8"),
        own_css=OWN_CSS,
        heading=html.escape(f"{run.name}  -  {run.op_mode}"),
        summary=html.escape(summary),
        group_index=group_index_html(groups),
        event_key=event_key_html(),
        charts="\n".join(
            f'  <section class="chart" id="group-{html.escape(name)}"></section>' for name in groups
        )
        or '  <p class="empty">no signal columns to plot</p>',
        event_rows=event_rows_html(run.events),
        vendor_js=UPLOT_JS.read_text(encoding="utf-8"),
        data_json=data_json,
        chart_height=CHART_HEIGHT,
        event_colors=json.dumps(EVENT_COLORS),
        fallback_color=EVENT_FALLBACK_COLOR,
    )


def group_index_html(groups: dict[str, list[str]]) -> str:
    if len(groups) <= GROUP_INDEX_THRESHOLD:
        return ""
    links = " ".join(
        f'<a href="#group-{html.escape(name)}">{html.escape(name)} <span>{len(columns)}</span></a>'
        for name, columns in groups.items()
    )
    return f'<nav class="index">{links}</nav>'


def event_key_html() -> str:
    swatches = "".join(
        f'<span class="swatch"><i style="background:{color}"></i>{html.escape(kind)}</span>'
        for kind, color in EVENT_COLORS.items()
    )
    other = f'<span class="swatch"><i style="background:{EVENT_FALLBACK_COLOR}"></i>other</span>'
    return f'<div class="key">{swatches}{other}</div>'


def event_rows_html(events: list[dict]) -> str:
    rows = []
    for event in events:
        kind = runlog.event_kind(event)
        color = EVENT_COLORS.get(kind, EVENT_FALLBACK_COLOR)
        time_value = event.get("t", 0.0)
        time_text = f"{float(time_value):.3f}" if isinstance(time_value, (int, float)) else "?"
        rows.append(
            f'    <tr><td class="t">{time_text}</td>'
            f'<td style="color:{color}">{html.escape(kind)}</td>'
            f"<td>{html.escape(runlog.event_label(event))}</td></tr>"
        )
    return "\n".join(rows) or '    <tr><td colspan="3">no events</td></tr>'


OWN_CSS = """
body { font: 13px/1.45 -apple-system, Segoe UI, Roboto, sans-serif; margin: 0 auto; padding: 16px;
       max-width: 1400px; color: #222; }
h1 { font-size: 18px; margin: 0 0 8px; }
pre.summary { background: #f6f6f6; border: 1px solid #ddd; border-radius: 4px; padding: 10px;
              overflow-x: auto; white-space: pre; }
nav.index { position: sticky; top: 0; background: #fff; border-bottom: 1px solid #ddd;
            padding: 6px 0; margin-bottom: 8px; z-index: 10; }
nav.index a { display: inline-block; margin-right: 10px; text-decoration: none; color: #1f77b4; }
nav.index span { color: #888; }
div.key { margin: 6px 0 12px; color: #555; }
div.key .swatch { margin-right: 12px; white-space: nowrap; }
div.key i { display: inline-block; width: 10px; height: 10px; margin-right: 4px; }
section.chart { margin-bottom: 18px; }
p.empty { color: #888; }
table.events { border-collapse: collapse; margin-top: 12px; width: 100%; }
table.events th, table.events td { border-bottom: 1px solid #eee; padding: 3px 8px;
                                   text-align: left; vertical-align: top; }
table.events td.t { font-variant-numeric: tabular-nums; white-space: nowrap; }
"""

TEMPLATE = """<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<title>{title}</title>
<style>
{vendor_css}
{own_css}
</style>
</head>
<body>
<h1>{heading}</h1>
<pre class="summary">{summary}</pre>
{group_index}
{event_key}
{charts}
<table class="events">
  <thead><tr><th>t (s)</th><th>event</th><th>detail</th></tr></thead>
  <tbody>
{event_rows}
  </tbody>
</table>
<script>
{vendor_js}
</script>
<script>
const DATA = {data_json};
const EVENT_COLORS = {event_colors};
const HEIGHT = {chart_height};
(function () {{
  const SYNC = uPlot.sync("run");
  function eventLines(events) {{
    return {{ hooks: {{ draw: u => {{
      const ctx = u.ctx, bb = u.bbox;
      ctx.save();
      ctx.setLineDash([4, 4]);
      ctx.lineWidth = 1;
      for (const e of events) {{
        const x = u.valToPos(e.t, "x", true);
        if (x < bb.left || x > bb.left + bb.width) continue;
        ctx.strokeStyle = EVENT_COLORS[e.kind] || "{fallback_color}";
        ctx.beginPath();
        ctx.moveTo(x, bb.top);
        ctx.lineTo(x, bb.top + bb.height);
        ctx.stroke();
      }}
      ctx.restore();
    }} }} }};
  }}
  const charts = [];
  for (const group of DATA.groups) {{
    const el = document.getElementById("group-" + group.name);
    if (!el) continue;
    const data = [DATA.x].concat(group.series.map(s => s.values));
    const series = [{{}}].concat(group.series.map(s => {{
      const opts = {{ label: s.label, stroke: s.stroke, width: 1.25 }};
      if (s.flag) opts.paths = uPlot.paths.stepped({{ align: 1 }});
      return opts;
    }}));
    const chart = new uPlot({{
      title: group.title,
      width: el.clientWidth || 900,
      height: HEIGHT,
      scales: {{ x: {{ time: false }} }},
      axes: [{{ label: "t (s)" }}, {{}}],
      cursor: {{ sync: {{ key: SYNC.key }} }},
      series: series,
      plugins: [eventLines(DATA.events)],
    }}, data, el);
    charts.push([chart, el]);
  }}
  // Exposed so a chart can be poked at from the browser console.
  window.RUN_CHARTS = charts.map(([chart]) => chart);
  window.addEventListener("resize", () => {{
    for (const [chart, el] of charts) {{
      chart.setSize({{ width: el.clientWidth || 900, height: HEIGHT }});
    }}
  }});
}})();
</script>
</body>
</html>
"""


if __name__ == "__main__":
    sys.exit(main())
