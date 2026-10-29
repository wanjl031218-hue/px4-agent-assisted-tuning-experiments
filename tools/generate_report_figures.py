#!/usr/bin/env python3
"""Generate the small, dependency-free SVG figures used by the reports.

The data below is copied from the final Markdown tables. Keeping the generator
in the repository makes the figures reproducible without requiring matplotlib.
"""
from __future__ import annotations

import html
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def esc(value: object) -> str:
    return html.escape(str(value), quote=True)


def text(x: float, y: float, value: object, *, size: int = 12,
         anchor: str = "start", weight: str = "normal", fill: str = "#222",
         rotate: float | None = None) -> str:
    transform = f' transform="rotate({rotate} {x:.1f} {y:.1f})"' if rotate is not None else ""
    return (f'<text x="{x:.1f}" y="{y:.1f}" text-anchor="{anchor}" '
            f'font-family="Arial,sans-serif" font-size="{size}px" '
            f'font-weight="{weight}" fill="{fill}"{transform}>{esc(value)}</text>')


def start_svg(width: int, height: int, title: str) -> list[str]:
    return [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        '<style>.grid{stroke:#d9dee7;stroke-width:1}.axis{stroke:#6b7280;stroke-width:1}.baseline{fill:#5470c6;stroke:#314f9b;stroke-width:1}.candidate{fill:#91cc75;stroke:#4b8f35;stroke-width:1}.base-line{stroke:#5470c6;fill:none;stroke-width:2.5}.cand-line{stroke:#91cc75;fill:none;stroke-width:2.5}.base-dot{fill:#5470c6}.cand-dot{fill:#91cc75}.whisker{stroke:#374151;stroke-width:1.2}.median{stroke:#111827;stroke-width:2}.mean{fill:#f59e0b;stroke:#92400e;stroke-width:1}</style>',
        text(width / 2, 28, title, size=18, anchor="middle", weight="bold"),
    ]


def finish_svg(parts: list[str]) -> str:
    parts.append("</svg>")
    return "\n".join(parts) + "\n"


def nice_max(values: list[float]) -> float:
    return max(values + [1e-9]) * 1.22


def q(values: list[float], p: float) -> float:
    xs = sorted(values)
    if len(xs) == 1:
        return xs[0]
    pos = (len(xs) - 1) * p
    lo, hi = int(math.floor(pos)), int(math.ceil(pos))
    return xs[lo] + (xs[hi] - xs[lo]) * (pos - lo)


def grouped(path: Path, title: str, metrics: list[tuple[str, str, list[float], list[float]]]) -> None:
    width, height = 1500, 560
    parts = start_svg(width, height, title)
    left, top, bottom, gap = 78, 70, 82, 18
    panel_w = (width - left - 32 - gap * (len(metrics) - 1)) / len(metrics)
    plot_h = height - top - bottom
    for i, (label, unit, base, cand) in enumerate(metrics):
        x0 = left + i * (panel_w + gap)
        y0 = top
        ymax = nice_max(base + cand)
        for tick in range(5):
            y = y0 + plot_h - plot_h * tick / 4
            parts.append(f'<line class="grid" x1="{x0:.1f}" x2="{x0 + panel_w:.1f}" y1="{y:.1f}" y2="{y:.1f}"/>')
            parts.append(text(x0 - 7, y + 4, f"{ymax * tick / 4:.3f}", size=9, anchor="end", fill="#4b5563"))
        parts.append(f'<line class="axis" x1="{x0:.1f}" x2="{x0:.1f}" y1="{y0:.1f}" y2="{y0 + plot_h:.1f}"/>')
        parts.append(f'<line class="axis" x1="{x0:.1f}" x2="{x0 + panel_w:.1f}" y1="{y0 + plot_h:.1f}" y2="{y0 + plot_h:.1f}"/>')
        vals = [sum(base) / len(base), sum(cand) / len(cand)]
        colors = ["baseline", "candidate"]
        centers = [x0 + panel_w * 0.34, x0 + panel_w * 0.68]
        bar_w = panel_w * 0.21
        for value, color, cx, raw in zip(vals, colors, centers, [base, cand]):
            bh = plot_h * value / ymax
            by = y0 + plot_h - bh
            parts.append(f'<rect class="{color}" x="{cx - bar_w / 2:.1f}" y="{by:.1f}" width="{bar_w:.1f}" height="{bh:.1f}" rx="2"/>')
            parts.append(text(cx, by - 8, f"{value:.4f}", size=10, anchor="middle"))
            lo, hi = min(raw), max(raw)
            ylo = y0 + plot_h - plot_h * lo / ymax
            yhi = y0 + plot_h - plot_h * hi / ymax
            parts.append(f'<line class="whisker" x1="{cx:.1f}" x2="{cx:.1f}" y1="{yhi:.1f}" y2="{ylo:.1f}"/>')
            parts.append(f'<line class="whisker" x1="{cx - 8:.1f}" x2="{cx + 8:.1f}" y1="{yhi:.1f}" y2="{yhi:.1f}"/>')
            parts.append(f'<line class="whisker" x1="{cx - 8:.1f}" x2="{cx + 8:.1f}" y1="{ylo:.1f}" y2="{ylo:.1f}"/>')
        change = (vals[1] - vals[0]) / vals[0] * 100 if vals[0] else float("nan")
        parts.append(text(x0 + panel_w / 2, y0 + plot_h + 23, label, size=11, anchor="middle", weight="bold"))
        parts.append(text(x0 + panel_w / 2, y0 + plot_h + 40, unit, size=10, anchor="middle", fill="#4b5563"))
        parts.append(text(x0 + panel_w / 2, y0 + plot_h + 61, f"变化 {change:+.2f}%", size=10, anchor="middle", fill="#374151"))
    legend_y = height - 18
    parts.append('<rect class="baseline" x="560" y="{0}" width="14" height="14"/>'.format(legend_y - 11))
    parts.append(text(580, legend_y, "baseline", size=11))
    parts.append('<rect class="candidate" x="700" y="{0}" width="14" height="14"/>'.format(legend_y - 11))
    parts.append(text(720, legend_y, "candidate", size=11))
    path.write_text(finish_svg(parts), encoding="utf-8")


def lines(path: Path, title: str, series: list[tuple[str, list[float], list[float | None], str]]) -> None:
    width, height = 1250, 820
    parts = start_svg(width, height, title)
    left, right, top, bottom, gap = 100, 42, 68, 80, 28
    panel_h = (height - top - bottom - gap * (len(series) - 1)) / len(series)
    x0, x1 = left, width - right
    for pi, (label, base, cand, unit) in enumerate(series):
        y0 = top + pi * (panel_h + gap)
        allv = base + [v for v in cand if v is not None]
        ymin, ymax = 0.0, nice_max(allv)
        for tick in range(5):
            y = y0 + panel_h - panel_h * tick / 4
            parts.append(f'<line class="grid" x1="{x0:.1f}" x2="{x1:.1f}" y1="{y:.1f}" y2="{y:.1f}"/>')
            parts.append(text(x0 - 9, y + 4, f"{ymax * tick / 4:.3f}", size=9, anchor="end", fill="#4b5563"))
        parts.append(text(27, y0 + panel_h / 2 + 4, label, size=11, anchor="middle", weight="bold", rotate=-90))
        def point(idx: int, val: float) -> tuple[float, float]:
            x = x0 + (x1 - x0) * idx / 9
            y = y0 + panel_h - panel_h * (val - ymin) / (ymax - ymin)
            return x, y
        for vals, cls in [(base, "base-line"), (cand, "cand-line")]:
            seg: list[tuple[float, float]] = []
            for idx, value in enumerate(vals):
                if value is None:
                    if len(seg) > 1:
                        parts.append('<polyline class="{}" points="{}"/>'.format(cls, " ".join(f"{x:.1f},{y:.1f}" for x, y in seg)))
                    seg = []
                    continue
                p = point(idx, value)
                seg.append(p)
            if len(seg) > 1:
                parts.append('<polyline class="{}" points="{}"/>'.format(cls, " ".join(f"{x:.1f},{y:.1f}" for x, y in seg)))
            dot_cls = "base-dot" if cls == "base-line" else "cand-dot"
            for idx, value in enumerate(vals):
                if value is not None:
                    x, y = point(idx, value)
                    parts.append(f'<circle class="{dot_cls}" cx="{x:.1f}" cy="{y:.1f}" r="3.5"/>')
        if pi == len(series) - 1:
            for idx in range(10):
                x = x0 + (x1 - x0) * idx / 9
                parts.append(text(x, y0 + panel_h + 19, idx + 1, size=10, anchor="middle"))
            parts.append(text((x0 + x1) / 2, y0 + panel_h + 43, "运行序号（candidate 的参数错配运行保留为空）", size=11, anchor="middle", fill="#4b5563"))
    parts.append('<line class="base-line" x1="1010" x2="1040" y1="43" y2="43"/>')
    parts.append(text(1050, 47, "baseline 1.800", size=11))
    parts.append('<line class="cand-line" x1="1130" x2="1160" y1="43" y2="43"/>')
    parts.append(text(1170, 47, "candidate 1.900", size=11, anchor="end"))
    path.write_text(finish_svg(parts), encoding="utf-8")


def boxplots(path: Path, title: str, metrics: list[tuple[str, list[float], list[float]]]) -> None:
    width, height = 1500, 560
    parts = start_svg(width, height, title)
    left, top, bottom, gap = 78, 70, 82, 18
    panel_w = (width - left - 32 - gap * (len(metrics) - 1)) / len(metrics)
    plot_h = height - top - bottom
    for i, (label, base, cand) in enumerate(metrics):
        x0 = left + i * (panel_w + gap)
        ymax = nice_max(base + cand)
        for tick in range(5):
            y = top + plot_h - plot_h * tick / 4
            parts.append(f'<line class="grid" x1="{x0:.1f}" x2="{x0 + panel_w:.1f}" y1="{y:.1f}" y2="{y:.1f}"/>')
            parts.append(text(x0 - 7, y + 4, f"{ymax * tick / 4:.3f}", size=9, anchor="end", fill="#4b5563"))
        parts.append(text(x0 + panel_w / 2, top - 16, label, size=11, anchor="middle", weight="bold"))
        for value, color, values, cx in [(base, "baseline", base, x0 + panel_w * .34), (cand, "candidate", cand, x0 + panel_w * .68)]:
            lo, q1, med, q3, hi = min(values), q(values, .25), q(values, .5), q(values, .75), max(values)
            def yy(v: float) -> float:
                return top + plot_h - plot_h * v / ymax
            parts.append(f'<line class="whisker" x1="{cx:.1f}" x2="{cx:.1f}" y1="{yy(hi):.1f}" y2="{yy(lo):.1f}"/>')
            parts.append(f'<line class="whisker" x1="{cx - 9:.1f}" x2="{cx + 9:.1f}" y1="{yy(hi):.1f}" y2="{yy(hi):.1f}"/>')
            parts.append(f'<line class="whisker" x1="{cx - 9:.1f}" x2="{cx + 9:.1f}" y1="{yy(lo):.1f}" y2="{yy(lo):.1f}"/>')
            parts.append(f'<rect class="{color}" x="{cx - 18:.1f}" y="{yy(q3):.1f}" width="36" height="{yy(q1) - yy(q3):.1f}" rx="2"/>')
            parts.append(f'<line class="median" x1="{cx - 18:.1f}" x2="{cx + 18:.1f}" y1="{yy(med):.1f}" y2="{yy(med):.1f}"/>')
            parts.append(f'<circle class="mean" cx="{cx:.1f}" cy="{yy(sum(values) / len(values)):.1f}" r="3.5"/>')
        parts.append(text(x0 + panel_w * .34, top + plot_h + 22, "baseline", size=10, anchor="middle"))
        parts.append(text(x0 + panel_w * .68, top + plot_h + 22, "candidate", size=10, anchor="middle"))
    parts.append(text(width / 2, height - 18, "箱体：四分位距；横线：中位数；橙点：均值；须：最小/最大值", size=11, anchor="middle", fill="#4b5563"))
    path.write_text(finish_svg(parts), encoding="utf-8")


S1 = {
    "main": ([.0227,.0487,.0140,.0659,.0184,.0310,.0468,.0382,.0303,.0301], [.0389,.0334,.0158,.0239,.0624,.0194,.0481,.0194,.0273,.0148]),
    "overshoot": ([.0918,.0601,.1332,.1833,.0540,.0945,.1554,.1582,.1330,.0754], [.1058,.0428,.0930,.0629,.1124,.0611,.1413,.0743,.0808,.0704]),
    "settling": ([3.492,3.436,3.540,3.492,3.460,3.484,3.484,3.452,3.492,3.432], [3.388,3.392,3.392,3.360,3.400,3.412,3.352,3.400,3.400,3.448]),
    "speed": ([.0668,.0519,.1420,.0905,.0504,.0548,.0722,.0694,.0519,.0568], [.0921,.0609,.0504,.0568,.0824,.0551,.0719,.0410,.0450,.0376]),
    "hold3d": ([.0956,.0999,.1472,.1582,.0525,.0818,.1158,.1351,.0860,.0681], [.1051,.0954,.0789,.0920,.0996,.0831,.1205,.0718,.0831,.0463]),
}

S2 = {
    "main": ([.011689,.015114,.012809,.019027,.012402,.017122,.009676,.018282,.014979,.011561], [.016438,.020232,.017307,.013527,.013659,.013074,None,.009854,.012879,.017851]),
    "pos3d": ([.044669,.038687,.068545,.053604,.057101,.058608,.041339,.052945,.051405,.041026], [.051656,.035343,.034182,.041832,.057866,.037027,None,.035825,.045140,.046479]),
    "speed": ([.042087,.034351,.067477,.054151,.056358,.055354,.035720,.053538,.049447,.039055], [.051698,.034350,.030470,.040792,.059794,.033943,None,.031931,.042574,.045188]),
}


def main() -> None:
    (ROOT / "report_figures").mkdir(exist_ok=True)
    (ROOT / "report_figures_subject2").mkdir(exist_ok=True)
    # Use the explicit arrays from the final report for each metric.
    grouped(ROOT / "report_figures/metric_group_comparison.svg", "科目 1 单轴响应（n=10/组）：baseline 1.800 vs candidate 1.900", [
        ("主轴 RMSE", "m", S1["main"][0], S1["main"][1]),
        ("最大超调", "m", S1["overshoot"][0], S1["overshoot"][1]),
        ("稳定时间", "s", S1["settling"][0], S1["settling"][1]),
        ("speed RMS", "m/s", S1["speed"][0], S1["speed"][1]),
        ("HOLD 最大三维误差", "m", S1["hold3d"][0], S1["hold3d"][1]),
    ])
    lines(ROOT / "report_figures/per_run_primary_metrics.svg", "科目 1 逐运行主要指标（10 次/组）", [
        ("主轴 RMSE (m)", S1["main"][0], S1["main"][1], "m"),
        ("最大超调 (m)", S1["overshoot"][0], S1["overshoot"][1], "m"),
        ("speed RMS (m/s)", S1["speed"][0], S1["speed"][1], "m/s"),
    ])
    boxplots(ROOT / "report_figures/metric_distribution_boxplot.svg", "科目 1 指标分布（n=10/组；箱体为四分位距）", [
        ("主轴 RMSE", S1["main"][0], S1["main"][1]),
        ("最大超调", S1["overshoot"][0], S1["overshoot"][1]),
        ("稳定时间", S1["settling"][0], S1["settling"][1]),
        ("speed RMS", S1["speed"][0], S1["speed"][1]),
        ("HOLD 三维误差", S1["hold3d"][0], S1["hold3d"][1]),
    ])
    grouped(ROOT / "report_figures_subject2/group_comparison.svg", "科目 2 悬停：baseline 1.800 vs candidate 1.900（有效 candidate n=9）", [
        ("主轴 RMSE", "m", S2["main"][0], [v for v in S2["main"][1] if v is not None]),
        ("三维 RMSE", "m", S2["pos3d"][0], [v for v in S2["pos3d"][1] if v is not None]),
        ("速度 RMS", "m/s", S2["speed"][0], [v for v in S2["speed"][1] if v is not None]),
    ])
    lines(ROOT / "report_figures_subject2/per_run.svg", "科目 2 逐运行指标（candidate 运行 7 为参数错配，留空）", [
        ("主轴 RMSE (m)", S2["main"][0], S2["main"][1], "m"),
        ("三维 RMSE (m)", S2["pos3d"][0], S2["pos3d"][1], "m"),
        ("速度 RMS (m/s)", S2["speed"][0], S2["speed"][1], "m/s"),
    ])


if __name__ == "__main__":
    main()
