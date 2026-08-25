"""Dependency-light PNG, CSV, and Markdown exports for four-bar studies."""

from __future__ import annotations

import csv
from math import degrees
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw

from .fourbar import FourBar


def _map(value: float, low: float, high: float, pixel_low: float, pixel_high: float) -> float:
    if high == low:
        return (pixel_low + pixel_high) / 2
    return pixel_low + (value - low) / (high - low) * (pixel_high - pixel_low)


def _draw_plot(linkage: FourBar, states: list, path: Path) -> None:
    image = Image.new("RGB", (1200, 560), "#f8fafc")
    draw = ImageDraw.Draw(image)
    draw.text((36, 22), "MechanismLab — Four-Bar Kinematic Study", fill="#0f172a", font=None)

    left = (55, 75, 575, 505)
    right = (650, 75, 1165, 505)
    for box, title in ((left, "Coupler-point path"), (right, "Output response")):
        draw.rectangle(box, outline="#94a3b8", width=2)
        draw.text((box[0], box[1] - 24), title, fill="#334155")

    points = np.array([linkage.coupler_point(s, 0.55, 0.2 * linkage.coupler) for s in states])
    pad_x = max(1.0, np.ptp(points[:, 0]) * 0.08)
    pad_y = max(1.0, np.ptp(points[:, 1]) * 0.08)
    x0, x1 = float(points[:, 0].min() - pad_x), float(points[:, 0].max() + pad_x)
    y0, y1 = float(points[:, 1].min() - pad_y), float(points[:, 1].max() + pad_y)
    curve = [(_map(x, x0, x1, left[0], left[2]), _map(y, y0, y1, left[3], left[1])) for x, y in points]
    draw.line(curve, fill="#0e7490", width=4, joint="curve")
    for x in (0.0, linkage.ground):
        px, py = _map(x, x0, x1, left[0], left[2]), _map(0, y0, y1, left[3], left[1])
        draw.ellipse((px - 6, py - 6, px + 6, py + 6), fill="#111827")

    angles = np.array([degrees(s.input_angle) for s in states])
    velocity = np.array([s.omega_output for s in states])
    mu = np.array([degrees(s.transmission_angle) / 90 for s in states])
    low, high = float(min(velocity.min(), mu.min())), float(max(velocity.max(), mu.max()))
    margin = max(0.1, (high - low) * 0.08)
    low, high = low - margin, high + margin
    for values, color in ((velocity, "#2563eb"), (mu, "#f97316")):
        line = [(_map(a, 0, 360, right[0], right[2]), _map(v, low, high, right[3], right[1])) for a, v in zip(angles, values)]
        draw.line(line, fill=color, width=3)
    draw.text((right[0] + 12, right[1] + 12), "blue: output angular velocity", fill="#2563eb")
    draw.text((right[0] + 12, right[1] + 32), "orange: transmission angle / 90°", fill="#c2410c")
    draw.text((right[0], right[3] + 10), "0° input", fill="#475569")
    draw.text((right[2] - 54, right[3] + 10), "360°", fill="#475569")
    image.save(path)


def export_study(linkage: FourBar, destination: str | Path, samples: int = 361) -> tuple[Path, Path, Path]:
    destination = Path(destination)
    destination.mkdir(parents=True, exist_ok=True)
    states = [state for state in linkage.sweep(samples=samples) if state is not None]
    if not states:
        raise ValueError("no valid linkage positions were found")

    csv_path = destination / "kinematics.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(("input_deg", "output_deg", "omega_output", "alpha_output", "transmission_deg", "coupler_x", "coupler_y"))
        for state in states:
            x, y = linkage.coupler_point(state, fraction=0.55, offset=0.2 * linkage.coupler)
            writer.writerow((degrees(state.input_angle), degrees(state.output_angle), state.omega_output, state.alpha_output, degrees(state.transmission_angle), x, y))

    figure_path = destination / "mechanism_study.png"
    _draw_plot(linkage, states, figure_path)

    report_path = destination / "REPORT.md"
    min_mu = min(degrees(s.transmission_angle) for s in states)
    report_path.write_text(
        "# Four-bar analysis report\n\n"
        f"- Ground: {linkage.ground:g}\n- Crank: {linkage.crank:g}\n"
        f"- Coupler: {linkage.coupler:g}\n- Rocker: {linkage.rocker:g}\n"
        f"- Assembly: {linkage.assembly}\n- Grübler mobility: {linkage.mobility}\n"
        f"- Grashof condition: {'satisfied' if linkage.grashof else 'not satisfied'}\n"
        f"- Valid samples: {len(states)} of {samples}\n"
        f"- Minimum sampled acute transmission angle: {min_mu:.2f}°\n\n"
        "![Generated mechanism study](mechanism_study.png)\n\n"
        "> Results are numerical model outputs and should be checked against tolerances, loads, and physical prototypes before design release.\n",
        encoding="utf-8",
    )
    return csv_path, figure_path, report_path
