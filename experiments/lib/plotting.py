"""Shared matplotlib helpers used by the experiment drivers.

Lightweight wrappers around matplotlib primitives that bake in the paper's
figure size + font conventions.
"""
from __future__ import annotations

from pathlib import Path
from typing import Any

import matplotlib
matplotlib.use("Agg")             # non-interactive backend
import matplotlib.pyplot as plt
import numpy as np


# One place for every paper figure's element sizing so all experiments'
# plots match (label/tick/legend sizes, figure size). Labels and ticks are
# 1.5x the base font, matching the ranging (exp2) figure.
_PAPER_RC = {
    "figure.figsize":  (6.5, 4.0),
    "savefig.dpi":     200,
    "axes.grid":       True,
    "grid.alpha":      0.3,
    "font.size":       10,
    "axes.labelsize":  15,
    "axes.titlesize":  15,
    "xtick.labelsize": 15,
    "ytick.labelsize": 15,
    "legend.fontsize": 12,
}


def apply_paper_style() -> None:
    """Install paper-style rcParams onto matplotlib globally."""
    for k, v in _PAPER_RC.items():
        plt.rcParams[k] = v


def save_figure(fig: matplotlib.figure.Figure,
                path: str | Path,
                *,
                tight: bool = True) -> Path:
    """Write ``fig`` to ``path`` (creating parent dirs) and close it."""
    p = Path(path)
    p.parent.mkdir(parents=True, exist_ok=True)
    if tight:
        fig.tight_layout()
    fig.savefig(p)
    plt.close(fig)
    return p


def waterfall(grid: np.ndarray,
              *,
              x_axis: np.ndarray,
              y_axis: np.ndarray,
              cmap: str = "viridis",
              x_label: str = "",
              y_label: str = "",
              cbar_label: str = "",
              title: str = "",
              log_db: bool = False,
              vmin: float | None = None,
              vmax: float | None = None,
              savepath: str | Path | None = None) -> matplotlib.figure.Figure:
    """Render a 2D intensity grid as a waterfall (pcolormesh).

    ``grid`` has shape ``(len(y_axis), len(x_axis))``. With ``log_db=True``
    the grid is converted to ``20*log10(|grid|)`` before plotting and
    ``vmin``/``vmax`` are interpreted in dB; the floor is clamped to
    ``10**(vmin/20)`` so the colormap clips cleanly.
    """
    apply_paper_style()
    fig, ax = plt.subplots()
    if log_db:
        floor_lin = 10.0 ** ((vmin if vmin is not None else -60.0) / 20.0)
        plot_grid = 20.0 * np.log10(np.maximum(grid, floor_lin))
    else:
        plot_grid = grid
    mesh = ax.pcolormesh(x_axis, y_axis, plot_grid, cmap=cmap, shading="auto",
                         vmin=vmin, vmax=vmax)
    ax.set_xlabel(x_label)
    ax.set_ylabel(y_label)
    ax.set_title(title)
    cbar = fig.colorbar(mesh, ax=ax)
    cbar.set_label(cbar_label)
    if savepath is not None:
        save_figure(fig, savepath)
    return fig


def overlay_tap_lines(fig: matplotlib.figure.Figure,
                      *,
                      ranges_m: np.ndarray,
                      delays_per_path: dict[str, np.ndarray],
                      x_unit: str = "ms",
                      colors: dict[str, str] | None = None,
                      ) -> None:
    """Overlay analytical-tap excess-delay curves onto a waterfall figure.

    ``delays_per_path`` maps path name -> array of excess delays (in the
    same x-unit as the waterfall x-axis, default milliseconds) aligned
    with ``ranges_m``. Path names get distinct colours and labels.
    """
    palette = {"direct": "tab:red", "surface": "white", "bottom": "tab:orange"}
    if colors:
        palette.update(colors)
    if not fig.axes:
        return
    ax = fig.axes[0]
    for name, delays in delays_per_path.items():
        ax.plot(np.asarray(delays), np.asarray(ranges_m),
                color=palette.get(name, "white"),
                linewidth=1.2, linestyle="--", label=f"τ_{name}")
    ax.legend(loc="upper right", framealpha=0.85, fontsize=8)
    ax.set_xlabel(f"Excess delay ({x_unit})")


def ecdf(values: np.ndarray,
         *,
         x_label: str = "",
         title: str = "",
         savepath: str | Path | None = None
         ) -> matplotlib.figure.Figure:
    """Empirical CDF curve."""
    apply_paper_style()
    v = np.asarray(values, dtype=float)
    v = v[~np.isnan(v)]
    v.sort()
    y = np.arange(1, len(v) + 1) / max(len(v), 1)
    fig, ax = plt.subplots()
    ax.plot(v, y)
    ax.set_xlabel(x_label)
    ax.set_ylabel("ECDF")
    ax.set_ylim(0, 1)
    ax.set_title(title)
    if savepath is not None:
        save_figure(fig, savepath)
    return fig


def violin_by_category(values_by_category: dict[str, np.ndarray],
                       *,
                       order: list[str] | None = None,
                       y_label: str = "",
                       x_label: str = "",
                       savepath: str | Path | None = None
                       ) -> matplotlib.figure.Figure:
    """Violin plot with one violin per category.

    ``values_by_category`` maps category label -> array of samples. ``order``
    pins category order on the x-axis; default is insertion order.
    """
    apply_paper_style()
    labels = order if order is not None else list(values_by_category.keys())
    data = [np.asarray(values_by_category[k], dtype=float) for k in labels]
    # matplotlib's violinplot rejects empty arrays; substitute a single-NaN
    # array so the slot is preserved on the x-axis.
    data = [d[~np.isnan(d)] if d.size else np.array([np.nan]) for d in data]

    fig, ax = plt.subplots()
    parts = ax.violinplot(data, showmeans=False, showmedians=True,
                          showextrema=True)
    for body in parts["bodies"]:
        body.set_alpha(0.6)
    ax.set_xticks(np.arange(1, len(labels) + 1))
    ax.set_xticklabels(labels)
    ax.set_xlabel(x_label)
    ax.set_ylabel(y_label)
    if savepath is not None:
        save_figure(fig, savepath)
    return fig


def per_vs_range(ranges_m: np.ndarray,
                 per: np.ndarray,
                 *,
                 label: str = "",
                 ax: matplotlib.axes.Axes | None = None,
                 savepath: str | Path | None = None
                 ) -> matplotlib.figure.Figure:
    """Packet-error-rate vs range, suitable for overlay across categories."""
    apply_paper_style()
    if ax is None:
        fig, ax = plt.subplots()
    else:
        fig = ax.figure
    ax.plot(ranges_m, per, marker="o", label=label)
    ax.set_xlabel("Range (m)")
    ax.set_ylabel("Packet error rate")
    ax.set_ylim(0, 1)
    if label:
        ax.legend()
    if savepath is not None:
        save_figure(fig, savepath)
    return fig
