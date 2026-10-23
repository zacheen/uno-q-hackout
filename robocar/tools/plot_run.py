"""Plot one run log, showing ball and goal image positions over time, shaded by phase.

The log comes from make pull-logs or run_sim.py --log-dir.

    conda run -n ML python robocar/tools/plot_run.py robocar/build/runs/run-20261024-101500.csv
"""
import csv
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

PHASE_COLOURS = {"search": "#dddddd", "approach": "#cfe8ff", "lineup": "#fff2b3",
                 "scan": "#e6d5ff", "push": "#c8f0c8", "recover": "#ffd0d0"}


def _float(value: str):
    return float(value) if value else None


def main(path: Path) -> Path:
    with path.open(encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    t = [float(r["t"]) for r in rows]
    fig, ax = plt.subplots(figsize=(12, 5))
    for i, row in enumerate(rows[:-1]):
        ax.axvspan(t[i], t[i + 1], color=PHASE_COLOURS.get(row["phase"], "white"), lw=0)
    for key, style, label in (("ball_x", "o-", "ball x"), ("goal_x", "s-", "goal x"), ("ball_y", "^:", "ball y")):
        points = [(ti, _float(r[key])) for ti, r in zip(t, rows) if r[key]]
        if points:
            ax.plot(*zip(*points), style, ms=3, label=label)
    ax.axhline(0.5, color="grey", lw=0.5)
    ax.set_ylim(0, 1)
    ax.set_xlabel("seconds since start")
    ax.set_ylabel("fraction of image")
    ax.set_title(f"{path.name}, ended: {rows[-1]['reason']}")
    handles = [plt.Rectangle((0, 0), 1, 1, color=c) for c in PHASE_COLOURS.values()]
    ax.legend(ax.get_legend_handles_labels()[0] + handles,
              ax.get_legend_handles_labels()[1] + list(PHASE_COLOURS), loc="upper right", fontsize=8)
    out = path.with_suffix(".png")
    fig.tight_layout()
    fig.savefig(out, dpi=110)
    return out


if __name__ == "__main__":
    print(main(Path(sys.argv[1])))
