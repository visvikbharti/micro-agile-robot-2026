"""Visualization for quadsim: 3D path plots, tracking panels, and GIF animations.

All functions work headless (Agg backend) and never call ``plt.show()``. Figures that are
saved to disk are closed immediately afterwards. Obstacles (:class:`Box`, :class:`Gate`) are
visual props only — trajectories avoid them by waypoint choice; there is no collision engine.
"""
from __future__ import annotations

import os
from dataclasses import dataclass
from typing import Any, List, Optional, Sequence, Tuple

import numpy as np
import matplotlib.pyplot as plt
from matplotlib import animation
from matplotlib.figure import Figure
from mpl_toolkits.mplot3d.art3d import Line3DCollection, Poly3DCollection

__all__ = ["Box", "Gate", "plot_trajectory_3d", "plot_tracking", "animate_quads"]


@dataclass
class Box:
    """Axis-aligned cuboid obstacle (visual prop only).

    Attributes:
        center: (3,) world-frame center of the cuboid [m].
        size:   (3,) full edge lengths along x, y, z [m].
    """

    center: np.ndarray
    size: np.ndarray


@dataclass
class Gate:
    """Rectangular frame to fly through; the opening is perpendicular to ``axis``.

    Attributes:
        center: (3,) world-frame center of the opening [m].
        width:  opening width [m] (horizontal extent of the frame).
        height: opening height [m] (vertical extent of the frame).
        axis:   "x" or "y" — the direction of flight through the gate.
    """

    center: np.ndarray
    width: float
    height: float
    axis: str = "x"


def _quat_to_rot(q: np.ndarray) -> np.ndarray:
    """Rotation matrix (body->world) of a unit quaternion ``[w, x, y, z]``."""
    q = np.asarray(q, dtype=float)
    q = q / np.linalg.norm(q)
    w, x, y, z = q
    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - w * z), 2.0 * (x * z + w * y)],
            [2.0 * (x * y + w * z), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - w * x)],
            [2.0 * (x * z - w * y), 2.0 * (y * z + w * x), 1.0 - 2.0 * (x * x + y * y)],
        ]
    )


def _as_history_list(histories: Any) -> List[Any]:
    """Accept a single History-like object or a sequence of them; return a list."""
    if hasattr(histories, "p") and hasattr(histories, "t"):
        return [histories]
    return list(histories)


def _ensure_parent_dir(path: str) -> None:
    """Create the parent directory of ``path`` if it does not exist."""
    parent = os.path.dirname(path)
    if parent:
        os.makedirs(parent, exist_ok=True)


def _gate_corners(gate: Gate) -> np.ndarray:
    """(4,3) corners of the gate frame, in drawing order (closed by repeating corner 0)."""
    c = np.asarray(gate.center, dtype=float)
    if gate.axis == "x":
        u = np.array([0.0, 0.5 * gate.width, 0.0])
    elif gate.axis == "y":
        u = np.array([0.5 * gate.width, 0.0, 0.0])
    else:
        raise ValueError(f"Gate.axis must be 'x' or 'y', got {gate.axis!r}")
    v = np.array([0.0, 0.0, 0.5 * gate.height])
    return np.array([c - u - v, c + u - v, c + u + v, c - u + v])


def _box_corners(box: Box) -> np.ndarray:
    """(8,3) corners of an axis-aligned box."""
    c = np.asarray(box.center, dtype=float)
    h = 0.5 * np.asarray(box.size, dtype=float)
    signs = np.array(
        [[sx, sy, sz] for sx in (-1.0, 1.0) for sy in (-1.0, 1.0) for sz in (-1.0, 1.0)]
    )
    return c + signs * h


def _draw_obstacles(ax: Any, obstacles: Sequence[Any]) -> None:
    """Draw Box and Gate obstacles onto a 3D axes."""
    for ob in obstacles:
        if isinstance(ob, Box):
            k = _box_corners(ob)
            faces = [
                [k[0], k[1], k[3], k[2]],
                [k[4], k[5], k[7], k[6]],
                [k[0], k[1], k[5], k[4]],
                [k[2], k[3], k[7], k[6]],
                [k[0], k[2], k[6], k[4]],
                [k[1], k[3], k[7], k[5]],
            ]
            poly = Poly3DCollection(
                faces, facecolors="tab:gray", edgecolors="dimgray", linewidths=0.6, alpha=0.25
            )
            ax.add_collection3d(poly)
        elif isinstance(ob, Gate):
            corners = _gate_corners(ob)
            loop = np.vstack([corners, corners[:1]])
            ax.plot(loop[:, 0], loop[:, 1], loop[:, 2], color="tab:orange", linewidth=2.5)


def _obstacle_points(obstacles: Sequence[Any]) -> List[np.ndarray]:
    """Corner points of all obstacles, for axis-limit computation."""
    pts: List[np.ndarray] = []
    for ob in obstacles:
        if isinstance(ob, Box):
            pts.append(_box_corners(ob))
        elif isinstance(ob, Gate):
            pts.append(_gate_corners(ob))
    return pts


def _set_equal_3d(ax: Any, pts: np.ndarray, pad: float = 0.25) -> None:
    """Near-equal aspect: cubical limits around ``pts`` (K,3) with padding [m]."""
    lo = pts.min(axis=0)
    hi = pts.max(axis=0)
    center = 0.5 * (lo + hi)
    half = 0.5 * float(np.max(hi - lo)) + pad
    ax.set_xlim(center[0] - half, center[0] + half)
    ax.set_ylim(center[1] - half, center[1] + half)
    ax.set_zlim(center[2] - half, center[2] + half)
    ax.set_box_aspect((1.0, 1.0, 1.0))


def plot_trajectory_3d(
    histories: Any,
    obstacles: Sequence[Any] = (),
    waypoints: Optional[np.ndarray] = None,
    save: Optional[str] = None,
    title: str = "",
) -> Optional[Figure]:
    """Plot 3D flight path(s) with waypoint markers and obstacles, near-equal aspect.

    Args:
        histories: a ``quadsim.sim.History``-like object, or a sequence of them.
        obstacles: iterable of :class:`Box` / :class:`Gate` props to draw.
        waypoints: optional (M,3) array of waypoints to mark.
        save: if given, save a PNG to this path and close the figure.
        title: figure title.

    Returns:
        The figure if ``save`` is None, else None (the figure is saved and closed).
    """
    hists = _as_history_list(histories)
    fig = plt.figure(figsize=(7.0, 6.0))
    ax = fig.add_subplot(projection="3d")
    cmap = plt.get_cmap("tab10")
    for i, h in enumerate(hists):
        label = "flight path" if len(hists) == 1 else f"quad {i}"
        ax.plot(h.p[:, 0], h.p[:, 1], h.p[:, 2], color=cmap(i % 10), linewidth=1.4, label=label)
    if waypoints is not None:
        wp = np.asarray(waypoints, dtype=float)
        ax.scatter(
            wp[:, 0], wp[:, 1], wp[:, 2], color="crimson", edgecolor="black",
            s=45, depthshade=False, label="waypoints",
        )
    _draw_obstacles(ax, obstacles)
    pts = [h.p for h in hists] + _obstacle_points(obstacles)
    if waypoints is not None:
        pts.append(np.asarray(waypoints, dtype=float))
    _set_equal_3d(ax, np.vstack(pts))
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_zlabel("z [m]")
    if title:
        ax.set_title(title)
    if len(hists) > 1 or waypoints is not None:
        ax.legend(loc="upper left", fontsize=8)
    if save is not None:
        _ensure_parent_dir(save)
        fig.savefig(save, dpi=150, bbox_inches="tight")
        plt.close(fig)
        return None
    return fig


def plot_tracking(
    history: Any,
    save: Optional[str] = None,
    title: str = "",
    f_max: float = 0.64,
) -> Optional[Figure]:
    """Plot a 4-panel tracking summary for a single flight.

    Panels: xyz vs t with dashed references; position error norm; thrust ``f`` with the
    ``f_max`` line; body rates.

    Args:
        history: a ``quadsim.sim.History``-like object.
        save: if given, save a PNG to this path and close the figure.
        title: figure suptitle.
        f_max: total thrust limit [N] drawn as a horizontal line in the thrust panel
            (default 0.64 N = 4 * 0.16 N, matching the default ``QuadParams``).

    Returns:
        The figure if ``save`` is None, else None (the figure is saved and closed).
    """
    h = history
    t = np.asarray(h.t, dtype=float)
    fig, axs = plt.subplots(2, 2, figsize=(11.0, 7.5), sharex=True)

    ax = axs[0, 0]
    for k, name in enumerate(("x", "y", "z")):
        ax.plot(t, h.p[:, k], color=f"C{k}", linewidth=1.2, label=name)
        ax.plot(t, h.ref_p[:, k], color=f"C{k}", linewidth=1.0, linestyle="--")
    ax.set_ylabel("position [m]")
    ax.set_title("position (refs dashed)")
    ax.legend(loc="best", fontsize=8)
    ax.grid(True, alpha=0.3)

    ax = axs[0, 1]
    err = np.linalg.norm(np.asarray(h.p) - np.asarray(h.ref_p), axis=1)
    ax.plot(t, err, color="tab:red", linewidth=1.2)
    ax.set_ylabel("|p - ref_p| [m]")
    ax.set_title("position error norm")
    ax.grid(True, alpha=0.3)

    ax = axs[1, 0]
    ax.plot(t, h.f, color="tab:blue", linewidth=1.2, label="f")
    ax.axhline(f_max, color="tab:red", linestyle="--", linewidth=1.0, label="f_max")
    ax.set_xlabel("t [s]")
    ax.set_ylabel("thrust [N]")
    ax.set_title("total thrust")
    ax.legend(loc="best", fontsize=8)
    ax.grid(True, alpha=0.3)

    ax = axs[1, 1]
    for k, name in enumerate(("wx", "wy", "wz")):
        ax.plot(t, h.w[:, k], color=f"C{k}", linewidth=1.2, label=name)
    ax.set_xlabel("t [s]")
    ax.set_ylabel("body rate [rad/s]")
    ax.set_title("body rates")
    ax.legend(loc="best", fontsize=8)
    ax.grid(True, alpha=0.3)

    if title:
        fig.suptitle(title)
    fig.tight_layout()
    if save is not None:
        _ensure_parent_dir(save)
        fig.savefig(save, dpi=150)
        plt.close(fig)
        return None
    return fig


def animate_quads(
    histories: Any,
    params: Any,
    obstacles: Sequence[Any] = (),
    save: str = "out/anim.gif",
    fps: int = 25,
    max_frames: int = 400,
    trail: float = 3.0,
    elev: float = 25,
    azim: float = -60,
    title: str = "",
    scale: float = 4.0,
) -> str:
    """Animate quadrotor flight(s) in 3D and save a GIF; return the saved path.

    Each quad is drawn as its two arm segments (rotated by the body attitude R), four rotor
    dots, and a fading trail of the last ``trail`` seconds. Frames are uniformly subsampled
    so the total number of frames is <= ``max_frames``.

    Args:
        histories: a ``quadsim.sim.History``-like object, or a sequence of them (all sharing
            the same time grid, as produced by lockstep simulation).
        params: ``QuadParams``-like object; ``params.L`` sets the true arm length.
        obstacles: iterable of :class:`Box` / :class:`Gate` props to draw.
        save: output GIF path (parent directory is created if needed).
        fps: GIF playback frame rate.
        max_frames: upper bound on the number of animation frames.
        trail: trail length in seconds of flight time.
        elev: 3D view elevation angle [deg].
        azim: 3D view azimuth angle [deg].
        title: axes title.
        scale: visual magnification of the true airframe geometry. Quads are drawn at their
            true size (arm length ``params.L``) times this factor; the default 4.0 keeps a
            ~9 cm robot visible in a ~4 m scene.

    Returns:
        The path the GIF was saved to (``save``).
    """
    hists = _as_history_list(histories)
    t = np.asarray(hists[0].t, dtype=float)
    n = t.shape[0]
    n_frames = int(min(n, max_frames))
    frame_idx = np.unique(np.round(np.linspace(0, n - 1, n_frames)).astype(int))

    d = float(params.L) / np.sqrt(2.0)
    r_body = scale * np.array(
        [[+d, +d, 0.0], [-d, +d, 0.0], [-d, -d, 0.0], [+d, -d, 0.0]]
    )
    reach = scale * float(params.L)

    fig = plt.figure(figsize=(7.0, 6.0))
    ax = fig.add_subplot(projection="3d")
    pts = [np.asarray(h.p, dtype=float) for h in hists] + _obstacle_points(obstacles)
    _set_equal_3d(ax, np.vstack(pts), pad=0.25 + reach)
    _draw_obstacles(ax, obstacles)
    ax.view_init(elev=elev, azim=azim)
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_zlabel("z [m]")
    if title:
        ax.set_title(title)
    time_text = ax.text2D(0.02, 0.95, "", transform=ax.transAxes, fontsize=9)

    cmap = plt.get_cmap("tab10")
    arms1, arms2, dots, trails = [], [], [], []
    for i, h in enumerate(hists):
        color = cmap(i % 10)
        (a1,) = ax.plot([], [], [], color="black", linewidth=1.6)
        (a2,) = ax.plot([], [], [], color="black", linewidth=1.6)
        (dt_,) = ax.plot([], [], [], linestyle="", marker="o", markersize=4, color=color)
        p0 = np.asarray(h.p[0], dtype=float)
        lc = Line3DCollection([np.stack([p0, p0])], linewidths=1.4)
        ax.add_collection3d(lc)
        arms1.append(a1)
        arms2.append(a2)
        dots.append(dt_)
        trails.append(lc)

    base_colors = [np.array(cmap(i % 10)) for i in range(len(hists))]

    def _update(k: int) -> Tuple[Any, ...]:
        i_now = frame_idx[k]
        t_now = t[i_now]
        time_text.set_text(f"t = {t_now:5.2f} s")
        for j, h in enumerate(hists):
            R = _quat_to_rot(np.asarray(h.q[i_now], dtype=float))
            rotors = np.asarray(h.p[i_now], dtype=float) + r_body @ R.T
            arms1[j].set_data_3d(rotors[[0, 2], 0], rotors[[0, 2], 1], rotors[[0, 2], 2])
            arms2[j].set_data_3d(rotors[[1, 3], 0], rotors[[1, 3], 1], rotors[[1, 3], 2])
            dots[j].set_data_3d(rotors[:, 0], rotors[:, 1], rotors[:, 2])
            past = frame_idx[: k + 1]
            past = past[t[past] >= t_now - trail]
            tail = np.asarray(h.p, dtype=float)[past]
            if tail.shape[0] >= 2:
                segs = np.stack([tail[:-1], tail[1:]], axis=1)
                alphas = np.linspace(0.1, 0.9, segs.shape[0])
                colors = np.tile(base_colors[j], (segs.shape[0], 1))
                colors[:, 3] = alphas
                trails[j].set_segments(list(segs))
                trails[j].set_color(colors)
            else:
                trails[j].set_segments([])
        return tuple(arms1 + arms2 + dots + trails + [time_text])

    anim = animation.FuncAnimation(
        fig, _update, frames=len(frame_idx), interval=1000.0 / fps, blit=False
    )
    _ensure_parent_dir(save)
    anim.save(save, writer=animation.PillowWriter(fps=fps))
    plt.close(fig)
    return save
