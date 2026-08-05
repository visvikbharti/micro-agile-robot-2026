"""Dimensioned two-view engineering drawing of the micro quadrotor airframe.

Renders out/frame_drawing.png (landscape, 160 dpi). Every number is tied to
quadsim/params.py: m = 0.033 kg (33 g), L = 0.046 m (46 mm), f_motor_max =
0.16 N/rotor, X mixer with rotors 1,3 (+x+y / -x-y) CCW and rotors 2,4 CW.

Top view is drawn with body +x (forward) up the page and body +y to the left
(FLU, +z out of the page), so the page matches the sim's body frame exactly:
R1 upper-left, R2 lower-left, R3 lower-right, R4 upper-right.
"""
from __future__ import annotations

import math
import os

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Circle, Ellipse, FancyBboxPatch, Rectangle

# ----------------------------------------------------------------------------
# Governing numbers (all mm unless noted) — keep consistent with quadsim/params.py
# ----------------------------------------------------------------------------
ARM_L = 46.0                          # params.L = 0.046 m
DIAG = 2.0 * ARM_L                    # motor-to-motor diagonal = 92
ADJ = DIAG / math.sqrt(2.0)           # adjacent motor spacing = 65.05
PROP_D = 55.0                         # prop diameter (55 mm Hubsan X4 class)
CLEAR = ADJ - PROP_D                  # tip-to-tip clearance = 10.05
FOOT = DIAG + PROP_D                  # overall footprint = 147
MOTOR_D = 7.0                         # motor can diameter
MOTOR_H = 16.0                        # motor can height (7x16 coreless)
PLATE_T = 3.0                         # frame plate thickness
FC_H = 5.0                            # FC stack height above plate
BATT_H = 7.0                          # battery slung below plate (GNB 300 LiHV ~7 mm thick)
OVERALL_H = BATT_H + PLATE_T + MOTOR_H  # 26 mm, battery bottom -> prop plane

D = ARM_L / math.sqrt(2.0)            # 32.527 — motor x/y offset

INK = "#141414"       # primary linework
MID = "#4d4d4d"       # secondary linework (spin arrows, leaders)
GHOST = "#8c8c8c"     # phantom lines (prop discs, prop plane)
FAINT = "#a9a9a9"     # dotted FC outline
ACC = "#0E63B4"       # accent: dimension lines only

OUT_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "out")
OUT_PNG = os.path.join(OUT_DIR, "frame_drawing.png")


# ----------------------------------------------------------------------------
# Drafting helpers
# ----------------------------------------------------------------------------

def dim(ax, p1, p2, offset, text, *, outside=False, text_pos=None, text_rot=None,
        fs=8.5, gap1=1.5, gap2=1.5, over=2.2, tshift=3.4, arrow_len=8.0):
    """Linear dimension between p1 and p2, dimension line offset perpendicular."""
    p1 = np.asarray(p1, float)
    p2 = np.asarray(p2, float)
    u = p2 - p1
    u = u / np.hypot(*u)
    n = np.array([-u[1], u[0]])
    q1 = p1 + offset * n
    q2 = p2 + offset * n
    s = 1.0 if offset >= 0 else -1.0
    ne = n * s
    for p, q, g in ((p1, q1, gap1), (p2, q2, gap2)):
        a = p + g * ne
        b = q + over * ne
        ax.plot([a[0], b[0]], [a[1], b[1]], color=ACC, lw=0.7, zorder=6)
    arrow_kw = dict(color=ACC, lw=0.9, mutation_scale=11, shrinkA=0, shrinkB=0)
    if outside:
        ax.plot([q1[0], q2[0]], [q1[1], q2[1]], color=ACC, lw=0.9, zorder=6)
        for q, sgn in ((q1, -1.0), (q2, 1.0)):
            tail = q + sgn * arrow_len * u
            ax.annotate("", xy=q, xytext=tail, zorder=6,
                        arrowprops=dict(arrowstyle="-|>", **arrow_kw))
    else:
        ax.annotate("", xy=q1, xytext=q2, zorder=6,
                    arrowprops=dict(arrowstyle="<|-|>", **arrow_kw))
    mid = 0.5 * (q1 + q2)
    if text_pos is None:
        text_pos = mid + tshift * ne
    if text_rot is None:
        ang = math.degrees(math.atan2(u[1], u[0]))
        if ang > 90.01:
            ang -= 180.0
        elif ang < -90.01:
            ang += 180.0
        text_rot = ang
    ax.text(text_pos[0], text_pos[1], text, rotation=text_rot, fontsize=fs,
            color=ACC, ha="center", va="center", zorder=8,
            bbox=dict(fc="white", ec="none", pad=0.6))


def leader(ax, tip, txt_xy, text, *, fs=7.2, color=MID, ha="left"):
    """Part-name leader with a small arrow."""
    ax.annotate(text, xy=tip, xytext=txt_xy, fontsize=fs, color=INK, ha=ha,
                va="center", zorder=8,
                bbox=dict(fc="white", ec="none", pad=0.4),
                arrowprops=dict(arrowstyle="-|>", color=color, lw=0.8,
                                mutation_scale=8, shrinkA=1, shrinkB=1))


def spin_arc(ax, c, r, a0, a1, *, color=MID):
    """Arc from a0 to a1 deg (direction of travel = spin direction), arrow at end."""
    th = np.radians(np.linspace(a0, a1, 60))
    x = c[0] + r * np.cos(th)
    y = c[1] + r * np.sin(th)
    ax.plot(x, y, color=color, lw=1.3, zorder=5, solid_capstyle="round")
    ax.annotate("", xy=(x[-1], y[-1]), xytext=(x[-4], y[-4]), zorder=5,
                arrowprops=dict(arrowstyle="-|>", color=color, lw=1.3,
                                mutation_scale=12, shrinkA=0, shrinkB=0))


def cross(ax, c, s=2.6, lw=0.6):
    ax.plot([c[0] - s, c[0] + s], [c[1], c[1]], color=INK, lw=lw, zorder=5)
    ax.plot([c[0], c[0]], [c[1] - s, c[1] + s], color=INK, lw=lw, zorder=5)


# ----------------------------------------------------------------------------
# Figure & sheet
# ----------------------------------------------------------------------------
fig = plt.figure(figsize=(16.5, 10.4), facecolor="white")
fig.add_artist(Rectangle((0.005, 0.005), 0.990, 0.990, transform=fig.transFigure,
                         fill=False, ec=INK, lw=1.5))

# ============================================================================
# TOP VIEW
# ============================================================================
ax = fig.add_axes([0.010, 0.118, 0.600, 0.862])
ax.set_aspect("equal")
ax.axis("off")
ax.set_xlim(-131, 101)
ax.set_ylim(-129, 97)

# page positions (u = -y_body, v = +x_body):  R1 UL, R2 LL, R3 LR, R4 UR
motors = {
    1: np.array([-D, D]),    # body (+d,+d)  CCW
    2: np.array([-D, -D]),   # body (-d,+d)  CW
    3: np.array([D, -D]),    # body (-d,-d)  CCW
    4: np.array([D, D]),     # body (+d,-d)  CW
}
phi = {1: 135.0, 2: 225.0, 3: 315.0, 4: 45.0}   # outward arm angle on page
ccw = {1: True, 2: False, 3: True, 4: False}

# --- arms (white fill polygons + edges), then plate on top -------------------
for k, m in motors.items():
    a = math.radians(phi[k])
    du = np.array([math.cos(a), math.sin(a)])
    pe = np.array([-du[1], du[0]])
    t1, t2, w1, w2 = 12.0, ARM_L, 5.0, 3.6
    poly = np.array([t1 * du + w1 * pe, t2 * du + w2 * pe,
                     t2 * du - w2 * pe, t1 * du - w1 * pe])
    ax.fill(poly[:, 0], poly[:, 1], fc="white", ec="none", zorder=2)
    ax.plot(poly[[0, 1], 0], poly[[0, 1], 1], color=INK, lw=1.3, zorder=3)
    ax.plot(poly[[2, 3], 0], poly[[2, 3], 1], color=INK, lw=1.3, zorder=3)

plate = FancyBboxPatch((-13, -13), 26, 26, boxstyle="round,pad=0,rounding_size=5",
                       fc="white", ec=INK, lw=1.3, zorder=3)
ax.add_patch(plate)

# FC footprint (phantom, 26x26 whoop AIO)
fc_sq = Rectangle((-13, -13), 26, 26, fill=False, ec=FAINT, lw=0.9,
                  ls=(0, (1.5, 2.2)), zorder=3.4)
ax.add_patch(fc_sq)
leader(ax, (6.0, -13.0), (0.0, -27.0), "AIO FC 26×26 (FC+4 ESC+RX)", ha="center", fs=6.8)

# --- prop discs, motor mounts, motors ---------------------------------------
for k, m in motors.items():
    ax.add_patch(Circle(m, PROP_D / 2.0, fill=False, ec=GHOST, lw=1.2,
                        ls=(0, (6, 4)), zorder=2.5))
    ax.add_patch(Circle(m, 5.5, fc="white", ec=INK, lw=0.8, zorder=3.6))
    ax.add_patch(Circle(m, MOTOR_D / 2.0, fc="white", ec=INK, lw=1.4, zorder=3.8))
    cross(ax, m)
cross(ax, (0, 0), s=3.2, lw=0.7)

# --- rotor numbers, spin arrows ---------------------------------------------
arc_span = {1: (110, 205), 2: (205, 110), 3: (-70, 70), 4: (70, -70)}
for k, m in motors.items():
    a = math.radians(phi[k])
    du = np.array([math.cos(a), math.sin(a)])
    pe = np.array([du[1], -du[0]])          # rotate -90 deg
    c_num = m + 31.0 * du
    ax.add_patch(Circle(c_num, 4.2, fc="white", ec=INK, lw=1.2, zorder=7))
    ax.text(c_num[0], c_num[1], str(k), fontsize=9.5, fontweight="bold",
            color=INK, ha="center", va="center", zorder=8)
    t_spin = m + 31.0 * du + 10.5 * pe
    ax.text(t_spin[0], t_spin[1], "CCW" if ccw[k] else "CW", fontsize=7.0,
            color=MID, ha="center", va="center", zorder=8,
            bbox=dict(fc="white", ec="none", pad=0.3))
    a0, a1 = arc_span[k]
    spin_arc(ax, m, 13.0, a0, a1)

# --- body axes ---------------------------------------------------------------
ax.annotate("", xy=(0, 17), xytext=(0, 0), zorder=7,
            arrowprops=dict(arrowstyle="-|>", color=INK, lw=1.5, mutation_scale=13))
ax.annotate("", xy=(-20, 0), xytext=(0, 0), zorder=7,
            arrowprops=dict(arrowstyle="-|>", color=INK, lw=1.5, mutation_scale=13))
ax.text(-2.5, 17.5, "x (FWD)", fontsize=8.5, fontweight="bold", color=INK,
        ha="right", va="center", zorder=8, bbox=dict(fc="white", ec="none", pad=0.3))
ax.text(-23.5, 3.2, "y", fontsize=8.5, fontweight="bold", color=INK,
        ha="center", va="center", zorder=8)
ax.text(-7.5, -7.5, "z⊙", fontsize=7.0, color=MID, ha="center", va="center",
        zorder=8)

# --- dimensions --------------------------------------------------------------
M1, M2, M3, M4 = motors[1], motors[2], motors[3], motors[4]

# arm length 46 (center -> R1), dim line on the NE side of the arm
dim(ax, (0, 0), M1, -19.0, "46", gap1=4.0, gap2=7.0)

# adjacent motor spacing 65.05 (R1 -> R4), above the vehicle
dim(ax, M1, M4, 86.0 - D, "65.05")

# motor-to-motor diagonal 92 (R1 -> R3) and overall footprint 143, stacked LL
dim(ax, M1, M3, -86.0, "92")
tip1 = (ARM_L + PROP_D / 2.0) * np.array([math.cos(math.radians(135)),
                                          math.sin(math.radians(135))])
tip3 = -tip1
dim(ax, tip1, tip3, -100.0, f"{FOOT:.0f} TIP-TO-TIP")

# prop tip clearance 14.05 between adjacent discs (left pair R1/R2), outside arrows
ea = np.array([-D, -D + PROP_D / 2.0])  # top edge of R2 disc
eb = np.array([-D, D - PROP_D / 2.0])   # bottom edge of R1 disc
for e in (ea, eb):
    ax.plot([e[0] - 3.2, e[0] + 2.2], [e[1], e[1]], color=ACC, lw=0.7, zorder=6)
ax.plot([ea[0], eb[0]], [ea[1], eb[1]], color=ACC, lw=0.9, zorder=6)
for e, sgn in ((ea, -1.0), (eb, 1.0)):
    ax.annotate("", xy=(e[0], e[1]), xytext=(e[0], e[1] + sgn * 9.0), zorder=6,
                arrowprops=dict(arrowstyle="-|>", color=ACC, lw=0.9,
                                mutation_scale=10, shrinkA=0, shrinkB=0))
ax.plot([-D - 1.8, -37.5], [0, 0], color=ACC, lw=0.7, zorder=6)
ax.text(-38.5, 0, f"{CLEAR:.2f} CLR", fontsize=8.0, color=ACC, ha="right",
        va="center", zorder=8, bbox=dict(fc="white", ec="none", pad=0.5))

# prop diameter leader on R3 disc
pt = M3 + (PROP_D / 2.0) * np.array([math.cos(math.radians(20)),
                                     math.sin(math.radians(20))])
ax.annotate("Ø55 (2-BLADE)", xy=pt, xytext=(70, -14), fontsize=8.5, color=ACC,
            ha="left", va="center", zorder=8,
            bbox=dict(fc="white", ec="none", pad=0.5),
            arrowprops=dict(arrowstyle="-|>", color=ACC, lw=0.9, mutation_scale=10))

# view title
ax.text(-128, 90, "TOP VIEW", fontsize=12, fontweight="bold", color=INK,
        ha="left", va="center")
ax.text(-128, 83, "X CONFIGURATION — LOOKING DOWN (+z OUT OF PAGE)\n"
                  "R1, R3 CCW · R2, R4 CW (quadsim mixer)",
        fontsize=7.5, color=MID, ha="left", va="top")

# ============================================================================
# SIDE VIEW
# ============================================================================
sx = fig.add_axes([0.622, 0.545, 0.370, 0.420])
sx.set_aspect("equal")
sx.axis("off")
sx.set_xlim(-76, 80)
sx.set_ylim(-25, 43)

# plate
sx.add_patch(Rectangle((-40, 0), 80, PLATE_T, fc="white", ec=INK, lw=1.3, zorder=3))
# battery slung below plate
sx.add_patch(Rectangle((-30, -BATT_H), 60, BATT_H, fc="white", ec=INK, lw=1.2, zorder=2.8))
# FC stack above plate
sx.add_patch(Rectangle((-13, PLATE_T), 26, FC_H, fc="white", ec=INK, lw=1.1, zorder=3))
# motors + props (projected pair at +-D)
for xm in (-D, D):
    sx.add_patch(Rectangle((xm - MOTOR_D / 2.0, PLATE_T), MOTOR_D, MOTOR_H,
                           fc="white", ec=INK, lw=1.3, zorder=3.2))
    sx.add_patch(Ellipse((xm, PLATE_T + MOTOR_H), PROP_D, 2.0, fc="white",
                         ec=INK, lw=1.1, zorder=3.4))
# prop plane (phantom)
yp = PLATE_T + MOTOR_H
sx.plot([-66, 66], [yp, yp], color=GHOST, lw=0.9, ls=(0, (8, 3, 1.5, 3)), zorder=2.5)
sx.text(0, 23.4, "PROP PLANE", fontsize=6.5, color=MID, ha="center", va="center",
        zorder=8, bbox=dict(fc="white", ec="none", pad=0.3))

# stack-height dimension chain (left, dim lines at x = -52)
dim(sx, (-36.03, PLATE_T), (-36.03, yp), 16.0, "16", text_pos=(-58.5, 11.0),
    text_rot=0, gap2=1.0)
dim(sx, (-40, 0), (-40, PLATE_T), 12.0, "3", outside=True, arrow_len=6.0,
    text_pos=(-58.5, 1.5), text_rot=0)
dim(sx, (-30, -BATT_H), (-30, 0), 22.0, f"{BATT_H:.0f}", outside=True, arrow_len=6.0,
    text_pos=(-58.5, -3.0), text_rot=0, gap2=11.5)
# FC height 5
dim(sx, (13, PLATE_T), (13, PLATE_T + FC_H), -8.0, "5", outside=True, arrow_len=6.0,
    text_pos=(25.5, 5.5), text_rot=0)
# overall height (battery bottom -> prop plane), right side
sx.plot([31.5, 68.0], [-BATT_H, -BATT_H], color=ACC, lw=0.7, zorder=6)
dim(sx, (D + PROP_D / 2.0, -BATT_H), (D + PROP_D / 2.0, yp), -8.0,
    f"{OVERALL_H:.0f} OVERALL", fs=8.0, gap1=1.0, gap2=1.0)

# part labels
leader(sx, (D + 1.5, 14.0), (46.0, 31.0), "7×16 CORELESS\nMOTOR (×4)")
leader(sx, (0.0, PLATE_T + FC_H + 0.4), (0.0, 14.8), "AIO FC (BRUSHED)", ha="center")
leader(sx, (0.0, -BATT_H - 0.4), (0.0, -14.5), "BATTERY 1S 300 mAh LiHV (SLUNG)",
       ha="center")
leader(sx, (-38.0, 1.5), (-46.0, -12.0), "FRAME PLATE t=3", ha="right")

sx.text(-74, 39.5, "SIDE VIEW", fontsize=12, fontweight="bold", color=INK,
        ha="left", va="center")
sx.text(-74, 34.5, "STACK HEIGHTS", fontsize=7.5, color=MID, ha="left", va="center")

# ============================================================================
# NOTES
# ============================================================================
nx = fig.add_axes([0.622, 0.118, 0.370, 0.402])
nx.axis("off")
nx.set_xlim(0, 1)
nx.set_ylim(0, 1)
nx.add_patch(Rectangle((0, 0), 1, 1, fill=False, ec=INK, lw=1.0))
nx.text(0.03, 0.955, "NOTES — PARTS & CONSISTENCY", fontsize=9.5,
        fontweight="bold", color=INK, ha="left", va="top")
notes = (
    "1. MATCHES quadsim/params.py: m = 0.033 kg (33 g), L = 0.046 m (46 mm),\n"
    "   f_motor_max = 0.16 N/rotor (16.3 gf). MIXER: R1(+x+y), R3(−x−y) CCW;\n"
    "   R2(−x+y), R4(+x−y) CW.  d = L/√2 = 32.53 mm.\n"
    "2. ALL-UP MASS 33 g TARGET, 36 g HARD MAX.  BUDGET: MOTORS 4×3.2 = 12.8 +\n"
    "   PROPS 4×0.4 = 1.6 + AIO FC 3.0 + BATTERY 7.8 + FRAME 6.0 +\n"
    "   SCREWS/STRAP 0.9 = 32.1 g.\n"
    "3. MOTORS: 7×16 mm BRUSHED CORELESS, ~19000 KV (BETAFPV / HAPPYMODEL\n"
    "   7×16 CLASS).  STATIC THRUST ≥ 16.5 gf EACH WITH Ø55 PROP ON 1S HV →\n"
    "   TOTAL ≥ 66 gf, THRUST/WEIGHT ≈ 2.0 AT 33 g.\n"
    "4. PROPS: Ø55 mm 2-BLADE, 2× CW + 2× CCW (HUBSAN X4 H107 CLASS,\n"
    "   1.0 mm BORE).  SPACING 65.05 − 55 → TIP CLEARANCE 10.05 mm (≥ 10 REQ'D).\n"
    "5. FC: WHOOP-CLASS BRUSHED AIO — FC + 4× BRUSHED ESC + RADIO RX ON ONE\n"
    "   26×26 mm BOARD (BETAFPV F4 1S BRUSHED CLASS).\n"
    "6. BATTERY: 1S 260–350 mAh LiPo/LiHV (PH2.0 300 mAh HV TYP., GNB CLASS),\n"
    "   SLUNG 7 mm UNDER PLATE.\n"
    "7. ALL DIMENSIONS mm.  FOOTPRINT 92 + 55 = 147 mm TIP-TO-TIP.  ARMS AT 45°."
)
nx.text(0.03, 0.875, notes, fontsize=7.6, color=INK, ha="left", va="top",
        linespacing=1.55)

# ============================================================================
# TITLE BLOCK
# ============================================================================
tb = fig.add_axes([0.008, 0.010, 0.984, 0.096])
tb.axis("off")
tb.set_xlim(0, 1)
tb.set_ylim(0, 1)
tb.add_patch(Rectangle((0, 0), 1, 1, fill=False, ec=INK, lw=1.5))
for xdiv in (0.40, 0.70, 0.795, 0.90):
    tb.plot([xdiv, xdiv], [0, 1], color=INK, lw=1.0)

tb.text(0.012, 0.66, "MICRO-AGILE-ROBOT-2026", fontsize=13, fontweight="bold",
        color=INK, ha="left", va="center")
tb.text(0.012, 0.24, "92 mm BRUSHED X-QUADROTOR — FRAME, STACK & PROP CLEARANCE",
        fontsize=7.8, color=MID, ha="left", va="center")
tb.text(0.55, 0.66, "matches quadsim/params.py: m = 33 g, L = 46 mm",
        fontsize=8.6, fontweight="bold", color=INK, ha="center", va="center")
tb.text(0.55, 0.24, "f_motor_max = 0.16 N/rotor  ·  X-MIXER: R1, R3 CCW / R2, R4 CW",
        fontsize=7.6, color=MID, ha="center", va="center")
tb.text(0.7475, 0.66, "DATE: 2026-08-06", fontsize=8.2, color=INK,
        ha="center", va="center")
tb.text(0.7475, 0.24, "UNITS: mm", fontsize=8.2, color=INK, ha="center", va="center")
tb.text(0.8475, 0.66, "SCALE: NTS", fontsize=8.2, color=INK, ha="center", va="center")
tb.text(0.8475, 0.24, "SHEET 1 OF 1", fontsize=8.2, color=INK, ha="center", va="center")
tb.text(0.95, 0.66, "DWG MAR26-HW-001", fontsize=7.6, color=INK,
        ha="center", va="center")
tb.text(0.95, 0.24, "REV A", fontsize=8.2, color=INK, ha="center", va="center")

# ----------------------------------------------------------------------------
os.makedirs(OUT_DIR, exist_ok=True)
fig.savefig(OUT_PNG, dpi=160, facecolor="white")
print(f"wrote {OUT_PNG}")
