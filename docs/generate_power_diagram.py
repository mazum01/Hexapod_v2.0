#!/usr/bin/env python3
"""
Generate MARS Hexapod Power Wiring Diagram (PDF) — Dual Battery Pack
Includes Star Ground, Inline Signal Resistors, External Charge Port, and Transient Capacitor.
"""

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, Rectangle
import os

# ── Colour palette (LCARS-inspired) ────────────────────────────────
COL_BG       = "#1A1A2E"
COL_WIRE     = "#FF9966"       # Main power wire (hot)
COL_WIRE_GND = "#6699CC"       # Ground wire
COL_WIRE_SIG = "#99CC66"       # Signal / UART
COL_WIRE_5V  = "#66CC99"       # 5V rail
COL_CELL_A   = "#CC6699"       # Servo pack cells
COL_CELL_B   = "#9977BB"       # Electronics pack cells
COL_BMS_A    = "#9966CC"       # DALY BMS
COL_BMS_B    = "#7799BB"       # Amazon BMS
COL_FUSE     = "#FFCC66"       # Fuses
COL_SWITCH   = "#66CCCC"       # Switches
COL_LOAD     = "#FF6666"       # Load boxes
COL_CONV     = "#66CC99"       # Converter
COL_HUB      = "#44AA88"       # Star Ground Hub
COL_EXT      = "#FF88AA"       # External Port
COL_TEXT     = "#EEEEFF"
COL_LABEL    = "#AABBDD"
COL_TITLE    = "#FF9966"
COL_NOTE     = "#888899"
COL_OUTLINE  = "#556677"


def draw_box(ax, x, y, w, h, label, color, sublabel=None, fontsize=8, bold=True):
    box = FancyBboxPatch(
        (x, y), w, h, boxstyle="round,pad=0.02",
        facecolor=color, edgecolor="#FFFFFF", linewidth=1.2, alpha=0.9
    )
    ax.add_patch(box)
    weight = "bold" if bold else "normal"
    offset = 0.015 if sublabel else 0
    ax.text(x + w / 2, y + h / 2 + offset, label,
            ha="center", va="center", fontsize=fontsize,
            fontweight=weight, color="#FFFFFF", family="monospace")
    if sublabel:
        ax.text(x + w / 2, y + h / 2 - 0.025, sublabel,
                ha="center", va="center", fontsize=5.5,
                color="#DDDDDD", family="monospace")


def draw_cell(ax, x, y, label, color):
    w, h = 0.055, 0.042
    rect = Rectangle((x, y), w, h, facecolor=color,
                      edgecolor="#FFFFFF", linewidth=0.8, alpha=0.9)
    ax.add_patch(rect)
    nub = Rectangle((x + w, y + h * 0.3), 0.006, h * 0.4,
                     facecolor="#FFFFFF", edgecolor="none")
    ax.add_patch(nub)
    ax.text(x + w / 2, y + h / 2, label, ha="center", va="center",
            fontsize=4.5, color="#FFFFFF", fontweight="bold", family="monospace")


def draw_fuse(ax, x, y, rating):
    w, h = 0.065, 0.028
    box = FancyBboxPatch(
        (x, y), w, h, boxstyle="round,pad=0.004",
        facecolor=COL_FUSE, edgecolor="#FFFFFF", linewidth=1.0, alpha=0.9
    )
    ax.add_patch(box)
    ax.text(x + w / 2, y + h / 2, f"F {rating}", ha="center", va="center",
            fontsize=5, color="#333333", fontweight="bold", family="monospace")


def draw_switch(ax, x, y, label):
    w, h = 0.075, 0.032
    box = FancyBboxPatch(
        (x, y), w, h, boxstyle="round,pad=0.005",
        facecolor=COL_SWITCH, edgecolor="#FFFFFF", linewidth=1.2, alpha=0.9
    )
    ax.add_patch(box)
    ax.text(x + w / 2, y + h / 2, label, ha="center", va="center",
            fontsize=5, color="#111111", fontweight="bold", family="monospace")


def draw_capacitor(ax, x, y, label):
    w, h = 0.03, 0.05
    # Capacitor symbol lines
    ax.plot([x - 0.01, x + 0.01], [y, y], color=COL_OUTLINE, lw=2)      # Top plate
    ax.plot([x - 0.01, x + 0.01], [y - 0.01, y - 0.01], color=COL_OUTLINE, lw=2) # Bottom plate
    ax.plot([x, x], [y, y + 0.02], color=COL_WIRE, lw=1.5)             # Top wire
    ax.plot([x, x], [y - 0.01, y - 0.03], color=COL_WIRE_GND, lw=1.5)  # Bottom wire
    
    # Body fill (just to look nice)
    rect = FancyBboxPatch((x - 0.015, y - 0.04), 0.03, 0.07, boxstyle="round,pad=0.005", 
                          facecolor="#AAAAAA", alpha=0.3, edgecolor="none", zorder=-1)
    ax.add_patch(rect)
    ax.text(x + 0.025, y - 0.005, label, ha="left", va="center", fontsize=4.5, 
            color="#FFDD77", family="monospace", fontweight="bold")
    ax.text(x, y + 0.005, "+", ha="center", va="bottom", fontsize=5, color=COL_WIRE, fontweight="bold")


def draw_wire(ax, points, color=COL_WIRE, lw=1.5, style="-", zorder=2):
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    ax.plot(xs, ys, color=color, linewidth=lw, linestyle=style,
            solid_capstyle="round", zorder=zorder)


def draw_dot(ax, x, y, color=COL_WIRE):
    ax.plot(x, y, "o", color=color, markersize=3.5, zorder=5)


def draw_label(ax, x, y, text, fontsize=5.5, color=COL_LABEL, ha="center", va="center"):
    ax.text(x, y, text, ha=ha, va=va, fontsize=fontsize,
            color=color, family="monospace", style="italic")


def draw_pack_outline(ax, x, y, w, h, title):
    rect = FancyBboxPatch(
        (x, y), w, h, boxstyle="round,pad=0.008",
        facecolor="none", edgecolor=COL_OUTLINE, linewidth=1.5,
        linestyle="--", alpha=0.7
    )
    ax.add_patch(rect)
    ax.text(x + w / 2, y + h + 0.012, title,
            ha="center", va="center", fontsize=6.5, fontweight="bold",
            color=COL_LABEL, family="monospace")


def draw_resistor(ax, x, y, label):
    w, h = 0.025, 0.015
    rect = Rectangle((x - w/2, y - h/2), w, h, facecolor="#DDCC33", 
                     edgecolor="#FFFFFF", lw=0.8, zorder=10)
    ax.add_patch(rect)
    ax.text(x, y, label, ha="center", va="center", fontsize=4.0, 
            color="#222222", fontweight="bold", family="monospace", zorder=11)


def generate_diagram():
    fig, ax = plt.subplots(1, 1, figsize=(16, 10))
    fig.patch.set_facecolor(COL_BG)
    ax.set_facecolor(COL_BG)
    ax.set_xlim(0, 1.6)
    ax.set_ylim(0, 1.0)
    ax.set_aspect("equal")
    ax.axis("off")

    # ── Title ───────────────────────────────────────────────────────
    ax.text(0.80, 0.97, "MARS — Hexapod v2.0  Power Wiring Diagram",
            ha="center", va="center", fontsize=15, fontweight="bold",
            color=COL_TITLE, family="monospace")
    ax.text(0.80, 0.945,
            "Dual Battery  |  Isolated Charge Port  |  Star Ground Protected  |  Transient Buffered",
            ha="center", va="center", fontsize=6.5, color=COL_NOTE,
            family="monospace")

    # ================================================================
    #  TOP HALF — SERVO POWER RAIL
    # ================================================================
    ax.text(0.80, 0.90,
            "━━━━━━━━━━━━━━━━━  SERVO POWER RAIL  ━━━━━━━━━━━━━━━━━",
            ha="center", fontsize=7, fontweight="bold", color=COL_WIRE,
            family="monospace")

    # ── Servo Battery Pack (3S2P) ───────────────────────────────────
    draw_pack_outline(ax, 0.03, 0.58, 0.26, 0.28,
                      "SERVO PACK — 3S2P  (11.1V nom, 12.6V full)")

    cx_a = 0.055
    cells_a_ys = [0.77, 0.70, 0.63]
    for i, cy in enumerate(cells_a_ys):
        draw_cell(ax, cx_a, cy, f"A{i+1}", COL_CELL_A)
        if i < 2:
            draw_wire(ax, [(cx_a + 0.061, cy + 0.021), (cx_a + 0.061, cy - 0.021),
                           (cx_a, cy - 0.021)], color=COL_WIRE, lw=1.0)

    cx_b = 0.175
    for i, cy in enumerate(cells_a_ys):
        draw_cell(ax, cx_b, cy, f"B{i+1}", COL_CELL_A)
        if i < 2:
            draw_wire(ax, [(cx_b + 0.061, cy + 0.021), (cx_b + 0.061, cy - 0.021),
                           (cx_b, cy - 0.021)], color=COL_WIRE, lw=1.0)

    top_par_y = 0.835
    draw_wire(ax, [(cx_a + 0.061, 0.812), (cx_a + 0.061, top_par_y),
                   (cx_b + 0.061, top_par_y), (cx_b + 0.061, 0.812)], color=COL_WIRE, lw=1.5)
    draw_dot(ax, cx_a + 0.061, 0.812)
    draw_dot(ax, cx_b + 0.061, 0.812)
    draw_label(ax, 0.145, top_par_y + 0.012, "P+", color=COL_WIRE)

    bot_par_y = 0.615
    draw_wire(ax, [(cx_a, 0.651), (cx_a, bot_par_y),
                   (cx_b, bot_par_y), (cx_b, 0.651)], color=COL_WIRE_GND, lw=1.5)
    draw_dot(ax, cx_a, 0.651, COL_WIRE_GND)
    draw_dot(ax, cx_b, 0.651, COL_WIRE_GND)
    draw_label(ax, 0.145, bot_par_y - 0.012, "P–", color=COL_WIRE_GND)

    ax.text(0.16, 0.585, "Samsung 50S 21700  (5000mAh, 25A)",
            ha="center", fontsize=5, color=COL_NOTE, family="monospace")

    # ── DALY BMS ────────────────────────────────────────────────────
    bms1_x, bms1_y = 0.33, 0.62
    bms1_w, bms1_h = 0.14, 0.22
    draw_box(ax, bms1_x, bms1_y, bms1_w, bms1_h, "DALY BMS", COL_BMS_A,
             sublabel="3S 40A Li-ion\nUART 9600 8N1\nBal 100mA", fontsize=9)

    bal_ys = [0.80, 0.76, 0.72, 0.68]
    bal_labels = ["B+", "B1", "B2", "B–"]
    for label, by in zip(bal_labels, bal_ys):
        draw_wire(ax, [(0.29, by), (bms1_x, by)], color="#AAAACC", lw=0.7, style="--")
        ax.text(bms1_x - 0.012, by, label, ha="right", va="center", fontsize=4.5, color="#AAAACC", family="monospace")

    draw_wire(ax, [(cx_b + 0.061, top_par_y), (0.30, top_par_y), (0.30, 0.80), (bms1_x, 0.80)], color=COL_WIRE, lw=2)
    draw_wire(ax, [(cx_b, bot_par_y), (0.30, bot_par_y), (0.30, 0.68), (bms1_x, 0.68)], color=COL_WIRE_GND, lw=2)

    bms1_r = bms1_x + bms1_w
    out_plus_y = 0.80
    out_minus_y = 0.68

    draw_wire(ax, [(bms1_r, out_plus_y), (0.55, out_plus_y)], color=COL_WIRE, lw=3)
    draw_label(ax, 0.50, out_plus_y + 0.012, "P+", color=COL_WIRE)

    # ── Fuse 1 ──────────────────────────────────────────────────────
    draw_fuse(ax, 0.55, out_plus_y - 0.014, "50A")
    draw_wire(ax, [(0.615, out_plus_y), (0.65, out_plus_y)], color=COL_WIRE, lw=3)

    # ── Switch 1 ────────────────────────────────────────────────────
    draw_switch(ax, 0.65, out_plus_y - 0.016, "SW1 SERVO")
    draw_wire(ax, [(0.725, out_plus_y), (0.80, out_plus_y)], color=COL_WIRE, lw=3)

    # ── Transient Capacitor ─────────────────────────────────────────
    cap_x = 0.77
    cap_y = out_plus_y - 0.05
    draw_capacitor(ax, cap_x, cap_y, "3300µF 35V\nLow-ESR Cap")
    draw_wire(ax, [(cap_x, out_plus_y), (cap_x, cap_y + 0.02)], color=COL_WIRE, lw=1.5)
    draw_dot(ax, cap_x, out_plus_y, COL_WIRE)


    # ── Servo Bus Load ──────────────────────────────────────────────
    servo_box_y = out_plus_y - 0.045
    draw_box(ax, 0.84, servo_box_y, 0.28, 0.09,
             "SERVO BUS  (11.1V direct)", COL_LOAD,
             sublabel="18× HTS-35S  |  6 UART buses × 3 servos  |  6-12V rated",
             fontsize=8)
    draw_label(ax, 0.80, out_plus_y + 0.012, "11.1V–12.6V", fontsize=5, color=COL_WIRE)


    # ================================================================
    #  DIVIDER
    # ================================================================
    iso_y = 0.525
    ax.plot([0.3, 1.55], [iso_y, iso_y], color="#333344", linewidth=1.5, linestyle="--", alpha=0.5)

    # ================================================================
    #  EXTERNAL CHARGING / POWER UMBILICAL
    # ================================================================
    port_x, port_y = 0.05, 0.485
    port_w, port_h = 0.16, 0.08
    draw_box(ax, port_x, port_y, port_w, port_h, "EXTERNAL UMBILICAL", COL_EXT,
             sublabel="4-Pin Charging & Power Input\nIsolates the two loops", fontsize=7)

    port_rx = port_x + port_w
    # Pin 1: Servo Charge/Power (+12.6V)
    draw_wire(ax, [(port_rx, port_y + 0.06), (0.48, port_y + 0.06), (0.48, out_plus_y)], color="#FFAA88", lw=1.5, style="--")
    draw_dot(ax, 0.48, out_plus_y, COL_WIRE)
    ax.text(port_rx + 0.01, port_y + 0.065, "P1: Servo V_CHG", ha="left", va="bottom", fontsize=5, color="#FFAA88", family="monospace")

    # Pin 2: Elec Charge/Power (+12.6V)
    draw_wire(ax, [(port_rx, port_y + 0.04), (0.45, port_y + 0.04), (0.45, 0.40)], color="#FFAA88", lw=1.5, style="--")
    draw_dot(ax, 0.45, 0.40, COL_WIRE)
    ax.text(port_rx + 0.01, port_y + 0.045, "P2: Elec V_CHG", ha="left", va="bottom", fontsize=5, color="#FFAA88", family="monospace")

    # Pins 3/4: Common Ground Return
    draw_wire(ax, [(port_rx, port_y + 0.02), (0.35, port_y + 0.02), (0.35, iso_y - 0.02), (0.51, iso_y - 0.02)], color=COL_WIRE_GND, lw=2.0)
    ax.text(port_rx + 0.01, port_y + 0.025, "P3/4: GND Ext", ha="left", va="bottom", fontsize=5, color=COL_WIRE_GND, family="monospace")


    # ================================================================
    #  BOTTOM HALF — ELECTRONICS POWER RAIL
    # ================================================================
    ax.text(0.80, 0.50,
            "━━━━━━━━━━━━━━━━  ELECTRONICS POWER RAIL  ━━━━━━━━━━━━━━━━",
            ha="center", fontsize=7, fontweight="bold", color=COL_CONV,
            family="monospace")

    draw_pack_outline(ax, 0.03, 0.22, 0.22, 0.22,
                      "ELECTRONICS PACK — 3S1P  (11.1V nom, 12.6V full)")

    ecx = 0.10
    ecells_ys = [0.37, 0.30, 0.23]
    for i, cy in enumerate(ecells_ys):
        draw_cell(ax, ecx, cy, f"C{i+1}", COL_CELL_B)
        if i < 2:
            draw_wire(ax, [(ecx + 0.061, cy + 0.021), (ecx + 0.061, cy - 0.021),
                           (ecx, cy - 0.021)], color=COL_WIRE, lw=1.0)

    e_plus_y = 0.43
    draw_wire(ax, [(ecx + 0.061, 0.412), (ecx + 0.061, e_plus_y)], color=COL_WIRE, lw=1.5)
    draw_label(ax, ecx + 0.085, e_plus_y + 0.01, "+", color=COL_WIRE)

    e_minus_y = 0.215
    draw_wire(ax, [(ecx, 0.251), (ecx, e_minus_y)], color=COL_WIRE_GND, lw=1.5)
    draw_label(ax, ecx - 0.02, e_minus_y, "–", color=COL_WIRE_GND)

    ax.text(0.14, 0.21, "Samsung 35E 18650  (3500mAh, 8A)", ha="center", fontsize=5, color=COL_NOTE, family="monospace")

    # ── Amazon BMS ──────────────────────────────────────────────────
    bms2_x, bms2_y = 0.29, 0.24
    bms2_w, bms2_h = 0.14, 0.20
    draw_box(ax, bms2_x, bms2_y, bms2_w, bms2_h, "AMAZON", COL_BMS_B,
             sublabel="BMS 3S 10A\nLi-ion", fontsize=9)

    bal2_ys = [0.40, 0.37, 0.34, 0.31]
    bal2_labels = ["B+", "B1", "B2", "B–"]
    for label, by in zip(bal2_labels, bal2_ys):
        draw_wire(ax, [(0.25, by), (bms2_x, by)], color="#AAAACC", lw=0.7, style="--")
        ax.text(bms2_x - 0.012, by, label, ha="right", va="center", fontsize=4.5, color="#AAAACC", family="monospace")

    draw_wire(ax, [(ecx + 0.061, e_plus_y), (0.26, e_plus_y), (0.26, 0.40), (bms2_x, 0.40)], color=COL_WIRE, lw=2)
    draw_wire(ax, [(ecx, e_minus_y), (0.26, e_minus_y), (0.26, 0.31), (bms2_x, 0.31)], color=COL_WIRE_GND, lw=2)

    bms2_r = bms2_x + bms2_w
    e_out_plus_y = 0.40
    e_out_minus_y = 0.31

    draw_wire(ax, [(bms2_r, e_out_plus_y), (0.55, e_out_plus_y)], color=COL_WIRE, lw=2.5)
    draw_label(ax, 0.46, e_out_plus_y + 0.012, "P+", color=COL_WIRE)

    # ── Fuse 2 ──────────────────────────────────────────────────────
    draw_fuse(ax, 0.55, e_out_plus_y - 0.014, "10A")
    draw_wire(ax, [(0.615, e_out_plus_y), (0.65, e_out_plus_y)], color=COL_WIRE, lw=2)

    # ── Switch 2 ────────────────────────────────────────────────────
    draw_switch(ax, 0.65, e_out_plus_y - 0.016, "SW2 ELEC")
    draw_wire(ax, [(0.725, e_out_plus_y), (0.80, e_out_plus_y)], color=COL_WIRE, lw=2)

    # ── Smart USB Converter ─────────────────────────────────────────
    draw_box(ax, 0.80, e_out_plus_y - 0.04, 0.16, 0.08,
             "SMART USB", COL_CONV, sublabel="Auto Voltage\n12V → 5V USB", fontsize=8)

    v5_y = e_out_plus_y
    draw_wire(ax, [(0.96, v5_y), (1.00, v5_y)], color=COL_WIRE_5V, lw=2.5)
    draw_label(ax, 0.98, v5_y + 0.015, "5V USB", fontsize=6, color=COL_WIRE_5V)

    # ── Pi 5 ────────────────────────────────────────────────────────
    pi_x, pi_y = 1.00, v5_y - 0.01
    draw_box(ax, pi_x, pi_y, 0.15, 0.06, "Raspberry Pi 5", COL_LOAD, sublabel="5V / 3A  (USB-C)", fontsize=7)

    # ── Teensy 4.1 ──────────────────────────────────────────────────
    tnsy_x, tnsy_y = 1.01, v5_y - 0.12
    draw_wire(ax, [(1.07, v5_y), (1.07, v5_y - 0.065)], color=COL_WIRE_5V, lw=1.2)
    draw_box(ax, tnsy_x, tnsy_y, 0.13, 0.05, "Teensy 4.1", COL_LOAD, sublabel="5V Vin", fontsize=6)

    # ── Peripherals ───────────────────────────────────────────────
    draw_wire(ax, [(1.15, v5_y), (1.20, v5_y), (1.20, v5_y + 0.04)], color=COL_WIRE_5V, lw=1.0)
    draw_box(ax, 1.15, v5_y + 0.04, 0.10, 0.04, "PAM8403", COL_LOAD, sublabel="Audio Amp", fontsize=5)

    draw_wire(ax, [(1.20, v5_y), (1.20, v5_y - 0.055)], color=COL_WIRE_5V, lw=1.0)
    draw_box(ax, 1.15, v5_y - 0.095, 0.10, 0.04, "ST7789", COL_LOAD, sublabel="Display", fontsize=5)

    draw_wire(ax, [(1.15, v5_y + 0.02), (1.30, v5_y + 0.02), (1.30, v5_y + 0.04)], color=COL_WIRE_5V, lw=0.8)
    draw_box(ax, 1.26, v5_y + 0.04, 0.09, 0.035, "USB DAC", COL_LOAD, sublabel="Sabrent", fontsize=5)

    draw_wire(ax, [(1.30, v5_y), (1.37, v5_y), (1.37, v5_y - 0.03)], color=COL_WIRE_5V, lw=0.8)
    draw_box(ax, 1.30, v5_y - 0.07, 0.14, 0.04, "IMU + ToF", COL_LOAD, sublabel="BNO085 + VL53L5CX", fontsize=5)

    # GND for elec loads
    gnd_elec_y = e_out_minus_y - 0.07
    draw_wire(ax, [(0.88, e_out_minus_y), (0.88, gnd_elec_y), (1.37, gnd_elec_y)], color=COL_WIRE_GND, lw=1.0)
    draw_label(ax, 1.10, gnd_elec_y - 0.012, "GND (electronics common)", fontsize=5, color=COL_WIRE_GND)


    # ================================================================
    #  STAR GROUND HUB & CAPACITOR RETURN
    # ================================================================
    hub_x, hub_y = 0.51, iso_y - 0.035
    hub_w, hub_h = 0.16, 0.07
    draw_box(ax, hub_x, hub_y, hub_w, hub_h, "STAR GROUND HUB", COL_HUB,
             sublabel="(Heavy Busbar) 1 Point Only", fontsize=7)

    # Servo BMS P- -> Hub
    draw_wire(ax, [(0.47, out_minus_y), (0.49, out_minus_y), (0.49, hub_y + hub_h - 0.015), (hub_x, hub_y + hub_h - 0.015)], color=COL_WIRE_GND, lw=3.0)
    draw_label(ax, 0.49, out_minus_y + 0.015, "P–", color=COL_WIRE_GND)

    # Elec BMS P- -> Hub
    draw_wire(ax, [(0.47, e_out_minus_y), (0.49, e_out_minus_y), (0.49, hub_y + 0.015), (hub_x, hub_y + 0.015)], color=COL_WIRE_GND, lw=3.0)
    draw_label(ax, 0.49, e_out_minus_y - 0.015, "P–", color=COL_WIRE_GND)
    
    # Hub -> Cap Ground Return
    draw_wire(ax, [(cap_x, cap_y - 0.01), (cap_x, iso_y + 0.02), (hub_x + hub_w/2, iso_y + 0.02), (hub_x + hub_w/2, hub_y + hub_h)], color=COL_WIRE_GND, lw=1.5)
    draw_dot(ax, hub_x + hub_w/2, hub_y + hub_h, COL_WIRE_GND)

    # Hub -> Servo Bus
    draw_wire(ax, [(hub_x + hub_w, hub_y + hub_h - 0.015), (0.72, hub_y + hub_h - 0.015), 
                   (0.72, out_minus_y), (0.94, out_minus_y), (0.94, servo_box_y)], color=COL_WIRE_GND, lw=2.5)

    # Hub -> Elec Bus
    draw_wire(ax, [(hub_x + hub_w, hub_y + 0.015), (0.72, hub_y + 0.015), 
                   (0.72, e_out_minus_y), (0.83, e_out_minus_y), (0.83, e_out_plus_y - 0.04)], color=COL_WIRE_GND, lw=2.5)


    # ================================================================
    #  INLINE SIGNAL RESISTORS (UART Crossover Protection)
    # ================================================================
    uart1_y = bms1_y - 0.015
    draw_wire(ax, [(bms1_x + bms1_w / 2, bms1_y), (bms1_x + bms1_w / 2, uart1_y - 0.075), 
                   (pi_x + 0.02, uart1_y - 0.075), (pi_x + 0.02, pi_y)], color=COL_WIRE_SIG, lw=1.2, style=":")
    draw_resistor(ax, bms1_x + bms1_w / 2, uart1_y - 0.04, "100Ω")
    ax.text(bms1_x + bms1_w / 2 + 0.02, uart1_y - 0.04, "GF Protect", ha="left", va="center", fontsize=4.5, color="#DDCC33", family="monospace")

    t_out_x, t_out_y = tnsy_x + 0.065, tnsy_y + 0.05
    s_in_x, s_in_y = 0.94, servo_box_y
    draw_wire(ax, [(t_out_x, t_out_y), (t_out_x, iso_y), (s_in_x - 0.05, iso_y), (s_in_x - 0.05, s_in_y)], color=COL_WIRE_SIG, lw=1.2, style=":")
    draw_resistor(ax, t_out_x, iso_y - 0.02, "100Ω")
    ax.text(t_out_x + 0.015, iso_y - 0.02, "x6 TX/RX", ha="left", va="center", fontsize=4.5, color="#DDCC33", family="monospace")


    # ================================================================
    #  NOTES
    # ================================================================
    notes_x = 0.03
    notes_y = 0.165
    notes = [
        "EXTERNAL POWER & CHARGING PROTECTIONS:",
        "1. UMBILICAL CABLE: Because the batteries have highly disparate capacities (10Ah vs 3.5Ah), plugging",
        "   a single 12.6V jack directly across both would force violent current from the big pack to the small pack.",
        "   Use a 4-pin connector to bring in independent V+ lines from a dual-channel RC charger.",
        "2. TRANSIENT / ARC PROTECTION: Hot-plugging a 12V 40A supply into motor controllers can create a massive",
        "   LC ringing spike (up to 25V). A 3300µF Low-ESR capacitor is placed directly on the main Servo Bus line",
        "   to absorb the inrush spark and act as a buffer for the servos' regenerative braking when walking.",
        "",
        "OTHER SAFETY SYSTEMS:",
        "• STAR GROUND HUB: Ensures safe common signal reference without ground-loop fire hazards.",
        "• UART RESISTORS: ~100Ω current limiting protects Pi/Teensy ICs if the main servo ground wire detaches.",
        "• DUAL SWITCHES: Allows you to boot Pi, enable charging, or run code without servos powered."
    ]
    for i, note in enumerate(notes):
        weight = "bold" if i <= 6 else "normal"
        color_note = "#FFAA66" if "PROTECTION" in note or i < 7 else COL_NOTE
        ax.text(notes_x, notes_y - i * 0.0125, note,
                ha="left", va="center", fontsize=4.8,
                fontweight=weight, color=color_note, family="monospace")

    # ── Revision block ──────────────────────────────────────────────
    ax.text(1.57, 0.015,
            "MARS Hexapod v2.0\nPower Wiring Rev 4.0\n(External Charge + Antispark Capacitor)\n2026-03-04",
            ha="right", va="bottom", fontsize=5, color=COL_NOTE,
            family="monospace")

    # ── Legend ───────────────────────────────────────────────────────
    leg_x = 1.35
    leg_y = 0.90
    ax.text(leg_x, leg_y, "LEGEND", fontsize=6, fontweight="bold",
            color=COL_TEXT, family="monospace")
    legend_items = [
        (COL_WIRE, "─── Power (+)"),
        (COL_WIRE_GND, "─── Ground (–)"),
        (COL_WIRE_5V, "─── 5V Rail"),
        (COL_WIRE_SIG, "····· Data / UART"),
        ("#FFAA88", "--- Ext. Charge"),
        ("#DDCC33", "▀[100Ω] Inline Res")
    ]
    for i, (col, label) in enumerate(legend_items):
        ly = leg_y - 0.02 - i * 0.018
        if "Res" in label:
            rect = Rectangle((leg_x, ly - 0.005), 0.025, 0.01, facecolor=col, edgecolor="#FFF", lw=0.5)
            ax.add_patch(rect)
            ax.text(leg_x + 0.035, ly, label[2:], va="center", fontsize=5, color="#EEEECC", family="monospace")
        else:
            style = "--" if "Charge" in label else ":" if "Data" in label else "-"
            ax.plot([leg_x, leg_x + 0.03], [ly, ly], color=col, linewidth=2, linestyle=style)
            ax.text(leg_x + 0.035, ly, label, va="center", fontsize=5, color=col, family="monospace")

    # ── Save ────────────────────────────────────────────────────────
    out_dir = os.path.dirname(os.path.abspath(__file__))
    out_path = os.path.join(out_dir, "MARS_Power_Wiring_Diagram.pdf")
    fig.savefig(out_path, format="pdf", dpi=300, bbox_inches="tight",
                facecolor=fig.get_facecolor(), edgecolor="none")
    plt.close(fig)
    print(f"✅  Saved: {out_path}")
    return out_path


if __name__ == "__main__":
    generate_diagram()
