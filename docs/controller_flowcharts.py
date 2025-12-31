#!/usr/bin/env python3
"""
Generate PDF flowcharts for MARS controller.py architecture.

Produces three flowcharts:
  1. Startup Sequence
  2. Main Loop Phases
  3. Shutdown Sequence

Usage:
    python3 controller_flowcharts.py

Output:
    controller_flowcharts.pdf (multi-page PDF)
"""

import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.backends.backend_pdf import PdfPages
import numpy as np


def draw_flowchart_box(ax, x, y, text, width=2.2, height=0.6, color='#e0e0e0', 
                       text_color='black', font_size=8, shape='box', edgecolor='#333'):
    """Draw a flowchart box at the specified position."""
    if shape == 'box':
        box = mpatches.FancyBboxPatch((x - width/2, y - height/2), width, height,
                                       boxstyle="round,pad=0.05,rounding_size=0.1",
                                       facecolor=color, edgecolor=edgecolor, linewidth=1.5)
    elif shape == 'diamond':
        # Diamond (decision)
        verts = [(x, y + height/2), (x + width/3, y), (x, y - height/2), (x - width/3, y)]
        box = mpatches.Polygon(verts, closed=True, facecolor=color, edgecolor=edgecolor, linewidth=1.5)
    elif shape == 'parallelogram':
        # Parallelogram (I/O)
        skew = 0.15
        verts = [(x - width/2 + skew, y + height/2), (x + width/2 + skew, y + height/2),
                 (x + width/2 - skew, y - height/2), (x - width/2 - skew, y - height/2)]
        box = mpatches.Polygon(verts, closed=True, facecolor=color, edgecolor=edgecolor, linewidth=1.5)
    elif shape == 'terminal':
        # Rounded terminal (start/end)
        box = mpatches.FancyBboxPatch((x - width/2, y - height/2), width, height,
                                       boxstyle="round,pad=0.05,rounding_size=0.3",
                                       facecolor=color, edgecolor=edgecolor, linewidth=2)
    else:
        box = mpatches.FancyBboxPatch((x - width/2, y - height/2), width, height,
                                       boxstyle="round,pad=0.05,rounding_size=0.1",
                                       facecolor=color, edgecolor=edgecolor, linewidth=1.5)
    ax.add_patch(box)
    ax.text(x, y, text, ha='center', va='center', fontsize=font_size, 
            color=text_color, fontweight='normal', wrap=True)


def draw_arrow(ax, x1, y1, x2, y2, color='#333'):
    """Draw an arrow between two points."""
    ax.annotate('', xy=(x2, y2), xytext=(x1, y1),
                arrowprops=dict(arrowstyle='->', color=color, lw=1.5))


def draw_startup_flowchart(ax):
    """Draw the startup sequence flowchart."""
    ax.set_xlim(-3, 3)
    ax.set_ylim(-12, 1.5)
    ax.set_aspect('equal')
    ax.axis('off')
    ax.set_title('MARS Controller Startup Sequence', fontsize=14, fontweight='bold', pad=20)
    
    y = 1
    spacing = 0.85
    
    # Start
    draw_flowchart_box(ax, 0, y, 'START', shape='terminal', color='#90EE90', width=1.8, height=0.5)
    y -= spacing
    
    # Display init
    draw_arrow(ax, 0, 1 - 0.25, 0, y + 0.3)
    draw_flowchart_box(ax, 0, y, 'Initialize Display\n(st7789)', color='#87CEEB')
    y -= spacing
    
    # Startup splash
    draw_arrow(ax, 0, y + spacing - 0.3, 0, y + 0.3)
    draw_flowchart_box(ax, 0, y, 'Create StartupSplash\n(MARS banner)', color='#87CEEB')
    y -= spacing
    
    # Touch screen
    draw_arrow(ax, 0, y + spacing - 0.3, 0, y + 0.3)
    draw_flowchart_box(ax, 0, y, 'Initialize Touch Screen\n(cst816d)', color='#87CEEB')
    y -= spacing
    
    # Menu systems
    draw_arrow(ax, 0, y + spacing - 0.3, 0, y + 0.3)
    draw_flowchart_box(ax, 0, y, 'Create Menu Systems\n(GUIMenu + MarsMenu)', color='#DDA0DD')
    y -= spacing
    
    # Eye display
    draw_arrow(ax, 0, y + spacing - 0.3, 0, y + 0.3)
    draw_flowchart_box(ax, 0, y, 'Initialize Eye Display\n(SimpleEyes)', color='#DDA0DD')
    y -= spacing
    
    # Display thread
    draw_arrow(ax, 0, y + spacing - 0.3, 0, y + 0.3)
    draw_flowchart_box(ax, 0, y, 'Start Display Thread\n(async render)', color='#FFB347')
    y -= spacing
    
    # IMU thread
    draw_arrow(ax, 0, y + spacing - 0.3, 0, y + 0.3)
    draw_flowchart_box(ax, 0, y, 'Start IMU Thread\n(BNO085 @ 100Hz)', color='#FFB347')
    y -= spacing
    
    # ToF thread
    draw_arrow(ax, 0, y + spacing - 0.3, 0, y + 0.3)
    draw_flowchart_box(ax, 0, y, 'Start ToF Thread\n(VL53L5CX @ 15Hz)', color='#FFB347')
    y -= spacing
    
    # PointCloud server
    draw_arrow(ax, 0, y + spacing - 0.3, 0, y + 0.3)
    draw_flowchart_box(ax, 0, y, 'Start PointCloud Server\n(WebSocket + HTTP)', color='#FFB347')
    y -= spacing
    
    # Teensy connection
    draw_arrow(ax, 0, y + spacing - 0.3, 0, y + 0.3)
    draw_flowchart_box(ax, 0, y, 'Connect to Teensy 4.1\n(serial @ 1Mbaud)', color='#98FB98')
    y -= spacing
    
    # Controller connection
    draw_arrow(ax, 0, y + spacing - 0.3, 0, y + 0.3)
    draw_flowchart_box(ax, 0, y, 'Connect Controller\n(joy daemon or evdev)', color='#98FB98')
    y -= spacing
    
    # Start telemetry
    draw_arrow(ax, 0, y + spacing - 0.3, 0, y + 0.3)
    draw_flowchart_box(ax, 0, y, 'Start Telemetry\n(Y 1 command)', color='#98FB98')
    y -= spacing
    
    # End startup
    draw_arrow(ax, 0, y + spacing - 0.3, 0, y + 0.3)
    draw_flowchart_box(ax, 0, y, 'Enter Main Loop', shape='terminal', color='#90EE90', width=1.8, height=0.5)


def draw_main_loop_flowchart(ax):
    """Draw the main loop three-layer architecture flowchart."""
    ax.set_xlim(-5, 5)
    ax.set_ylim(-14.5, 2)
    ax.set_aspect('equal')
    ax.axis('off')
    ax.set_title('MARS Controller — Three-Layer Architecture (166 Hz base)', 
                 fontsize=14, fontweight='bold', pad=20)
    
    # Colors by layer
    c_layer1 = '#98FB98'    # Layer 1 - Controller (green - reflexive/fast)
    c_layer2 = '#FFD700'    # Layer 2 - Sequencer (gold - behaviors)
    c_layer3 = '#DDA0DD'    # Layer 3 - Deliberator (purple - planning)
    c_sync = '#B0E0E6'      # Sync operations
    c_control = '#FFB347'   # Control flow
    
    y = 1.5
    spacing = 0.72
    
    # Start of loop
    draw_flowchart_box(ax, 0, y, 'while _run:', shape='terminal', color='#90EE90', width=1.8, height=0.5)
    y -= spacing
    
    # Sync globals
    draw_arrow(ax, 0, y + spacing - 0.25, 0, y + 0.3)
    draw_flowchart_box(ax, 0, y, 'sync_globals_to_ctrl()', color=c_sync, width=2.4)
    y -= spacing
    
    # Increment tick counters
    draw_arrow(ax, 0, y + spacing - 0.3, 0, y + 0.3)
    draw_flowchart_box(ax, 0, y, 'Increment tick counters\n(_layer2TickCount++)', color=c_control, width=2.4)
    y -= spacing * 0.7
    
    # === LAYER 1 BOX (always runs) ===
    layer1_top = y - 0.1
    draw_arrow(ax, 0, y + spacing * 0.4, 0, y + 0.1)
    
    # Layer 1 header
    y -= spacing * 0.5
    ax.text(-3.8, y + 0.15, 'LAYER 1', fontsize=9, fontweight='bold', color='#2E7D32')
    ax.text(-3.8, y - 0.15, 'Controller', fontsize=7, color='#2E7D32')
    ax.text(-3.8, y - 0.4, '166 Hz', fontsize=7, fontstyle='italic', color='#666')
    
    # Layer 1 contents (stacked vertically, compact)
    draw_flowchart_box(ax, 0, y, 'phase_timing_update()', color=c_layer1, width=2.4, height=0.45)
    y -= spacing * 0.65
    draw_flowchart_box(ax, 0, y, 'phase_teensy_connection()', color=c_layer1, width=2.4, height=0.45)
    y -= spacing * 0.65
    draw_flowchart_box(ax, 0, y, 'ctrl.housekeeping()', color=c_layer1, width=2.4, height=0.45)
    y -= spacing * 0.65
    draw_flowchart_box(ax, 0, y, 'phase_gait_tick()', color=c_layer1, width=2.4, height=0.45)
    y -= spacing * 0.65
    draw_flowchart_box(ax, 0, y, 'phase_auto_disable()', color=c_layer1, width=2.4, height=0.45)
    y -= spacing * 0.65
    draw_flowchart_box(ax, 0, y, 'phase_pointcloud()', color=c_layer1, width=2.4, height=0.45)
    layer1_bottom = y - 0.3
    
    # Draw Layer 1 bracket
    ax.plot([-3.2, -3.2], [layer1_top, layer1_bottom], color='#2E7D32', lw=2)
    ax.plot([-3.2, -3.0], [layer1_top, layer1_top], color='#2E7D32', lw=2)
    ax.plot([-3.2, -3.0], [layer1_bottom, layer1_bottom], color='#2E7D32', lw=2)
    
    y -= spacing
    
    # === LAYER 2 DECISION ===
    draw_arrow(ax, 0, y + spacing - 0.25, 0, y + 0.35)
    draw_flowchart_box(ax, 0, y, 'tick >= divisor?', shape='diamond', color=c_control, width=2.0, height=0.6)
    
    # "No" path (skip to sleep)
    ax.annotate('', xy=(2.8, y), xytext=(0.7, y),
                arrowprops=dict(arrowstyle='->', color='#333', lw=1.5))
    ax.text(1.7, y + 0.15, 'No', fontsize=7, color='#666')
    
    # "Yes" path (run Layer 2)
    y -= spacing
    draw_arrow(ax, 0, y + spacing - 0.3, 0, y + 0.3)
    ax.text(0.15, y + spacing - 0.15, 'Yes', fontsize=7, color='#666')
    
    # === LAYER 2 BOX ===
    layer2_top = y + 0.25
    
    # Layer 2 header
    ax.text(-3.8, y + 0.15, 'LAYER 2', fontsize=9, fontweight='bold', color='#B8860B')
    ax.text(-3.8, y - 0.15, 'Sequencer', fontsize=7, color='#B8860B')
    ax.text(-3.8, y - 0.4, '~33 Hz', fontsize=7, fontstyle='italic', color='#666')
    
    draw_flowchart_box(ax, 0, y, 'phase_gamepad_connection()', color=c_layer2, width=2.4, height=0.45)
    y -= spacing * 0.65
    draw_flowchart_box(ax, 0, y, 'poll_gamepad() / poll_joy_client()', color=c_layer2, width=2.6, height=0.45)
    y -= spacing * 0.65
    draw_flowchart_box(ax, 0, y, 'phase_touch_input()', color=c_layer2, width=2.4, height=0.45)
    y -= spacing * 0.65
    draw_flowchart_box(ax, 0, y, 'phase_keyboard_input()', color=c_layer2, width=2.4, height=0.45)
    y -= spacing * 0.65
    draw_flowchart_box(ax, 0, y, 'phase_autonomy()', color=c_layer2, width=2.4, height=0.45)
    y -= spacing * 0.65
    draw_flowchart_box(ax, 0, y, 'phase_display_update()', color=c_layer2, width=2.4, height=0.45)
    layer2_bottom = y - 0.3
    
    # Draw Layer 2 bracket
    ax.plot([-3.2, -3.2], [layer2_top, layer2_bottom], color='#B8860B', lw=2)
    ax.plot([-3.2, -3.0], [layer2_top, layer2_top], color='#B8860B', lw=2)
    ax.plot([-3.2, -3.0], [layer2_bottom, layer2_bottom], color='#B8860B', lw=2)
    
    y -= spacing * 0.8
    
    # Layer 2 "No" path joins here
    skip_y = y + 0.2
    ax.plot([2.8, 2.8], [layer2_top + spacing - 0.35, skip_y], color='#333', lw=1.5)
    ax.annotate('', xy=(0.8, skip_y), xytext=(2.8, skip_y),
                arrowprops=dict(arrowstyle='->', color='#333', lw=1.5))
    
    # === LAYER 3 (placeholder) ===
    y -= spacing * 0.4
    draw_flowchart_box(ax, 0, y, 'Layer 3: Deliberator (~1 Hz)\n[Future: SLAM, Planning]', 
                       color=c_layer3, width=2.8, height=0.55, font_size=7)
    ax.text(-3.8, y, 'LAYER 3', fontsize=8, fontweight='bold', color='#8B008B')
    ax.text(-3.8, y - 0.25, '~1 Hz', fontsize=7, fontstyle='italic', color='#666')
    
    y -= spacing
    
    # Sleep and sync
    draw_arrow(ax, 0, y + spacing - 0.25, 0, y + 0.3)
    draw_flowchart_box(ax, 0, y, 'phase_loop_sleep()', color=c_sync, width=2.4)
    y -= spacing
    
    draw_arrow(ax, 0, y + spacing - 0.3, 0, y + 0.3)
    draw_flowchart_box(ax, 0, y, 'sync_ctrl_to_globals()', color=c_sync, width=2.4)
    
    # Loop back arrow
    loop_x = -4.2
    ax.annotate('', xy=(loop_x, 1.5), xytext=(loop_x, y),
                arrowprops=dict(arrowstyle='->', color='#333', lw=1.5))
    ax.plot([loop_x, 0], [y, y], color='#333', lw=1.5)
    ax.plot([loop_x, 0], [1.5, 1.5], color='#333', lw=1.5)
    
    # Legend
    legend_y = -13.8
    ax.text(-4.5, legend_y, 'Layers:', fontsize=8, fontweight='bold')
    draw_flowchart_box(ax, -2.8, legend_y - 0.5, 'L1: Controller\n166 Hz', color=c_layer1, width=1.4, height=0.5, font_size=6)
    draw_flowchart_box(ax, -1.0, legend_y - 0.5, 'L2: Sequencer\n~33 Hz', color=c_layer2, width=1.4, height=0.5, font_size=6)
    draw_flowchart_box(ax, 0.8, legend_y - 0.5, 'L3: Deliberator\n~1 Hz', color=c_layer3, width=1.4, height=0.5, font_size=6)
    draw_flowchart_box(ax, 2.6, legend_y - 0.5, 'Sync/Sleep', color=c_sync, width=1.2, height=0.5, font_size=6)


def draw_shutdown_flowchart(ax):
    """Draw the shutdown sequence flowchart."""
    ax.set_xlim(-3, 3)
    ax.set_ylim(-10, 1.5)
    ax.set_aspect('equal')
    ax.axis('off')
    ax.set_title('MARS Controller Shutdown Sequence', fontsize=14, fontweight='bold', pad=20)
    
    y = 1
    spacing = 0.9
    
    # Start
    draw_flowchart_box(ax, 0, y, 'Exit Main Loop\n(_run = False)', shape='terminal', color='#FFB6C1', width=2, height=0.6)
    y -= spacing
    
    # Stop display thread
    draw_arrow(ax, 0, y + spacing - 0.3, 0, y + 0.3)
    draw_flowchart_box(ax, 0, y, 'Stop Display Thread\n(.stop())', color='#FFD700', width=2.4)
    y -= spacing
    
    # Stop IMU thread
    draw_arrow(ax, 0, y + spacing - 0.3, 0, y + 0.3)
    draw_flowchart_box(ax, 0, y, 'Stop IMU Thread\n(.stop())', color='#FFD700', width=2.4)
    y -= spacing
    
    # Stop ToF thread
    draw_arrow(ax, 0, y + spacing - 0.3, 0, y + 0.3)
    draw_flowchart_box(ax, 0, y, 'Stop ToF Thread\n(.stop())', color='#FFD700', width=2.4)
    y -= spacing
    
    # Stop PointCloud server
    draw_arrow(ax, 0, y + spacing - 0.3, 0, y + 0.3)
    draw_flowchart_box(ax, 0, y, 'Stop PointCloud Server\n(.stop())', color='#FFD700', width=2.4)
    y -= spacing
    
    # Cleanup keyboard
    draw_arrow(ax, 0, y + spacing - 0.3, 0, y + 0.3)
    draw_flowchart_box(ax, 0, y, 'Cleanup Keyboard\n(restore terminal)', color='#87CEEB', width=2.4)
    y -= spacing
    
    # Disable servos
    draw_arrow(ax, 0, y + spacing - 0.3, 0, y + 0.3)
    draw_flowchart_box(ax, 0, y, 'Disable Servos\n(LEG ALL DISABLE)', color='#FF6B6B', width=2.4)
    y -= spacing
    
    # Disable robot
    draw_arrow(ax, 0, y + spacing - 0.3, 0, y + 0.3)
    draw_flowchart_box(ax, 0, y, 'Disable Robot\n(DISABLE)', color='#FF6B6B', width=2.4)
    y -= spacing
    
    # Stop telemetry
    draw_arrow(ax, 0, y + spacing - 0.3, 0, y + 0.3)
    draw_flowchart_box(ax, 0, y, 'Stop Telemetry\n(Y 0)', color='#98FB98', width=2.4)
    y -= spacing
    
    # Clear display
    draw_arrow(ax, 0, y + spacing - 0.3, 0, y + 0.3)
    draw_flowchart_box(ax, 0, y, 'Clear Display\n(bl_DutyCycle(0))', color='#87CEEB', width=2.4)
    y -= spacing
    
    # End
    draw_arrow(ax, 0, y + spacing - 0.3, 0, y + 0.3)
    draw_flowchart_box(ax, 0, y, 'END', shape='terminal', color='#90EE90', width=1.5, height=0.5)


def main():
    """Generate the PDF with all three flowcharts."""
    output_path = 'controller_flowcharts.pdf'
    
    with PdfPages(output_path) as pdf:
        # Page 1: Startup Sequence
        fig1, ax1 = plt.subplots(figsize=(8.5, 11))
        draw_startup_flowchart(ax1)
        plt.tight_layout()
        pdf.savefig(fig1, bbox_inches='tight')
        plt.close(fig1)
        
        # Page 2: Main Loop
        fig2, ax2 = plt.subplots(figsize=(8.5, 11))
        draw_main_loop_flowchart(ax2)
        plt.tight_layout()
        pdf.savefig(fig2, bbox_inches='tight')
        plt.close(fig2)
        
        # Page 3: Shutdown Sequence
        fig3, ax3 = plt.subplots(figsize=(8.5, 11))
        draw_shutdown_flowchart(ax3)
        plt.tight_layout()
        pdf.savefig(fig3, bbox_inches='tight')
        plt.close(fig3)
    
    print(f"Generated: {output_path}")
    print("  - Page 1: Startup Sequence")
    print("  - Page 2: Main Loop Phases (166 Hz)")
    print("  - Page 3: Shutdown Sequence")


if __name__ == '__main__':
    main()
