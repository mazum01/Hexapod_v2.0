"""
menu_controller.py — Extracted menu callback logic from controller.py

Part of M3 Modularization: Menu Logic Extraction
Created: 2026-01-06

This module contains callback setup functions for MarsMenu categories.
Each setup function takes:
  - menu: MarsMenu instance
  - ctx: SimpleNamespace with references to globals from controller.py

The ctx object provides access to mutable state without circular imports.
"""

import time
from types import SimpleNamespace
from MarsMenu import MarsMenu, MenuCategory
from config_manager import (
    save_eye_shape, save_eye_crt_mode, save_eye_vertical_offset,
    save_human_eye_settings, save_pid_settings, save_imp_settings,
    save_est_settings, save_tof_settings,
    save_gait_settings, save_pounce_settings, save_behavior_settings,
    save_safety_display_settings, save_low_battery_settings,
    save_collision_settings,
)


def setup_eyes_callbacks(menu: MarsMenu, ctx: SimpleNamespace) -> None:
    """Setup EYES menu callbacks.
    
    ctx must have:
        - eyes: SimpleEyes instance
        - eye_colors: list of RGB tuples per eye shape
        - display_thread: DisplayThread instance (or None)
        - set_force_display_update: callable to set _forceDisplayUpdate = True
        - set_eye_crt_mode: callable(bool) to set _eyeCrtMode
        - set_eye_vertical_offset: callable(int) to set _eyeVerticalOffset
    """
    
    def on_eye_style_change(val):
        ctx.eyes.left_shape = val
        ctx.eyes.right_shape = val
        ctx.eyes.eye_color = ctx.eye_colors[val] if val < len(ctx.eye_colors) else ctx.eye_colors[0]
        ctx.eyes.update(force_update=True)
        ctx.set_force_display_update()
        save_eye_shape(val)
    
    def on_human_color_change(val):
        ctx.eyes.human_eye_color_idx = val
        ctx.eyes.update(force_update=True)
        ctx.set_force_display_update()
    
    def on_eye_size_change(val):
        ctx.eyes.human_eye_size = val
        ctx.eyes.update(force_update=True)
        ctx.set_force_display_update()
    
    def on_eye_spacing_change(val):
        ctx.eyes.human_eye_spacing_pct = val / 100.0
        ctx.eyes.update(force_update=True)
        ctx.set_force_display_update()
    
    def on_crt_change(val):
        crt_mode = (val == 1)
        ctx.set_eye_crt_mode(crt_mode)
        ctx.eyes.crt_mode = crt_mode
        ctx.eyes.update(force_update=True)
        ctx.set_force_display_update()
        save_eye_crt_mode(crt_mode)
    
    def on_eye_vcenter_change(val):
        ctx.set_eye_vertical_offset(val)
        ctx.eyes.eye_vertical_offset = val
        if ctx.get_display_thread() is not None:
            ctx.get_display_thread().set_base_vertical_offset(val)
        ctx.eyes.update(force_update=True)
        ctx.set_force_display_update()
        save_eye_vertical_offset(val)
    
    # Register callbacks
    menu.set_callback(MenuCategory.EYES, "Style", "on_change", on_eye_style_change)
    menu.set_callback(MenuCategory.EYES, "Human Color", "on_change", on_human_color_change)
    menu.set_callback(MenuCategory.EYES, "Size", "on_change", on_eye_size_change)
    menu.set_callback(MenuCategory.EYES, "V Center", "on_change", on_eye_vcenter_change)
    menu.set_callback(MenuCategory.EYES, "Spacing", "on_change", on_eye_spacing_change)
    menu.set_callback(MenuCategory.EYES, "CRT Effect", "on_change", on_crt_change)


def setup_system_callbacks(menu: MarsMenu, ctx: SimpleNamespace) -> None:
    """Setup SYSTEM menu callbacks.
    
    ctx must have:
        - disp: Display instance
        - display_thread: DisplayThread instance (or None)
        - get_verbose: callable returning current _verbose
        - set_verbose: callable(bool)
        - set_mirror: callable(bool)
        - set_brightness: callable(int)
        - set_auto_disable_s: callable(float)
        - set_force_display_update: callable
        - set_run: callable(bool) for shutdown
        - save_eye_shape_fn: callable(int)
    """
    from config_manager import save_menu_settings, save_eye_shape
    
    def on_brightness_change(val):
        ctx.set_brightness(val)
        ctx.disp.bl_DutyCycle(val)
    
    def on_verbose_change(val):
        ctx.set_verbose(val == 1)
    
    def on_mirror_change(val):
        ctx.set_mirror(val == 1)
        ctx.set_force_display_update()
    
    def on_collision_overlay_change(val):
        enabled = (val == 1)
        dt = ctx.get_display_thread()
        if dt is not None:
            dt.set_collision_overlay(enabled)
        if ctx.get_verbose():
            print(f"Collision overlay -> {'On' if enabled else 'Off'}", end="\r\n")
    
    def on_theme_change(val):
        ctx.menu.theme = val
        save_menu_settings(theme=val)
    
    def on_palette_change(val):
        ctx.menu.lcars_palette = val
        if ctx.get_display_thread() is not None:
            ctx.get_display_thread().set_lcars_palette(val)
        save_menu_settings(palette=val)
    
    def on_save_all():
        save_eye_shape(ctx.eyes.left_shape)
        if ctx.get_verbose():
            print("Settings saved", end="\r\n")
    
    def on_shutdown():
        ctx.set_run(False)
        if ctx.get_verbose():
            print("Shutdown requested via menu", end="\r\n")
    
    def on_auto_disable_change(val):
        ctx.set_auto_disable_s(float(val))
        if ctx.get_verbose():
            print(f"Auto-disable timeout set to {val}s", end="\r\n")
    
    # Register callbacks
    menu.set_callback(MenuCategory.SYSTEM, "Theme", "on_change", on_theme_change)
    menu.set_callback(MenuCategory.SYSTEM, "Palette", "on_change", on_palette_change)
    menu.set_callback(MenuCategory.SYSTEM, "Brightness", "on_change", on_brightness_change)
    menu.set_callback(MenuCategory.SYSTEM, "Auto-Disable", "on_change", on_auto_disable_change)
    menu.set_callback(MenuCategory.SYSTEM, "Verbose", "on_change", on_verbose_change)
    menu.set_callback(MenuCategory.SYSTEM, "Mirror Display", "on_change", on_mirror_change)
    menu.set_callback(MenuCategory.SYSTEM, "Col Overlay", "on_change", on_collision_overlay_change)
    menu.set_callback(MenuCategory.SYSTEM, "Save All", "on_select", on_save_all)
    menu.set_callback(MenuCategory.SYSTEM, "Shutdown", "on_select", on_shutdown)


def sync_system_initial_values(menu: MarsMenu, ctx: SimpleNamespace) -> None:
    """Sync SYSTEM menu items with current state.
    
    ctx must have:
        - menu_theme: int
        - menu_palette: int
        - display_brightness: int
        - auto_disable_s: float
        - get_verbose: callable
        - get_mirror: callable
    """
    menu.set_value(MenuCategory.SYSTEM, "Theme", ctx.menu_theme)
    menu.set_value(MenuCategory.SYSTEM, "Palette", ctx.menu_palette)
    menu.set_value(MenuCategory.SYSTEM, "Brightness", ctx.display_brightness)
    menu.set_value(MenuCategory.SYSTEM, "Auto-Disable", int(ctx.auto_disable_s))
    menu.set_value(MenuCategory.SYSTEM, "Verbose", 1 if ctx.get_verbose() else 0)
    menu.set_value(MenuCategory.SYSTEM, "Mirror Display", 1 if ctx.get_mirror() else 0)


def sync_eyes_initial_values(menu: MarsMenu, ctx: SimpleNamespace) -> None:
    """Sync EYES menu items with current state.
    
    ctx must have:
        - eyes: SimpleEyes instance
        - eye_vertical_offset: int
    """
    menu.set_value(MenuCategory.EYES, "Style", ctx.eyes.left_shape)
    menu.set_value(MenuCategory.EYES, "Human Color", ctx.eyes.human_eye_color_idx)
    menu.set_value(MenuCategory.EYES, "Size", ctx.eyes.human_eye_size)
    menu.set_value(MenuCategory.EYES, "V Center", ctx.eye_vertical_offset)
    menu.set_value(MenuCategory.EYES, "Spacing", int(ctx.eyes.human_eye_spacing_pct * 100))
    menu.set_value(MenuCategory.EYES, "CRT Effect", 1 if ctx.eyes.crt_mode else 0)


# ===========================================================================
# M3.3: PID / IMP / EST callbacks
# ===========================================================================

def setup_pid_imp_est_callbacks(menu: MarsMenu, ctx: SimpleNamespace) -> None:
    """Setup PID, IMP, and EST menu callbacks.
    
    ctx must have:
        - pid_state: dict (mutable, shared with controller)
        - imp_state: dict (mutable, shared with controller)
        - est_state: dict (mutable, shared with controller)
        - send_cmd: callable(cmd, force=True)
        - get_verbose: callable returning bool
        - kick_list_poll: callable(which: str) to schedule re-poll
    """
    
    # === PID callbacks ===
    def on_pid_enabled_change(val):
        try:
            ctx.pid_state['enabled'] = (val == 1)
            save_pid_settings(ctx.pid_state)
            ctx.get_send_cmd()(b'PID ENABLE' if val == 1 else b'PID DISABLE', force=True)
            ctx.kick_list_poll('PID')
        except Exception:
            pass

    def on_pid_mode_change(val):
        try:
            ctx.pid_state['mode'] = 'active' if val == 0 else 'shadow'
            save_pid_settings(ctx.pid_state)
            ctx.get_send_cmd()(b'PID MODE ACTIVE' if val == 0 else b'PID MODE SHADOW', force=True)
            ctx.kick_list_poll('PID')
        except Exception:
            pass

    def on_pid_shadow_hz_change(val):
        try:
            ctx.pid_state['shadow_hz'] = int(val)
            save_pid_settings(ctx.pid_state)
            ctx.get_send_cmd()(f"PID SHADOW_RATE {int(val)}".encode('ascii'), force=True)
            ctx.kick_list_poll('PID')
        except Exception:
            pass

    def _make_pid_gain_cb(kind: str, joint: str):
        def _cb(val):
            try:
                key = kind.lower()
                arr = ctx.pid_state.get(key)
                if arr is None or len(arr) != 3:
                    arr = [0, 0, 0]
                idx = 0 if joint == 'COXA' else (1 if joint == 'FEMUR' else 2)
                arr[idx] = int(val)
                ctx.pid_state[key] = arr
                save_pid_settings(ctx.pid_state)
                ctx.get_send_cmd()(f"PID {kind} {joint} {int(val)}".encode('ascii'), force=True)
                ctx.kick_list_poll('PID')
            except Exception:
                pass
        return _cb

    def _make_pid_kdalpha_cb(joint: str):
        def _cb(val):
            try:
                arr = ctx.pid_state.get('kdalph')
                if arr is None or len(arr) != 3:
                    arr = [200, 200, 200]
                idx = 0 if joint == 'COXA' else (1 if joint == 'FEMUR' else 2)
                arr[idx] = int(val)
                ctx.pid_state['kdalph'] = arr
                save_pid_settings(ctx.pid_state)
                ctx.get_send_cmd()(f"PID KDALPHA {joint} {int(val)}".encode('ascii'), force=True)
                ctx.kick_list_poll('PID')
            except Exception:
                pass
        return _cb

    # === IMP callbacks ===
    def on_imp_enabled_change(val):
        try:
            ctx.imp_state['enabled'] = (val == 1)
            save_imp_settings(ctx.imp_state)
            ctx.get_send_cmd()(b'IMP ENABLE' if val == 1 else b'IMP DISABLE', force=True)
            ctx.kick_list_poll('IMP')
        except Exception:
            pass

    def on_imp_mode_change(val):
        try:
            ctx.imp_state['mode'] = 'joint' if val == 1 else ('cart' if val == 2 else 'off')
            save_imp_settings(ctx.imp_state)
            if val == 1:
                cmd = b'IMP MODE JOINT'
            elif val == 2:
                cmd = b'IMP MODE CART'
            else:
                cmd = b'IMP MODE OFF'
            ctx.get_send_cmd()(cmd, force=True)
            ctx.kick_list_poll('IMP')
        except Exception:
            pass

    def on_imp_scale_change(val):
        try:
            ctx.imp_state['scale'] = int(val)
            save_imp_settings(ctx.imp_state)
            ctx.get_send_cmd()(f"IMP SCALE {int(val)}".encode('ascii'), force=True)
            ctx.kick_list_poll('IMP')
        except Exception:
            pass

    def _make_imp_joint_gain_cb(kind: str, joint: str):
        def _cb(val):
            try:
                key = kind.lower()
                arr = ctx.imp_state.get(key)
                if arr is None or len(arr) != 3:
                    arr = [0, 0, 0]
                idx = 0 if joint == 'COXA' else (1 if joint == 'FEMUR' else 2)
                arr[idx] = int(val)
                ctx.imp_state[key] = arr
                save_imp_settings(ctx.imp_state)
                ctx.get_send_cmd()(f"IMP {kind} {joint} {int(val)}".encode('ascii'), force=True)
                ctx.kick_list_poll('IMP')
            except Exception:
                pass
        return _cb

    def _make_imp_cart_gain_cb(kind: str, axis: str):
        def _cb(val):
            try:
                key = kind.lower()
                arr = ctx.imp_state.get(key)
                if arr is None or len(arr) != 3:
                    arr = [0, 0, 0]
                idx = 0 if axis == 'X' else (1 if axis == 'Y' else 2)
                arr[idx] = int(val)
                ctx.imp_state[key] = arr
                save_imp_settings(ctx.imp_state)
                ctx.get_send_cmd()(f"IMP {kind} {axis} {int(val)}".encode('ascii'), force=True)
                ctx.kick_list_poll('IMP')
            except Exception:
                pass
        return _cb

    def on_imp_jdb_change(val):
        try:
            ctx.imp_state['jdb_cd'] = int(val)
            save_imp_settings(ctx.imp_state)
            ctx.get_send_cmd()(f"IMP JDB {int(val)}".encode('ascii'), force=True)
            ctx.kick_list_poll('IMP')
        except Exception:
            pass

    def on_imp_cdb_change(val):
        try:
            ctx.imp_state['cdb_mm'] = float(val)
            save_imp_settings(ctx.imp_state)
            ctx.get_send_cmd()(f"IMP CDB {float(val):.3f}".encode('ascii'), force=True)
            ctx.kick_list_poll('IMP')
        except Exception:
            pass

    # === EST callbacks ===
    def on_est_cmd_alpha_change(val):
        try:
            ctx.est_state['cmd_alpha_milli'] = int(val)
            save_est_settings(ctx.est_state)
            ctx.get_send_cmd()(f"EST CMD_ALPHA {int(val)}".encode('ascii'), force=True)
            ctx.kick_list_poll('EST')
        except Exception:
            pass

    def on_est_meas_alpha_change(val):
        try:
            ctx.est_state['meas_alpha_milli'] = int(val)
            save_est_settings(ctx.est_state)
            ctx.get_send_cmd()(f"EST MEAS_ALPHA {int(val)}".encode('ascii'), force=True)
            ctx.kick_list_poll('EST')
        except Exception:
            pass

    def on_est_vel_alpha_change(val):
        try:
            ctx.est_state['meas_vel_alpha_milli'] = int(val)
            save_est_settings(ctx.est_state)
            ctx.get_send_cmd()(f"EST MEAS_VEL_ALPHA {int(val)}".encode('ascii'), force=True)
            ctx.kick_list_poll('EST')
        except Exception:
            pass

    # Register PID callbacks
    menu.set_callback(MenuCategory.PID, "Enabled", "on_change", on_pid_enabled_change)
    menu.set_callback(MenuCategory.PID, "Mode", "on_change", on_pid_mode_change)
    menu.set_callback(MenuCategory.PID, "Shadow Hz", "on_change", on_pid_shadow_hz_change)
    menu.set_callback(MenuCategory.PID, "Kp Coxa", "on_change", _make_pid_gain_cb('KP', 'COXA'))
    menu.set_callback(MenuCategory.PID, "Kp Femur", "on_change", _make_pid_gain_cb('KP', 'FEMUR'))
    menu.set_callback(MenuCategory.PID, "Kp Tibia", "on_change", _make_pid_gain_cb('KP', 'TIBIA'))
    menu.set_callback(MenuCategory.PID, "Ki Coxa", "on_change", _make_pid_gain_cb('KI', 'COXA'))
    menu.set_callback(MenuCategory.PID, "Ki Femur", "on_change", _make_pid_gain_cb('KI', 'FEMUR'))
    menu.set_callback(MenuCategory.PID, "Ki Tibia", "on_change", _make_pid_gain_cb('KI', 'TIBIA'))
    menu.set_callback(MenuCategory.PID, "Kd Coxa", "on_change", _make_pid_gain_cb('KD', 'COXA'))
    menu.set_callback(MenuCategory.PID, "Kd Femur", "on_change", _make_pid_gain_cb('KD', 'FEMUR'))
    menu.set_callback(MenuCategory.PID, "Kd Tibia", "on_change", _make_pid_gain_cb('KD', 'TIBIA'))
    menu.set_callback(MenuCategory.PID, "Kdα Coxa", "on_change", _make_pid_kdalpha_cb('COXA'))
    menu.set_callback(MenuCategory.PID, "Kdα Femur", "on_change", _make_pid_kdalpha_cb('FEMUR'))
    menu.set_callback(MenuCategory.PID, "Kdα Tibia", "on_change", _make_pid_kdalpha_cb('TIBIA'))

    # Register IMP callbacks
    menu.set_callback(MenuCategory.IMP, "Enabled", "on_change", on_imp_enabled_change)
    menu.set_callback(MenuCategory.IMP, "Mode", "on_change", on_imp_mode_change)
    menu.set_callback(MenuCategory.IMP, "Scale", "on_change", on_imp_scale_change)
    menu.set_callback(MenuCategory.IMP, "J Spring Coxa", "on_change", _make_imp_joint_gain_cb('JSPRING', 'COXA'))
    menu.set_callback(MenuCategory.IMP, "J Spring Femur", "on_change", _make_imp_joint_gain_cb('JSPRING', 'FEMUR'))
    menu.set_callback(MenuCategory.IMP, "J Spring Tibia", "on_change", _make_imp_joint_gain_cb('JSPRING', 'TIBIA'))
    menu.set_callback(MenuCategory.IMP, "J Damp Coxa", "on_change", _make_imp_joint_gain_cb('JDAMP', 'COXA'))
    menu.set_callback(MenuCategory.IMP, "J Damp Femur", "on_change", _make_imp_joint_gain_cb('JDAMP', 'FEMUR'))
    menu.set_callback(MenuCategory.IMP, "J Damp Tibia", "on_change", _make_imp_joint_gain_cb('JDAMP', 'TIBIA'))
    menu.set_callback(MenuCategory.IMP, "C Spring X", "on_change", _make_imp_cart_gain_cb('CSPRING', 'X'))
    menu.set_callback(MenuCategory.IMP, "C Spring Y", "on_change", _make_imp_cart_gain_cb('CSPRING', 'Y'))
    menu.set_callback(MenuCategory.IMP, "C Spring Z", "on_change", _make_imp_cart_gain_cb('CSPRING', 'Z'))
    menu.set_callback(MenuCategory.IMP, "C Damp X", "on_change", _make_imp_cart_gain_cb('CDAMP', 'X'))
    menu.set_callback(MenuCategory.IMP, "C Damp Y", "on_change", _make_imp_cart_gain_cb('CDAMP', 'Y'))
    menu.set_callback(MenuCategory.IMP, "C Damp Z", "on_change", _make_imp_cart_gain_cb('CDAMP', 'Z'))
    menu.set_callback(MenuCategory.IMP, "J Deadband", "on_change", on_imp_jdb_change)
    menu.set_callback(MenuCategory.IMP, "C Deadband", "on_change", on_imp_cdb_change)

    # Register EST callbacks
    menu.set_callback(MenuCategory.EST, "Cmd α", "on_change", on_est_cmd_alpha_change)
    menu.set_callback(MenuCategory.EST, "Meas α", "on_change", on_est_meas_alpha_change)
    menu.set_callback(MenuCategory.EST, "Vel α", "on_change", on_est_vel_alpha_change)


# ===========================================================================
# M3.4: IMU / Leveling callbacks
# ===========================================================================

def setup_imu_callbacks(menu: MarsMenu, ctx: SimpleNamespace) -> None:
    """Setup IMU and Leveling menu callbacks.
    
    ctx must have:
        - display_thread: DisplayThread instance (or None)
        - leveling_state: LevelingState instance (or None)
        - get_verbose: callable returning bool
        - set_leveling_enabled: callable(bool)
        - set_leveling_gain: callable(float)
        - set_leveling_max_corr_mm: callable(float)
        - set_leveling_tilt_limit_deg: callable(float)
        - set_lean_enabled: callable(bool)
        - set_lean_max_deg: callable(float)
    """

    def on_imu_overlay_change(val):
        show_overlay = (val == 1)
        if ctx.get_display_thread() is not None:
            ctx.get_display_thread().update_imu(show_overlay=show_overlay)

    def on_leveling_enabled_change(val):
        enabled = (val == 1)
        ctx.set_leveling_enabled(enabled)
        if ctx.get_leveling_state() is not None:
            ctx.get_leveling_state().config.enabled = enabled
            if not enabled:
                ctx.get_leveling_state().reset()
        if ctx.get_verbose():
            print(f"Body leveling: {'ON' if enabled else 'OFF'}", end="\r\n")

    def on_leveling_gain_change(val):
        ctx.set_leveling_gain(val)
        if ctx.get_leveling_state() is not None:
            ctx.get_leveling_state().config.gain = val
        if ctx.get_verbose():
            print(f"Leveling gain: {val:.1f}", end="\r\n")

    def on_leveling_max_corr_change(val):
        ctx.set_leveling_max_corr_mm(val)
        if ctx.get_leveling_state() is not None:
            ctx.get_leveling_state().config.max_correction_mm = val
        if ctx.get_verbose():
            print(f"Leveling max correction: {val:.0f}mm", end="\r\n")

    def on_leveling_tilt_limit_change(val):
        ctx.set_leveling_tilt_limit_deg(val)
        if ctx.get_leveling_state() is not None:
            ctx.get_leveling_state().config.tilt_limit_deg = val
        if ctx.get_verbose():
            print(f"Leveling tilt limit: {val:.0f}°", end="\r\n")

    def on_lean_enabled_change(val):
        enabled = (val == 1)
        ctx.set_lean_enabled(enabled)
        if ctx.get_leveling_state() is not None:
            ctx.get_leveling_state().config.lean_enabled = enabled
            if not enabled:
                ctx.get_leveling_state()._filtered_lean_pitch = 0.0
                ctx.get_leveling_state()._filtered_lean_roll = 0.0
        if ctx.get_verbose():
            print(f"Motion lean: {'ON' if enabled else 'OFF'}", end="\r\n")

    def on_lean_max_change(val):
        ctx.set_lean_max_deg(val)
        if ctx.get_leveling_state() is not None:
            ctx.get_leveling_state().config.lean_max_deg = val
        if ctx.get_verbose():
            print(f"Lean max: {val:.0f}°", end="\r\n")

    # Register callbacks
    menu.set_callback(MenuCategory.IMU, "Show Overlay", "on_change", on_imu_overlay_change)
    menu.set_callback(MenuCategory.IMU, "Leveling", "on_change", on_leveling_enabled_change)
    menu.set_callback(MenuCategory.IMU, "LVL Gain", "on_change", on_leveling_gain_change)
    menu.set_callback(MenuCategory.IMU, "Max Corr", "on_change", on_leveling_max_corr_change)
    menu.set_callback(MenuCategory.IMU, "Tilt Limit", "on_change", on_leveling_tilt_limit_change)
    menu.set_callback(MenuCategory.IMU, "Motion Lean", "on_change", on_lean_enabled_change)
    menu.set_callback(MenuCategory.IMU, "Lean Max", "on_change", on_lean_max_change)


# ===========================================================================
# M3.5: ToF callbacks
# ===========================================================================

def setup_tof_callbacks(menu: MarsMenu, ctx: SimpleNamespace) -> None:
    """Setup ToF menu callbacks.
    
    ctx must have:
        - get_tof_enabled: callable returning bool
        - set_tof_enabled: callable(bool)
        - get_tof_resolution: callable returning int (16 or 64)
        - set_tof_resolution: callable(int)
        - get_tof_hz: callable returning float
        - set_tof_hz: callable(float)
        - get_tof_bus: callable returning int
        - set_tof_bus: callable(int)
        - get_tof_sensors: callable returning list of (name, addr) tuples
        - set_tof_sensors: callable(list)
        - get_tof_thread: callable returning thread or None
        - set_tof_thread: callable(thread or None)
        - get_verbose: callable returning bool
    """

    def on_tof_enabled_change(val):
        ctx.set_tof_enabled(val == 1)
        if ctx.get_verbose():
            print(f"ToF enabled: {'On' if val == 1 else 'Off'} (restart required)", end="\r\n")

    def on_tof_resolution_change(val):
        # val: 0=4x4, 1=8x8
        ctx.set_tof_resolution(64 if val == 1 else 16)
        if ctx.get_verbose():
            grid = 8 if val == 1 else 4
            print(f"ToF resolution: {grid}x{grid} (restart required)", end="\r\n")

    def on_tof_framerate_change(val):
        ctx.set_tof_hz(float(val))
        if ctx.get_verbose():
            print(f"ToF frame rate: {float(val):.0f} Hz (restart required)", end="\r\n")

    def on_tof_bus_change(val):
        ctx.set_tof_bus(int(val))
        if ctx.get_verbose():
            print(f"ToF I2C bus: {int(val)} (restart required)", end="\r\n")

    def on_tof_add_sensor():
        sensors = ctx.get_tof_sensors()
        used_addrs = {addr for _, addr in sensors}
        next_addr = 0x29
        for i in range(16):
            if (0x29 + i) not in used_addrs:
                next_addr = 0x29 + i
                break
        sensor_num = len(sensors) + 1
        sensors.append((f"sensor{sensor_num}", next_addr))
        ctx.set_tof_sensors(sensors)
        if ctx.get_verbose():
            print(f"Added ToF sensor{sensor_num} at 0x{next_addr:02X} (restart required)", end="\r\n")

    def on_tof_remove_sensor1():
        sensors = ctx.get_tof_sensors()
        if sensors:
            removed = sensors.pop(0)
            ctx.set_tof_sensors(sensors)
            if ctx.get_verbose():
                print(f"Removed ToF sensor '{removed[0]}' (restart required)", end="\r\n")

    def on_tof_apply_restart():
        tof_state = {
            'enabled': ctx.get_tof_enabled(),
            'hz': ctx.get_tof_hz(),
            'bus': ctx.get_tof_bus(),
            'resolution': 8 if ctx.get_tof_resolution() >= 64 else 4,
            'sensors': ctx.get_tof_sensors(),
        }
        if save_tof_settings(tof_state):
            if ctx.get_verbose():
                print("ToF settings saved", end="\r\n")
        else:
            if ctx.get_verbose():
                print("ToF settings save failed", end="\r\n")

        # Stop existing thread if running
        tof_thread = ctx.get_tof_thread()
        if tof_thread is not None and tof_thread.is_alive():
            tof_thread.stop()
            if ctx.get_verbose():
                print("ToF thread stopped", end="\r\n")
            ctx.set_tof_thread(None)

        if ctx.get_tof_enabled():
            if ctx.get_verbose():
                print("ToF restart: settings saved. Full restart recommended for sensor re-init.", end="\r\n")
        else:
            if ctx.get_verbose():
                print("ToF disabled and stopped", end="\r\n")

    # Register callbacks
    menu.set_callback(MenuCategory.TOF, "Enabled", "on_change", on_tof_enabled_change)
    menu.set_callback(MenuCategory.TOF, "Resolution", "on_change", on_tof_resolution_change)
    menu.set_callback(MenuCategory.TOF, "Frame Rate", "on_change", on_tof_framerate_change)
    menu.set_callback(MenuCategory.TOF, "I2C Bus", "on_change", on_tof_bus_change)
    menu.set_callback(MenuCategory.TOF, "Add Sensor", "on_select", on_tof_add_sensor)
    menu.set_callback(MenuCategory.TOF, "Remove Sensor 1", "on_select", on_tof_remove_sensor1)
    menu.set_callback(MenuCategory.TOF, "Apply & Restart", "on_select", on_tof_apply_restart)


# ===========================================================================
# M3.6: GAIT callbacks
# ===========================================================================

def setup_gait_callbacks(menu: MarsMenu, ctx: SimpleNamespace) -> None:
    """Setup GAIT menu callbacks.
    
    ctx must have:
        - get_verbose: callable returning bool
        - get_gait_engine: callable returning gait engine or None
        - set_gait_lift_mm: callable(float)
        - get_gait_lift_mm: callable returning float
        - set_gait_cycle_ms: callable(int)
        - get_gait_cycle_ms: callable returning int
        - set_gait_turn_max_deg_s: callable(float)
        - get_gait_turn_max_deg_s: callable returning float
        - get_gait_width_mm: callable returning float
        - start_gait: callable() to start gait
        - stop_gait: callable() to stop gait
    """

    def on_gait_type_change(val):
        if ctx.get_verbose():
            gait_names = ["Tripod", "Wave", "Ripple", "Stationary"]
            print(f"Gait type: {gait_names[val]}", end="\r\n")

    def on_step_height_change(val):
        ctx.set_gait_lift_mm(val)
        engine = ctx.get_gait_engine()
        if engine is not None:
            engine.params.lift_mm = val
        if ctx.get_verbose():
            print(f"Step height: {val}mm", end="\r\n")

    def on_step_length_change(val):
        engine = ctx.get_gait_engine()
        if engine is not None:
            engine.params.step_length_mm = val
        if ctx.get_verbose():
            print(f"Step length: {val}mm", end="\r\n")

    def on_turn_rate_change(val):
        ctx.set_gait_turn_max_deg_s(float(val))
        save_gait_settings(
            ctx.get_gait_width_mm(),
            ctx.get_gait_lift_mm(),
            cycle_ms=ctx.get_gait_cycle_ms(),
            turn_max_deg_s=float(val)
        )
        if ctx.get_verbose():
            print(f"Max turn rate: {float(val):.1f} deg/s", end="\r\n")

    def on_cycle_time_change(val):
        ctx.set_gait_cycle_ms(int(val))
        engine = ctx.get_gait_engine()
        if engine is not None and hasattr(engine, 'params'):
            engine.params.cycle_ms = int(val)
        save_gait_settings(
            ctx.get_gait_width_mm(),
            ctx.get_gait_lift_mm(),
            cycle_ms=int(val)
        )
        if ctx.get_verbose():
            print(f"Cycle time: {int(val)}ms", end="\r\n")

    def on_start_gait():
        ctx.start_gait()
        if ctx.get_verbose():
            print("Gait started via menu", end="\r\n")

    def on_stop_gait():
        ctx.stop_gait()
        if ctx.get_verbose():
            print("Gait stopped via menu", end="\r\n")

    # Register callbacks
    menu.set_callback(MenuCategory.GAIT, "Type", "on_change", on_gait_type_change)
    menu.set_callback(MenuCategory.GAIT, "Step Height", "on_change", on_step_height_change)
    menu.set_callback(MenuCategory.GAIT, "Step Length", "on_change", on_step_length_change)
    menu.set_callback(MenuCategory.GAIT, "Turn Rate", "on_change", on_turn_rate_change)
    menu.set_callback(MenuCategory.GAIT, "Cycle Time", "on_change", on_cycle_time_change)
    menu.set_callback(MenuCategory.GAIT, "Start Gait", "on_select", on_start_gait)
    menu.set_callback(MenuCategory.GAIT, "Stop Gait", "on_select", on_stop_gait)


# ===========================================================================
# M3.7: POSTURE + Pounce callbacks
# ===========================================================================

def setup_posture_callbacks(menu: MarsMenu, ctx: SimpleNamespace) -> None:
    """Setup POSTURE menu callbacks.
    
    ctx must have:
        - on_stand: callable() for stand action
        - on_tuck: callable() for tuck action
        - on_home: callable() for home action
        - on_pounce: callable() for pounce action
        - get/set_pounce_*: getters/setters for pounce timing params
        - persist_pounce: callable() to save pounce settings
        - hide_menu: callable() to hide menu after action
    """

    def on_stand():
        ctx.on_stand()
        ctx.hide_menu()

    def on_tuck():
        ctx.on_tuck()
        ctx.hide_menu()

    def on_home():
        ctx.on_home()
        ctx.hide_menu()

    def on_pounce():
        ctx.on_pounce()

    # Pounce timing callbacks
    def on_p_prep(val):
        ctx.set_pounce_prep_ms(int(val))
        ctx.persist_pounce()

    def on_p_rear(val):
        ctx.set_pounce_rear_ms(int(val))
        ctx.persist_pounce()

    def on_p_lunge(val):
        ctx.set_pounce_lunge_ms(int(val))
        ctx.persist_pounce()

    def on_p_recov(val):
        ctx.set_pounce_recover_ms(int(val))
        ctx.persist_pounce()

    def on_p_back1(val):
        ctx.set_pounce_back1_z(float(val))
        ctx.persist_pounce()

    def on_p_back2(val):
        ctx.set_pounce_back2_z(float(val))
        ctx.persist_pounce()

    def on_p_push(val):
        ctx.set_pounce_push_z(float(val))
        ctx.persist_pounce()

    def on_p_strike(val):
        ctx.set_pounce_strike_z(float(val))
        ctx.persist_pounce()

    def on_p_crouch(val):
        ctx.set_pounce_crouch_dy(float(val))
        ctx.persist_pounce()

    def on_p_lift(val):
        ctx.set_pounce_lift_dy(float(val))
        ctx.persist_pounce()

    def on_p_frontz(val):
        ctx.set_pounce_front_z(float(val))
        ctx.persist_pounce()

    # Register callbacks
    menu.set_callback(MenuCategory.POSTURE, "Stand", "on_select", on_stand)
    menu.set_callback(MenuCategory.POSTURE, "Tuck", "on_select", on_tuck)
    menu.set_callback(MenuCategory.POSTURE, "Home", "on_select", on_home)
    menu.set_callback(MenuCategory.POSTURE, "Pounce", "on_select", on_pounce)
    menu.set_callback(MenuCategory.POSTURE, "P Prep", "on_change", on_p_prep)
    menu.set_callback(MenuCategory.POSTURE, "P Rear", "on_change", on_p_rear)
    menu.set_callback(MenuCategory.POSTURE, "P Lunge", "on_change", on_p_lunge)
    menu.set_callback(MenuCategory.POSTURE, "P Recov", "on_change", on_p_recov)
    menu.set_callback(MenuCategory.POSTURE, "P Back1", "on_change", on_p_back1)
    menu.set_callback(MenuCategory.POSTURE, "P Back2", "on_change", on_p_back2)
    menu.set_callback(MenuCategory.POSTURE, "P Push", "on_change", on_p_push)
    menu.set_callback(MenuCategory.POSTURE, "P Strike", "on_change", on_p_strike)
    menu.set_callback(MenuCategory.POSTURE, "P Crouch", "on_change", on_p_crouch)
    menu.set_callback(MenuCategory.POSTURE, "P Lift", "on_change", on_p_lift)
    menu.set_callback(MenuCategory.POSTURE, "P FrontZ", "on_change", on_p_frontz)


# ===========================================================================
# M3.8: AUTONOMY callbacks
# ===========================================================================

def setup_autonomy_callbacks(menu: MarsMenu, ctx: SimpleNamespace) -> None:
    """Setup AUTONOMY menu callbacks.
    
    ctx must have:
        - get_verbose: callable returning bool
        - display_thread: DisplayThread or None
        - get_behavior_arbiter: callable returning arbiter or None
        - set_behavior_arbiter: callable(arbiter)
        - init_behavior_arbiter: callable() to initialize arbiter
        - get/set_autonomy_enabled: getters/setters for autonomy state
        - get/set for obstacle avoidance, cliff, caught foot, patrol, wall follow, etc.
    """

    def on_autonomy_enabled_change(val):
        ctx.set_autonomy_enabled(val == 1)
        if val == 1 and ctx.get_behavior_arbiter() is None:
            init_fn = ctx.get_init_behavior_arbiter()
            if init_fn is not None:
                ctx.set_behavior_arbiter(init_fn())
        if ctx.get_verbose():
            print(f"Autonomy: {'ENABLED' if val == 1 else 'DISABLED'}", end="\r\n")

    def on_obstacle_avoid_change(val):
        ctx.set_obstacle_avoidance(val == 1)
        arbiter = ctx.get_behavior_arbiter()
        if arbiter is not None:
            arbiter.set_behavior_enabled("ObstacleAvoidance", val == 1)
        if ctx.get_verbose():
            print(f"Obstacle avoidance: {'On' if val == 1 else 'Off'}", end="\r\n")

    def on_cliff_detect_change(val):
        ctx.set_cliff_detection(val == 1)
        arbiter = ctx.get_behavior_arbiter()
        if arbiter is not None:
            arbiter.set_behavior_enabled("CliffDetection", val == 1)
        if ctx.get_verbose():
            print(f"Cliff detection: {'On' if val == 1 else 'Off'}", end="\r\n")

    def on_caught_foot_change(val):
        ctx.set_caught_foot_recovery(val == 1)
        arbiter = ctx.get_behavior_arbiter()
        if arbiter is not None:
            arbiter.set_behavior_enabled("CaughtFootRecovery", val == 1)
        if ctx.get_verbose():
            print(f"Caught foot recovery: {'On' if val == 1 else 'Off'}", end="\r\n")

    def on_patrol_change(val):
        ctx.set_patrol(val == 1)
        arbiter = ctx.get_behavior_arbiter()
        if arbiter is not None:
            arbiter.set_behavior_enabled("Patrol", val == 1)
        if ctx.get_display_thread() is not None:
            ctx.get_display_thread().show_notification(f"Patrol {'ON' if val == 1 else 'OFF'}", 2.0)
        if ctx.get_verbose():
            print(f"Patrol: {'On' if val == 1 else 'Off'}", end="\r\n")

    def on_wall_follow_change(val):
        ctx.set_wall_follow(val == 1)
        arbiter = ctx.get_behavior_arbiter()
        if arbiter is not None:
            arbiter.set_behavior_enabled("WallFollow", val == 1)
        wall_side = ctx.get_wall_side()
        if ctx.get_display_thread() is not None:
            ctx.get_display_thread().show_notification(f"Wall Follow {'ON' if val == 1 else 'OFF'} ({wall_side.capitalize()})", 2.0)
        if ctx.get_verbose():
            print(f"Wall follow: {'On' if val == 1 else 'Off'} ({wall_side})", end="\r\n")

    def on_wall_side_change(val):
        side = 'left' if val == 0 else 'right'
        ctx.set_wall_side(side)
        arbiter = ctx.get_behavior_arbiter()
        if arbiter is not None:
            behavior = arbiter.get_behavior("WallFollow")
            if behavior is not None:
                behavior.wall_side = side
        if ctx.get_verbose():
            print(f"Wall side: {side}", end="\r\n")

    def on_wall_dist_change(val):
        ctx.set_wall_dist_mm(int(val))
        arbiter = ctx.get_behavior_arbiter()
        if arbiter is not None:
            behavior = arbiter.get_behavior("WallFollow")
            if behavior is not None:
                behavior.wall_distance_mm = int(val)
        if ctx.get_verbose():
            print(f"Wall distance: {int(val)}mm", end="\r\n")

    def on_stop_dist_change(val):
        ctx.set_stop_dist_mm(int(val))
        arbiter = ctx.get_behavior_arbiter()
        if arbiter is not None:
            behavior = arbiter.get_behavior("ObstacleAvoidance")
            if behavior is not None:
                behavior.stop_distance_mm = int(val)
        if ctx.get_verbose():
            print(f"Stop distance: {int(val)}mm", end="\r\n")

    def on_slow_dist_change(val):
        ctx.set_slow_dist_mm(int(val))
        arbiter = ctx.get_behavior_arbiter()
        if arbiter is not None:
            behavior = arbiter.get_behavior("ObstacleAvoidance")
            if behavior is not None:
                behavior.slow_distance_mm = int(val)
        if ctx.get_verbose():
            print(f"Slow distance: {int(val)}mm", end="\r\n")

    def on_cliff_thresh_change(val):
        ctx.set_cliff_threshold_mm(int(val))
        arbiter = ctx.get_behavior_arbiter()
        if arbiter is not None:
            behavior = arbiter.get_behavior("CliffDetection")
            if behavior is not None:
                behavior.cliff_threshold_mm = int(val)
        if ctx.get_verbose():
            print(f"Cliff threshold: {int(val)}mm", end="\r\n")

    def on_snag_error_change(val):
        ctx.set_snag_error_deg(float(val))
        arbiter = ctx.get_behavior_arbiter()
        if arbiter is not None:
            behavior = arbiter.get_behavior("CaughtFootRecovery")
            if behavior is not None:
                behavior.position_error_threshold = float(val)
        if ctx.get_verbose():
            print(f"Snag error threshold: {float(val):.1f}°", end="\r\n")

    def on_snag_timeout_change(val):
        ctx.set_snag_timeout_ms(int(val))
        arbiter = ctx.get_behavior_arbiter()
        if arbiter is not None:
            behavior = arbiter.get_behavior("CaughtFootRecovery")
            if behavior is not None:
                behavior.timeout_ms = int(val)
        if ctx.get_verbose():
            print(f"Snag timeout: {int(val)}ms", end="\r\n")

    def on_recovery_lift_change(val):
        ctx.set_recovery_lift_mm(float(val))
        arbiter = ctx.get_behavior_arbiter()
        if arbiter is not None:
            behavior = arbiter.get_behavior("CaughtFootRecovery")
            if behavior is not None:
                behavior.recovery_lift_mm = float(val)
        if ctx.get_verbose():
            print(f"Recovery lift: {float(val):.0f}mm", end="\r\n")

    def on_patrol_time_change(val):
        ctx.set_patrol_duration_s(float(val))
        arbiter = ctx.get_behavior_arbiter()
        if arbiter is not None:
            behavior = arbiter.get_behavior("Patrol")
            if behavior is not None:
                behavior.patrol_duration_s = float(val)
        if ctx.get_verbose():
            print(f"Patrol duration: {float(val):.0f}s", end="\r\n")

    def on_turn_interval_change(val):
        ctx.set_turn_interval_s(float(val))
        arbiter = ctx.get_behavior_arbiter()
        if arbiter is not None:
            behavior = arbiter.get_behavior("Patrol")
            if behavior is not None:
                behavior.turn_interval_s = float(val)
        if ctx.get_verbose():
            print(f"Turn interval: {float(val):.0f}s", end="\r\n")

    def on_autonomy_save():
        settings = {
            'enabled': ctx.get_autonomy_enabled(),
            'obstacle_avoidance': ctx.get_obstacle_avoidance(),
            'cliff_detection': ctx.get_cliff_detection(),
            'caught_foot_recovery': ctx.get_caught_foot_recovery(),
            'wall_follow': ctx.get_wall_follow(),
            'wall_side': ctx.get_wall_side(),
            'wall_distance_mm': ctx.get_wall_dist_mm(),
            'patrol': ctx.get_patrol(),
            'stop_distance_mm': ctx.get_stop_dist_mm(),
            'slow_distance_mm': ctx.get_slow_dist_mm(),
            'cliff_threshold_mm': ctx.get_cliff_threshold_mm(),
            'snag_position_error_deg': ctx.get_snag_error_deg(),
            'snag_timeout_ms': ctx.get_snag_timeout_ms(),
            'recovery_lift_mm': ctx.get_recovery_lift_mm(),
            'patrol_duration_s': ctx.get_patrol_duration_s(),
            'turn_interval_s': ctx.get_turn_interval_s(),
        }
        if save_behavior_settings(settings):
            if ctx.get_verbose():
                print("Autonomy settings saved", end="\r\n")
        else:
            if ctx.get_verbose():
                print("Failed to save autonomy settings", end="\r\n")

    # Register callbacks
    menu.set_callback(MenuCategory.AUTO, "Autonomy", "on_change", on_autonomy_enabled_change)
    menu.set_callback(MenuCategory.AUTO, "Obstacle Avoid", "on_change", on_obstacle_avoid_change)
    menu.set_callback(MenuCategory.AUTO, "Cliff Detect", "on_change", on_cliff_detect_change)
    menu.set_callback(MenuCategory.AUTO, "Caught Foot", "on_change", on_caught_foot_change)
    menu.set_callback(MenuCategory.AUTO, "Wall Follow", "on_change", on_wall_follow_change)
    menu.set_callback(MenuCategory.AUTO, "Wall Side", "on_change", on_wall_side_change)
    menu.set_callback(MenuCategory.AUTO, "Wall Dist", "on_change", on_wall_dist_change)
    menu.set_callback(MenuCategory.AUTO, "Patrol", "on_change", on_patrol_change)
    menu.set_callback(MenuCategory.AUTO, "Stop Dist", "on_change", on_stop_dist_change)
    menu.set_callback(MenuCategory.AUTO, "Slow Dist", "on_change", on_slow_dist_change)
    menu.set_callback(MenuCategory.AUTO, "Cliff Thresh", "on_change", on_cliff_thresh_change)
    menu.set_callback(MenuCategory.AUTO, "Snag Error", "on_change", on_snag_error_change)
    menu.set_callback(MenuCategory.AUTO, "Snag Timeout", "on_change", on_snag_timeout_change)
    menu.set_callback(MenuCategory.AUTO, "Recovery Lift", "on_change", on_recovery_lift_change)
    menu.set_callback(MenuCategory.AUTO, "Patrol Time", "on_change", on_patrol_time_change)
    menu.set_callback(MenuCategory.AUTO, "Turn Interval", "on_change", on_turn_interval_change)
    menu.set_callback(MenuCategory.AUTO, "Save Settings", "on_select", on_autonomy_save)


# ===========================================================================
# M3.9: SAFETY callbacks
# ===========================================================================

def setup_safety_callbacks(menu: MarsMenu, ctx: SimpleNamespace) -> None:
    """Setup SAFETY menu callbacks.
    
    ctx must have:
        - send_cmd: callable(cmd, force=True)
        - get_verbose: callable returning bool
        - display_thread: DisplayThread or None
        - get/set for safety display thresholds (volt_min, volt_warn, volt_max, temp_min, temp_max)
        - get/set for low battery settings (enabled, volt_critical, volt_recovery, filter_alpha)
                - get/set for collision settings (enabled, stop_on_collision, warn_only,
                    leg_radius_mm, safety_margin_mm, body_keepout_radius_mm,
                    time_horizon_s, max_velocity_margin_mm,
                    pose_log_enabled, pose_log_hz)
    """

    def on_clear_safety():
        ctx.get_send_cmd()(b'SAFETY CLEAR', force=True)
        if ctx.get_verbose():
            print("SAFETY CLEAR command sent to Teensy", end="\r\n")

    def _send_safety_override(keyword: bytes, label: str):
        cmd = b'SAFETY OVERRIDE ' + keyword
        ctx.get_send_cmd()(cmd, force=True)
        if ctx.get_verbose():
            print(f"SAFETY OVERRIDE {label} command sent to Teensy", end="\r\n")

    def on_override_all():
        _send_safety_override(b'ALL', 'ALL')

    def on_override_temp():
        _send_safety_override(b'TEMP', 'TEMP')

    def on_override_collision():
        _send_safety_override(b'COLLISION', 'COLLISION')

    def on_override_none():
        _send_safety_override(b'NONE', 'NONE')

    # Safety display threshold callbacks
    def on_volt_min_change(value):
        ctx.set_safety_volt_min(float(value))
        if ctx.get_display_thread():
            ctx.get_display_thread().set_safety_thresholds(volt_min=float(value))
        save_safety_display_settings({'volt_min': float(value)})

    def on_volt_warn_change(value):
        ctx.set_safety_volt_warn(float(value))
        if ctx.get_display_thread():
            ctx.get_display_thread().set_safety_thresholds(volt_warn=float(value))
        save_safety_display_settings({'volt_warn': float(value)})

    def on_volt_max_change(value):
        ctx.set_safety_volt_max(float(value))
        if ctx.get_display_thread():
            ctx.get_display_thread().set_safety_thresholds(volt_max=float(value))
        save_safety_display_settings({'volt_max': float(value)})

    def on_temp_min_change(value):
        ctx.set_safety_temp_min(float(value))
        if ctx.get_display_thread():
            ctx.get_display_thread().set_safety_thresholds(temp_min=float(value))
        save_safety_display_settings({'temp_min': float(value)})

    def on_temp_max_change(value):
        ctx.set_safety_temp_max(float(value))
        if ctx.get_display_thread():
            ctx.get_display_thread().set_safety_thresholds(temp_max=float(value))
        save_safety_display_settings({'temp_max': float(value)})

    # Low battery protection callbacks
    def on_lowbatt_prot_change(value):
        ctx.set_low_battery_enabled(value == 1)
        save_low_battery_settings({'enabled': value == 1})

    def on_volt_critical_change(value):
        ctx.set_low_battery_volt_critical(float(value))
        save_low_battery_settings({'volt_critical': float(value)})

    def on_volt_recovery_change(value):
        ctx.set_low_battery_recovery_volt(float(value))
        save_low_battery_settings({'volt_recovery': float(value)})

    def on_lowbatt_filter_change(value):
        alpha = max(0.01, min(1.0, float(value)))
        ctx.set_low_battery_filter_alpha(alpha)
        save_low_battery_settings({'filter_alpha': alpha})

    # Collision model tuning callbacks
    def on_col_enabled_change(value):
        enabled = (value == 1)
        ctx.set_collision_enabled(enabled)
        save_collision_settings({'enabled': enabled})

    def on_col_stop_change(value):
        stop_on_collision = (value == 1)
        ctx.set_collision_stop_on_collision(stop_on_collision)
        save_collision_settings({'stop_on_collision': stop_on_collision})

    def on_col_warn_only_change(value):
        warn_only = (value == 1)
        ctx.set_collision_warn_only(warn_only)
        save_collision_settings({'warn_only': warn_only})

    def on_leg_radius_change(value):
        v = max(0.0, min(100.0, float(value)))
        ctx.set_collision_leg_radius_mm(v)
        save_collision_settings({'leg_radius_mm': v})

    def on_safety_margin_change(value):
        v = max(0.0, min(100.0, float(value)))
        ctx.set_collision_safety_margin_mm(v)
        save_collision_settings({'safety_margin_mm': v})

    def on_body_keepout_change(value):
        v = max(0.0, float(value))
        ctx.set_collision_body_keepout_radius_mm(v)
        save_collision_settings({'body_keepout_radius_mm': v})

    def on_vel_horizon_ms_change(value):
        ms = max(0.0, float(value))
        sec = ms / 1000.0
        ctx.set_collision_time_horizon_s(sec)
        save_collision_settings({'time_horizon_s': sec})

    def on_vel_margin_max_change(value):
        v = max(0.0, float(value))
        ctx.set_collision_max_velocity_margin_mm(v)
        save_collision_settings({'max_velocity_margin_mm': v})

    # Collision diagnostics callbacks
    def on_pose_log_change(value):
        enabled = (value == 1)
        ctx.set_collision_pose_log_enabled(enabled)
        save_collision_settings({'pose_log_enabled': enabled})

    def on_pose_log_hz_change(value):
        hz = max(0.1, min(200.0, float(value)))
        ctx.set_collision_pose_log_hz(hz)
        save_collision_settings({'pose_log_hz': hz})

    # Register callbacks
    menu.set_callback(MenuCategory.SAFETY, "Clear Safety", "on_select", on_clear_safety)
    menu.set_callback(MenuCategory.SAFETY, "Override ALL", "on_select", on_override_all)
    menu.set_callback(MenuCategory.SAFETY, "Override TEMP", "on_select", on_override_temp)
    menu.set_callback(MenuCategory.SAFETY, "Override COLLISION", "on_select", on_override_collision)
    menu.set_callback(MenuCategory.SAFETY, "Override NONE", "on_select", on_override_none)
    menu.set_callback(MenuCategory.SAFETY, "Volt Min", "on_change", on_volt_min_change)
    menu.set_callback(MenuCategory.SAFETY, "Volt Warn", "on_change", on_volt_warn_change)
    menu.set_callback(MenuCategory.SAFETY, "Volt Max", "on_change", on_volt_max_change)
    menu.set_callback(MenuCategory.SAFETY, "Temp Min", "on_change", on_temp_min_change)
    menu.set_callback(MenuCategory.SAFETY, "Temp Max", "on_change", on_temp_max_change)
    menu.set_callback(MenuCategory.SAFETY, "LowBatt Prot", "on_change", on_lowbatt_prot_change)
    menu.set_callback(MenuCategory.SAFETY, "Volt Critical", "on_change", on_volt_critical_change)
    menu.set_callback(MenuCategory.SAFETY, "Volt Recovery", "on_change", on_volt_recovery_change)
    menu.set_callback(MenuCategory.SAFETY, "LowBatt Filter", "on_change", on_lowbatt_filter_change)

    menu.set_callback(MenuCategory.SAFETY, "Col Enabled", "on_change", on_col_enabled_change)
    menu.set_callback(MenuCategory.SAFETY, "Col Stop", "on_change", on_col_stop_change)
    menu.set_callback(MenuCategory.SAFETY, "Col WarnOnly", "on_change", on_col_warn_only_change)
    menu.set_callback(MenuCategory.SAFETY, "Leg Radius", "on_change", on_leg_radius_change)
    menu.set_callback(MenuCategory.SAFETY, "Safety Margin", "on_change", on_safety_margin_change)
    menu.set_callback(MenuCategory.SAFETY, "Body Keepout", "on_change", on_body_keepout_change)
    menu.set_callback(MenuCategory.SAFETY, "Vel Horizon", "on_change", on_vel_horizon_ms_change)
    menu.set_callback(MenuCategory.SAFETY, "Vel Margin Max", "on_change", on_vel_margin_max_change)
    menu.set_callback(MenuCategory.SAFETY, "Pose Log", "on_change", on_pose_log_change)
    menu.set_callback(MenuCategory.SAFETY, "Pose Log Hz", "on_change", on_pose_log_hz_change)


# ===========================================================================
# M3.10: Sync initial values helpers
# ===========================================================================

def sync_posture_initial_values(menu: MarsMenu, ctx: SimpleNamespace) -> None:
    """Sync POSTURE (pounce) menu items with current state.
    
    ctx must have getters for all pounce parameters.
    """
    menu.set_value(MenuCategory.POSTURE, "P Prep", int(ctx.get_pounce_prep_ms()))
    menu.set_value(MenuCategory.POSTURE, "P Rear", int(ctx.get_pounce_rear_ms()))
    menu.set_value(MenuCategory.POSTURE, "P Lunge", int(ctx.get_pounce_lunge_ms()))
    menu.set_value(MenuCategory.POSTURE, "P Recov", int(ctx.get_pounce_recover_ms()))
    menu.set_value(MenuCategory.POSTURE, "P Back1", int(round(ctx.get_pounce_back1_z())))
    menu.set_value(MenuCategory.POSTURE, "P Back2", int(round(ctx.get_pounce_back2_z())))
    menu.set_value(MenuCategory.POSTURE, "P Push", int(round(ctx.get_pounce_push_z())))
    menu.set_value(MenuCategory.POSTURE, "P Strike", int(round(ctx.get_pounce_strike_z())))
    menu.set_value(MenuCategory.POSTURE, "P Crouch", int(round(ctx.get_pounce_crouch_dy())))
    menu.set_value(MenuCategory.POSTURE, "P Lift", int(round(ctx.get_pounce_lift_dy())))
    menu.set_value(MenuCategory.POSTURE, "P FrontZ", int(round(ctx.get_pounce_front_z())))


def sync_gait_initial_values(menu: MarsMenu, ctx: SimpleNamespace) -> None:
    """Sync GAIT menu items with current state.
    
    ctx must have getters for gait parameters.
    """
    menu.set_value(MenuCategory.GAIT, "Step Height", int(ctx.get_gait_lift_mm()))
    menu.set_value(MenuCategory.GAIT, "Turn Rate", int(ctx.get_gait_turn_max_deg_s()))
    menu.set_value(MenuCategory.GAIT, "Cycle Time", int(ctx.get_gait_cycle_ms()))


def sync_safety_initial_values(menu: MarsMenu, ctx: SimpleNamespace) -> None:
    """Sync SAFETY menu items with current state.
    
    ctx must have getters for safety settings.
    """
    menu.set_value(MenuCategory.SAFETY, "Volt Min", ctx.get_safety_volt_min())
    menu.set_value(MenuCategory.SAFETY, "Volt Warn", ctx.get_safety_volt_warn())
    menu.set_value(MenuCategory.SAFETY, "Volt Max", ctx.get_safety_volt_max())
    menu.set_value(MenuCategory.SAFETY, "Temp Min", ctx.get_safety_temp_min())
    menu.set_value(MenuCategory.SAFETY, "Temp Max", ctx.get_safety_temp_max())
    menu.set_value(MenuCategory.SAFETY, "LowBatt Prot", 1 if ctx.get_low_battery_enabled() else 0)
    menu.set_value(MenuCategory.SAFETY, "Volt Critical", ctx.get_low_battery_volt_critical())
    menu.set_value(MenuCategory.SAFETY, "Volt Recovery", ctx.get_low_battery_recovery_volt())
    menu.set_value(MenuCategory.SAFETY, "LowBatt Filter", ctx.get_low_battery_filter_alpha())
    menu.set_value(MenuCategory.SAFETY, "LowBatt Status", "OK")

    menu.set_value(MenuCategory.SAFETY, "Col Enabled", 1 if ctx.get_collision_enabled() else 0)
    menu.set_value(MenuCategory.SAFETY, "Col Stop", 1 if ctx.get_collision_stop_on_collision() else 0)
    menu.set_value(MenuCategory.SAFETY, "Col WarnOnly", 1 if ctx.get_collision_warn_only() else 0)
    menu.set_value(MenuCategory.SAFETY, "Leg Radius", float(ctx.get_collision_leg_radius_mm()))
    menu.set_value(MenuCategory.SAFETY, "Safety Margin", float(ctx.get_collision_safety_margin_mm()))
    menu.set_value(MenuCategory.SAFETY, "Body Keepout", float(ctx.get_collision_body_keepout_radius_mm()))
    menu.set_value(MenuCategory.SAFETY, "Vel Horizon", int(round(float(ctx.get_collision_time_horizon_s()) * 1000.0)))
    menu.set_value(MenuCategory.SAFETY, "Vel Margin Max", float(ctx.get_collision_max_velocity_margin_mm()))

    menu.set_value(MenuCategory.SAFETY, "Pose Log", 1 if ctx.get_collision_pose_log_enabled() else 0)
    menu.set_value(MenuCategory.SAFETY, "Pose Log Hz", float(ctx.get_collision_pose_log_hz()))
