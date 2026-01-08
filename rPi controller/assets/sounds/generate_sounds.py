#!/usr/bin/env python3
"""
Generate sound effect WAV files for MARS hexapod audio feedback system.

All sounds are synthesized programmatically - no external assets needed.
Sounds are optimized for small speakers (28mm) and robot feedback use cases.

Run this script to regenerate all sound files:
    python3 generate_sounds.py

Output: .wav files in the current directory (assets/sounds/)
"""

import numpy as np
import wave
import struct
import os

# Audio parameters
SAMPLE_RATE = 44100
AMPLITUDE = 0.7  # Master amplitude (0-1)


def save_wav(filename: str, samples: np.ndarray, sample_rate: int = SAMPLE_RATE):
    """Save numpy array as 16-bit mono WAV file."""
    # Normalize and convert to 16-bit
    samples = np.clip(samples, -1.0, 1.0)
    samples_int16 = (samples * 32767).astype(np.int16)
    
    with wave.open(filename, 'w') as wav:
        wav.setnchannels(1)  # Mono
        wav.setsampwidth(2)  # 16-bit
        wav.setframerate(sample_rate)
        wav.writeframes(samples_int16.tobytes())
    
    print(f"  Created: {filename} ({len(samples)/sample_rate:.3f}s)")


def envelope(samples: np.ndarray, attack_ms: float = 5, release_ms: float = 10) -> np.ndarray:
    """Apply attack/release envelope to avoid clicks."""
    attack_samples = int(SAMPLE_RATE * attack_ms / 1000)
    release_samples = int(SAMPLE_RATE * release_ms / 1000)
    
    env = np.ones(len(samples))
    
    if attack_samples > 0 and attack_samples < len(samples):
        env[:attack_samples] = np.linspace(0, 1, attack_samples)
    
    if release_samples > 0 and release_samples < len(samples):
        env[-release_samples:] = np.linspace(1, 0, release_samples)
    
    return samples * env


def tone(freq: float, duration_ms: float, amplitude: float = AMPLITUDE) -> np.ndarray:
    """Generate a sine wave tone."""
    samples = int(SAMPLE_RATE * duration_ms / 1000)
    t = np.linspace(0, duration_ms / 1000, samples, False)
    wave = np.sin(2 * np.pi * freq * t) * amplitude
    return envelope(wave)


def silence(duration_ms: float) -> np.ndarray:
    """Generate silence."""
    return np.zeros(int(SAMPLE_RATE * duration_ms / 1000))


def noise_burst(duration_ms: float, amplitude: float = 0.3) -> np.ndarray:
    """Generate filtered noise burst (for clicks/impacts)."""
    samples = int(SAMPLE_RATE * duration_ms / 1000)
    noise = np.random.randn(samples) * amplitude
    # Simple lowpass by averaging
    kernel = np.ones(5) / 5
    filtered = np.convolve(noise, kernel, mode='same')
    return envelope(filtered, attack_ms=1, release_ms=duration_ms * 0.8)


def sweep(start_freq: float, end_freq: float, duration_ms: float, 
          amplitude: float = AMPLITUDE) -> np.ndarray:
    """Generate frequency sweep (chirp)."""
    samples = int(SAMPLE_RATE * duration_ms / 1000)
    t = np.linspace(0, duration_ms / 1000, samples, False)
    # Logarithmic sweep
    freq = start_freq * (end_freq / start_freq) ** (t / (duration_ms / 1000))
    phase = 2 * np.pi * np.cumsum(freq) / SAMPLE_RATE
    wave = np.sin(phase) * amplitude
    return envelope(wave)


# =============================================================================
# Sound Generators
# =============================================================================

def generate_startup():
    """3-tone ascending chime: A4 → C#5 → E5"""
    parts = [
        tone(440, 150),   # A4
        silence(30),
        tone(554, 150),   # C#5
        silence(30),
        tone(659, 200),   # E5
    ]
    return np.concatenate(parts)


def generate_shutdown():
    """2-tone descending: E5 → A4"""
    parts = [
        tone(659, 150),   # E5
        silence(50),
        tone(440, 250),   # A4 (longer, fading)
    ]
    return np.concatenate(parts)


def generate_enable():
    """Rising confirmation: G4 → C5"""
    parts = [
        tone(392, 80),    # G4
        silence(20),
        tone(523, 120),   # C5
    ]
    return np.concatenate(parts)


def generate_disable():
    """Falling tone: C5 → G4"""
    parts = [
        tone(523, 80),    # C5
        silence(20),
        tone(392, 120),   # G4
    ]
    return np.concatenate(parts)


def generate_click():
    """Short percussive click for button presses."""
    # Damped impulse - short and snappy
    samples = int(SAMPLE_RATE * 0.025)  # 25ms
    t = np.linspace(0, 0.025, samples, False)
    # Damped sine with fast decay
    freq = 1200
    decay = np.exp(-t * 150)
    wave = np.sin(2 * np.pi * freq * t) * decay * AMPLITUDE
    return wave


def generate_error():
    """Error buzz - two low harsh tones."""
    parts = [
        tone(200, 80),
        silence(30),
        tone(200, 100),
    ]
    return np.concatenate(parts)


def generate_alert_low_battery():
    """Urgent 3-beep alert."""
    parts = [
        tone(880, 100),
        silence(50),
        tone(880, 100),
        silence(50),
        tone(880, 150),
    ]
    return np.concatenate(parts)


def generate_alert_safety():
    """Urgent escalating warning."""
    parts = [
        tone(440, 80),
        silence(30),
        tone(660, 80),
        silence(30),
        tone(880, 120),
    ]
    return np.concatenate(parts)


def generate_gait_start():
    """Quick chirp for gait activation."""
    return sweep(400, 800, 80)


def generate_gait_stop():
    """Quick descending chirp for gait stop."""
    return sweep(800, 400, 80)


def generate_mode_change():
    """Mode switch confirmation."""
    parts = [
        tone(660, 60),
        silence(30),
        tone(880, 80),
    ]
    return np.concatenate(parts)


def generate_connect():
    """Device connected - rising tones."""
    parts = [
        tone(523, 60),    # C5
        silence(40),
        tone(659, 60),    # E5
        silence(40),
        tone(784, 80),    # G5
    ]
    return np.concatenate(parts)


def generate_disconnect():
    """Device disconnected - falling tones."""
    parts = [
        tone(784, 60),    # G5
        silence(40),
        tone(523, 60),    # C5
        silence(40),
        tone(392, 100),   # G4
    ]
    return np.concatenate(parts)


def generate_menu_nav():
    """Subtle tick for menu navigation."""
    # Very short, quiet tick
    samples = int(SAMPLE_RATE * 0.015)  # 15ms
    t = np.linspace(0, 0.015, samples, False)
    decay = np.exp(-t * 300)
    wave = np.sin(2 * np.pi * 1200 * t) * decay * 0.4
    return wave


def generate_menu_tab():
    """Tab change sound - slightly more noticeable."""
    return tone(880, 25, amplitude=0.5)


def generate_menu_adjust():
    """Value adjustment blip."""
    return tone(1000, 20, amplitude=0.5)


def generate_menu_select():
    """Selection confirmation."""
    parts = [
        tone(660, 30, amplitude=0.6),
        silence(30),
        tone(880, 40, amplitude=0.6),
    ]
    return np.concatenate(parts)


def generate_autonomy_on():
    """Autonomy mode enabled - sci-fi ascending."""
    parts = [
        tone(440, 60),    # A4
        silence(40),
        tone(554, 60),    # C#5
        silence(40),
        tone(659, 80),    # E5
    ]
    return np.concatenate(parts)


def generate_autonomy_off():
    """Autonomy mode disabled - descending."""
    parts = [
        tone(659, 60),    # E5
        silence(50),
        tone(440, 80),    # A4
    ]
    return np.concatenate(parts)


def generate_stand():
    """Posture: stand confirmation."""
    parts = [
        tone(523, 60),    # C5
        silence(30),
        tone(659, 80),    # E5
    ]
    return np.concatenate(parts)


def generate_tuck():
    """Posture: tuck confirmation."""
    parts = [
        tone(659, 60),    # E5
        silence(30),
        tone(440, 80),    # A4
    ]
    return np.concatenate(parts)


def generate_home():
    """Posture: home confirmation."""
    return tone(587, 100)  # D5


# =============================================================================
# Main
# =============================================================================

SOUNDS = {
    # System
    'startup': generate_startup,
    'shutdown': generate_shutdown,
    'enable': generate_enable,
    'disable': generate_disable,
    'click': generate_click,
    'error': generate_error,
    
    # Alerts
    'alert_low_battery': generate_alert_low_battery,
    'alert_safety': generate_alert_safety,
    
    # Gait
    'gait_start': generate_gait_start,
    'gait_stop': generate_gait_stop,
    'mode_change': generate_mode_change,
    
    # Connectivity
    'connect': generate_connect,
    'disconnect': generate_disconnect,
    
    # Menu
    'menu_nav': generate_menu_nav,
    'menu_tab': generate_menu_tab,
    'menu_adjust': generate_menu_adjust,
    'menu_select': generate_menu_select,
    
    # Autonomy
    'autonomy_on': generate_autonomy_on,
    'autonomy_off': generate_autonomy_off,
    
    # Postures
    'stand': generate_stand,
    'tuck': generate_tuck,
    'home': generate_home,
}


def main():
    print("MARS Hexapod Sound Generator")
    print("=" * 40)
    print(f"Sample rate: {SAMPLE_RATE} Hz")
    print(f"Generating {len(SOUNDS)} sound files...\n")
    
    # Get script directory for output
    script_dir = os.path.dirname(os.path.abspath(__file__))
    
    for name, generator in SOUNDS.items():
        samples = generator()
        filepath = os.path.join(script_dir, f"{name}.wav")
        save_wav(filepath, samples)
    
    print(f"\nDone! Generated {len(SOUNDS)} sound files.")


if __name__ == "__main__":
    main()
