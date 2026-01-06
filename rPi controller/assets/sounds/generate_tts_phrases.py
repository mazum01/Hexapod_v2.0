#!/usr/bin/env python3
"""
Generate pre-cached TTS phrase WAV files for MARS hexapod.

This pre-generates all fixed speech phrases so they can play instantly
without the ~300ms latency of real-time TTS generation.

Supports both Piper (natural) and espeak-ng (robotic) engines.

Run: python3 generate_tts_phrases.py [--engine piper|espeak]

Output: tts_*.wav files in the current directory (assets/sounds/)
"""

import subprocess
import os
import sys
import argparse

# All fixed TTS phrases used by the controller
PHRASES = {
    "tts_startup": "Mars online",
    "tts_shutdown": "Mars shutting down",
    "tts_enabled": "Robot enabled",
    "tts_disabled": "Robot disabled",
    "tts_standing": "Standing",
    "tts_tucking": "Tucking",
    "tts_safety_lockout": "Safety lockout activated",
    "tts_autonomy_on": "Autonomy mode on",
    "tts_autonomy_off": "Autonomy mode off",
    "tts_obstacle": "Obstacle detected",
    "tts_cliff": "Cliff detected",
    # Battery phrases for common percentages (dynamic fallback for others)
    "tts_battery_10": "Battery low, 10 percent remaining",
    "tts_battery_15": "Battery low, 15 percent remaining",
    "tts_battery_20": "Battery low, 20 percent remaining",
    "tts_battery_25": "Battery low, 25 percent remaining",
}

# Default Piper model
DEFAULT_PIPER_MODEL = os.path.expanduser("~/.local/share/piper/en_US-lessac-medium.onnx")

# Default espeak settings
DEFAULT_ESPEAK_VOICE = "en-us"
DEFAULT_ESPEAK_RATE = 140
DEFAULT_ESPEAK_PITCH = 50


def generate_piper(text: str, output_path: str, model_path: str, gain_db: int = 10):
    """Generate WAV using Piper TTS."""
    try:
        # Piper outputs raw PCM, sox converts to WAV and applies gain
        piper = subprocess.Popen(
            ['piper', '-m', model_path, '--output-raw'],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL
        )
        sox = subprocess.Popen(
            ['sox', '-t', 'raw', '-r', '22050', '-e', 'signed', '-b', '16', '-c', '1', '-',
             '-t', 'wav', output_path, 'gain', str(gain_db)],
            stdin=piper.stdout,
            stderr=subprocess.DEVNULL
        )
        piper.stdin.write(text.encode('utf-8'))
        piper.stdin.close()
        sox.wait()
        piper.wait()
        return piper.returncode == 0 and sox.returncode == 0
    except Exception as e:
        print(f"    Error: {e}")
        return False


def generate_espeak(text: str, output_path: str, voice: str = DEFAULT_ESPEAK_VOICE,
                    rate: int = DEFAULT_ESPEAK_RATE, pitch: int = DEFAULT_ESPEAK_PITCH,
                    gain_db: int = 30):
    """Generate WAV using espeak-ng."""
    try:
        # espeak outputs WAV, sox applies gain
        espeak = subprocess.Popen(
            ['espeak-ng', '-v', voice, '-s', str(rate), '-p', str(pitch),
             '-a', '200', '--stdout', text],
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL
        )
        sox = subprocess.Popen(
            ['sox', '-t', 'wav', '-', '-t', 'wav', output_path, 'gain', str(gain_db)],
            stdin=espeak.stdout,
            stderr=subprocess.DEVNULL
        )
        sox.wait()
        espeak.wait()
        return espeak.returncode == 0 and sox.returncode == 0
    except Exception as e:
        print(f"    Error: {e}")
        return False


def main():
    parser = argparse.ArgumentParser(description="Generate TTS phrase WAV files")
    parser.add_argument('--engine', choices=['piper', 'espeak'], default='piper',
                        help='TTS engine to use (default: piper)')
    parser.add_argument('--model', default=DEFAULT_PIPER_MODEL,
                        help='Piper model path')
    parser.add_argument('--voice', default=DEFAULT_ESPEAK_VOICE,
                        help='espeak voice')
    parser.add_argument('--gain', type=int, default=10,
                        help='Sox gain in dB')
    args = parser.parse_args()

    script_dir = os.path.dirname(os.path.abspath(__file__))
    
    print(f"MARS TTS Phrase Generator")
    print(f"=" * 40)
    print(f"Engine: {args.engine}")
    if args.engine == 'piper':
        print(f"Model: {os.path.basename(args.model)}")
    else:
        print(f"Voice: {args.voice}")
    print(f"Gain: +{args.gain}dB")
    print(f"Output: {script_dir}")
    print()

    success = 0
    failed = 0

    for name, phrase in PHRASES.items():
        output_path = os.path.join(script_dir, f"{name}.wav")
        print(f"  Generating: {name}.wav")
        print(f"    \"{phrase}\"")
        
        if args.engine == 'piper':
            ok = generate_piper(phrase, output_path, args.model, args.gain)
        else:
            ok = generate_espeak(phrase, output_path, args.voice, gain_db=args.gain)
        
        if ok and os.path.isfile(output_path):
            size = os.path.getsize(output_path)
            print(f"    OK ({size/1024:.1f} KB)")
            success += 1
        else:
            print(f"    FAILED")
            failed += 1

    print()
    print(f"Done! Generated {success} files, {failed} failed.")
    
    if failed > 0:
        sys.exit(1)


if __name__ == "__main__":
    main()
