# audio_manager.py
# Audio feedback system for MARS hexapod
# Uses pygame.mixer for sound playback via USB DAC
#
# Hardware chain:
#   Pi 5 USB → Sabrent DAC → 3.5mm → PAM8403 amp → 2x 28mm speakers
#
# Changelog:
# 2026-01-03  v1.0: Initial implementation - pygame mixer, sound pool, volume control
#----------------------------------------------------------------------------------------------------------------------

import os
import threading
from typing import Dict, Optional, Callable
from dataclasses import dataclass, field

# Try to import pygame.mixer
try:
    import pygame
    import pygame.mixer
    PYGAME_AVAILABLE = True
except ImportError:
    PYGAME_AVAILABLE = False
    print("[AudioManager] pygame not available - audio disabled")


@dataclass
class AudioConfig:
    """Configuration for audio manager."""
    enabled: bool = True
    volume: float = 0.7          # Master volume 0.0-1.0
    sounds_dir: str = "assets/sounds"
    device: str = "hw:2,0"       # ALSA device: hw:2,0 = USB Audio (Sabrent DAC)
    sample_rate: int = 44100
    buffer_size: int = 512       # Smaller = lower latency, larger = more stable
    channels: int = 2            # Stereo


class AudioManager:
    """
    Non-blocking audio playback manager for MARS hexapod.
    
    Features:
    - Sound pool for preloaded common sounds
    - Non-blocking playback
    - Volume control
    - Priority channels (alerts override ambient)
    
    Usage:
        audio = AudioManager(config)
        audio.play("startup")
        audio.play("alert_low_battery", priority=True)
        audio.set_volume(0.5)
        audio.stop_all()
    """
    
    # Sound categories with relative volumes
    SOUND_VOLUMES = {
        "startup": 1.0,
        "shutdown": 1.0,
        "enable": 0.8,
        "disable": 0.8,
        "gait_change": 0.6,
        "mode_change": 0.7,
        "alert_low_battery": 1.0,
        "alert_safety": 1.0,
        "alert_collision": 0.9,
        "click": 1.0,  # Boosted from 0.5 - short click needs full volume
        "beep": 0.6,
        "error": 0.8,
    }
    
    def __init__(self, config: Optional[AudioConfig] = None):
        """Initialize audio manager.
        
        Args:
            config: Audio configuration. Uses defaults if None.
        """
        self.config = config or AudioConfig()
        self._sounds: Dict[str, pygame.mixer.Sound] = {}
        self._initialized = False
        self._lock = threading.Lock()
        self._muted = False
        
        if not PYGAME_AVAILABLE:
            print("[AudioManager] pygame not available - audio disabled")
            return
            
        if not self.config.enabled:
            print("[AudioManager] Audio disabled by config")
            return
        
        self._init_mixer()
    
    def _init_mixer(self) -> bool:
        """Initialize pygame mixer with configured settings."""
        if not PYGAME_AVAILABLE:
            return False
            
        try:
            # Set SDL audio driver for ALSA
            if self.config.device:
                os.environ['SDL_AUDIODRIVER'] = 'alsa'
                os.environ['AUDIODEV'] = self.config.device
            
            # Initialize pygame (only mixer, not display)
            if not pygame.mixer.get_init():
                pygame.mixer.pre_init(
                    frequency=self.config.sample_rate,
                    size=-16,  # 16-bit signed
                    channels=self.config.channels,
                    buffer=self.config.buffer_size
                )
                pygame.mixer.init()
            
            # Reserve channels: 0-3 for normal sounds, 4-7 for priority/alerts
            pygame.mixer.set_num_channels(8)
            
            self._initialized = True
            print(f"[AudioManager] Initialized: {pygame.mixer.get_init()}")
            return True
            
        except Exception as e:
            print(f"[AudioManager] Failed to initialize: {e}")
            self._initialized = False
            return False
    
    def preload(self, sound_names: Optional[list] = None) -> int:
        """Preload sounds into memory for faster playback.
        
        Args:
            sound_names: List of sound names to preload. If None, preloads all
                        sounds found in sounds_dir.
        
        Returns:
            Number of sounds successfully loaded.
        """
        if not self._initialized:
            return 0
        
        sounds_dir = self._get_sounds_dir()
        if not os.path.isdir(sounds_dir):
            print(f"[AudioManager] Sounds directory not found: {sounds_dir}")
            return 0
        
        loaded = 0
        
        # If no specific sounds requested, scan directory
        if sound_names is None:
            sound_names = []
            for filename in os.listdir(sounds_dir):
                if filename.endswith(('.wav', '.ogg', '.mp3')):
                    name = os.path.splitext(filename)[0]
                    sound_names.append(name)
        
        for name in sound_names:
            if self._load_sound(name):
                loaded += 1
        
        print(f"[AudioManager] Preloaded {loaded} sounds")
        return loaded
    
    def _get_sounds_dir(self) -> str:
        """Get absolute path to sounds directory."""
        if os.path.isabs(self.config.sounds_dir):
            return self.config.sounds_dir
        # Relative to this file's directory
        base_dir = os.path.dirname(os.path.abspath(__file__))
        return os.path.join(base_dir, self.config.sounds_dir)
    
    def _load_sound(self, name: str) -> bool:
        """Load a single sound file into the pool.
        
        Args:
            name: Sound name (without extension)
            
        Returns:
            True if loaded successfully.
        """
        if not self._initialized:
            return False
            
        if name in self._sounds:
            return True  # Already loaded
        
        sounds_dir = self._get_sounds_dir()
        
        # Try common audio formats
        for ext in ('.wav', '.ogg', '.mp3'):
            filepath = os.path.join(sounds_dir, name + ext)
            if os.path.isfile(filepath):
                try:
                    sound = pygame.mixer.Sound(filepath)
                    with self._lock:
                        self._sounds[name] = sound
                    return True
                except Exception as e:
                    print(f"[AudioManager] Failed to load {filepath}: {e}")
                    return False
        
        return False
    
    def play(self, name: str, priority: bool = False, loop: bool = False,
             volume: Optional[float] = None) -> bool:
        """Play a sound by name.
        
        Args:
            name: Sound name (matches filename without extension)
            priority: If True, use priority channel (won't be interrupted)
            loop: If True, loop indefinitely until stopped
            volume: Override volume (0.0-1.0). Uses category default if None.
            
        Returns:
            True if sound started playing.
        """
        if not self._initialized or self._muted:
            return False
        
        # Load on demand if not preloaded
        if name not in self._sounds:
            if not self._load_sound(name):
                print(f"[AudioManager] Sound not found: {name}")
                return False
        
        try:
            sound = self._sounds[name]
            
            # Calculate effective volume
            category_vol = self.SOUND_VOLUMES.get(name, 0.7)
            if volume is not None:
                effective_vol = volume * self.config.volume
            else:
                effective_vol = category_vol * self.config.volume
            
            sound.set_volume(effective_vol)
            
            # Choose channel based on priority
            if priority:
                # Use channels 4-7 for priority sounds
                channel = pygame.mixer.find_channel(True)  # Force find
                if channel is None:
                    channel = pygame.mixer.Channel(4)
            else:
                # Use channels 0-3 for normal sounds
                channel = pygame.mixer.find_channel(False)
                if channel is None:
                    channel = pygame.mixer.Channel(0)
            
            loops = -1 if loop else 0
            channel.play(sound, loops=loops)
            return True
            
        except Exception as e:
            print(f"[AudioManager] Playback error for {name}: {e}")
            return False
    
    def stop(self, name: str) -> None:
        """Stop a specific sound if it's playing."""
        if name in self._sounds:
            self._sounds[name].stop()
    
    def stop_all(self) -> None:
        """Stop all currently playing sounds."""
        if self._initialized:
            pygame.mixer.stop()
    
    def set_volume(self, volume: float) -> None:
        """Set master volume.
        
        Args:
            volume: Volume level 0.0-1.0
        """
        self.config.volume = max(0.0, min(1.0, volume))
    
    def get_volume(self) -> float:
        """Get current master volume."""
        return self.config.volume
    
    def mute(self, muted: bool = True) -> None:
        """Mute or unmute audio."""
        self._muted = muted
        if muted:
            self.stop_all()
    
    def is_muted(self) -> bool:
        """Check if audio is muted."""
        return self._muted
    
    def is_playing(self, name: Optional[str] = None) -> bool:
        """Check if any sound (or specific sound) is playing.
        
        Args:
            name: Check specific sound. If None, checks any sound.
        """
        if not self._initialized:
            return False
        
        if name is not None and name in self._sounds:
            return self._sounds[name].get_num_channels() > 0
        
        return pygame.mixer.get_busy()
    
    def beep(self, frequency: int = 880, duration_ms: int = 100) -> bool:
        """Play a simple beep tone.
        
        Args:
            frequency: Tone frequency in Hz
            duration_ms: Duration in milliseconds
            
        Returns:
            True if played successfully.
        """
        if not self._initialized or self._muted:
            return False
        
        try:
            import numpy as np
            
            # Generate sine wave
            sample_rate = self.config.sample_rate
            duration = duration_ms / 1000.0
            samples = int(sample_rate * duration)
            t = np.linspace(0, duration, samples, False)
            wave = np.sin(2 * np.pi * frequency * t)
            
            # Apply envelope to avoid clicks (5ms fade in/out)
            env_samples = int(sample_rate * 0.005)
            envelope = np.ones(samples)
            envelope[:env_samples] = np.linspace(0, 1, env_samples)
            envelope[-env_samples:] = np.linspace(1, 0, env_samples)
            wave = wave * envelope * self.config.volume
            
            # Convert to 16-bit stereo
            wave16 = (wave * 32767).astype(np.int16)
            stereo = np.zeros((samples, 2), dtype=np.int16)
            stereo[:, 0] = wave16
            stereo[:, 1] = wave16
            
            sound = pygame.mixer.Sound(buffer=stereo.tobytes())
            sound.play()
            return True
            
        except ImportError:
            print("[AudioManager] numpy required for beep()")
            return False
        except Exception as e:
            print(f"[AudioManager] Beep error: {e}")
            return False
    
    def shutdown(self) -> None:
        """Clean shutdown of audio system."""
        if self._initialized:
            self.stop_all()
            pygame.mixer.quit()
            self._initialized = False
            print("[AudioManager] Shutdown complete")


# =============================================================================
# Standalone test
# =============================================================================
if __name__ == "__main__":
    import time
    
    print("AudioManager Test")
    print("=" * 40)
    
    # Create manager with defaults
    config = AudioConfig(
        enabled=True,
        volume=0.8,
        sounds_dir="assets/sounds"
    )
    audio = AudioManager(config)
    
    # Test beep
    print("\nTest 1: Beep tones")
    audio.beep(440, 200)  # A4
    time.sleep(0.3)
    audio.beep(880, 200)  # A5
    time.sleep(0.3)
    audio.beep(1760, 200)  # A6
    time.sleep(0.5)
    
    # Test preload
    print("\nTest 2: Preload sounds")
    loaded = audio.preload()
    print(f"Loaded {loaded} sounds")
    
    # Test playback
    print("\nTest 3: Play startup sound")
    if audio.play("startup"):
        print("Playing startup...")
        time.sleep(2)
    else:
        print("No startup sound found")
    
    # Test volume
    print("\nTest 4: Volume control")
    audio.set_volume(0.3)
    audio.beep(660, 300)
    time.sleep(0.5)
    audio.set_volume(0.8)
    audio.beep(660, 300)
    time.sleep(0.5)
    
    # Shutdown
    audio.shutdown()
    print("\nDone!")
