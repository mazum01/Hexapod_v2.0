# audio_manager.py
# Audio feedback system for MARS hexapod
# Uses pygame.mixer for sound playback via USB DAC
#
# Hardware chain:
#   Pi 5 USB → Sabrent DAC → 3.5mm → PAM8403 amp → 2x 28mm speakers
#
# Changelog:
# 2026-01-04  v1.1: Added sound queue for sequential playback; priority sounds can interrupt
# 2026-01-03  v1.0: Initial implementation - pygame mixer, sound pool, volume control
#----------------------------------------------------------------------------------------------------------------------

import os
import threading
import queue
import time
import os
from typing import Dict, Optional, Callable
from dataclasses import dataclass, field

# Try to import pygame.mixer (suppress welcome banner)
try:
    os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = "1"
    import pygame
    import pygame.mixer
    PYGAME_AVAILABLE = True
except ImportError:
    PYGAME_AVAILABLE = False
    print("[AudioManager] pygame not available - audio disabled", end="\r\n")


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
    queue_enabled: bool = True   # Enable sequential sound queue
    queue_max_size: int = 10     # Max pending sounds in queue


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
    
    # Sound categories with relative volumes (0.0-1.0)
    # Generated WAV files are in assets/sounds/
    SOUND_VOLUMES = {
        # System
        "startup": 1.0,
        "shutdown": 1.0,
        "enable": 0.8,
        "disable": 0.8,
        "click": 1.0,
        "error": 0.8,
        
        # Alerts
        "alert_low_battery": 1.0,
        "alert_safety": 1.0,
        "alert_collision": 0.9,
        
        # Gait
        "gait_start": 0.7,
        "gait_stop": 0.7,
        "gait_change": 0.6,
        "mode_change": 0.7,
        
        # Connectivity
        "connect": 0.8,
        "disconnect": 0.8,
        
        # Menu navigation
        "menu_nav": 0.4,      # Subtle - rapid navigation
        "menu_tab": 0.5,      # Slightly more noticeable
        "menu_adjust": 0.4,   # Subtle value changes
        "menu_select": 0.6,   # Selection confirmation
        
        # Autonomy
        "autonomy_on": 0.8,
        "autonomy_off": 0.8,
        
        # Postures
        "stand": 0.7,
        "tuck": 0.7,
        "home": 0.7,
        
        # Fallback for beeps
        "beep": 0.6,
        
        # TTS Pre-cached phrases (Piper-generated, priority playback)
        "tts_startup": 1.0,
        "tts_shutdown": 1.0,
        "tts_enabled": 1.0,
        "tts_disabled": 1.0,
        "tts_standing": 0.9,
        "tts_tucking": 0.9,
        "tts_safety_lockout": 1.0,
        "tts_autonomy_on": 0.9,
        "tts_autonomy_off": 0.9,
        "tts_obstacle": 0.9,
        "tts_cliff": 1.0,
        "tts_battery_10": 1.0,
        "tts_battery_15": 1.0,
        "tts_battery_20": 1.0,
        "tts_battery_25": 1.0,
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
        
        # Sound queue for sequential playback
        self._queue: queue.Queue = queue.Queue(maxsize=self.config.queue_max_size)
        self._queue_thread: Optional[threading.Thread] = None
        self._queue_stop = threading.Event()
        self._current_channel: Optional[pygame.mixer.Channel] = None
        
        # Short sounds that bypass queue (play immediately, don't interrupt speech)
        self._immediate_sounds = {
            'click', 'menu_nav', 'menu_tab', 'menu_adjust', 'menu_select',
            'beep', 'gait_start', 'gait_stop', 'gait_change'
        }
        
        if not PYGAME_AVAILABLE:
            print("[AudioManager] pygame not available - audio disabled", end="\r\n")
            return
            
        if not self.config.enabled:
            print("[AudioManager] Audio disabled by config", end="\r\n")
            return
        
        self._init_mixer()
        
        # Start queue worker thread
        if self.config.queue_enabled and self._initialized:
            self._start_queue_worker()
    
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
            print(f"[AudioManager] Initialized: {pygame.mixer.get_init()}", end="\r\n")
            return True
            
        except Exception as e:
            print(f"[AudioManager] Failed to initialize: {e}", end="\r\n")
            self._initialized = False
            return False
    
    def _start_queue_worker(self) -> None:
        """Start the background queue worker thread."""
        if self._queue_thread is not None and self._queue_thread.is_alive():
            return  # Already running
        
        self._queue_stop.clear()
        self._queue_thread = threading.Thread(target=self._queue_worker, daemon=True)
        self._queue_thread.start()
        print("[AudioManager] Queue worker started", end="\r\n")
    
    def _queue_worker(self) -> None:
        """Background worker that plays queued sounds sequentially."""
        while not self._queue_stop.is_set():
            try:
                # Wait for a sound with timeout (allows checking stop flag)
                item = self._queue.get(timeout=0.1)
            except queue.Empty:
                continue
            
            if item is None:  # Poison pill for shutdown
                break
            
            name, priority, volume = item
            
            # Play the sound and wait for it to finish
            self._play_immediate(name, priority, volume)
            
            # Wait for sound to finish (poll channel)
            while self._current_channel and self._current_channel.get_busy():
                if self._queue_stop.is_set():
                    break
                time.sleep(0.02)  # 20ms poll interval
            
            self._queue.task_done()
    
    def _play_immediate(self, name: str, priority: bool = False,
                        volume: Optional[float] = None) -> bool:
        """Play a sound immediately without queuing.
        
        Internal method used by queue worker and for short sounds.
        """
        if not self._initialized or self._muted:
            return False
        
        # Load on demand if not preloaded
        if name not in self._sounds:
            if not self._load_sound(name):
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
                # Use channel 4 for queued priority (TTS) sounds
                channel = pygame.mixer.Channel(4)
            else:
                # Use channels 0-3 for immediate/short sounds
                channel = pygame.mixer.find_channel(False)
                if channel is None:
                    channel = pygame.mixer.Channel(0)
            
            channel.play(sound)
            self._current_channel = channel
            return True
            
        except Exception as e:
            print(f"[AudioManager] Playback error for {name}: {e}", end="\r\n")
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
            print(f"[AudioManager] Sounds directory not found: {sounds_dir}", end="\r\n")
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
        
        print(f"[AudioManager] Preloaded {loaded} sounds", end="\r\n")
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
                    print(f"[AudioManager] Failed to load {filepath}: {e}", end="\r\n")
                    return False
        
        return False
    
    def play(self, name: str, priority: bool = False, loop: bool = False,
             volume: Optional[float] = None, interrupt: bool = False) -> bool:
        """Play a sound by name.
        
        Sounds are queued for sequential playback unless they are short sounds
        (clicks, menu navigation) which play immediately on separate channels.
        
        Args:
            name: Sound name (matches filename without extension)
            priority: If True, sound plays on priority channel (for TTS/alerts)
            loop: If True, loop indefinitely until stopped (bypasses queue)
            volume: Override volume (0.0-1.0). Uses category default if None.
            interrupt: If True and priority, stop current sound and play immediately
            
        Returns:
            True if sound queued/started successfully.
        """
        if not self._initialized or self._muted:
            return False
        
        # Load on demand if not preloaded
        if name not in self._sounds:
            if not self._load_sound(name):
                print(f"[AudioManager] Sound not found: {name}", end="\r\n")
                return False
        
        # Looping sounds bypass queue (used for continuous effects)
        if loop:
            return self._play_loop(name, volume)
        
        # Short/UI sounds play immediately on separate channels (don't queue)
        if name in self._immediate_sounds:
            return self._play_immediate(name, priority=False, volume=volume)
        
        # Priority interrupt: stop current and skip queue
        if priority and interrupt:
            self._clear_queue()
            if self._current_channel:
                self._current_channel.stop()
            return self._play_immediate(name, priority=True, volume=volume)
        
        # Queue enabled: add to sequential playback queue
        if self.config.queue_enabled:
            try:
                self._queue.put_nowait((name, priority, volume))
                return True
            except queue.Full:
                print(f"[AudioManager] Queue full, dropping sound: {name}", end="\r\n")
                return False
        
        # Queue disabled: play immediately
        return self._play_immediate(name, priority, volume)
    
    def _play_loop(self, name: str, volume: Optional[float] = None) -> bool:
        """Play a looping sound (bypasses queue)."""
        if not self._initialized or self._muted:
            return False
        
        if name not in self._sounds:
            return False
        
        try:
            sound = self._sounds[name]
            category_vol = self.SOUND_VOLUMES.get(name, 0.7)
            effective_vol = (volume if volume else category_vol) * self.config.volume
            sound.set_volume(effective_vol)
            
            channel = pygame.mixer.find_channel(False)
            if channel is None:
                channel = pygame.mixer.Channel(0)
            channel.play(sound, loops=-1)
            return True
        except Exception as e:
            print(f"[AudioManager] Loop error for {name}: {e}", end="\r\n")
            return False
    
    def _clear_queue(self) -> None:
        """Clear all pending sounds from the queue."""
        try:
            while True:
                self._queue.get_nowait()
                self._queue.task_done()
        except queue.Empty:
            pass
    
    def queue_pending(self) -> int:
        """Return number of sounds waiting in the queue."""
        return self._queue.qsize()
    
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
            print("[AudioManager] numpy required for beep()", end="\r\n")
            return False
        except Exception as e:
            print(f"[AudioManager] Beep error: {e}", end="\r\n")
            return False
    
    def shutdown(self) -> None:
        """Clean shutdown of audio system."""
        # Stop queue worker
        if self._queue_thread is not None:
            self._queue_stop.set()
            self._clear_queue()
            # Send poison pill
            try:
                self._queue.put_nowait(None)
            except queue.Full:
                pass
            self._queue_thread.join(timeout=1.0)
            self._queue_thread = None
        
        if self._initialized:
            self.stop_all()
            pygame.mixer.quit()
            self._initialized = False
            print("[AudioManager] Shutdown complete", end="\r\n")


# =============================================================================
# Standalone test
# =============================================================================
if __name__ == "__main__":
    import time
    
    print("AudioManager Test", end="\r\n")
    print("=" * 40, end="\r\n")
    
    # Create manager with defaults
    config = AudioConfig(
        enabled=True,
        volume=0.8,
        sounds_dir="assets/sounds"
    )
    audio = AudioManager(config)
    
    # Test beep
    print("\nTest 1: Beep tones", end="\r\n")
    audio.beep(440, 200)  # A4
    time.sleep(0.3)
    audio.beep(880, 200)  # A5
    time.sleep(0.3)
    audio.beep(1760, 200)  # A6
    time.sleep(0.5)
    
    # Test preload
    print("\nTest 2: Preload sounds", end="\r\n")
    loaded = audio.preload()
    print(f"Loaded {loaded} sounds", end="\r\n")
    
    # Test playback
    print("\nTest 3: Play startup sound", end="\r\n")
    if audio.play("startup"):
        print("Playing startup...", end="\r\n")
        time.sleep(2)
    else:
        print("No startup sound found", end="\r\n")
    
    # Test volume
    print("\nTest 4: Volume control", end="\r\n")
    audio.set_volume(0.3)
    audio.beep(660, 300)
    time.sleep(0.5)
    audio.set_volume(0.8)
    audio.beep(660, 300)
    time.sleep(0.5)
    
    # Shutdown
    audio.shutdown()
    print("\nDone!", end="\r\n")
