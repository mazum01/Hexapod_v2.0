#!/usr/bin/env python3
"""
Button Code Recording Test Script
Records actual button codes from Xbox controller for each button press.
"""

import sys
import time
from datetime import datetime

try:
    from evdev import InputDevice, list_devices
except ImportError:
    print("Error: evdev not installed. Run: pip install evdev")
    sys.exit(1)

# Test order - buttons to press
TEST_ORDER = [
    "A (green, bottom)",
    "B (red, right)",
    "X (blue, left)",
    "Y (yellow, top)",
    "LB (left bumper)",
    "RB (right bumper)",
    "Back/View (left center)",
    "Start/Menu (right center)",
    "Left Stick Click",
    "Right Stick Click",
    "Xbox Guide (center logo)",
    "D-pad Up",
    "D-pad Down",
    "D-pad Left",
    "D-pad Right",
]


def find_controller():
    """Find Xbox controller."""
    for path in list_devices():
        try:
            dev = InputDevice(path)
            if 'xbox' in dev.name.lower() or 'controller' in dev.name.lower():
                return dev
        except (OSError, PermissionError):
            continue
    return None


def wait_for_input(dev, timeout=30):
    """Wait for any button/axis input, return description string."""
    import select
    
    start = time.time()
    while time.time() - start < timeout:
        r, _, _ = select.select([dev.fd], [], [], 0.1)
        if not r:
            continue
        
        try:
            for event in dev.read():
                # Button press (type 1, value 1)
                if event.type == 1 and event.value == 1:
                    return f"button code {event.code}"
                # D-pad / axis (type 3, code 16 or 17, non-zero)
                elif event.type == 3 and event.code in (16, 17) and event.value != 0:
                    direction = "positive" if event.value > 0 else "negative"
                    return f"axis {event.code} {direction} (value={event.value})"
        except BlockingIOError:
            continue
    
    return "TIMEOUT"


def main():
    print("=" * 60)
    print("  XBOX CONTROLLER BUTTON CODE RECORDER")
    print("=" * 60)
    print()
    
    # Find controller
    dev = find_controller()
    if not dev:
        print("ERROR: No Xbox controller found!")
        print("Make sure controller is connected and paired.")
        sys.exit(1)
    
    print(f"Found: {dev.name}")
    print(f"Path:  {dev.path}")
    print()
    print("Press each button when prompted. You have 30 seconds per button.")
    print("Press Ctrl+C to abort.")
    print()
    input("Press ENTER to begin...")
    print()
    
    # Results storage
    results = []
    
    for i, button_name in enumerate(TEST_ORDER, 1):
        print(f"[{i:2}/{len(TEST_ORDER)}] Press: {button_name}")
        
        received = wait_for_input(dev)
        print(f"        Received: {received}")
        
        results.append({
            "button": button_name,
            "received": received
        })
        
        # Small delay between tests
        time.sleep(0.5)
    
    # Summary
    print()
    print("=" * 60)
    print("  RECORDED BUTTON CODES")
    print("=" * 60)
    print()
    
    print(f"{'Button':<35} {'Code Received':<30}")
    print("-" * 65)
    
    for r in results:
        print(f"{r['button']:<35} {r['received']:<30}")
    
    print("-" * 65)
    print()
    
    # Save results to file
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"button_codes_{timestamp}.txt"
    
    with open(filename, "w") as f:
        f.write("XBOX CONTROLLER BUTTON CODE RECORDING\n")
        f.write(f"Date: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
        f.write(f"Controller: {dev.name}\n")
        f.write(f"Path: {dev.path}\n")
        f.write("\n")
        f.write(f"{'Button':<35} {'Code Received':<30}\n")
        f.write("-" * 65 + "\n")
        
        for r in results:
            f.write(f"{r['button']:<35} {r['received']:<30}\n")
        
        f.write("-" * 65 + "\n")
        
        # Extract just the codes for easy reference
        f.write("\nQUICK REFERENCE (for joy_controller.py):\n")
        for r in results:
            f.write(f"  {r['button']}: {r['received']}\n")
    
    print(f"Results saved to: {filename}")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\nTest aborted by user.")
        sys.exit(1)
