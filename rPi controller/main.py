#!/usr/bin/env python3
#----------------------------------------------------------------------------------------------------------------------
#    main.py
#----------------------------------------------------------------------------------------------------------------------
# New entry point for MARS Hexapod.
# Currently wraps the legacy controller.py script which runs on import.
# Future refactoring will move the execution logic here.
#----------------------------------------------------------------------------------------------------------------------

import sys
import os
import signal
import time

# Global reference to controller logic for signal handling
_controller_module = None

def _main_sigterm_handler(signum, frame):
    """Handle SIGTERM by propagating to controller stop."""
    print(f"\n[MAIN] Signal {signum} received, stopping controller...", flush=True)
    if _controller_module:
        _controller_module.stop()
    else:
        sys.exit(0)

if __name__ == "__main__":
    # Ensure the script directory is in the path
    script_dir = os.path.dirname(os.path.abspath(__file__))
    if script_dir not in sys.path:
        sys.path.append(script_dir)
    
    # Register signal handler for SIGTERM
    signal.signal(signal.SIGTERM, _main_sigterm_handler)

    print("Starting MARS Hexapod (Legacy Wrapper)...", end="\r\n", flush=True)
    
    try:
        # Importing controller.py currently triggers the full initialization and event loop
        # entirely within the global scope of that module.
        import controller
        _controller_module = controller
        
        # Keep main alive if controller runs in a thread (it doesn't, it blocks on import,
        # but just in case future refactoring changes that).
        # Since controller.py blocks on import until exit, we are done when import returns.
        
    except KeyboardInterrupt:
        print("\nMARS execution interrupted.")
        if _controller_module:
            _controller_module.stop()
            
    except Exception as e:
        import traceback
        traceback.print_exc()
        print(f"\nMARS execution failed: {e}")
        # Try to stop cleanly if possible
        if _controller_module:
            try:
                _controller_module.stop()
            except:
                pass
        sys.exit(1)
