#import evdev
from evdev import InputDevice, categorize, ecodes, list_devices
import cv2 as cv
#creates object 'gamepad' to store the data
#you can call it whatever you like

gamePad = None
cv.imshow('Gamepad Input', cv.imread('./controller.png'))

#cv.namedWindow('Gamepad Input', cv.WINDOW_NORMAL)
while True:
    devices = [InputDevice(path) for path in list_devices()]
    if (gamePad == None):
        print('Connecting to xbox controller...')
        found = False
        for device in devices:
            print(device.path, device.name) 
            if str.lower(device.name) == 'xbox wireless controller':
                gamePad = InputDevice(str(device.path))
                found = True
                break
    else: # device event loop
        try:
            print("Xbox controller connected")
            #prints out device info at start
            #print(gamePad)

            leds = gamePad.leds(verbose=True)
            print(leds) 

            events = gamePad.read()  # Read events from the gamepad
            #print(events)
            for event in events:

                print(event)
        except BlockingIOError as bioe:
            # Handle the case where no events are available
            print("No events available, waiting...")
            #continue
        except Exception as e:
            print(f"Error reading from gamepad: {e}")
            gamePad = None
    
    if cv.waitKey(1) & 0xFF == ord('q'):
        break