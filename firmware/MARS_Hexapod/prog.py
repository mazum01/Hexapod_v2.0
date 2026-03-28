import RPi.GPIO as GPIO
import time

# Set up GPIO
GPIO.setmode(GPIO.BCM)
program_pin = 21  # Use the GPIO pin number you connected to Teensy
GPIO.setup(program_pin, GPIO.OUT)

# Trigger the program pin
GPIO.output(program_pin, GPIO.LOW)  # Pull pin low
time.sleep(0.1)                      # Hold for a short duration
GPIO.output(program_pin, GPIO.HIGH)  # Release pin

# Clean up
GPIO.cleanup()