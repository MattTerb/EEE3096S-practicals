#!/usr/bin/python3
"""
Python Practical Template
Keegan Crankshaw
Readjust this Docstring as follows:
Names: Mattthew Terblanche
Student Number: TRBMAT002
Prac: 1
Date: 22/07/2019
"""

# import Relevant Librares
import RPi.GPIO as GPIO
import time

# Logic that you write

LED_status = 0		# Create variable for LED status and set it to off
def setup():

    GPIO.setmode(GPIO.BCM)		# Set numbering system to BCM
    GPIO.setup(17, GPIO.OUT)		# Set LED on pin 17 to output
    GPIO.setup(23, GPIO.IN)		# Set button on pin 23 to input
    GPIO.output(17, LED_status)		# Set LED to off

def switchLED(self):
    global LED_status
    LED_status = not LED_status		# Turn LED on if off or off if on
    GPIO.output(17, LED_status)		# Turn LED on

def main():

    GPIO.add_event_detect(23, GPIO.RISING, callback=switchLED, bouncetime=200)		# Wait for button press

    while True:
        time.sleep(1)

# Only run the functions if
if __name__ == "__main__":
    # Make sure the GPIO is stopped correctly
    try:

        setup()
        while True:
            main()
    except KeyboardInterrupt:
        print("Exiting gracefully")
        # Turn off your GPIOs here
        GPIO.cleanup()
    except e:
        GPIO.cleanup()
        print("Some other error occurred")
        print(e.message)
