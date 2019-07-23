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

def setup():

    GPIO.setmode(GPIO.BCM)		# Set numbering system to BCM
    GPIO.setup(17, GPIO.OUT)		# Set LED on pin 17 to output


def main():

    GPIO.output(17, True)		# Turn LED on
    time.sleep(2)			# Wait for 5 seconds
    GPIO.output(17, False)		# Turn LED off
    time.sleep(2)




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
