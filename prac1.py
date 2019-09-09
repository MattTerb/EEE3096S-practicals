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
import itertools		# Import itertools to use itertools.product to create binary array

# Logic that you write

counter = 0		# Variable for index to binary array
binary = list(itertools.product([0,1], repeat=3))		# Array of binary from 000 to 111
LED_pin = [17,27,22]			# Array for 3 LED pin numbers
btn_pin = [23,24]			# Array for 2 button pin numbers

def setup():

    GPIO.setmode(GPIO.BCM)		# Set numbering system to BCM

    GPIO.setup(LED_pin, GPIO.OUT)		# Set LED on pin 17,27,22 to output
    GPIO.setup(btn_pin, GPIO.IN)		# Set button on pin 23,24 to input

    GPIO.output(LED_pin, 0)		# Set LEDs to off

    GPIO.add_event_detect(23, GPIO.RISING, bouncetime=200)		# Detect button press on pin 23
    GPIO.add_event_detect(24, GPIO.RISING, bouncetime=200)		# Detect button press on pin 24

def buttonPress():

    global counter

    if (GPIO.input(23) == 1 and counter != 7):		# Check for left button press and count not equal to 7
        counter += 1					# Increase count by 1
        time.sleep(.2)					# Create .2 sec delay

    if (GPIO.input(23) == 1 and counter == 7):		# Check for left button press and count equal to 7
        counter = 0					# Set count back to 0 for wrap around
        time.sleep(.2)

    if (GPIO.input(24) == 1 and counter != 0 ):		# Check for right button press and count not equal to 0
        counter -= 1					# Decrease count by 1
        time.sleep(.2)

    if (GPIO.input(24) == 1 and counter == 0):		# Check for right button press and count equal to 0
        counter = 7					# Set count back to 7 for wrap around
        time.sleep(.2)

    return


def switchLED(index,state):

    GPIO.output(LED_pin[index], state)		# Turn LED on if binary value equals 1


    return

def binaryCounter(count):

    for index, value in enumerate(binary[count]):	# Loop through binary array

        switchLED(index,value)				# Pass LED number and state of LED to switchLED

    return


def main():

    buttonPress()		# Call buttonPress
    binaryCounter(counter)	# Call binaryCounter with global counter variable


# Only run the functions if
if __name__ == "__main__":
    # Make sure the GPIO is stopped correctly
    try:

        setup()			# Setup LEDs and buttons

        while True:
            main()		# Run main function until program is cancelled
    except KeyboardInterrupt:
        print("Exiting gracefully")
        # Turn off your GPIOs here
        GPIO.cleanup()
    except e:
        GPIO.cleanup()
        print("Some other error occurred")
        print(e.message)
