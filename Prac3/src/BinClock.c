/*
 * BinClock.c
 * Jarrod Olivier
 * Modified for EEE3095S/3096S by Keegan Crankshaw
 * August 2019
 * 
 * <TRBMAT002> <WTZMIC001>
 * 19 August 2019
*/

#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <stdio.h> //For printf functions
#include <stdlib.h> // For system functions
#include <stdbool.h>
#include <signal.h> // For keyboard interrupt

#include "BinClock.h"
#include "CurrentTime.h"


//Global variables
int hours, mins, secs, numOfPlaces;
long lastInterruptTime = 0; //Used for button debounce
int RTC; //Holds the RTC instance
int display[6] = { 0, 0, 0, 0, 0, 0 };
int HH,MM,SS;

void initGPIO(void){
	/* 
	 * Sets GPIO using wiringPi pins. see pinout.xyz for specific wiringPi pins
	 * You can also use "gpio readall" in the command line to get the pins
	 * Note: wiringPi does not use GPIO or board pin numbers (unless specifically set to that mode)
	 */
	printf("Setting up\n");
	wiringPiSetup(); //This is the default mode. If you want to change pinouts, be aware
	
	RTC = wiringPiI2CSetup(RTCAddr); //Set up the RTC
	
	//Set up the LEDS
	for(int i=0; i < sizeof(LEDS)/sizeof(LEDS[0]); i++){
	    pinMode(LEDS[i], OUTPUT);
	}
	
	//Set Up the Seconds LED for PWM
	//Write your logic here
	pinMode(SEC, PWM_OUTPUT);
	
	printf("LEDS done\n");

	
	//Set up the Buttons
	for(int j=0; j < sizeof(BTNS)/sizeof(BTNS[0]); j++){
		pinMode(BTNS[j], INPUT);
		pullUpDnControl(BTNS[j], PUD_UP);
	}

	
	//Attach interrupts to Buttons
	//Write your logic here
	
	wiringPiISR(BTNS[0], INT_EDGE_RISING, hourInc);
   	wiringPiISR(BTNS[1], INT_EDGE_RISING, minInc);
	
	printf("BTNS done\n");
	printf("Setup done\n");
	
	}


/*
 * The main function
 * This function is called, and calls all relevant functions we've written
 */
int main(void){
	initGPIO();

	signal(SIGINT, cleanUp);
	//Set random time (3:04PM)
	//You can comment this file out later
	wiringPiI2CWriteReg8(RTC, HOUR, 0x2+TIMEZONE);
	wiringPiI2CWriteReg8(RTC, MIN, 0x4);
	wiringPiI2CWriteReg8(RTC, SEC, 0x00);

 	printf("Start");

	// Repeat this until we shut down
	for (;;){
		//Fetch the time from the RTC
		fetchTime();

        	//Function calls to toggle LEDS
		lightHours(hours);
		lightMins(mins);
		secPWM(secs);
		// Print out the time we have stored on our RTC
		printf("The current time is: %x:%x:%x\n", hours, mins, secs);

		//using a delay to make our program "less CPU hungry"
		delay(1000); //milliseconds
	}
	return 0;
}

void cleanUp(int signal) {

	printf("Program Ended");
	for(int i=0; i < sizeof(LEDS)/sizeof(LEDS[0]); i++){
            digitalWrite(LEDS[i], 0);	//Turns LEDs off
        }
	 for(int i=0; i < sizeof(LEDS)/sizeof(LEDS[0]); i++){
            pinMode(LEDS[i], INPUT);	//Sets LEDs to input
        } 
	pinMode(SECS, INPUT);		//Sets seconds LED to input
	exit(1);	//Exits program

}


void fetchTime(void){

	hours = wiringPiI2CReadReg8(RTC, HOUR);	//Reads HOUR value from RTC
	mins = wiringPiI2CReadReg8(RTC, MIN);	//Reads MIN value from RTC
	secs = wiringPiI2CReadReg8(RTC, SEC);	//Reads SEC value from RTC

    
}

/*
 * Change the hour format to 12 hours
 */
int hFormat(int hours){
	/*formats to 12h*/
	if (hours >= 24){
		hours = 0;
	}
	else if (hours > 12){
		hours -= 12;
	}
	return (int)hours;
}


void decToBinary(int n){
    
    int binaryNum[numOfPlaces];
    int i = 0;
    while (i < numOfPlaces){      //Fill binarynum array with zeros
        binaryNum[i] = 0;
        i ++;
    }
    i = 0;
    while (n > 0) {
        binaryNum[i] = n%2; //convert n to binary
        n = n/ 2;
        i ++;
    }
    
    i = 0;
    for (int j = numOfPlaces - 1; j >= 0; j--){
        display[i] = binaryNum[j];                  //populate display array - because one can't return an array in c
        i ++;
    } 

    printf("\n");
}

/*
 *	Turns on corresponding LED's for hours
 */

void lightHours(int units){
	// Write your logic to light up the hour LEDs here	
    numOfPlaces = 4;
	int h = 0;
	h = hexCompensation(units);
	h = hFormat(h);                 //turn hours into right format
    decToBinary(h);
    for (int i = 0; i < numOfPlaces; i++){
        if (display[i] == 1){
            printf("H1");          //display on LED's
            digitalWrite(LEDS[i], 1);
        } else {
            printf("H0");
            digitalWrite(LEDS[i], 0);
        }
    }

}


/*
 * Turn on the Minute LEDs
 */

void lightMins(int units){
    numOfPlaces = 6;
	int m = 0;
	m = hexCompensation(units); 
    decToBinary(m);
    for (int i = 0; i < numOfPlaces; i++){
        if (display[i] == 1){
            printf("m1");
            digitalWrite(LEDS[4+i],1);
        } else {
            printf("m0");
            digitalWrite(LEDS[4+i],0);
        }
    }
}

/*
 * PWM on the Seconds LED
 * The LED should have 60 brightness levels
 * The LED should be "off" at 0 seconds, and fully bright at 59 seconds
 */
void secPWM(int units){
    printf("Sec works");
    int sec = units;
	int brightness = (sec/60) * 1024;
    pwmWrite(SEC, brightness);
}

/*
 * hexCompensation
 * This function may not be necessary if you use bit-shifting rather than decimal checking for writing out time values
 */
int hexCompensation(int units){
	/*Convert HEX or BCD value to DEC where 0x45 == 0d45 
	  This was created as the lighXXX functions which determine what GPIO pin to set HIGH/LOW
	  perform operations which work in base10 and not base16 (incorrect logic) 
	*/
	int unitsU = units%0x10;

	if (units >= 0x50){
		units = 50 + unitsU;
	}
	else if (units >= 0x40){
		units = 40 + unitsU;
	}
	else if (units >= 0x30){
		units = 30 + unitsU;
	}
	else if (units >= 0x20){
		units = 20 + unitsU;
	}
	else if (units >= 0x10){
		units = 10 + unitsU;
	}
	return units;
}


/*
 * decCompensation
 * This function "undoes" hexCompensation in order to write the correct base 16 value through I2C
 */
int decCompensation(int units){
	int unitsU = units%10;

	if (units >= 50){
		units = 0x50 + unitsU;
	}
	else if (units >= 40){
		units = 0x40 + unitsU;
	}
	else if (units >= 30){
		units = 0x30 + unitsU;
	}
	else if (units >= 20){
		units = 0x20 + unitsU;
	}
	else if (units >= 10){
		units = 0x10 + unitsU;
	}
	return units;
}


/*  
 * hourInc
 * Fetch the hour value off the RTC, increase it by 1, and write back
 * Be sure to cater for there only being 23 hours in a day
 * Software Debouncing should be used
 */
void hourInc(void){
	//Debounce
   	long interruptTime = millis();

	if (interruptTime - lastInterruptTime>200){
		printf("Interrupt 1 triggered, %x\n", hours);
		//Fetch RTC Time
		fetchTime();
        
		//Increase hours by 1, ensuring not to overflow
		if (hours < 0x17){	//Ensures hours doesn't go above 23
			hours = hours + 0x01;
      		}
		else {
			hours = 0x00;
       		} 

		//Write hours back to the RTC
       
		wiringPiI2CWriteReg8(RTC, HOUR, hours);
  	}
	lastInterruptTime = interruptTime;
}

/* 
 * minInc
 * Fetch the minute value off the RTC, increase it by 1, and write back
 * Be sure to cater for there only being 60 minutes in an hour
 * Software Debouncing should be used
 */
void minInc(void){
	long interruptTime = millis();

	if (interruptTime - lastInterruptTime>200){
		printf("Interrupt 2 triggered, %x\n", mins);
		//Fetch RTC Time
		fetchTime();
		//Increase minutes by 1, ensuring not to overflow
		if (mins < 0x3C){	//Ensures minutes doesn't go above 60
                        mins += 0x01;
                }
                else {
                        mins = 0x00;
                } 
                //Write minutes back to the RTC
                wiringPiI2CWriteReg8(RTC, MIN, mins);
	}
	lastInterruptTime = interruptTime;
}

//This interrupt will fetch current time from another script and write it to the clock registers
//This functions will toggle a flag that is checked in main
void toggleTime(void){
	long interruptTime = millis();

	if (interruptTime - lastInterruptTime>200){
		HH = getHours();
		MM = getMins();
		SS = getSecs();

		HH = hFormat(HH);
		HH = decCompensation(HH);
		wiringPiI2CWriteReg8(RTC, HOUR, HH);

		MM = decCompensation(MM);
		wiringPiI2CWriteReg8(RTC, MIN, MM);

		SS = decCompensation(SS);
		wiringPiI2CWriteReg8(RTC, SEC, 0b10000000+SS);

	}
	lastInterruptTime = interruptTime;
}
