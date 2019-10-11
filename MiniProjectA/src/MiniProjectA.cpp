/*
 * MiniProjectA.cpp
 * 
 * Written by Matthew Terblanche and Michael Wetzel
 * 
 * This file is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include "MiniProjectA.h"
#include "CurrentTime.c"

#define NUM_THREADS 2

unsigned char bufferADC[3];
unsigned char bufferDAC[2];
bool monitoring = true; // should be set false when stopped
bool stopped = false; // If set to true, program should close
int hoursRTC, minsRTC, secsRTC;
int hoursSys = 0;
int minsSys = 0;
int secsSys = 0;
int freq = 1;
long lastInterruptTime = 0;	// Used for button debounce
int vOutBin[10];
bool alarmOn = false;
int alarmTimer = 200;

int RTC; //Holds the RTC instance

double humidity = 0;
int temperature = 0;
int light = 0;
double vOut = 0;


// Configure your interrupts here.
// Don't forget to use debouncing.
void stop_start_isr(void){

    //Debounce
    long interruptTime = millis();

    if (interruptTime - lastInterruptTime>200){
//        printf("Stop/Start interrupt triggered\n");

	monitor();


}
    lastInterruptTime = interruptTime;
}

void dismiss_alarm_isr(void){

    //Debounce
    long interruptTime = millis();

    if (interruptTime - lastInterruptTime>200){
  //      printf("Dismiss Alarm interrupt triggered\n");

	dismiss_alarm();

    }
    lastInterruptTime = interruptTime;
}

void reset_isr(void){

    //Debounce
    long interruptTime = millis();

    if (interruptTime - lastInterruptTime>200){
    //    printf("Reset interrupt triggered\n");

        reset();

    }
    lastInterruptTime = interruptTime;
}

void frequency_isr(void){

    //Debounce
    long interruptTime = millis();

    if (interruptTime - lastInterruptTime>200){
      //  printf("Change frequency interrupt triggered\n");

        change_frequency();

    }
    lastInterruptTime = interruptTime;
}



void monitor(void){


if (monitoring) {
        monitoring = false;
    }
    else{
        monitoring = true;
    }
}


void reset(void){

    printf("\e[1;1H\e[2J");	//Regex - Clear screen
    secsSys = 0;
    minsSys = 0;
    hoursSys = 0;
    alarmTimer = 200;
    alarmOn = false;
   // digitalWrite(ALARM_LED, 0);
    printf("----------------------------------------------------------------------------------------------\n"); 
    printf("|   RTC Time   |   Sys Timer   |   Humidity   |   Temp   |  Light  |   DAC out   |   Alarm   |\n"); 
    printf("----------------------------------------------------------------------------------------------\n"); 

   

}

void change_frequency(void){

    if (freq == 1) {
        freq = 2;
   // printf("Freq = %d\n", freq);
	return;
}
    if (freq == 2){
        freq = 5;
// printf("Freq = %d\n", freq);
       return;
}
    if (freq == 5){
        freq = 1; 
// printf("Freq = %d\n", freq);
       return;
}
}


void dismiss_alarm(void){

	alarmOn = false;

}


double humidityVoltage(double value){

    return ((3.3/1023)*value);

}


int temperatureCelsius(int value){

    double adcVoltage = ((3.3/1023)*value);

    return ((adcVoltage - 0.5) / 0.01);	// 0.5 = OFFSET, 0.01 = Temp Coefficient

}

void setAlarm(double vOut){

    if (vOut < 0.65 || vOut > 2.65){
	if (alarmTimer > 180 && (monitoring == true)) {
//        printf("\nALARM set = %f\n",vOut);
        alarmOn = true;
        alarmTimer = 0;
       // digitalWrite(ALARM_LED, 1);
 
}
    }
    


}


double dacOUT(double light, double humidityV){

    double vOut = ( (light/1023) * humidityV );
   // decToBinary(vOut);
   // bufferDAC[0] = 0b01110000 | (vOut >> 4);
   // bufferDAC[1] = vOut << 4;
   // wiringPiSPIDataRW(SPI_CHAN, bufferDAC, 2);
    // setAlarm(vOut);
     return vOut;
}

void decToBinary(int n){
    
    int binaryNum[10];
    int i = 0;
    while (i < 10){      //Fill binarynum array with zeros
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
    for (int j = 10 - 1; j >= 0; j--){
        vOutBin[i] = binaryNum[j];                  //populate display array - because one can't return an array in c
        i ++;
    } 

  
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




/*
 * Setup Function. Called once 
 */
int setup_gpio(void){
    //Set up wiring Pi
    wiringPiSetup();

    //setting up the led
    pinMode(ALARM_LED, PWM_OUTPUT);


    //setting up the buttons
    pinMode(CHANGE_FREQ_BTN, INPUT);
    pullUpDnControl(CHANGE_FREQ_BTN, PUD_UP);

    pinMode(RESET_TIME_BTN, INPUT);
    pullUpDnControl(RESET_TIME_BTN, PUD_UP);

    pinMode(STOP_START_BTN, INPUT);
    pullUpDnControl(STOP_START_BTN, PUD_UP);

    pinMode(DISMISS_ALARM_BTN, INPUT);
    pullUpDnControl(DISMISS_ALARM_BTN, PUD_UP);

    //Attach interrupts to button
    wiringPiISR(CHANGE_FREQ_BTN, INT_EDGE_FALLING, frequency_isr);
    wiringPiISR(RESET_TIME_BTN, INT_EDGE_FALLING, reset_isr);
    wiringPiISR(STOP_START_BTN, INT_EDGE_FALLING, stop_start_isr);
    wiringPiISR(DISMISS_ALARM_BTN, INT_EDGE_FALLING, dismiss_alarm_isr);
   
    //setting up the SPI interface
    wiringPiSPISetup(SPI_CHAN, SPI_SPEED);

    RTC = wiringPiI2CSetup(RTC_ADDR); //Set up the RTC

    return 0;
}


int analogReadADC(int analogChannel){


bufferADC[0] = 1;
bufferADC[1] = (8 + analogChannel) << 4;
bufferADC[2] = 0;

wiringPiSPIDataRW(SPI_CHAN, bufferADC, 3);

return (((bufferADC[1] & 3) << 8) + bufferADC[2]);
}

void fetchTime(void){

//	hoursRTC = getHours();
  //      minsRTC = getMins();
//	secsRTC = getSecs();

	hoursRTC = hexCompensation(wiringPiI2CReadReg8(RTC, HOUR)) + TIMEZONE;	//Reads HOUR value from RTC
	minsRTC = hexCompensation(wiringPiI2CReadReg8(RTC, MIN));	//Reads MIN value from RTC
	secsRTC = hexCompensation(wiringPiI2CReadReg8(RTC, SEC)-0b10000000);	//Reads SEC value from RTC


}

void sysTime(void){

    alarmTimer += freq ;
    secsSys += freq;
    if(secsSys > 59){
        minsSys += 1;
        secsSys = 0;
    }

    if(minsSys > 59){
       hoursSys += 1;
       minsSys = 0;
    }

}




 void alarmLED(void){

    for(int i=0;i<1024;i++){

        if(alarmOn){
        pwmWrite(ALARM_LED,i);
        delay(1);
    }
	else{
	pwmWrite(ALARM_LED,0);
	}
    }

//    sleep(1);
    for(int i=1023;i>=0;i--){

        if(alarmOn){
        pwmWrite(ALARM_LED,i);
        delay(1);

    }    
	else{
        pwmWrite(ALARM_LED,0);
        }
}

}

void *systemTimeThread(void *threadargs){

    while(1){

        sysTime();
        sleep(freq);
    }


    pthread_exit(NULL);
}
/* 
 * Thread that handles reading from SPI
 * 
 */
void *monitorThread(void *threadargs){


    while(1){

       while(!monitoring){

       }
    
    humidity = humidityVoltage(analogReadADC(2));
    temperature = temperatureCelsius(analogReadADC(0));
    light = analogReadADC(1);
    vOut = dacOUT(analogReadADC(1), humidityVoltage(analogReadADC(2)));
    
    alarmLED();
    

}
    pthread_exit(NULL);
}

int main(){
    // Call the setup GPIO function
	if(setup_gpio()==-1){
        printf("Setup error");
	 return 0;
    }

	signal(SIGINT, cleanUp);

    	toggleTime();



    /* Initialize thread with parameters
     */ 
    
    //Write your logic here
    pthread_attr_t tattr;
    pthread_t thread_id[NUM_THREADS];
    int newprio = 99;
    sched_param param;
    
    pthread_attr_init (&tattr);
    pthread_attr_getschedparam (&tattr, &param); /* safe to get existing scheduling param */
    param.sched_priority = newprio; /* set the priority; others are unchanged */
    pthread_attr_setschedparam (&tattr, &param); /* setting the new scheduling param */
    
    pthread_create(&thread_id[0], &tattr, monitorThread, (void *)1); /* with new priority specified */
    
    pthread_create(&thread_id[1], &tattr, systemTimeThread, (void *)1); /* with new priority specified */

	 
   printf("----------------------------------------------------------------------------------------------\n"); 
   printf("|   RTC Time   |   Sys Timer   |   Humidity   |   Temp   |  Light  |   DAC out   |   Alarm   |\n"); 
   printf("----------------------------------------------------------------------------------------------\n"); 

//You need to only be monitoring if the stopped flag is false
    while(1){


        while(!monitoring){
        }

        //Fetch the time from the RTC
        fetchTime();


        setAlarm(vOut);

        printf("| %02d:%02d:%02d     | %02d:%02d:%02d      | %-3.1f V        | %-2d C     | %-3d     | %-3.2f V      |     %1s     |\n", hoursRTC, minsRTC, secsRTC,hoursSys, minsSys, secsSys, humidity, temperature , light, vOut, (alarmOn == true) ? "*":" ");
        printf("----------------------------------------------------------------------------------------------\n");

        
        sleep(freq);


} 
    pthread_exit(NULL);



    return 0;
}

void cleanUp(int signal) {

	printf("\nProgram Ended\n");
 
	pinMode(ALARM_LED, 0);		//Sets seconds LED to input
	exit(0);	//Exits program

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
