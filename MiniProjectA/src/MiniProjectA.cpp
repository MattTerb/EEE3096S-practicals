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
 * Setup Function. Called once 
 */
int setup_gpio(void){
    //Set up wiring Pi
    wiringPiSetup();

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

   // RTC = wiringPiI2CSetup(RTC_ADDR); //Set up the RTC

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

	hoursRTC = getHours();
        minsRTC = getMins();
	secsRTC = getSecs();

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


/* 
 * Thread that handles reading from SPI
 * 
 */
void *monitorThread(void *threadargs){


	printf("----------------------------------------------------------------------------------------------\n"); 
	printf("|   RTC Time   |   Sys Timer   |   Humidity   |   Temp   |  Light  |   DAC out   |   Alarm   |\n"); 
 	printf("----------------------------------------------------------------------------------------------\n");   

 //You need to only be monitoring if the stopped flag is false
    while(!stopped){
       //Code to suspend playing if paused
//	while (!monitoring){
//	    sysTime();
  //          sleep(freq);
    //    }

//Fetch the time from the RTC
	fetchTime();

        setAlarm(dacOUT(analogReadADC(1), humidityVoltage(analogReadADC(2))));

	printf("| %02d:%02d:%02d     | %02d:%02d:%02d      | %-3.1f V        | %-2d C     | %-3d     | %-3.2f V      |     %1s     |\n", hoursRTC, minsRTC, secsRTC,hoursSys, minsSys, secsSys, (monitoring == true) ? humidityVoltage(analogReadADC(2)): 0, (monitoring == true) ? temperatureCelsius(analogReadADC(0)) : 0 , (monitoring == true) ? analogReadADC(1) : 0, 
(monitoring == true) ? dacOUT(analogReadADC(1), humidityVoltage(analogReadADC(2))) : 0, (alarmOn == true) ? "*":" ");
	printf("----------------------------------------------------------------------------------------------\n");

        sysTime();
	sleep(freq);
       // setAlarm(dacOUT(analogReadADC(1), humidityVoltage(analogReadADC(2))));
}

    
    
    pthread_exit(NULL);
}

int main(){
    // Call the setup GPIO function
	if(setup_gpio()==-1){
        printf("Setup error");
	 return 0;
    }



    /* Initialize thread with parameters
     */ 
    
    //Write your logic here
    pthread_attr_t tattr;
    pthread_t thread_id;
    int newprio = 99;
    sched_param param;
    
    pthread_attr_init (&tattr);
    pthread_attr_getschedparam (&tattr, &param); /* safe to get existing scheduling param */
    param.sched_priority = newprio; /* set the priority; others are unchanged */
    pthread_attr_setschedparam (&tattr, &param); /* setting the new scheduling param */
    pthread_create(&thread_id, &tattr, monitorThread, (void *)1); /* with new priority specified */
    
	 
    //Join and exit the playthread
    pthread_join(thread_id, NULL); 
    pthread_exit(NULL);



    return 0;
}

