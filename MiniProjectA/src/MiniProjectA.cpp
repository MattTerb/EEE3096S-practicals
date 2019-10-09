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

using namespace std;

bool monitoring = true; // should be set false when stopped
bool stopped = false; // If set to true, program should close
unsigned char buffer[3] = {1};
int buffer_location = 0;
bool bufferReading = 0; //using this to switch between column 0 and 1 - the first column
bool threadReady = false; //using this to finish writing the first column at the start of the song, before the column is played

long lastInterruptTime = 0;	// Used for button debounce


// Configure your interrupts here.
// Don't forget to use debouncing.
void stop_start_isr(void){

    //Debounce
    long interruptTime = millis();

    if (interruptTime - lastInterruptTime>200){
        printf("Stop/Start interrupt triggered");

	monitor();


}
    lastInterruptTime = interruptTime;
}

void dismiss_alarm_isr(void){

    //Debounce
    long interruptTime = millis();

    if (interruptTime - lastInterruptTime>200){
        printf("Dismiss Alarm interrupt triggered");

	dismiss_alarm();

    }
    lastInterruptTime = interruptTime;
}

void reset_isr(void){

    //Debounce
    long interruptTime = millis();

    if (interruptTime - lastInterruptTime>200){
        printf("Reset interrupt triggered");

        reset();

    }
    lastInterruptTime = interruptTime;
}

void frequency_isr(void){

    //Debounce
    long interruptTime = millis();

    if (interruptTime - lastInterruptTime>200){
        printf("Change frequency interrupt triggered");

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




   

}

void change_frequency(void){




    

}


void dismiss_alarm(void){




    

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

 return 0;
}

/* 
 * Thread that handles reading from SPI
 * 
 */
void *monitorThread(void *threadargs){
    // If the thread isn't ready, don't do anything
    while(!threadReady)
        continue;

    //You need to only be monitoring if the stopped flag is false
    while(!stopped){
        //Code to suspend playing if paused
	while (!monitoring){
//	    printf("Paused");

        }


	buffer[1] = (8+0) << 4;
        //Write the buffer to the ADC
	wiringPiSPIDataRW(SPI_CHAN, buffer, 3);

	printf("CH0 Result  %d ", ((buffer[1] & 3) << 8 )+buffer[2]);


}

    }
    
    mthread_exit(NULL);
}

int main(){
    // Call the setup GPIO function
	if(setup_gpio()==-1){
        return 0;
    }
    
    /* Initialize thread with parameters
     */ 
    
    //Write your logic here
    mthread_attr_t tattr;
    mthread_t thread_id;
    int newprio = 99;
    sched_param param;
    
    mthread_attr_init (&tattr);
    mthread_attr_getschedparam (&tattr, &param); /* safe to get existing scheduling param */
    param.sched_priority = newprio; /* set the priority; others are unchanged */
    mthread_attr_setschedparam (&tattr, &param); /* setting the new scheduling param */
    mthread_create(&thread_id, &tattr, monitorThread, (void *)1); /* with new priority specified */
    
	 
    //Join and exit the playthread
    mthread_join(thread_id, NULL); 
    mthread_exit(NULL);
	
    return 0;
}

