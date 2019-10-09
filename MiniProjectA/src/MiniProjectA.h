/*
 * MiniProjectA.cpp
 *
 * Written for MiniProjectA - EEE3096S 2019 by Matthew Terblanche and Michael Wetzel
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

#ifndef MINIPROJECTA_H
#define MINIPROJECTA_H

//Includes
#include <wiringPi.h>
#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include <wiringPiSPI.h>
#include <unistd.h>
#include <math.h>
#include <pthread.h>
#include <iostream>

//Define buttons
#define CHANGE_INTERVAL 0	//BCM 17
#define RESET_TIME 2	//BCM 27
#define STOP_START 3	//BCM 22
#define DISMISS_ALARM 21	//BCM 5

//Define LED
#define ALARM_LED 1       //BCM 18

//SPI Settings
#define SPI_CHAN 0	//CEO
#define SPI_SPEED 409600	// Hz

//Define constants
const char RTC_ADDR = 0x6f;
const char SEC = 0x00;
const char MIN = 0x01;
const char HOUR = 0x02;
const char TIMEZONE = 2; // +02H00 (RSA)

//Function definitions
void play_audio(void);
void stop_audio(void);
int setup_gpio(void);
int main(void);


#endif
