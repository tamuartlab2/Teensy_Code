/*
	Name: main.h
	Created: 11/28/2021
	Author:	Yuan Wei
	Last Modified: 10/14/2022
*/

#ifndef MAIN_H
#define MAIN_H

#include <arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Adafruit_GPS.h>
#include <XBee.h>
#include <EEPROM.h>
#include "float32byte.h"
#include "RGBLED.h"
#include "artlab.h"

#define GPSSerial Serial1
#define IMUPeriod 10000  //microseconds
#define GPSPeriod 1000  //microseconds

void IMUGPSISR();
void GPSDATAISR();
void bnoIMU();
void adafruitGPS();
double dm_dd(double loc, char dir);
void sendMSG();

struct imu_data_structure 
{
    uint8_t sensorType;
    uint8_t sys;
    uint8_t gyro;
    uint8_t accel;
    float quat_x;
    float quat_y;
    float quat_z;
    float quat_w;
    float Gyroscope_x;
    float Gyroscope_y;
    float Gyroscope_z;
    float LinearAccel_x;
    float LinearAccel_y;
    float LinearAccel_z;
    float compass;    //ENU
};

#endif
