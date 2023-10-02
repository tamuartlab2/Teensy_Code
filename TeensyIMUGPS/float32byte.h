/*  File Name: float32byte.h
 *  Author: Yuan Wei
 *  Date: July 13, 2021
 *  Description: head file of float32byte
 *  Last Changed: Dec 27, 2021 by Yuan Wei
 */

#ifndef FLOAT32BYTE_H
#define FLOAT32BYTE_H

#include "Arduino.h"

void float2Byte(float val, uint8_t* byteArr);
float byte2Float(uint8_t* byteArr);
void double2Byte(double val, uint8_t* byteArr);
double byte2Double(uint8_t* byteArr);

#endif
