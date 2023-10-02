/*  File Name: artlab.h
 *  Author: Yuan Wei
 *  Date: July 14, 2021
 *  Description: head file of artlab.cpp, which consists of some useful functions
 *  Last Changed: July 28, 2021 by Yuan Wei
 */

#ifndef ARTLAB_H
#define ARTLAB_H

#include "Arduino.h"

float arraySum(float* Arr, uint16_t arraySize);
uint16_t maxNumberofArray(float *Arr, uint16_t arraySize);
void normalize(float* Arr, uint16_t arraySize);
float findDecisiveness(float* Arr, uint16_t arraySize);
float log_2(float para);
void randomArrayGenerator(float *Arr, uint16_t arraySize, long minNumber, long maxNumber);
void clearArray(float* Arr, uint16_t arraySize);

#endif
 
