/*  File Name: artlab.cpp
 *  Author: Yuan Wei
 *  Date: July 14, 2021
 *  Description: This program consists of some useful functions
 *  Last Changed: July 28, 2021 by Yuan Wei
 */

#include "artlab.h"

float arraySum(float* Arr, uint16_t arraySize)
{
    float sum = 0;
    for (int i = 0; i < arraySize; i++)
    {
        sum = sum + Arr[i];
    }
    return sum;
}

uint16_t maxNumberofArray(float* Arr, uint16_t arraySize)
{
    uint16_t num = 0;
    float maxN = Arr[0];
    for (uint16_t i = 1; i < arraySize; i++)
    {
        if (maxN < Arr[i])
        {
            maxN = Arr[i];
            num = i;
        }
    }
    return num;
}

void normalize(float* Arr, uint16_t arraySize)
{
    float sum = arraySum(Arr, arraySize);
    for (uint16_t i = 0; i < arraySize; i++)
    {
        Arr[i] = Arr[i] / sum;
    }
}

float findDecisiveness(float* Arr, uint16_t arraySize)
{
    float Hk = 0;
    float omega = 1;
    for (uint16_t i = 0; i < arraySize; i++)
    {
        if ((Arr[i] == 0) || (Arr[i] == 1)){}
        else 
        {
            Hk = Hk - Arr[i] * log_2(Arr[i]);
        }
    }
    omega = 1 / (1 + Hk);
    return omega;
}

float log_2(float para)
{
    float value = log(para) / log(2);
    return value;
}

void randomArrayGenerator(float *Arr, uint16_t arraySize, long minNumber, long maxNumber)
{
    for (uint16_t i = 0; i < arraySize; i++)
    {
        Arr[i] = random(minNumber, maxNumber);
    }
}

void clearArray(float* Arr, uint16_t arraySize)
{
    for (uint16_t i = 0; i < arraySize; i++)
    {
        Arr[i] = 0;
    }
}