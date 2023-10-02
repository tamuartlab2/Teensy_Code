/*  File Name: float32byte.cpp
 *  Author: Yuan Wei
 *  Date: July 13, 2021
 *  Description: transfer a float32 number to 4 byte and vice versa
 *  Last Changed: Dec 27, 2021 by Yuan Wei
 */

#include "float32byte.h"

void float2Byte(float val, uint8_t* byteArr)
{
    // define union
    union
    {
        float floatVal;
        uint8_t byteArr[4];
    }uFloatByteArr;

    uFloatByteArr.floatVal = val;
    memcpy(byteArr, uFloatByteArr.byteArr, 4);
}

float byte2Float(uint8_t* byteArr)
{
    // define union
    union
    {
        float floatVal;
        uint8_t byteArr[4];
    }uFloatByteArr;
    
    memcpy(uFloatByteArr.byteArr, byteArr, 4);
    return uFloatByteArr.floatVal;
}

void double2Byte(double val, uint8_t* byteArr)
{
    // define union
    union
    {
        double doubleVal;
        uint8_t byteArr[8];
    }uDoubleByteArr;

    uDoubleByteArr.doubleVal = val;
    memcpy(byteArr, uDoubleByteArr.byteArr, 8);
}

double byte2Double(uint8_t* byteArr)
{
    // define union
    union
    {
        double DoubleVal;
        uint8_t byteArr[4];
    }uDoubleByteArr;
    
    memcpy(uDoubleByteArr.byteArr, byteArr, 8);
    return uDoubleByteArr.DoubleVal;
}