/*  File Name: RGBLED.h
 *  Author: Yuan Wei
 *  Date: Oct 13, 2021
 *  Description: head file of RGBLED.cpp
 *  Last Changed: Oct 13, 2021 by Yuan Wei
 */

#ifndef RGBLED_H
#define RGBLED_H

#include "Arduino.h"

class RGBLED
{
    public:
        uint8_t red_pin = 0;
        uint8_t green_pin = 1;
        uint8_t blue_pin = 2;
        RGBLED(uint8_t R_pin, uint8_t G_pin, uint8_t B_pin);
        void setColor(uint8_t R_color, uint8_t G_color, uint8_t B_color);
};

#endif