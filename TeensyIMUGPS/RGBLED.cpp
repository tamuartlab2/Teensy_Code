/*  File Name: RGBLED.cpp
 *  Author: Yuan Wei
 *  Date: Oct 13, 2021
 *  Description: head file of RGBLED.cpp
 *  Last Changed: Oct 13, 2021 by Yuan Wei
 */

#include "RGBLED.h"

RGBLED::RGBLED(uint8_t R_pin, uint8_t G_pin, uint8_t B_pin)
{
    red_pin = R_pin;
    green_pin = G_pin;
    blue_pin = B_pin;
}

void RGBLED::setColor(uint8_t R_color, uint8_t G_color, uint8_t B_color)
{
    analogWrite(red_pin, R_color);
    analogWrite(green_pin, G_color);
    analogWrite(blue_pin, B_color);
}
