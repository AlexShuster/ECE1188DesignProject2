/*
 * racingBot.c
 *
 *  Created on: Apr 24, 2023
 *      Author: jsb19
 */

#include <stdint.h>
#include <math.h>
#include "msp.h"
#include "pidController.h"
#include "distanceDriver.h"
#include "odometry.h"

int targetSpeed = 12000;
int leftSpeed = 0, rightSpeed = 0;

void setCenterSpeed(uint32_t *distances)
{
    int newSpeed = targetSpeed;
    if(distances[1] < 500)
        newSpeed *= distances[1] / 500.0;
    if(distances[0] > 750 && distances[2] > 1000 && distances[0] < 1200)
    {
        setMotorSpeed(3000, 0);
        Clock_Delay1ms(200);
        return;
    }
    leftSpeed = newSpeed * sin((float)distances[2] * 90.0 / 1500.0 * 0.0174533);
    rightSpeed = newSpeed * sin((float)distances[0] * 90.0 / 1500.0 * 0.0174533);

    rightSpeed *= 1.1;
}

int main()
{
    uint32_t distances[3];
    int32_t x, y, theta;
    Clock_Init48MHz();
    motorPWMInit(15000, 0, 0);
    EnableInterrupts();
    initDistanceDriver();
    Odometry_Init(0, 0, 0);
    while(1)
    {
        getDistances(distances);
        setCenterSpeed(distances);
        UpdatePosition();
        Odometry_Get(&x, &y, &theta);
        setMotorSpeed(leftSpeed, rightSpeed);
        Clock_Delay1ms(25);
    }

}

