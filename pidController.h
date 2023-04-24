/*
 * pidController.h
 *
 *  Created on: Apr 24, 2023
 *      Author: jsb19
 */

#ifndef PIDCONTROLLER_H_
#define PIDCONTROLLER_H_

#include "motorDriver.h"
#include "Clock.h"
#include "CortexM.h"
#include "TimerA1.h"
#include "Tachometer.h"

void initPIDMotorControl(int leftDesired, int rightDesired);
void setMotorSpeedDesired(int leftSpeed, int rightSpeed);
void updateController(void);

void setK(float Kp, float Ki, float Kd);
float getKp(void);
float getKi(void);
float getKd(void);

#endif /* PIDCONTROLLER_H_ */
