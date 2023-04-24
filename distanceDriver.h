/*
 * distanceDriver.h
 *
 *  Created on: Apr 24, 2023
 *      Author: jsb19
 */

#ifndef DISTANCEDRIVER_H_
#define DISTANCEDRIVER_H_

#include <stdint.h>
#include "msp.h"
#include "Clock.h"
#include "I2CB1.h"
#include "CortexM.h"
#include "opt3101.h"
#include "LPF.h"

void initDistanceDriver(void);
void getDistances(uint32_t *distanceBuf);


#endif /* DISTANCEDRIVER_H_ */
