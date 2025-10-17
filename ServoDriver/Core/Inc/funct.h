#ifndef FUNCT_H_
#define FUNCT_H_

#include "main.h"
#include "stm32h7xx_hal.h"

extern uint32_t minPulse;
extern uint32_t maxPulse;

void servoAngle(uint32_t angle);
void calculate_pwm_bounds(uint32_t psc, uint32_t arr);

#endif
