#include "funct.h"
uint32_t minPulse = 0;
uint32_t maxPulse = 0;

void servoAngle(uint32_t angle) {
	if (angle > 180)
		angle = 180;
	uint32_t pulse = minPulse + (angle * (maxPulse - minPulse)) / 180;
	TIM1->CCR1 = pulse;
}

void calculate_pwm_bounds(uint32_t psc, uint32_t arr) {
	uint32_t timer_clk;
	if (psc > 1) {
		timer_clk = HAL_RCC_GetPCLK2Freq() * 2;
	} else {
		timer_clk = HAL_RCC_GetPCLK2Freq();
	}
	float tick_freq = (float) timer_clk / (psc + 1);
	float ticks_per_ms = tick_freq / 1000;
//	float pwm_period = arr * tick_freq;

	minPulse = (uint32_t) (1.0f * ticks_per_ms);
	maxPulse = (uint32_t) (2.0f * ticks_per_ms);
}
