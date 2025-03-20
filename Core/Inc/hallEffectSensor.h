/*
 * hallEffectSensor.c
 *
 *  Created on: Mar 11, 2025
 *      Author: Avery Robertson
 */

#ifndef HALLEFFECTSENSOR_H
#define HALLEFFECTSENSOR_H

#include "stm32f3xx_hal.h"

#define PI 3.141592
#define RPM_CONVERSION 60.0

// Hall Effect Sensor data structure
typedef struct hallEffectSensor{
	TIM_HandleTypeDef *counter;
	TIM_HandleTypeDef *timeOutTimer;
	uint32_t time_current;
	uint32_t time_previous;
	uint16_t brake_disc_gaps;
	uint16_t rpm;
	float wheel_speed;
	float wheel_radius;
	float brake_radius;
	float tuner;
	uint16_t clock_ratio;
	uint8_t isReady;
	uint16_t interruptPin;
}hallEffectSensor;

HAL_StatusTypeDef hallEffectInit(hallEffectSensor *_hf,
								 float _wheel_radius,
								 float _brake_radius,
								 uint16_t brake_disc_gaps,
								 TIM_HandleTypeDef *_counter,
								 TIM_HandleTypeDef *_timeOutTimer,
								 float _tuner,
								 uint16_t _interruptPin);

void hallEffectCalculator(hallEffectSensor *_hf, UART_HandleTypeDef *huart);

#endif
