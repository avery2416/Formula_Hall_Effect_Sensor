/*
 * hallEffectSensor.c
 *
 *  Created on: Mar 11, 2025
 *      Author: Avery Robertson
 */

#include <stdio.h>
#include <string.h>
#include "hallEffectSensor.h"

// Initializes the hallEffectSensor Structure
HAL_StatusTypeDef hallEffectInit(hallEffectSensor *_hf,
								 float _wheel_radius,
								 float _brake_radius,
								 uint16_t _brake_disc_gaps,
								 TIM_HandleTypeDef *_counter,
								 TIM_HandleTypeDef *_timeOutTimer,
								 float _tuner,
								 uint16_t _interruptPin){
	_hf->isReady = 0;
	_hf->brake_radius = _brake_radius;
	_hf->wheel_radius = _wheel_radius;
	_hf->brake_disc_gaps = _brake_disc_gaps;
	_hf->counter = _counter;
	_hf->timeOutTimer = _timeOutTimer;
	_hf->tuner = _tuner;
	_hf->clock_ratio = HAL_RCC_GetHCLKFreq()/(_hf->counter->Init.Prescaler+1);
	_hf->interruptPin = _interruptPin;
	return HAL_TIM_Base_Start_IT(_hf->timeOutTimer);
}

void hallEffectCalculator(hallEffectSensor *_hf, UART_HandleTypeDef *huart){
	if(_hf->isReady==1){
		_hf->isReady = 0;
		_hf->time_current = __HAL_TIM_GET_COUNTER(_hf->counter);
		uint32_t delta_time = _hf->time_current - _hf->time_previous;
		_hf->rpm = (RPM_CONVERSION * _hf->clock_ratio) / (delta_time * _hf->brake_disc_gaps);
		float wheel_angular_velocity = ((2 * PI * _hf->rpm) / 60.0) * (_hf->brake_radius / _hf->wheel_radius);
		_hf->wheel_speed = wheel_angular_velocity * _hf->wheel_radius;
		_hf->time_previous = _hf->time_current;
		_hf->timeOutTimer->Instance->CNT = 0;

		// Add UART transmission here
		uint8_t MSG[100] = {'\0'};

		// ✅ Use %u for uint16_t RPM and %lu for uint32_t delta_time
		sprintf((char *)MSG, "RPM: %u, Wheel Speed: %.2f m/s, Delta Time: %lu\r\n",
		        (uint16_t)_hf->rpm,                // Cast to uint16_t
		        (float)_hf->wheel_speed,           // Cast to float
		        (uint32_t)delta_time);             // Cast to uint32_t

		HAL_UART_Transmit(huart, MSG, strlen((char *)MSG), 100);
	}
	return;
}

