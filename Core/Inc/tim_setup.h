/*
 * tim_setup.h
 *
 *  Created on: Mar 18, 2025
 *      Author: avery
 */

#ifndef INC_TIM_SETUP_H_
#define INC_TIM_SETUP_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim6;

// Initialization functions
void MX_TIM2_Init(void);
void MX_TIM6_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* INC_TIM_SETUP_H_ */
