/*
 * tim_setup.c
 *
 *  Created on: Mar 18, 2025
 *      Author: avery
 */

#include "tim_setup.h"

TIM_HandleTypeDef htim2;    // RPM Calculation Timer
TIM_HandleTypeDef htim6;    // Wheel Stop Timeout Timer

/**
 * @brief Initialize the RPM Calculation Timer (TIM2)
 * Uses the internal clock and free-runs for uptime tracking.
 * TIM2 is used to measure the time difference between consecutive
 * Hall effect sensor interrupts to calculate RPM.
 */
void MX_TIM2_Init(void)
{
    __HAL_RCC_TIM2_CLK_ENABLE();

    /* Configure TIM2 */
    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 64000 - 1;             // 64 MHz / 64000 = 1 kHz (1 ms steps)
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 0xFFFFFFFF;               // Max period (32-bit counter)
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

    if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
    {
        Error_Handler();
    }

    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;

    if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
    {
        Error_Handler();
    }

    /* Enable TIM2 interrupt */
    HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);
}

/**
 * @brief Initialize the Wheel Stop Timeout Timer (TIM6)
 * TIM6 is used to monitor the wheel's motion and set RPM to 0
 * if no signal is received within the timeout period.
 */
void MX_TIM6_Init(void)
{
    __HAL_RCC_TIM6_CLK_ENABLE();

    htim6.Instance = TIM6;
    htim6.Init.Prescaler = 64000 - 1;             // 64 MHz / 64000 = 1 kHz (1 ms steps)
    htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim6.Init.Period = 3000 - 1;                  // Timeout at 3 seconds (3000 ms)
    htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

    if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
    {
        Error_Handler();
    }

    /* Enable TIM6 interrupt */
    HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
}

/**
 * @brief TIM2 interrupt request handler.
 * Handles the capture of time for RPM calculation based on
 * Hall effect sensor signal interrupts.
 */
void TIM2_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&htim2);
}

/**
 * @brief TIM6 interrupt request handler.
 * Handles the timeout event to set RPM to 0 when the wheel stops moving.
 */
void TIM6_DAC_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&htim6);
}
