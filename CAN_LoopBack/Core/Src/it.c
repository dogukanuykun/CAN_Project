/*
 * it.c
 *
 *  Created on: Mar 22, 2024
 *      Author: uykun
 */

#include "main_app.h"


void SysTick_Handler (void)
{
	HAL_IncTick();
	HAL_SYSTICK_IRQHandler();
}

