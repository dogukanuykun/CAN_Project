

#include "stm32f4xx_hal.h"

void HAL_MspInit(void)
{
  //Here will do low level processor specific inits.
	//1. Set up the priority grouping of the arm cortex mx processor
	HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
	//2.enable the required system exceptions of the arm cortex mx processor
	SCB->SHCSR |= (0x7 << 16); //enable memfault, busfault and usgfault system exceptions

	//3. configure the priority for the system exceptions
	HAL_NVIC_SetPriority(MemoryManagement_IRQn, 0, 0);
	HAL_NVIC_SetPriority(BusFault_IRQn, 0, 0);
	HAL_NVIC_SetPriority(UsageFault_IRQn, 0, 0);

}


void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
	//low level inits. of the USART2 peripheral
	GPIO_InitTypeDef gpio_uart;
	//1.enable the clock for the USART2 peripheral as well as GPIOA peripheral
	__HAL_RCC_USART2_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	//pin muxing conf.
	gpio_uart.Pin = GPIO_PIN_2;
	gpio_uart.Mode = GPIO_MODE_AF_PP;
	gpio_uart.Pull = GPIO_PULLUP;
	gpio_uart.Speed = GPIO_SPEED_FREQ_LOW;
	gpio_uart.Alternate = GPIO_AF7_USART2; //UART2_TX
	HAL_GPIO_Init(GPIOA, &gpio_uart);

	gpio_uart.Pin = GPIO_PIN_3; //UART2_RX
	HAL_GPIO_Init(GPIOA, &gpio_uart);

	//enable the IRQ and set up the priority(NVIC settings)
	HAL_NVIC_EnableIRQ(USART2_IRQn);
	HAL_NVIC_SetPriority(USART2_IRQn, 15, 0);

}
