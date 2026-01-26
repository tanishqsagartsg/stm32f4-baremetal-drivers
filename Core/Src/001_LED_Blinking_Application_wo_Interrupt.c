/*
 * 001_LED_Blinking_Application_wo_Interrupt.c
 *
 *  Created on: Jan 27, 2026
 *      Author: tanis
 */


#include "stm32f407xx.h"
#include "stm32f407xx_gpio_driver.h"
#include <string.h>

#define HIGH 1
#define LOW  0

void delay(void)
{
	for(uint32_t i = 0; i <= 500000/2; i++);
}

int main(void)
{
	GPIO_Handle_t GpioLed, GPIOBtn;

	memset(&GpioLed, 0, sizeof(GpioLed));
	memset(&GPIOBtn, 0, sizeof(GPIOBtn));

	/************** LED CONFIGURATION ****************/
	GpioLed.pGPIOx = GPIOD;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PIN_PUPD;

	GPIO_PeriClockControl(GPIOD, EN);
	GPIO_Init(&GpioLed);

	/************** BUTTON CONFIGURATION **************/
	GPIOBtn.pGPIOx = GPIOD;
	GPIOBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;   // INPUT MODE
	GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU; // Pull-up

	GPIO_PeriClockControl(GPIOD, EN);
	GPIO_Init(&GPIOBtn);

	/************** MAIN LOOP *************************/
	while(1)
	{
		// Button is active LOW because pull-up is enabled
		if(GPIO_ReadFromInputPin(GPIOD, GPIO_PIN_NO_5) == LOW)
		{
			delay();  // Debounce
			GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);

			// Wait until button is released
			while(GPIO_ReadFromInputPin(GPIOD, GPIO_PIN_NO_5) == LOW);
		}
	}
}
