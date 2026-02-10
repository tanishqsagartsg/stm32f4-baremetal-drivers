/*
 * 006_I2C_Tx_w_slave_Arduino.c
 *
 *  Created on: Feb 10, 2026
 *      Author: tanis
 */


#include "stm32f407xx.h"
#include<string.h>

I2C_Handle_t I2C1_HANDLE;
uint8_t userdata[]= "HELLO WORLD/n";
#define SlaveAddr 	0x68


void delay(void)
{
	for(uint32_t i=0; i<=500000/2; i++);
}


/*
 * PB6 --> SCL
 * PB9 --> SDA
 * ALTENATE FUNCTION MODE: 5
 */

void I2C1_GPIO_Inits(void){
	GPIO_Handle_t I2CPins;

	I2CPins.pGPIOx = GPIOB;
	I2CPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	I2CPins.GPIO_PinConfig.GPIO_PinALtFunMode = 4;
	I2CPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	I2CPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	I2CPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	//SCL
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
	GPIO_Init(&I2CPins);

	//SDA
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_9;
	GPIO_Init(&I2CPins);

}



void I2C1_Init(void){

	I2C1_HANDLE.pI2Cx = I2C1;
	I2C1_HANDLE.I2C_Config.I2C_AckControl = I2C_ACK_ENABLE;
	I2C1_HANDLE.I2C_Config.I2C_DeviceAdress = 0x66;
	I2C1_HANDLE.I2C_Config.I2C_FMDutyCycle = I2C_FM_DUTY_2;
	I2C1_HANDLE.I2C_Config.I2C_SCLSPEED = I2C_SCL_SPEED_SM;


	I2C_Init(&I2C1_HANDLE);
}


void GPIO_ButtonInit(void){

	GPIO_Handle_t GPIOBtn;
	GPIOBtn.pGPIOx= GPIOD;
	GPIOBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;


	GPIO_Init(&GPIOBtn);
}


int main(void){

	GPIO_ButtonInit();

	I2C1_GPIO_Inits();

	I2C1_Init();

	I2C_PeriClockControl(I2C1, EN);

	while(1){
		while(! GPIO_ReadFromInputPin(GPIOA,GPIO_PIN_NO_0));

		delay();

		I2C_MasterSendData(&I2C1_HANDLE, userdata, strlen((char*)userdata), SlaveAddr);
	}

	return 0;
}
