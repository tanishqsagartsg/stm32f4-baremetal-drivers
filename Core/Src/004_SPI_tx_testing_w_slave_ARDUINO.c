/*
 * spi_tx_testing.c
 *
 *  Created on: Jan 25, 2026
 *      Author: tanis
 */
#include "stm32f407xx.h"
#include<string.h>

void delay(void)
{
	for(uint32_t i=0; i<=500000/2; i++);
}


/*
 * PB12 --> SPI2_NSS
 * PB23 --> SPI2_SCLK
 * PB14 --> SPI2_MISO
 * PB15 --> SPI2_MOSI
 * ALTENATE FUNCTION MODE: 5
 */

void SPI2_GPIO_Inits(void){
	GPIO_Handle_t SPIPins;

	SPIPins.pGPIOx = GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_PinALtFunMode = 5;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PIN_PUPD;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	//SCLK
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIO_Init(&SPIPins);

	//MOSI
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
	GPIO_Init(&SPIPins);

	//MISO
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	GPIO_Init(&SPIPins);

	//NSS
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIO_Init(&SPIPins);


}



void SPI2_Init(void){
	SPI_Handle_t SPI2_HANDLE;

	SPI2_HANDLE.pSPIx = SPI2;
	SPI2_HANDLE.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2_HANDLE.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2_HANDLE.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIVBY_8;   //generates sclk of 2MHz
	SPI2_HANDLE.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI2_HANDLE.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2_HANDLE.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2_HANDLE.SPIConfig.SPI_SSM = SPI_SSM_DI;    //Hardware Slave Management enabled for NSS pin

	SPI_Init(&SPI2_HANDLE);
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

	char userdata[]= "HELLO WORLD";

	SPI2_GPIO_Inits();

	SPI2_Init();

	SPI_SSOE_CONFIG(SPI2, EN);		//needed when hardware slave management is active

	while(1){
		while(! GPIO_ReadFromInputPin(GPIOA,GPIO_PIN_NO_0));

		delay();

		//SPI_SSI_CONFIG(SPI2, EN);      //not needed if we are using hardware slave management

		SPI_PeripheralContol(SPI2, EN);

		//first send length info to the slave so it knows what to expect
		uint8_t dataLen = strlen(userdata);
		SPI_SendData(SPI2, dataLen, 1);

		SPI_SendData(SPI2, (uint8_t*)userdata, strlen(userdata));

		//LETS CONFIRM IF THE SPI IS BUSY OR NOT
		while(SPI_GetFlagStatus(SPI2,SPI_BSY_FLAG));
		//THEN WE CAN DISABLE THE SPI
		SPI_PeripheralContol(SPI2, DI);
	}

	return 0;
}
