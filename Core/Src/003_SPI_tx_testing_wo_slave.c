/*
 * spi_tx_testing.c
 *
 *  Created on: Jan 25, 2026
 *      Author: tanis
 */
#include "stm32f407xx.h"
#include<string.h>

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
	SPI2_HANDLE.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIVBY_2;
	SPI2_HANDLE.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI2_HANDLE.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2_HANDLE.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2_HANDLE.SPIConfig.SPI_SSM = SPI_SSM_EN;

	SPI_Init(&SPI2_HANDLE);
}



int main(void){

	char userdata[]= "HELLO WORLD";

	SPI2_GPIO_Inits();

	SPI2_Init();

	SPI_SSI_CONFIG(SPI2, EN);

	SPI_PeripheralContol(SPI2, EN);

	SPI_SendData(SPI2, (uint8_t*)userdata, strlen(userdata));

	SPI_PeripheralContol(SPI2, DI);

	return 0;
}
