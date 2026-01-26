/*
 * stm32f407xx_spi_driver.c
 *
 *  Created on: Jan 19, 2026
 *      Author: tanis
 */

#include"stm32f407xx_spi_driver.h"
#include "stm32f407xx.h"

/*====================================================================
 * FUNCTION:
 *
 * BRIEF:
 *
 * PARAMETER 1:
 * PARAMETER 2:
 * PARAMETER 3:
 *
 * RETURN:
 *
 * NOTE:
 ====================================================================*/
//PERIPHERAL CLOCK SETUP===============================================
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi){
	if (EnorDi == EN)
	    {
	        if (pSPIx == SPI1)      { SPI1_PCLK_EN(); }
	        else if (pSPIx == SPI2) { SPI2_PCLK_EN(); }
	        else if (pSPIx == SPI3) { SPI3_PCLK_EN(); }

	    }
	    else
	    {
	    	if (pSPIx == SPI1)      { SPI1_PCLK_DN(); }
	    	else if (pSPIx == SPI1) { SPI2_PCLK_DN(); }
	    	else if (pSPIx == SPI1) { SPI3_PCLK_DN(); }

	    }
}

/*====================================================================
 * FUNCTION:
 *
 * BRIEF:
 *
 * PARAMETER 1:
 * PARAMETER 2:
 * PARAMETER 3:
 *
 * RETURN:
 *
 * NOTE:
 ====================================================================*/
//INIT DEINIT==========================================================
void SPI_Init(SPI_Handle_t *pSPIHandle){
	//first lets configure SPI_CR1 register
	uint32_t tempreg = 0;

	//ENABLE THE PHERIPHERAL CLOCK
	SPI_PeriClockControl(pSPIHandle->pSPIx, EN);

	//configure the device mode
	tempreg |= pSPIHandle->SPIConfig.SPI_DeviceMode << 2;

	//configure the bus speed
	if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD){
		//BIDIMODE should be cleared
		tempreg &= ~(1 << 15);
	}
	else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD){
		//BIDIMODE should be set
		tempreg |= (1 << 15);
	}
	else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY){
		//RX_ONLY bit is to be set
		tempreg |= (1 << 10);
		//BIDIMODE should be cleared
		tempreg &= ~(1 << 15);
	}
	//configure the SPI serial clock speed (baud rate)
	tempreg |= pSPIHandle->SPIConfig.SPI_SclkSpeed << 3;

	// configure the DFF
	tempreg |= pSPIHandle->SPIConfig.SPI_DFF << 11;

	// configure the CPOL
	tempreg |= pSPIHandle->SPIConfig.SPI_CPOL << 1;

	// configure the CPHA
	tempreg |= pSPIHandle->SPIConfig.SPI_CPHA << 0;

	pSPIHandle->pSPIx->CR1 = tempreg;

}
//==================================================




void SPI_DeInit(SPI_RegDef_t *pSPIx);
//===================================================



uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName)
{
	if(pSPIx->SR & FlagName){
		return FLAG_SET;
	}
	return FLAG_RESET;
}

/*====================================================================
 * FUNCTION:
 *
 * BRIEF:
 *
 * PARAMETER 1:
 * PARAMETER 2:
 * PARAMETER 3:
 *
 * RETURN:
 *
 * NOTE: this is a blocking call, which means any interrupt will not be entertained and will stop execution only after completion
 ====================================================================*/
//DATA SEND AND RECIEVE===================================================
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTXBuffer, uint32_t Len){
	while(Len > 0)
	{
		//wait until TXE flag is set
		while(SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG)== FLAG_RESET);

		//CHECK DFF BIT IN CR1
		if(pSPIx->CR1 & (1 << SPI_CR1_DFF)){
			//16 BIT DFF
			//LOAD DATA INTO DR
			pSPIx->DR = ((uint16_t)*pTXBuffer);
			Len--;
			Len--;
			(uint16_t*)pTXBuffer++;
		}
		else{
			//8 BIT DFF
			//LOAD DATA INTO DR
			pSPIx->DR = *pTXBuffer;
			Len--;
			pTXBuffer++;
		}
	}
}





void SPI_RecieveData(SPI_RegDef_t *pSPIx, uint8_t *pRXBuffer, uint32_t Len);
//===================================================

/*====================================================================
 * FUNCTION:
 *
 * BRIEF:
 *
 * PARAMETER 1:
 * PARAMETER 2:
 * PARAMETER 3:
 *
 * RETURN:
 *
 * NOTE:
 ====================================================================*/
//INTERRUPT HANDLING===================================================
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pHandle);
//===================================================



//OTHER PERIPHERAL CONTROL API SUPPORTED BY SPI====================================================================================

/*====================================================================
 * FUNCTION:
 *
 * BRIEF:
 *
 * PARAMETER 1:
 * PARAMETER 2:
 * PARAMETER 3:
 *
 * RETURN:
 *
 * NOTE:
 ====================================================================*/
//THIS IS TO ENABLE THE SPI PERIPHERAL AND WIHTOUT ENABLING THIS SPI WONT WORK
void SPI_PeripheralContol(SPI_RegDef_t *pSPIx, uint8_t EnOrDi){
	if(EnOrDi == EN){
		pSPIx->CR1 |= (1 << SPI_CR1_SPE);
	}
	else{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
	}
}
//=================================================================================================================================



//THIS IS TO ENABLE THE SPI SSI AS WHEN WE ARE NOT USING NSS THE SPI HAS TO BE PULLED TO HIGH ELSE IT WONT WORK AS MASTER WHEN WE INTIALIZE IT
void SPI_SSI_CONFIG(SPI_RegDef_t *pSPIx, uint8_t EnOrDi){
	if(EnOrDi == EN){
			pSPIx->CR1 |= (1 << SPI_CR1_SSI);
		}
		else{
			pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
		}
}
//=================================================================================================================================




//THIS IS TO ENABLE THE SPI SSI AS WHEN WE ARE NOT USING NSS THE SPI HAS TO BE PULLED TO HIGH ELSE IT WONT WORK AS MASTER WHEN WE INTIALIZE IT
void SPI_SSOE_CONFIG(SPI_RegDef_t *pSPIx, uint8_t EnOrDi){
	if(EnOrDi == EN){
			pSPIx->CR2 |= (1 << SPI_CR2_SSOE);
		}
		else{
			pSPIx->CR2 &= ~(1 << SPI_CR2_SSOE);
		}
}
//=================================================================================================================================


