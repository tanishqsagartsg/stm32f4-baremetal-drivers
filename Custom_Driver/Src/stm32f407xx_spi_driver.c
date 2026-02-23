/*
 * stm32f407xx_spi_driver.c
 *
 *  Created on: Jan 19, 2026
 *      Author: tanis
 */

#include"stm32f407xx_spi_driver.h"
#include "stm32f407xx.h"




static void spi_txe_interrupt_handle(SPI_Handle_t *pHandle);
static void spi_rxe_interrupt_handle(SPI_Handle_t *pHandle);
static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pHandle);




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




//INIT DEINIT==========================================================
void SPI_Init(SPI_Handle_t *pSPIHandle){
	//first lets configure SPI_CR1 register
	uint32_t tempreg = 0;

	//ENABLE THE PHERIPHERAL CLOCK
	SPI_PeriClockControl(pSPIHandle->pSPIx, EN);

	//configure the device mode
	tempreg |= pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR;

	//configure the bus speed
	if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD){
		//BIDIMODE should be cleared
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);
	}
	else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD){
		//BIDIMODE should be set
		tempreg |= (1 << SPI_CR1_BIDIMODE);
	}
	else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY){
		//RX_ONLY bit is to be set
		tempreg |= (1 << SPI_CR1_RXONLY);
		//BIDIMODE should be cleared
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);
	}
	//configure the SPI serial clock speed (baud rate)
	tempreg |= pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR;

	// configure the DFF
	tempreg |= pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF;

	// configure the CPOL
	tempreg |= pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL;

	// configure the CPHA
	tempreg |= pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA;

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





void SPI_RecieveData(SPI_RegDef_t *pSPIx, uint8_t *pRXBuffer, uint32_t Len){
	while(Len > 0)
		{
			//wait until RXNE flag is set
			while(SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG)== FLAG_RESET);

			//CHECK DFF BIT IN CR1
			if(pSPIx->CR1 & (1 << SPI_CR1_DFF)){
				//16 BIT DFF
				//LOAD DATA FROM DR
				*((uint16_t*)pRXBuffer) = pSPIx->DR;
				Len--;
				Len--;
				(uint16_t*)pRXBuffer++;
			}
			else{
				//8 BIT DFF
				//LOAD DATA FROM DR
				*pRXBuffer = pSPIx->DR;
				Len--;
				pRXBuffer++;
			}
		}
}
//===================================================





uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTXBuffer, uint32_t Len){

	uint8_t state= pSPIHandle->TxState;

	if(state != SPI_BUSY_IN_TX){

		// 1. save the Tx buffer address and Len info in some global variables
		pSPIHandle->pTxBuffer = pTXBuffer;
		pSPIHandle->TxLen = Len;

		// 2. mark the spi state as busy in transmission so that no other code can take over spi untill transmission is over
		pSPIHandle->TxState = SPI_BUSY_IN_TX;

		// 3. enable the txeie control bit to get interrupt whenever TXE flag is set in SR
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_TXEIE);
		// 4. data transmission will be handled by ISR code


	}
	return state;
}


uint8_t SPI_RecieveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRXBuffer, uint32_t Len){

	uint8_t state= pSPIHandle->RxState;

	if(state != SPI_BUSY_IN_RX){

		// 1. save the Tx buffer address and Len info in some global variables
		pSPIHandle->pRxBuffer = pRXBuffer;
		pSPIHandle->RxLen = Len;

		// 2. mark the spi state as busy in transmission so that no other code can take over spi until transmission is over
		pSPIHandle->RxState = SPI_BUSY_IN_RX;

		// 3. enable the txeie control bit to get interrupt whenever TXE flag is set in SR
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_RXNEIE);
		// 4. data transmission will be handled by ISR code


	}
	return state;

}

//=================================================================================================================================



//INTERRUPT HANDLING===================================================
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi){

	if(EnorDi==EN){
		if(IRQNumber <= 31)
		{
			*NVIC_ISER0 |= (1 << (IRQNumber));
		}
		else if(IRQNumber>31 && IRQNumber < 64)
		{
			*NVIC_ISER1 |= (1 << (IRQNumber % 32));
		}
		else if(IRQNumber>64 && IRQNumber < 96)
		{
			*NVIC_ISER2 |= (1 << (IRQNumber % 64));
		}
	}

	else
	{
		if(IRQNumber <= 31)
		{
			*NVIC_ICER0 |= (1 << (IRQNumber));
		}
		else if(IRQNumber>31 && IRQNumber < 64)
		{
			*NVIC_ICER1 |= (1 << (IRQNumber % 32));
		}
		else if(IRQNumber>64 && IRQNumber < 96)
		{
			*NVIC_ICER2 |= (1 << (IRQNumber % 64));
		}
	}

}


void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority){

	uint8_t iprx = IRQNumber/4;
	uint8_t iprx_section = IRQNumber%4;
	uint8_t shift_amount= (8*iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR + (iprx)) |= (IRQPriority << shift_amount);

}


void SPI_IRQHandling(SPI_Handle_t *pHandle){

	uint8_t temp1, temp2;


	//first check for TXE
	temp1 = pHandle->pSPIx->SR & (1 << SPI_SR_TXE);
	temp2 = pHandle->pSPIx->CR2 & (1 << SPI_CR2_TXEIE);

	if( temp1 && temp2){
		//handle TXE
		spi_txe_interrupt_handle(pHandle);
	}


	//first check for TXE
	temp1 = pHandle->pSPIx->SR & (1 << SPI_SR_RXNE);
	temp2 = pHandle->pSPIx->CR2 & (1 << SPI_CR2_RXNEIE);

	if( temp1 && temp2){
		//handle TXE
		spi_rxe_interrupt_handle(pHandle);
	}


	//first check for TXE
	temp1 = pHandle->pSPIx->SR & (1 << SPI_SR_OVR);
	temp2 = pHandle->pSPIx->CR2 & (1 << SPI_CR2_ERRIE);

	if( temp1 && temp2){
		//handle TXE
		spi_ovr_err_interrupt_handle(pHandle);
	}
}
//===================================================



static void spi_txe_interrupt_handle(SPI_Handle_t *pHandle){

	if(pHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF)){
		//16 BIT DFF
		//LOAD DATA INTO DR
		pHandle->pSPIx->DR = ((uint16_t)*pHandle->pTxBuffer);
		pHandle->TxLen--;
		pHandle->TxLen--;
		(uint16_t*)pHandle->pTxBuffer++;
	}

	else{
		//8 BIT DFF
		//LOAD DATA INTO DR
		pHandle->pSPIx->DR = *pHandle->pTxBuffer;
		pHandle->TxLen--;
		pHandle->pTxBuffer++;
	}

	if(!pHandle->TxLen){
		SPI_CloseTransmission(pHandle);
		SPI_ApplicationEventCallBack(pHandle,SPI_EVENT_TX_CMPLT);
	}

}


static void spi_rxe_interrupt_handle(SPI_Handle_t *pHandle){


	if(pHandle->pSPIx->CR1 & (1 << 11)){
		//16 BIT DFF
		//LOAD DATA INTO DR
		*((uint16_t*)pHandle->pRxBuffer) = (uint16_t)pHandle->pSPIx->DR;
		pHandle->RxLen-=2;
		pHandle->pRxBuffer--;
		pHandle->pRxBuffer--;
	}

	else{
		//8 BIT DFF
		//LOAD DATA INTO DR
		*(pHandle->pRxBuffer) = (uint8_t)pHandle->pSPIx->DR;
		pHandle->RxLen--;
		pHandle->pRxBuffer--;
	}

	if(!pHandle->RxLen){
		SPI_CloseReception(pHandle);
		SPI_ApplicationEventCallBack(pHandle,SPI_EVENT_RX_CMPLT);
	}
}


static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pHandle){
	uint8_t temp;

	//clear the ovr flag
	if(pHandle->TxState!=SPI_BUSY_IN_TX)
	{
		temp = pHandle->pSPIx->DR;
		temp = pHandle->pSPIx->SR;

	}
	(void)temp;
	//inform the application
	SPI_ApplicationEventCallBack(pHandle,SPI_EVENT_OVR_ERR);
}


void SPI_CloseTransmission(SPI_Handle_t *pHandle){

	pHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_TXEIE);
	pHandle->pTxBuffer = NULL;
	pHandle->TxLen = 0;
	pHandle->TxState = SPI_READY;

}


void SPI_CloseReception(SPI_Handle_t *pHandle){

	pHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_RXNEIE);
	pHandle->pRxBuffer = NULL;
	pHandle->RxLen = 0;
	pHandle->RxState = SPI_READY;
}


void SPI_ClearOvrFlag(SPI_RegDef_t *pSPIx){

	uint8_t temp;
	temp=pSPIx->DR;
	temp=pSPIx->SR;
	(void)temp;
}

__weak void SPI_ApplicationEventCallBack(SPI_Handle_t *pHandle,uint8_t AppEv){

		//this is a weak implementation. application may override this.
}




//OTHER PERIPHERAL CONTROL API SUPPORTED BY SPI====================================================================================
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


