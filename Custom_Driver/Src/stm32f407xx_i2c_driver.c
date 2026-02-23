/*
 * stm32f407xx_i2c_driver.c
 *
 *  Created on: Feb 5, 2026
 *      Author: tanis
 */

#include "stm32f407xx_i2c_driver.h"
#include "stm32f407xx.h"




//=================================================================================================================================
static void I2C_GenerateStartCondition(I2C_RegDef_t  *pI2Cx);
static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t  *pI2Cx, uint8_t SlaveAddr);
static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t  *pI2Cx, uint8_t SlaveAddr);
static void I2C_ClearADDRFlag(I2C_RegDef_t  *pI2Cx);
static void I2C_GenerateStopCondition(I2C_RegDef_t  *pI2Cx);

static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t *pI2CHandle );
static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t *pI2CHandle );


//===========================================================
static void I2C_GenerateStartCondition(I2C_RegDef_t  *pI2Cx){
	pI2Cx->CR1 |= (1 << I2C_CR1_START);
}

static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t  *pI2Cx, uint8_t SlaveAddr){
	SlaveAddr = SlaveAddr << 1;
	SlaveAddr &= ~(1); 		//Slave Addr = slave address + read/write bit which is set to 0 for write
	pI2Cx->DR = SlaveAddr;
}

static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t  *pI2Cx, uint8_t SlaveAddr){
	SlaveAddr = SlaveAddr << 1;
	SlaveAddr |= (1); 		//Slave Addr = slave address + read/write bit which is set to 1 for read
	pI2Cx->DR = SlaveAddr;
}

static void I2C_ClearADDRFlag(I2C_RegDef_t  *pI2Cx){
	uint32_t dummyread = pI2Cx->SR1;
	dummyread = pI2Cx->SR2;

	(void)dummyread;
}

static void I2C_GenerateStopCondition(I2C_RegDef_t  *pI2Cx){
	pI2Cx->CR1 |= (1 << I2C_CR1_STOP);
}
//=================================================================================================================================





//=================================================================================================================================
//*****************API_SUPPORTED_BY_I2C_PERIPHERAL*********************
//PERIPHERAL CLOCK SETUP===============================================
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	if (EnorDi == EN)
		    {
		        if (pI2Cx == I2C1)      { I2C1_PCLK_EN(); }
		        else if (pI2Cx == I2C2) { I2C2_PCLK_EN(); }
		        else if (pI2Cx == I2C3) { I2C3_PCLK_EN(); }

		    }
		    else
		    {
		    	if (pI2Cx == I2C1)      { I2C1_PCLK_DN(); }
		    	else if (pI2Cx == I2C1) { I2C2_PCLK_DN(); }
		    	else if (pI2Cx == I2C1) { I2C3_PCLK_DN(); }

		    }
}
//=================================================================================================================================



uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName)
{
	if(pI2Cx->SR1 & FlagName){
		return FLAG_SET;
	}
	return FLAG_RESET;
}



//=================================================================================================================================





//INIT DEINIT=============================================================================================================
void I2C_Init(I2C_Handle_t *pI2CHandle){
	uint32_t  tempreg=0;

	I2C_PeriClockControl(pI2CHandle->pI2Cx, EN);

	//ack control bit
	tempreg |=  pI2CHandle->I2C_Config.I2C_AckControl << I2C_CR1_ACK;
	pI2CHandle->pI2Cx->CR1 = tempreg;

	//configure the freq field of the CR2
	tempreg = 0;
	tempreg |=  RCC_GetPCLK1Value() / 1000000U;
	pI2CHandle->pI2Cx->CR2 = (tempreg & 0x3f);

	//program the device's own address
	tempreg = 0;
	tempreg |= pI2CHandle->I2C_Config.I2C_DeviceAdress << 1;
	tempreg |= (1<<14);
	pI2CHandle->pI2Cx->OAR1 = tempreg;


	//CCR value calculations
	uint16_t ccr_value = 0;
	tempreg = 0;

	if(pI2CHandle->I2C_Config.I2C_SCLSPEED == I2C_SCL_SPEED_SM){

		//mode is standard mode
		ccr_value = (RCC_GetPCLK1Value() / (2*(pI2CHandle->I2C_Config.I2C_SCLSPEED)));
		tempreg = (ccr_value & 0xfff);
	}

	else{

		//mode is fast mode
		tempreg |= (1<<15);
		tempreg |= (pI2CHandle->I2C_Config.I2C_FMDutyCycle << 14);

		if(pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2){
			ccr_value = (RCC_GetPCLK1Value() / (3*(pI2CHandle->I2C_Config.I2C_SCLSPEED)));
		}
		else{
			ccr_value = (RCC_GetPCLK1Value() / (25*(pI2CHandle->I2C_Config.I2C_SCLSPEED)));
		}
		tempreg |= (ccr_value & 0xfff);
	}

	pI2CHandle->pI2Cx->CCR = tempreg;

	//TRISE configuration
	if(pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2){
		//mode is standard mode
		tempreg = (  RCC_GetPCLK1Value() / 1000000U) + 1;
	}

	else{
		//mode is fast mode
		tempreg = (  (RCC_GetPCLK1Value() * 300) / 1000000000U) + 1;
	}

	pI2CHandle->pI2Cx->TRISE = (tempreg & 0x3f);
}
//=================================================================================================================================



//=================================================================================================================================
void I2C_DeInit(I2C_RegDef_t *pI2Cx);
//======================================================================================================








//=================================================================================================================================
//DATA SEND AND RECIEVE======================================================================================================
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTXBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t RS){

	// 1. Generate the Start Condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);


	// 2. CONFIRM THAT THE START GENERATION IS COMPLETED BY CHECKING SB FLAG IN SR1
	// Note: until SB is cleared SCL will be pulled to LOW or STRETCHED
	while( !I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_SB_FLAG));


	// 3. SEND THE ADDRESS TO THE SLAVE WITH READ/WRITE BIT SET TO 0(WRITE) (TOTAL 8 BITS)
	I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx, SlaveAddr);

	// 4. CONFIRM THAT ADDRESS PHASE IS COMPLETED BY CHECKING ADDR FLAG IN SR1
	while( !I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_ADDR_FLAG));


	// 5. CLEAR THE ADDR FLAG ACCORDING TO ITS SOFTWARE SEQUENCE
	// NOTE: UNTILL ADDR IS CLEARED THE SCL WILL BE PULLED TO LOW
	I2C_ClearADDRFlag(pI2CHandle->pI2Cx);

	// 6. SEND THE DATA UNTIL LEN BECOMES 0
	while(Len>0){
		while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_TxE_FLAG))
		pI2CHandle->pI2Cx->DR = *pTXBuffer;
		pTXBuffer++;
		Len--;
	}

	// 7. WHEN LEN BECOMES 0 WAIT FOR TXE = 1 AND BTF = 1 BEFORE GENERATING THE STOP CONDITION
	// nOTE: TXE=1 AND BTF=1 MEANS THAT BOTH SR AND DR ARE EMPTY AND NEXT TRANSMISSION SHOULD BEGIN
	// WHEN BTF=1 SCL WILL BE STRECHED
	while( !I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_TxE_FLAG));

	while( !I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_BTF_FLAG));

	// 8. GENERATE STOP CONDITION AND MASTER NEED NOT TO WAIT FOR COMPLETION OF STOP CONDITION
	// nOTE: GENERATING STOP CONDITION AUTOMATICALLY CLEARS BTF
	if(RS == I2C_RS_ENABLED){
		I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
	}
}




void I2C_MasterRecieveData(I2C_Handle_t *pI2CHandle, uint8_t *pRXBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t RS){

	// 1. Generate the Start Condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);


	// 2. CONFIRM THAT THE START GENERATION IS COMPLETED BY CHECKING SB FLAG IN SR1
	// Note: untill sb is cleared SCL will be pulled to LOW or STRETCHED
	while( !I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_SB_FLAG));


	// 3. SEND THE ADDRESS TO THE SLAVE WITH READ/WRITE BIT SET TO 1(READ) (TOTAL 8 BITS)
	I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx, SlaveAddr);

	// 4. CONFIRM THAT ADDRESS PHASE IS COMPLETED BY CHECKING ADDR FLAG IN SR1
	while( !I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_ADDR_FLAG));

	//procedure to read only one byte from  slave
	if(Len==1){
		//Disable acking
		I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);

		//clear the ADDR flag
		I2C_ClearADDRFlag(pI2CHandle->pI2Cx);

		//wait until RxNE flag becomes 1
		while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_RxNE_FLAG));

		//generate stop condition
		if(RS == I2C_RS_ENABLED){
			I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
		}


		//read data into the buffer
		*pRXBuffer=pI2CHandle->pI2Cx->DR;

	}

	if(Len>1){

		//clear the ADDR flag
		I2C_ClearADDRFlag(pI2CHandle->pI2Cx);

		for(uint32_t i= Len; i>0; i--){

			//wait until RxNE flag becomes 1
			while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_RxNE_FLAG));

			if(i==2){		//if last 2 bytes are remaining
				//Disable acking
				I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);

				//generate stop condition
				if(RS == I2C_RS_ENABLED){
					I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
				}
			}

			//read data into the buffer
			*pRXBuffer=pI2CHandle->pI2Cx->DR;

			pRXBuffer++;
		}

	}
	//re-enable acking
	if(pI2CHandle->I2C_Config.I2C_AckControl == I2C_ACK_ENABLE){
		I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_ENABLE);
	}
}
//======================================================================================================



void I2C_ManageAcking( I2C_RegDef_t *pI2Cx , uint8_t EnOrDi){

	if(EnOrDi == I2C_ACK_ENABLE){
		//enable the ack
		pI2Cx->CR1 |= ( 1 << I2C_CR1_ACK) ;
	}
	else{
		//enable the ack
		pI2Cx->CR1 &= ~( 1 << I2C_CR1_ACK) ;
	}
}




//======================================================================================================
uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTXBuffer,	uint32_t Len, uint8_t SlaveAddr, uint8_t RS) {

	uint8_t busystate = pI2CHandle->TxRxState;

	if ((busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX)) {
		pI2CHandle->pTxBuffer = pTXBuffer;
		pI2CHandle->TxLen = Len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_TX;
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Sr = RS;


		//Implement code to Generate START Condition
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

		//Implement the code to enable ITBUFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITBUFEN);

		//Implement the code to enable ITEVFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITEVTEN);

		//Implement the code to enable ITERREN Control Bit
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITERREN);


	}

	return busystate;

}



//======================================================================================================
uint8_t I2C_MasterRecieveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRXBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t RS) {

	uint8_t busystate = pI2CHandle->TxRxState;

	if ((busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX)) {
		pI2CHandle->pRxBuffer = pRXBuffer;
		pI2CHandle->RxLen = Len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_RX;
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Sr = RS;

		//Implement code to Generate START Condition
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

		//Implement the code to enable ITBUFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITBUFEN);

		//Implement the code to enable ITEVFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITEVTEN);

		//Implement the code to enable ITERREN Control Bit
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITERREN);
	}

	return busystate;
}



//INTERRUPT HANDLING======================================================================================================
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi){
	if (EnorDi == EN) {
		if (IRQNumber <= 31) {
			*NVIC_ISER0 |= (1 << (IRQNumber));
		} else if (IRQNumber > 31 && IRQNumber < 64) {
			*NVIC_ISER1 |= (1 << (IRQNumber % 32));
		} else if (IRQNumber > 64 && IRQNumber < 96) {
			*NVIC_ISER2 |= (1 << (IRQNumber % 64));
		}
	}
	else {
		if (IRQNumber <= 31) {
			*NVIC_ICER0 |= (1 << (IRQNumber));
		} else if (IRQNumber > 31 && IRQNumber < 64) {
			*NVIC_ICER1 |= (1 << (IRQNumber % 32));
		} else if (IRQNumber > 64 && IRQNumber < 96) {
			*NVIC_ICER2 |= (1 << (IRQNumber % 64));
		}
	}

}



//======================================================================================================
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority){

	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;
	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR + (iprx)) |= (IRQPriority << shift_amount);
}



static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t *pI2CHandle )
{

	if(pI2CHandle->TxLen > 0)
	{
		//1. load the data in to DR
		pI2CHandle->pI2Cx->DR = *(pI2CHandle->pTxBuffer);

		//2. decrement the TxLen
		pI2CHandle->TxLen--;

		//3. Increment the buffer address
		pI2CHandle->pTxBuffer++;

	}

}



static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t *pI2CHandle )
{
	//We have to do the data reception
	if(pI2CHandle->RxSize == 1)
	{
		*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->DR;
		pI2CHandle->RxLen--;

	}


	if(pI2CHandle->RxSize > 1)
	{
		if(pI2CHandle->RxLen == 2)
		{
			//clear the ack bit
			I2C_ManageAcking(pI2CHandle->pI2Cx,DI);
		}

			//read DR
			*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->DR;
			pI2CHandle->pRxBuffer++;
			pI2CHandle->RxLen--;
	}

	if(pI2CHandle->RxLen == 0 )
	{
		//close the I2C data reception and notify the application

		//1. generate the stop condition
		if(pI2CHandle->Sr == I2C_RS_DISABLED)
			I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

		//2 . Close the I2C rx
		I2C_CloseReceiveData(pI2CHandle);

		//3. Notify the application
		I2C_ApplicationEventCallback(pI2CHandle,I2C_EV_RX_CMPLT);
	}
}


void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle)
{
	//Implement the code to disable ITBUFEN Control Bit
	pI2CHandle->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITBUFEN);

	//Implement the code to disable ITEVFEN Control Bit
	pI2CHandle->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITEVTEN);

	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pRxBuffer = NULL;
	pI2CHandle->RxLen = 0;
	pI2CHandle->RxSize = 0;

	if(pI2CHandle->I2C_Config.I2C_AckControl == I2C_ACK_ENABLE)
	{
		I2C_ManageAcking(pI2CHandle->pI2Cx,EN);
	}

}


void I2C_CloseSendData(I2C_Handle_t *pI2CHandle)
{
	//Implement the code to disable ITBUFEN Control Bit
	pI2CHandle->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITBUFEN);

	//Implement the code to disable ITEVFEN Control Bit
	pI2CHandle->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITEVTEN);


	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pTxBuffer = NULL;
	pI2CHandle->TxLen = 0;
}



//======================================================================================================
void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle)
{
	//Interrupt handling for both master and slave mode of a device

	uint32_t temp1, temp2, temp3;

	temp1   = pI2CHandle->pI2Cx->CR2 & ( 1 << I2C_CR2_ITEVTEN) ;
	temp2   = pI2CHandle->pI2Cx->CR2 & ( 1 << I2C_CR2_ITBUFEN) ;

	temp3  = pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_SB);
	//1. Handle For interrupt generated by SB event
	//	Note : SB flag is only applicable in Master mode
	if(temp1 && temp3)
	{
		//The interrupt is generated because of SB event
		//This block will not be executed in slave mode because for slave SB is always zero
		//In this block lets executed the address phase
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
		{
			I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx,pI2CHandle->DevAddr);
		}else if (pI2CHandle->TxRxState == I2C_BUSY_IN_RX )
		{
			I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx,pI2CHandle->DevAddr);
		}
	}

	temp3  = pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_ADDR);
	//2. Handle For interrupt generated by ADDR event
	//Note : When master mode : Address is sent
	//		 When Slave mode   : Address matched with own address
	if(temp1 && temp3)
	{
		// interrupt is generated because of ADDR event
		I2C_ClearADDRFlag(pI2CHandle->pI2Cx);
	}

	temp3  = pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_BTF);
	//3. Handle For interrupt generated by BTF(Byte Transfer Finished) event
	if(temp1 && temp3)
	{
		//BTF flag is set
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
		{
			//make sure that TXE is also set .
			if(pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_TXE) )
			{
				//BTF, TXE = 1
				if(pI2CHandle->TxLen == 0 )
				{
					//1. generate the STOP condition
					if(pI2CHandle->Sr == I2C_RS_DISABLED)
						I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

					//2. reset all the member elements of the handle structure.
					I2C_CloseSendData(pI2CHandle);

					//3. notify the application about transmission complete
					I2C_ApplicationEventCallback(pI2CHandle,I2C_EV_TX_CMPLT);

				}
			}

		}else if (pI2CHandle->TxRxState == I2C_BUSY_IN_RX )
		{
			;
		}
	}

	temp3  = pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_STOPF);
	//4. Handle For interrupt generated by STOPF event
	// Note : Stop detection flag is applicable only slave mode . For master this flag will never be set
	//The below code block will not be executed by the master since STOPF will not set in master mode
	if(temp1 && temp3)
	{
		//STOF flag is set
		//Clear the STOPF ( i.e 1) read SR1 2) Write to CR1 )

		pI2CHandle->pI2Cx->CR1 |= 0x0000;

		//Notify the application that STOP is detected
		I2C_ApplicationEventCallback(pI2CHandle,I2C_EV_STOP);
	}


	temp3  = pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_TXE);
	//5. Handle For interrupt generated by TXE event
	if(temp1 && temp2 && temp3)
	{
		//Check for device mode
		if(pI2CHandle->pI2Cx->SR2 & ( 1 << I2C_SR2_MSL))
		{
			//TXE flag is set
			//We have to do the data transmission
			if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
			{
				I2C_MasterHandleTXEInterrupt(pI2CHandle);
			}
		}else
		{
			//slave
			//make sure that the slave is really in transmitter mode
		    if(pI2CHandle->pI2Cx->SR2 & ( 1 << I2C_SR2_TRA))
		    {
		    	I2C_ApplicationEventCallback(pI2CHandle,I2C_EV_DATA_REQ);
		    }
		}
	}

	temp3  = pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_RXNE);
	//6. Handle For interrupt generated by RXNE event
	if(temp1 && temp2 && temp3)
	{
		//check device mode .
		if(pI2CHandle->pI2Cx->SR2 & ( 1 << I2C_SR2_MSL))
		{
			//The device is master

			//RXNE flag is set
			if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
			{
				I2C_MasterHandleRXNEInterrupt(pI2CHandle);

			}

		}else
		{
			//slave
			//make sure that the slave is really in receiver mode
			if(!(pI2CHandle->pI2Cx->SR2 & ( 1 << I2C_SR2_TRA)))
			{
				I2C_ApplicationEventCallback(pI2CHandle,I2C_EV_DATA_RCV);
			}
		}
	}
}





//======================================================================================================
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle)
{

	uint32_t temp1,temp2;

    //Know the status of  ITERREN control bit in the CR2
	temp2 = (pI2CHandle->pI2Cx->CR2) & ( 1 << I2C_CR2_ITERREN);


//Check for Bus error
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1<< I2C_SR1_BERR);
	if(temp1  && temp2 )
	{
		//This is Bus error

		//Implement the code to clear the buss error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_BERR);

		//Implement the code to notify the application about the error
	   I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_BERR);
	}

//Check for arbitration lost error
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_ARLO );
	if(temp1  && temp2)
	{
		//This is arbitration lost error

		//Implement the code to clear the arbitration lost error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_ARLO);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_ARLO);

	}

//Check for ACK failure  error

	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_AF);
	if(temp1  && temp2)
	{
		//This is ACK failure error

	    //Implement the code to clear the ACK failure error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_AF);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_AF);
	}

//Check for Overrun/underrun error
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_OVR);
	if(temp1  && temp2)
	{
		//This is Overrun/underrun

	    //Implement the code to clear the Overrun/underrun error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_OVR);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_OVR);
	}

//Check for Time out error
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_TIMEOUT);
	if(temp1  && temp2)
	{
		//This is Time out error

	    //Implement the code to clear the Time out error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_TIMEOUT);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_TIMEOUT);
	}

}
//=================================================================================================================================



//OTHER PERIPHERAL CONTROL API SUPPORTED BY I2C====================================================================================

//THIS IS TO ENABLE THE I2C PERIPHERAL AND WIHTOUT ENABLING THIS I2C WONT WORK
void I2C_PeripheralContol(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi){
	if(EnOrDi == EN){
			pI2Cx->CR1 |= (1 << I2C_CR1_PE);
		}
		else{
			pI2Cx->CR1 &= ~(1 << 0);
		}
}
//=================================================================================================================================



__weak void I2C_ApplicationEventCallback(SPI_Handle_t *pI2CHandle,uint8_t AppEv){

		//this is a weak implementation. application may override this.
}



