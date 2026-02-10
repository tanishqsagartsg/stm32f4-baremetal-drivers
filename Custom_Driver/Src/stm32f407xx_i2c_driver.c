/*
 * stm32f407xx_i2c_driver.c
 *
 *  Created on: Feb 5, 2026
 *      Author: tanis
 */

#include "stm32f407xx_i2c_driver.h"
#include "stm32f407xx.h"



uint16_t AHB_PreScaler[8]= {2,4,8,16,64,128,256,512};
uint16_t APB1_PreScaler[4]= {2,4,8,16};




//=================================================================================================================================
static void I2C_GenerateStartCondition(I2C_RegDef_t  *pI2Cx);
static void I2C_ExecuteAddressPhase(I2C_RegDef_t  *pI2Cx, uint8_t SlaveAddr);
static void I2C_ClearADDRFlag(I2C_RegDef_t  *pI2Cx);
static void I2C_GenerateStopCondition(I2C_RegDef_t  *pI2Cx);




//===========================================================
static void I2C_GenerateStartCondition(I2C_RegDef_t  *pI2Cx){
	pI2Cx->CR1 |= (1 << I2C_CR1_START);
}

static void I2C_ExecuteAddressPhase(I2C_RegDef_t  *pI2Cx, uint8_t SlaveAddr){
	SlaveAddr = SlaveAddr << 1;
	SlaveAddr &= ~(1); 		//Slave Addr = slave address + read/write bit which is set to 0 for write
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



//TO GET THE PCLK VALUE WITH HE VALUES OF HSI, AHB AND APB1 PRESCALERS=============================================================
uint32_t RCC_GetPclk1Value(void){
	uint32_t pclk1, SystemClk;
	uint8_t clksrc, temp, ahbp, apb1p;

	clksrc = ((RCC->CFGR >> 2 ) & 0x3);

	if (clksrc == 0){
		SystemClk = 16000000;
	}

	else if (clksrc == 1){
		SystemClk = 8000000;
	}
	else if (clksrc == 2){
		SystemClk = 8000000;
		return 0;
	}


	// for ahb prescaler
	temp = ((RCC->CFGR>>4) & 0xf);

	if (temp<8){
		ahbp = 1;
	}
	else{
		ahbp = AHB_PreScaler[temp-8];
	}


	// for apb1 prescaler
	temp = ((RCC->CFGR>>10) & 0x7);

		if (temp<8){
			apb1p = 1;
		}
		else{
			apb1p = APB1_PreScaler[temp-4];
		}


	pclk1 = (SystemClk / ahbp) / apb1p;

	return pclk1;
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
	tempreg |=  RCC_GetPclk1Value() / 1000000U;
	pI2CHandle->pI2Cx->CR2 = (tempreg & 0x3f);

	//program the device's own address
	tempreg = 0;
	tempreg |= pI2CHandle->I2C_Config.I2C_DeviceAdress << 1;
	tempreg |= (1<<14);
	pI2CHandle->pI2Cx->OAR1 = tempreg;


	//CCR value calculations
	uint16_t ccr_value = 0;
	tempreg = 0;

	if(pI2CHandle->I2C_Config.I2C_SCLSPEED < I2C_SCL_SPEED_SM){

		//mode is standard mode
		ccr_value = (RCC_GetPclk1Value() / 2*(pI2CHandle->I2C_Config.I2C_SCLSPEED));
		tempreg = (ccr_value & 0xfff);
	}

	else{

		//mode is fast mode
		tempreg |= (1<<15);
		tempreg |= (pI2CHandle->I2C_Config.I2C_FMDutyCycle << 14);

		if(pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2){
			ccr_value = (RCC_GetPclk1Value() / 3*(pI2CHandle->I2C_Config.I2C_SCLSPEED));
		}
		else{
			ccr_value = (RCC_GetPclk1Value() / 25*(pI2CHandle->I2C_Config.I2C_SCLSPEED));
		}
		tempreg |= (ccr_value & 0xfff);
	}

	pI2CHandle->pI2Cx->CCR = tempreg;

	//TRISE configuration
	if(pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2){
		//mode is standard mode
		tempreg = (  RCC_GetPclk1Value() / 1000000U) + 1;
	}

	else{
		//mode is fast mode
		tempreg = (  (RCC_GetPclk1Value() * 300) / 1000000U) + 1;
	}

	pI2CHandle->pI2Cx->TRISE = (tempreg & 0x3f);
}
//=================================================================================================================================



//=================================================================================================================================
void I2C_DeInit(I2C_RegDef_t *pI2Cx);
//======================================================================================================








//=================================================================================================================================
//DATA SEND AND RECIEVE======================================================================================================
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTXBuffer, uint32_t Len, uint8_t SlaveAddr){

	// 1. Generate the Start Condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);


	// 2. CONFIRM THAT THE START GENERATION IS COMPLETED BY CHECKING SB FLAG IN SR1
	// Note: until sb is cleared SCL will be pulled to LOW or STRETCHED
	while( !I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_SB_FLAG));


	// 3. SEND THE ADDRESS TO THE SLAVE WITH READ/WRITE BIT SET TO 0(WRITE) (TOTAL 8 BITS)
	I2C_ExecuteAddressPhase(pI2CHandle->pI2Cx, SlaveAddr);

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
	I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
}




void I2C_RecieveData(I2C_RegDef_t *pI2Cx, uint8_t *pRXBuffer, uint32_t Len);
//======================================================================================================


//INTERRUPT HANDLING======================================================================================================
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
void I2C_IRQHandling(I2C_Handle_t *pHandle);
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







