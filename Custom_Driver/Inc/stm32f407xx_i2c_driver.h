/*
 * stm32f407xx_i2c_driver.h
 *
 *  Created on: Feb 5, 2026
 *      Author: tanis
 */

#ifndef INC_STM32F407XX_I2C_DRIVER_H_
#define INC_STM32F407XX_I2C_DRIVER_H_

#include "stm32f407xx.h"


typedef struct{
	uint32_t I2C_SCLSPEED;
	uint8_t I2C_DeviceAdress;
	uint8_t I2C_AckControl;
	uint8_t I2C_FMDutyCycle;

}I2C_Config_t;


typedef struct{
	I2C_RegDef_t *pI2Cx;
	I2C_Config_t I2C_Config;
	uint8_t *pTxBuffer;
	uint8_t *pRxBuffer;
	uint32_t TxLen;
	uint32_t RxLen;
	uint8_t TxRxState;
	uint8_t DevAddr;
	uint32_t RxSize;
	uint8_t Sr;
}I2C_Handle_t;



#define I2C_READY					0
#define I2C_BUSY_IN_RX				1
#define I2C_BUSY_IN_TX				2



//I2C_SPEED_CONTROL================================================================================================================
#define I2C_SCL_SPEED_SM		100000
#define I2C_SCL_SPEED_FM2K		200000
#define I2C_SCL_SPEED_FM4K		400000
//=================================================================================================================================


//I2C_ACK_CONTROL================================================================================================================
#define I2C_ACK_ENABLE			1
#define I2C_ACK_DISABLE			0
//=================================================================================================================================


//I2C_FM_DUTY_CYCLE================================================================================================================
#define I2C_FM_DUTY_2			0
#define I2C_FM_DUTY_16_9		1
//=================================================================================================================================





//=================================================================================================================================
//*****************API_SUPPORTED_BY_I2C_PERIPHERAL*********************
//PERIPHERAL CLOCK SETUP===============================================
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);


//INIT DEINIT==========================================================
void I2C_Init(I2C_Handle_t *pI2CHandle);
void I2C_DeInit(I2C_RegDef_t *pI2Cx);
//===================================================



//DATA SEND AND RECIEVE===================================================
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTXBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t RS);
void I2C_MasterRecieveData(I2C_Handle_t *pI2CHandle, uint8_t *pRXBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t RS);
//======================================================================================================


uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTXBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t RS);
uint8_t I2C_MasterRecieveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRXBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t RS);
void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle);
void I2C_CloseSendData(I2C_Handle_t *pI2CHandle);




//INTERRUPT HANDLING===================================================
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
void I2C_EV_IRQHandling(I2C_Handle_t *pHandle);
void I2C_ER_IRQHandling(I2C_Handle_t *pHandle);
//=================================================================================================================================



//OTHER PERIPHERAL CONTROL API SUPPORTED BY I2C====================================================================================

//THIS IS TO ENABLE THE I2C PERIPHERAL AND WIHTOUT ENABLING THIS I2C WONT WORK
void I2C_PeripheralContol(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi);


void I2C_ManageAcking( I2C_RegDef_t *pI2Cx , uint8_t EnOrDi);
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName);

//=================================================================================================================================








//I2C RELATED STATUS FLAGS DEFINITIONS
#define  I2C_SB_FLAG 		(1 << I2C_SR1_SB)
#define  I2C_ADDR_FLAG 		(1 << I2C_SR1_ADDR)
#define  I2C_BTF_FLAG 		(1 << I2C_SR1_BTF)
#define  I2C_ADD10_FLAG 	(1 << I2C_SR1_ADD10)
#define  I2C_STOPF_FLAG 	(1 << I2C_SR1_STOPF)
#define  I2C_RxNE_FLAG		(1 << I2C_SR1_RXNE)
#define  I2C_TxE_FLAG 		(1 << I2C_SR1_TXE)
#define  I2C_BERR_FLAG 		(1 << I2C_SR1_BERR)
#define  I2C_ARLO_FLAG 		(1 << I2C_SR1_ARLO)
#define  I2C_AF_FLAG 		(1 << I2C_SR1_AF)
#define  I2C_OVR_FLAG 		(1 << I2C_SR1_OVR)
#define  I2C_PECERR_FLAG 	(1 << I2C_SR1_PECERR)
#define  I2C_TIMEOUT_FLAG 	(1 << I2C_SR1_TIMEOUT)
#define  I2C_SMBALERT_FLAG 	(1 << I2C_SR1_SMBALERT)

//=================================================================================================================================



#define I2C_RS_ENABLED 			RESET
#define I2C_RS_DISABLED 		SET






//I2C application events macros

#define I2C_EV_TX_CMPLT  	 	0
#define I2C_EV_RX_CMPLT  	 	1
#define I2C_EV_STOP       		2
#define I2C_ERROR_BERR 	 		3
#define I2C_ERROR_ARLO  		4
#define I2C_ERROR_AF    		5
#define I2C_ERROR_OVR   		6
#define I2C_ERROR_TIMEOUT 		7
#define I2C_EV_DATA_REQ         8
#define I2C_EV_DATA_RCV         9

#endif /* INC_STM32F407XX_I2C_DRIVER_H_ */
