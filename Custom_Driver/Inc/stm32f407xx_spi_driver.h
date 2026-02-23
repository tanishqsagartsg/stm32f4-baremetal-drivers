/*
 * stm32f407xx_spi_driver.h
 *
 *  Created on: Jan 19, 2026
 *      Author: tanis
 */

#ifndef INC_STM32F407XX_SPI_DRIVER_H_
#define INC_STM32F407XX_SPI_DRIVER_H_

#include"stm32f407xx.h"


//=================================================================================================================================
typedef struct{
	uint8_t SPI_DeviceMode;
	uint8_t SPI_BusConfig;
	uint8_t SPI_SclkSpeed;
	uint8_t SPI_DFF;
	uint8_t SPI_CPOL;
	uint8_t SPI_CPHA;
	uint8_t SPI_SSM;
}SPI_Config_t;
//=================================================================================================================================




//=================================================================================================================================
typedef struct{
	SPI_RegDef_t *pSPIx;
	SPI_Config_t SPIConfig;
	uint8_t *pTxBuffer;
	uint8_t *pRxBuffer;
	uint32_t TxLen;
	uint32_t RxLen;
	uint8_t TxState;
	uint8_t RxState;
}SPI_Handle_t;
//=================================================================================================================================




//SPI APPLICATION STATES===========================
#define SPI_EVENT_TX_CMPLT						0
#define SPI_EVENT_RX_CMPLT						1
#define SPI_EVENT_OVR_ERR						2
#define SPI_EVENT_CRC_ERR						3




//******SPI_CONFIGURATIONS************************************************
//DEVICE MODES FOR SPI====================================================
#define SPI_DEVICE_MODE_SLAVE 					0
#define SPI_DEVICE_MODE_MASTER					1
//=================================================================================================================================
//

//SPI BUS CONFIG===================================================================================================================
#define SPI_BUS_CONFIG_FD						1
#define SPI_BUS_CONFIG_HD						2
//#define SPI_BUS_CONFIG_SIMPLEX_TXONLY			THIS MODE ACTUALLY WORKS LIKE FULL DUPLEX ONLY SO NO NEED TO INTIALISE THIS
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY			4
//=================================================================================================================================


//SPI CLOCK SPEED===================================================================================================================
#define SPI_SCLK_SPEED_DIVBY_2					0
#define SPI_SCLK_SPEED_DIVBY_4					1
#define SPI_SCLK_SPEED_DIVBY_8					2
#define SPI_SCLK_SPEED_DIVBY_16					3
#define SPI_SCLK_SPEED_DIVBY_32					4
#define SPI_SCLK_SPEED_DIVBY_64					5
#define SPI_SCLK_SPEED_DIVBY_128				6
#define SPI_SCLK_SPEED_DIVBY_256				7
//=================================================================================================================================


//SPI DATA FRAME FORMAT=============================================================================================================
#define SPI_DFF_8BITS							0
#define SPI_DFF_16BITS							1
//=================================================================================================================================



//SPI CLOCK PHASE===================================================================================================================
#define SPI_CPHA_LOW							0
#define SPI_CPHA_HIGH							1
//=================================================================================================================================




//SPI CLOCK POLARITY===============================================================================================================
#define SPI_CPOL_LOW							0
#define SPI_CPOL_HIGH							1
//=================================================================================================================================

//SPI SLAVE SELECT MANAGEMENT======================================================================================================
#define SPI_SSM_DI								0
#define SPI_SSM_EN								1
//=================================================================================================================================

//SPI RELATED STATUS FLAGS DEFINITIONS
#define  SPI_TXE_FLAG (1 << SPI_SR_TXE)
#define  SPI_RXNE_FLAG (1 << SPI_SR_RXNE)
#define  SPI_BSY_FLAG (1 << SPI_SR_BSY)



//
#define SPI_READY 						0
#define SPI_BUSY_IN_RX 					1
#define SPI_BUSY_IN_TX 					2

//=================================================================================================================================






//=================================================================================================================================
//*****************API_SUPPORTED_BY_SPI_PERIPHERAL*********************
//PERIPHERAL CLOCK SETUP===============================================
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);


//INIT DEINIT==========================================================
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);
//===================================================



//DATA SEND AND RECIEVE===================================================
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTXBuffer, uint32_t Len);
void SPI_RecieveData(SPI_RegDef_t *pSPIx, uint8_t *pRXBuffer, uint32_t Len);



uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTXBuffer, uint32_t Len);
uint8_t SPI_RecieveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRXBuffer, uint32_t Len);
//======================================================================================================


//INTERRUPT HANDLING===================================================
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pHandle);
//=================================================================================================================================



//OTHER PERIPHERAL CONTROL API SUPPORTED BY SPI====================================================================================

//THIS IS TO ENABLE THE SPI PERIPHERAL AND WIHTOUT ENABLING THIS SPI WONT WORK
void SPI_PeripheralContol(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);
//=================================================================================================================================


//THIS IS TO ENABLE THE SPI SSI AS WHEN WE ARE NOT USING NSS THE SPI HAS TO BE PULLED TO HIGH ELSE IT WONT WORK AS MASTER WHEN WE INTIALIZE IT
void SPI_SSI_CONFIG(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);
//=================================================================================================================================


//THIS IS TO ENABLE THE SPI SSI AS WHEN WE ARE NOT USING NSS THE SPI HAS TO BE PULLED TO HIGH ELSE IT WONT WORK AS MASTER WHEN WE INTIALIZE IT
void SPI_SSOE_CONFIG(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);
//=================================================================================================================================


void SPI_ClearOvrFlag(SPI_RegDef_t *pSPIx);
void SPI_CloseTransmission(SPI_Handle_t *pHandle);
void SPI_CloseReception(SPI_Handle_t *pHandle);
__weak void SPI_ApplicationEventCallBack(SPI_Handle_t *pHandle,uint8_t AppEv);



#endif /* INC_STM32F407XX_SPI_DRIVER_H_ */
