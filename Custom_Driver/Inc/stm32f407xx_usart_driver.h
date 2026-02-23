/*
 * stm32f407xx_usart_driver.h
 *
 *  Created on: Feb 20, 2026
 *      Author: tanis
 */

#ifndef INC_STM32F407XX_USART_DRIVER_H_
#define INC_STM32F407XX_USART_DRIVER_H_

#include "stm32f407xx.h"


typedef struct{
	uint32_t USART_MODE;
	uint8_t USART_BAUD;
	uint8_t USART_NoOfStopBits;
	uint8_t USART_WordLength;
	uint8_t USART_ParityControl;
	uint8_t USART_HWFlowControl;

}USART_Config_t;


typedef struct{
	USART_RegDef_t *pUSARTx;
	USART_Config_t USART_Config;
	uint8_t *pTxBuffer;
	uint8_t *pRxBuffer;
	uint32_t TxLen;
	uint32_t RxLen;
	uint8_t TxState;
	uint8_t RxState;
	uint32_t RxSize;
}USART_Handle_t;







#define USART_READY						0
#define USART_BUSY_IN_RX				1
#define USART_BUSY_IN_TX				2


/*
 *@USART_Mode
 *Possible options for USART_Mode
 */
#define USART_MODE_ONLY_TX 0
#define USART_MODE_ONLY_RX 1
#define USART_MODE_TXRX  2

/*
 *@USART_Baud
 *Possible options for USART_Baud
 */
#define USART_STD_BAUD_1200					1200
#define USART_STD_BAUD_2400					400
#define USART_STD_BAUD_9600					9600
#define USART_STD_BAUD_19200 				19200
#define USART_STD_BAUD_38400 				38400
#define USART_STD_BAUD_57600 				57600
#define USART_STD_BAUD_115200 				115200
#define USART_STD_BAUD_230400 				230400
#define USART_STD_BAUD_460800 				460800
#define USART_STD_BAUD_921600 				921600
#define USART_STD_BAUD_2M 					2000000
#define SUART_STD_BAUD_3M 					3000000


/*
 *@USART_ParityControl
 *Possible options for USART_ParityControl
 */
#define USART_PARITY_EN_ODD   2
#define USART_PARITY_EN_EVEN  1
#define USART_PARITY_DISABLE   0

/*
 *@USART_WordLength
 *Possible options for USART_WordLength
 */
#define USART_WORDLEN_8BITS  0
#define USART_WORDLEN_9BITS  1

/*
 *@USART_NoOfStopBits
 *Possible options for USART_NoOfStopBits
 */
#define USART_STOPBITS_1     0
#define USART_STOPBITS_0_5   1
#define USART_STOPBITS_2     2
#define USART_STOPBITS_1_5   3

/*
 *@USART_HWFlowControl
 *Possible options for USART_HWFlowControl
 */
#define USART_HW_FLOW_CTRL_NONE    	0
#define USART_HW_FLOW_CTRL_CTS    	1
#define USART_HW_FLOW_CTRL_RTS    	2
#define USART_HW_FLOW_CTRL_CTS_RTS	3



/******************************************************************************************
 *								APIs supported by this driver
 *		 For more information about the APIs check the function definitions
 ******************************************************************************************/
/*
 * Peripheral Clock setup
 */
void USART_PeriClockControl(USART_RegDef_t *pUSARTx, uint8_t EnorDi);

/*
 * Init and De-init
 */
void USART_Init(USART_Handle_t *pUSARTHandle);
void USART_DeInit(USART_RegDef_t *pUSARTx);


/*
 * Data Send and Receive
 */
void USART_SendData(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len);
void USART_ReceiveData(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len);
uint8_t USART_SendDataIT(USART_Handle_t *pUSARTHandle,uint8_t *pTxBuffer, uint32_t Len);
uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len);

/*
 * IRQ Configuration and ISR handling
 */
void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void USART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void USART_IRQHandling(USART_Handle_t *pHandle);

/*
 * Other Peripheral Control APIs
 */
void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t EnOrDi);
uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx , uint32_t FlagName);
void USART_ClearFlag(USART_RegDef_t *pUSARTx, uint16_t StatusFlagName);

/*
 * Application callback
 */
void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle,uint8_t AppEv);




/* --- USART Status Flags (USART_SR) --- */
#define USART_FLAG_PE        (1 << USART_SR_PE)        // Parity Error (Bit 0)
#define USART_FLAG_FE        (1 << USART_SR_FE)        // Framing Error (Bit 1)
#define USART_FLAG_NF        (1 << USART_SR_NF)        // Noise Flag (Bit 2)
#define USART_FLAG_ORE       (1 << USART_SR_ORE)       // Overrun Error (Bit 3)
#define USART_FLAG_IDLE      (1 << USART_SR_IDLE)      // IDLE line detected (Bit 4)
#define USART_FLAG_RXNE      (1 << USART_SR_RXNE)      // Read data reg not empty (Bit 5)
#define USART_FLAG_TC        (1 << USART_SR_TC)        // Transmission complete (Bit 6)
#define USART_FLAG_TXE       (1 << USART_SR_TXE)       // Transmit data reg empty (Bit 7)

/* --- USART Control Flags (USART_CR1) --- */
#define USART_CR1_RE_BIT     (1 << USART_CR1_RE)       // Receiver enable (Bit 2)
#define USART_CR1_TE_BIT     (1 << USART_CR1_TE)       // Transmitter enable (Bit 3)
#define USART_CR1_RXNEIE_BIT (1 << USART_CR1_RXNEIE)   // RXNE interrupt enable (Bit 5)
#define USART_CR1_TCIE_BIT   (1 << USART_CR1_TCIE)     // TC interrupt enable (Bit 6)
#define USART_CR1_TXEIE_BIT  (1 << USART_CR1_TXEIE)    // TXE interrupt enable (Bit 7)
#define USART_CR1_PCE_BIT    (1 << USART_CR1_PCE)      // Parity control enable (Bit 10)
#define USART_CR1_M_BIT      (1 << USART_CR1_M)        // Word length (0=8b, 1=9b) (Bit 12)
#define USART_CR1_UE_BIT     (1 << USART_CR1_UE)       // USART enable (Bit 13)
#define USART_CR1_OVER8_BIT  (1 << USART_CR1_OVER8)    // Oversampling mode (Bit 15)

/* --- USART Control Flags (USART_CR2 & CR3) --- */
#define USART_CR2_STOP_BIT   (3 << 12)                 // STOP bits mask (Bits 12:13)
#define USART_CR3_RTSE_BIT   (1 << USART_CR3_RTSE)     // RTS hardware flow control (Bit 8)
#define USART_CR3_CTSE_BIT   (1 << USART_CR3_CTSE)     // CTS hardware flow control (Bit 9)
#define USART_CR3_DMAT_BIT   (1 << USART_CR3_DMAT)     // DMA enable transmitter (Bit 7)
#define USART_CR3_DMAR_BIT   (1 << USART_CR3_DMAR)     // DMA enable receiver (Bit 6)








#endif /* INC_STM32F407XX_USART_DRIVER_H_ */
