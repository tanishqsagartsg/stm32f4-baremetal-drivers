/*
 * stm32f103xx.h
 *
 *  Created on: Dec 23, 2025
 *      Author: tanis
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_
#include<stdint.h>



//PROCESSOR SPECIFIC REGISTER ADDRESSES========================================================================================

//NVIC ISER REGISTER ADDRESSES=================================================================================================
#define NVIC_ISER0 			((volatile uint32_t*)0xE000E100)
#define NVIC_ISER1 			((volatile uint32_t*)0xE000E104)
#define NVIC_ISER2 			((volatile uint32_t*)0xE000E108)
#define NVIC_ISER3 			((volatile uint32_t*)0xE000E10C)

//NVIC ICER REGISTER ADDRESSES=================================================================================================
#define NVIC_ICER0 			((volatile uint32_t*)0xE000E180)
#define NVIC_ICER1 			((volatile uint32_t*)0xE000E184)
#define NVIC_ICER2 			((volatile uint32_t*)0xE000E188)
#define NVIC_ICER3 			((volatile uint32_t*)0xE000E18C)
//=============================================================================================================================

#define NVIC_PR_BASE_ADDR 	((volatile uint32_t*)0xE000E400)
#define NO_PR_BITS_IMPLEMENTED 		4


//SOME GENERIC MACROS==========================================================================================================
#define EN         				1U
#define DI       				0U
#define GPIO_PinSet       		EN
#define GPIO_PinReset      		DI

//=============================================================================================================================




// MEMORY ELEMENTS ADDRESSES===================================================================================================
#define FLASH_BASE_ADDR 			0x08000000
#define SRAM1_BASE_ADDR 			0x20000000
#define SRAM 						SRAM1_BASE_ADDR
#define SRAM2_BASE_ADDR 			0x2001C000
#define ROM_BASE_ADDR 				0x1FFF0000
//=============================================================================================================================



//MEMORY BUSES ADDRESSES=======================================================================================================
#define PERIPH_BASE_ADDR 			0x40000000
#define APB1_BASE_ADDR 				PERIPH_BASE_ADDR
#define APB2_BASE_ADDR 				0x40010000
#define AHB1_BASE_ADDR 				0x40020000
#define AHB2_BASE_ADDR 				0x50000000
//=============================================================================================================================


//PERIPHERALS ATTACHED TO AHB1 BUS BASE ADDR===================================================================================
#define GPIOA_BASE_ADDR 			(AHB1_BASE_ADDR + 0X0000)
#define GPIOB_BASE_ADDR 			(AHB1_BASE_ADDR + 0X0400)
#define GPIOC_BASE_ADDR 			(AHB1_BASE_ADDR + 0X0800)
#define GPIOD_BASE_ADDR 			(AHB1_BASE_ADDR + 0X0C00)
#define GPIOE_BASE_ADDR 			(AHB1_BASE_ADDR + 0X1000)
#define GPIOF_BASE_ADDR 			(AHB1_BASE_ADDR + 0X1400)
#define GPIOG_BASE_ADDR 			(AHB1_BASE_ADDR + 0X1800)
#define GPIOH_BASE_ADDR 			(AHB1_BASE_ADDR + 0X1C00)
#define GPIOI_BASE_ADDR 			(AHB1_BASE_ADDR + 0X2000)
#define RCC_BASE_ADDR				(AHB1_BASE_ADDR + 0X3800)
//=============================================================================================================================


//PHERIPHERALS ATTACHED TO APB1 BASE ADDR======================================================================================
#define I2C1_BASE_ADDR				(APB1_BASE_ADDR + 0X5400)
#define I2C2_BASE_ADDR				(APB1_BASE_ADDR + 0X5800)
#define I2C3_BASE_ADDR				(APB1_BASE_ADDR + 0X5C00)

#define SPI2_BASE_ADDR				(APB1_BASE_ADDR + 0X3800)
#define SPI3_BASE_ADDR				(APB1_BASE_ADDR + 0X3C00)

#define USART2_BASE_ADDR			(APB1_BASE_ADDR + 0X4400)
#define USART3_BASE_ADDR			(APB1_BASE_ADDR + 0X4800)
#define UART4_BASE_ADDR				(APB1_BASE_ADDR + 0X4C00)
#define UART5_BASE_ADDR				(APB1_BASE_ADDR + 0X5000)
//=============================================================================================================================


//PERIPHERAL ATTACHED TO APB2 BASE ADDR========================================================================================
#define EXTI_BASE_ADDR 				(APB2_BASE_ADDR + 0X3C00)
#define SPI1_BASE_ADDR 				(APB2_BASE_ADDR + 0X3000)
#define SYSCFG_BASE_ADDR 			(APB2_BASE_ADDR + 0X3800)
#define USART1_BASE_ADDR 			(APB2_BASE_ADDR + 0X1000)
#define USART6_BASE_ADDR 			(APB2_BASE_ADDR + 0X1400)
//=============================================================================================================================




//REGISTER PRESENT IN GPIO BASE ADDR===========================================================================================
typedef struct
{
	volatile uint32_t MODER;
	volatile uint32_t OTYPER;
	volatile uint32_t OSPEEDR;
	volatile uint32_t PUPDR;
	volatile uint32_t IDR;
	volatile uint32_t ODR;
	volatile uint32_t BSSRL;
	volatile uint32_t BSSRH;
	volatile uint32_t LKR;
	volatile uint32_t AFR[2];
}GPIO_RegDef_t;
//=============================================================================================================================




//RCC CLOCK PERIPHERAL REGISTER BASE ADDR======================================================================================
typedef struct
{
	volatile uint32_t CR;
	volatile uint32_t PLLCFGR;
	volatile uint32_t CFGR;
	volatile uint32_t CIR;
	volatile uint32_t AHB1RSTR;
	volatile uint32_t AHB2RSTR;
	volatile uint32_t AHB3RSTR;
	uint32_t RESERVED0;
	volatile uint32_t APB1RSTR;
	volatile uint32_t APB2RSTR;
	uint32_t RESRVED1[2];
	volatile uint32_t AHB1ENR;
	volatile uint32_t AHB2ENR;
	volatile uint32_t AHB3ENR;
	uint32_t RESERVED2;
	volatile uint32_t APB1ENR;
	volatile uint32_t APB2ENR;
	uint32_t RESRVED3[2];
	volatile uint32_t AHB1LPENR;
	volatile uint32_t AHB2LPENR;
	volatile uint32_t AHB3LPENR;
	uint32_t RESERVED4;
	volatile uint32_t APB1LPENR;
	volatile uint32_t APB2LPENR;
	uint32_t RESRVED5[2];
	volatile uint32_t BDCR;
	volatile uint32_t CSR;
	uint32_t RESRVED6[2];
	volatile uint32_t SSCGR;
	volatile uint32_t PLLI2SCFGR;
}RCC_RefDef_t;
//=================================================================================================================================


//=================================================================================================================================
typedef struct{
	volatile uint32_t IMR;
	volatile uint32_t EMR;
	volatile uint32_t RTSR;
	volatile uint32_t FTSR;
	volatile uint32_t SWIER;
	volatile uint32_t PR;
}EXTI_RegDef_t;
//=================================================================================================================================

typedef struct{
	volatile uint32_t MEMRMP;
	volatile uint32_t PMC;
	volatile uint32_t EXTICR[4];
	uint32_t RESERVED1[2];
	volatile uint32_t CMPCR;
	uint32_t RESERVED2[2];
	volatile uint32_t CFGR;
}SYSCFG_RegDef_t;



//INTRODUCING WHAT THE STRUCT ABOVE HOLDS AS BASE ADDR FOR EACH GPIO PIN=========================================================================
#define GPIOA ((GPIO_RegDef_t*)GPIOA_BASE_ADDR)
#define GPIOB ((GPIO_RegDef_t*)GPIOB_BASE_ADDR)
#define GPIOC ((GPIO_RegDef_t*)GPIOC_BASE_ADDR)
#define GPIOD ((GPIO_RegDef_t*)GPIOD_BASE_ADDR)
#define GPIOE ((GPIO_RegDef_t*)GPIOE_BASE_ADDR)
#define GPIOF ((GPIO_RegDef_t*)GPIOF_BASE_ADDR)
#define GPIOG ((GPIO_RegDef_t*)GPIOG_BASE_ADDR)
#define GPIOH ((GPIO_RegDef_t*)GPIOH_BASE_ADDR)
#define GPIOI ((GPIO_RegDef_t*)GPIOI_BASE_ADDR)
//================================================================================================================================================

//
#define RCC 	((RCC_RefDef_t*)RCC_BASE_ADDR)
//


//
#define EXTI  	((EXTI_RegDef_t*)EXTI_BASE_ADDR)
//

//
#define SYSCFG	((SYSCFG_RegDef_t*)SYSCFG_BASE_ADDR)
//

//CLOCK ENABLE MACROS FOR GPIO CLOCK ENABLE==========================================================================================
#define GPIOA_PCLK_EN() (RCC -> AHB1LPENR |= (1<<0))
#define GPIOB_PCLK_EN() (RCC -> AHB1LPENR |= (1<<1))
#define GPIOC_PCLK_EN() (RCC -> AHB1LPENR |= (1<<2))
#define GPIOD_PCLK_EN() (RCC -> AHB1LPENR |= (1<<3))
#define GPIOE_PCLK_EN() (RCC -> AHB1LPENR |= (1<<4))
#define GPIOF_PCLK_EN() (RCC -> AHB1LPENR |= (1<<5))
#define GPIOG_PCLK_EN() (RCC -> AHB1LPENR |= (1<<6))
#define GPIOH_PCLK_EN() (RCC -> AHB1LPENR |= (1<<7))
#define GPIOI_PCLK_EN() (RCC -> AHB1LPENR |= (1<<8))
//====================================================================================================================================



//CLOCK DISABLE MACROS FOR GPIO CLOCK ENABLE==========================================================================================
#define GPIOA_PCLK_DN() (RCC -> AHB1LPENR &= ~(1<<0))
#define GPIOB_PCLK_DN() (RCC -> AHB1LPENR &= ~(1<<1))
#define GPIOC_PCLK_DN() (RCC -> AHB1LPENR &= ~(1<<2))
#define GPIOD_PCLK_DN() (RCC -> AHB1LPENR &= ~(1<<3))
#define GPIOE_PCLK_DN() (RCC -> AHB1LPENR &= ~(1<<4))
#define GPIOF_PCLK_DN() (RCC -> AHB1LPENR &= ~(1<<5))
#define GPIOG_PCLK_DN() (RCC -> AHB1LPENR &= ~(1<<6))
#define GPIOH_PCLK_DN() (RCC -> AHB1LPENR &= ~(1<<7))
#define GPIOI_PCLK_DN() (RCC -> AHB1LPENR &= ~(1<<8))
//====================================================================================================================================



/* ==============================================================================================================================
 * CLOCK ENABLE MACROS
 * ============================================================ */
/* ---------------- APB1 : I2C ---------------- */
#define I2C1_PCLK_EN()    (RCC->APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN()    (RCC->APB1ENR |= (1 << 22))
#define I2C3_PCLK_EN()    (RCC->APB1ENR |= (1 << 23))

/* ---------------- APB1 : SPI ---------------- */
#define SPI2_PCLK_EN()    (RCC->APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN()    (RCC->APB1ENR |= (1 << 15))

/* ---------------- APB2 : SPI ---------------- */
#define SPI1_PCLK_EN()    (RCC->APB2ENR |= (1 << 12))

/* ---------------- APB1 : UART / USART ---------------- */
#define USART2_PCLK_EN()  (RCC->APB1ENR |= (1 << 17))
#define USART3_PCLK_EN()  (RCC->APB1ENR |= (1 << 18))
#define UART4_PCLK_EN()   (RCC->APB1ENR |= (1 << 19))
#define UART5_PCLK_EN()   (RCC->APB1ENR |= (1 << 20))

/* ---------------- APB2 : UART / USART ---------------- */
#define USART1_PCLK_EN()  (RCC->APB2ENR |= (1 << 4))
#define USART6_PCLK_EN()  (RCC->APB2ENR |= (1 << 5))

/* ---------------- APB2 : SYSCFG ---------------- */
#define SYSCFG_PCLK_EN()  (RCC->APB2ENR |= (1 << 14))
//====================================================================================================================================





/* ==============================================================================================================================
 * CLOCK DISABLE MACROS
 * ============================================================ */
/* ---------------- APB1 : I2C ---------------- */
#define I2C1_PCLK_DN()    (RCC->APB1ENR &= ~(1 << 21))
#define I2C2_PCLK_DN()    (RCC->APB1ENR &= ~(1 << 22))
#define I2C3_PCLK_DN()    (RCC->APB1ENR &= ~(1 << 23))

/* ---------------- APB1 : SPI ---------------- */
#define SPI2_PCLK_DN()    (RCC->APB1ENR &= ~(1 << 14))
#define SPI3_PCLK_DN()    (RCC->APB1ENR &= ~(1 << 15))

/* ---------------- APB2 : SPI ---------------- */
#define SPI1_PCLK_DN()    (RCC->APB2ENR &= ~(1 << 12))

/* ---------------- APB1 : UART / USART ---------------- */
#define USART2_PCLK_DN()  (RCC->APB1ENR &= ~(1 << 17))
#define USART3_PCLK_DN()  (RCC->APB1ENR &= ~(1 << 18))
#define UART4_PCLK_DN()   (RCC->APB1ENR &= ~(1 << 19))
#define UART5_PCLK_DN()   (RCC->APB1ENR &= ~(1 << 20))

/* ---------------- APB2 : UART / USART ---------------- */
#define USART1_PCLK_DN()  (RCC->APB2ENR &= ~(1 << 4))
#define USART6_PCLK_DN()  (RCC->APB2ENR &= ~(1 << 5))

/* ---------------- APB2 : SYSCFG ---------------- */
#define SYSCFG_PCLK_DN()  (RCC->APB2ENR &= ~(1 << 14))
//====================================================================================================================================




//MACROS TO RESET GPIO PERIPHERALS===================================================================================================
#define GPIOA_REG_RESET()	do{(RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~(1 << 0)); } while(0)
#define GPIOB_REG_RESET()	do{(RCC->AHB1RSTR |= (1 << 1)); (RCC->AHB1RSTR &= ~(1 << 1)); } while(0)
#define GPIOC_REG_RESET()	do{(RCC->AHB1RSTR |= (1 << 2)); (RCC->AHB1RSTR &= ~(1 << 2)); } while(0)
#define GPIOD_REG_RESET()	do{(RCC->AHB1RSTR |= (1 << 3)); (RCC->AHB1RSTR &= ~(1 << 3)); } while(0)
#define GPIOE_REG_RESET()	do{(RCC->AHB1RSTR |= (1 << 4)); (RCC->AHB1RSTR &= ~(1 << 4)); } while(0)
#define GPIOF_REG_RESET()	do{(RCC->AHB1RSTR |= (1 << 5)); (RCC->AHB1RSTR &= ~(1 << 5)); } while(0)
#define GPIOG_REG_RESET()	do{(RCC->AHB1RSTR |= (1 << 6)); (RCC->AHB1RSTR &= ~(1 << 6)); } while(0)
#define GPIOH_REG_RESET()	do{(RCC->AHB1RSTR |= (1 << 7)); (RCC->AHB1RSTR &= ~(1 << 7)); } while(0)
#define GPIOI_REG_RESET()	do{(RCC->AHB1RSTR |= (1 << 8)); (RCC->AHB1RSTR &= ~(1 << 8)); } while(0)
//==================================================================


//====================================================================================================================================
#define GPIO_BASE_ADDR_TO_CODE(x) 	((x==GPIOA)?0:\
									(x==GPIOB)?1:\
									(x==GPIOC)?2:\
									(x==GPIOD)?3:\
									(x==GPIOE)?4:\
									(x==GPIOF)?5:\
									(x==GPIOG)?6:\
									(x==GPIOH)?7:\
									(x==GPIOI)?8:0)
//====================================================================================================================================


//====================================================================================================================================
#define IRQ_NO_EXTI0		6
#define IRQ_NO_EXTI1		7
#define IRQ_NO_EXTI2		8
#define IRQ_NO_EXTI3		9
#define IRQ_NO_EXTI4		10
#define IRQ_NO_EXTI9_5		23
#define IRQ_NO_EXTI10_15	40
//====================================================================================================================================



//MACROS FOR ALL THE PRIORITY LEVELS=========
#define NVIC_IRQ_PRI0		0
#define NVIC_IRQ_PRI1		1
#define NVIC_IRQ_PRI2		2
#define NVIC_IRQ_PRI3		3
#define NVIC_IRQ_PRI4		4
#define NVIC_IRQ_PRI5		5
#define NVIC_IRQ_PRI6		6
#define NVIC_IRQ_PRI7		7
#define NVIC_IRQ_PRI8		8
#define NVIC_IRQ_PRI9		9
#define NVIC_IRQ_PRI10		10
#define NVIC_IRQ_PRI11		11
#define NVIC_IRQ_PRI12		12
#define NVIC_IRQ_PRI13		13
#define NVIC_IRQ_PRI14		14
#define NVIC_IRQ_PRI15		15


#endif /* INC_STM32F407XX_H_ */
