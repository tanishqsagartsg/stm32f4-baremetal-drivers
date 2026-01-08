/*
 * stm32f407xx_gpio_driver.c
 *
 *  Created on: Dec 29, 2025
 *      Author: tanis
 */


#include "../../Custom_Driver/Inc/stm32f407xx_gpio_driver.h"

#include "../../Custom_Driver/Inc/stm32f407xx.h"


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


//888888888888888888888888888888888888888888888888888888888888888888888
//===================   API SUPPORTED BY THIS DRIVER    ======================

/*====================================================================
 * FUNCTION:	GPIO_PeriClockControl
 *
 * BRIEF:
 *
 * PARAMETER 1: GPIO_RegDef_t *pGPIOx
 * PARAMETER 2: uint8_t EnorDi
 * PARAMETER 3:
 *
 * RETURN: 		NONE
 *
 * NOTE: 		NONE
 ====================================================================*/
//PERIPHERAL CLOCK SETUP===============================================
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
    if (EnorDi == EN)
    {
        if (pGPIOx == GPIOA)      { GPIOA_PCLK_EN(); }
        else if (pGPIOx == GPIOB) { GPIOB_PCLK_EN(); }
        else if (pGPIOx == GPIOC) { GPIOC_PCLK_EN(); }
        else if (pGPIOx == GPIOD) { GPIOD_PCLK_EN(); }
        else if (pGPIOx == GPIOE) { GPIOE_PCLK_EN(); }
        else if (pGPIOx == GPIOF) { GPIOF_PCLK_EN(); }
        else if (pGPIOx == GPIOG) { GPIOG_PCLK_EN(); }
        else if (pGPIOx == GPIOH) { GPIOH_PCLK_EN(); }
        else if (pGPIOx == GPIOI) { GPIOI_PCLK_EN(); }
    }
    else
    {
        if (pGPIOx == GPIOA)      { GPIOA_PCLK_DN(); }
        else if (pGPIOx == GPIOB) { GPIOB_PCLK_DN(); }
        else if (pGPIOx == GPIOC) { GPIOC_PCLK_DN(); }
        else if (pGPIOx == GPIOD) { GPIOD_PCLK_DN(); }
        else if (pGPIOx == GPIOE) { GPIOE_PCLK_DN(); }
        else if (pGPIOx == GPIOF) { GPIOF_PCLK_DN(); }
        else if (pGPIOx == GPIOG) { GPIOG_PCLK_DN(); }
        else if (pGPIOx == GPIOH) { GPIOH_PCLK_DN(); }
        else if (pGPIOx == GPIOI) { GPIOI_PCLK_DN(); }
    }
}
//====================================================================


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
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{

	uint32_t temp = 0;
	//CONFIGURE THE MODE OF THE GPIO PIN==============================
	//for non interrupt functionalities
	if(pGPIOHandle-> GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG){
		pGPIOHandle->pGPIOx->MODER &= ~(0x3<<pGPIOHandle-> GPIO_PinConfig.GPIO_PinNumber);//clearing
		temp = (pGPIOHandle-> GPIO_PinConfig.GPIO_PinMode << (2*pGPIOHandle-> GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER |= temp;
	}
	else{

	}
	//=================================================================

	temp=0;


	//CONFIGURE THE SPEED==============================================
	temp = (pGPIOHandle-> GPIO_PinConfig.GPIO_PinSpeed << (2*pGPIOHandle-> GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3<<pGPIOHandle-> GPIO_PinConfig.GPIO_PinNumber);//clearing
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;

	//=================================================================

	temp=0;

	//CONFIGURE THE PUPD SETTINGS======================================
	temp = (pGPIOHandle-> GPIO_PinConfig.GPIO_PinPuPdControl << (2*pGPIOHandle-> GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3<<pGPIOHandle-> GPIO_PinConfig.GPIO_PinNumber);//clearing
	pGPIOHandle->pGPIOx->PUPDR |= temp;

	//=================================================================

	temp=0;

	//CONFIGURE THE OPTYPE=============================================
	temp = (pGPIOHandle-> GPIO_PinConfig.GPIO_PinOPType << (pGPIOHandle-> GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x1<<pGPIOHandle-> GPIO_PinConfig.GPIO_PinNumber);//clearing
	pGPIOHandle->pGPIOx->OTYPER |= temp;

	//=================================================================

	temp=0;

	//CONFIGURE THE ALT FUNCTIONALITY==================================
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode==GPIO_MODE_ALTFN){
		uint8_t temp1, temp2;
		temp1 = (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8);
		temp2 = (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8);
		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF<<(4*temp2));//clearing
		pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle-> GPIO_PinConfig.GPIO_PinALtFunMode << (4*temp2));
	}
}
//=====================================================================




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
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	if (pGPIOx == GPIOA)      { GPIOA_REG_RESET(); }
	else if (pGPIOx == GPIOB) { GPIOB_REG_RESET(); }
	else if (pGPIOx == GPIOC) { GPIOC_REG_RESET(); }
	else if (pGPIOx == GPIOD) { GPIOD_REG_RESET(); }
	else if (pGPIOx == GPIOE) { GPIOE_REG_RESET(); }
	else if (pGPIOx == GPIOF) { GPIOF_REG_RESET(); }
	else if (pGPIOx == GPIOG) { GPIOG_REG_RESET(); }
	else if (pGPIOx == GPIOH) { GPIOH_REG_RESET(); }
	else if (pGPIOx == GPIOI) { GPIOI_REG_RESET(); }

}



//READ AND WRITE=======================================================
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
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t value;
	value= (uint8_t)(( pGPIOx->IDR >> PinNumber) & 0x00000001 );
	return value;

}

//====================================================================

uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value;
	value= (uint16_t)pGPIOx->IDR;
	return value;
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
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{
	if(Value==GPIO_PinSet){
		pGPIOx-> ODR |= (1 << PinNumber);
	}
	else{
		pGPIOx-> ODR &= ~(1 << PinNumber);
	}

}

//====================================================================
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{
	pGPIOx-> ODR = Value;

}

//====================================================================

void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR ^= (1 << PinNumber);

}



//INTERRUPT HANDLING===================================================
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
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnorDi)
{

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
void GPIO_IRQHandling(uint8_t PinNumber)
{

}





