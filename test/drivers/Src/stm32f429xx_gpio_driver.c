/*
 * stm32f429xx_gpio_driver.c
 *
 *  Created on: 19 jun. 2021
 *      Author: ruixi
 */

#include "stm32f429xx_gpio_driver.h"

// Peripheral clock setup
void GPIO_PClkControl(GPIO_RegDef_t *pGPIOx, uint8_t EnOrDi)
{
	if (EnOrDi == ENABLE)
	{
		if (pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		} else if (pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		} else if (pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		} else if (pGPIOx == GPIOD)
		{
			GPIOD_PCLK_EN();
		} else if (pGPIOx == GPIOE)
		{
			GPIOE_PCLK_EN();
		} else if (pGPIOx == GPIOF)
		{
			GPIOF_PCLK_EN();
		} else if (pGPIOx == GPIOG)
		{
			GPIOG_PCLK_EN();
		} else if (pGPIOx == GPIOH)
		{
			GPIOH_PCLK_EN();
		} else if (pGPIOx == GPIOI)
		{
			GPIOI_PCLK_EN();
		} else if (pGPIOx == GPIOJ)
		{
			GPIOJ_PCLK_EN();
		} else if (pGPIOx == GPIOK)
		{
			GPIOK_PCLK_EN();
		}
	} else {
		//TODO
	}
}


// Init and de-init
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	uint32_t aux;

	// Enable Clock
	GPIO_PClkControl(pGPIOHandle->pGPIOx, ENABLE);

	// 1. Configure moder
	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		// Clear moder
		pGPIOHandle->pGPIOx->MODER &= ~(0x3U << 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		// Set moder
		aux = pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		pGPIOHandle->pGPIOx->MODER |= aux;
	} else
	{
		// interrupts
	}

	// 2. Clear speed
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3U << 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	// 2. Set speed
	aux = pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OSPEEDR |= aux;

	// 3. Clear pull-up/pull-down
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3U << 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	// 3. Set pupd
	aux = pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->PUPDR |= aux;

	// 4. Clear output type
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x1U << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	// 4. Set output type
	aux = pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;
	pGPIOHandle->pGPIOx->OTYPER |= aux;

	// 5. Configure AltFunc
	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
	{
		uint8_t high, offset;

		high = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
		offset = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;

		// Clear AltFunc
		pGPIOHandle->pGPIOx->AFR[high] &= ~(0xFU << 4 * offset);
		// Set AltFunc
		pGPIOHandle->pGPIOx->AFR[high] |= pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * offset);
	}
}
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	if (pGPIOx == GPIOA)
	{
		GPIOA_REG_RESET();
	} else if (pGPIOx == GPIOB)
	{
		GPIOB_REG_RESET();
	} else if (pGPIOx == GPIOC)
	{
		GPIOC_REG_RESET();
	} else if (pGPIOx == GPIOD)
	{
		GPIOD_REG_RESET();
	} else if (pGPIOx == GPIOE)
	{
		GPIOE_REG_RESET();
	} else if (pGPIOx == GPIOF)
	{
		GPIOF_REG_RESET();
	} else if (pGPIOx == GPIOG)
	{
		GPIOG_REG_RESET();
	} else if (pGPIOx == GPIOH)
	{
		GPIOH_REG_RESET();
	} else if (pGPIOx == GPIOI)
	{
		GPIOI_REG_RESET();
	} else if (pGPIOx == GPIOJ)
	{
		GPIOJ_REG_RESET();
	} else if (pGPIOx == GPIOK)
	{
		GPIOK_REG_RESET();
	}
}


// Data read and write
uint8_t GPIO_ReadFromInputPin(GPIO_Handle_t *pGPIOHandle)
{
	// Shift to the left to move the input value to the last bit
	uint8_t value = (pGPIOHandle->pGPIOx->IDR >> pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) & 0x1U;
	return value;
}

uint16_t GPIO_ReadFromInputPort(GPIO_Handle_t *pGPIOHandle)
{
	uint16_t value = pGPIOHandle->pGPIOx->IDR;
	return value;
}

void GPIO_WriteToOutputPin(GPIO_Handle_t *pGPIOHandle, uint8_t value)
{
	if (value == GPIO_PIN_SET)
	{
		pGPIOHandle->pGPIOx->ODR |= 0x1U << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;
	} else if (value == GPIO_PIN_RESET)
	{
		pGPIOHandle->pGPIOx->ODR &= ~(0x1U << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	}
}

void GPIO_WriteToOutputPort(GPIO_Handle_t *pGPIOHandle, uint16_t value)
{
	pGPIOHandle->pGPIOx->ODR = value;
}

void GPIO_ToggleOutputPort(GPIO_Handle_t *pGPIOHandle)
{
	pGPIOHandle->pGPIOx->ODR ^= 0x1U << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;
}


// IRQ configuration and ISR handling
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnOrDi)
{

}
void GPIO_IRQHandling(uint8_t PinNumber)
{

}
