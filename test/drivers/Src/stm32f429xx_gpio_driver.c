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
		uint8_t gpio = GPIO_BASEADDR_TO_CODE(pGPIOx);
		switch (gpio)
		{
		case 0:
			GPIOA_PCLK_EN();
			break;
		case 1:
			GPIOB_PCLK_EN();
			break;
		case 2:
			GPIOC_PCLK_EN();
			break;
		case 3:
			GPIOD_PCLK_EN();
			break;
		case 4:
			GPIOE_PCLK_EN();
			break;
		case 5:
			GPIOF_PCLK_EN();
			break;
		case 6:
			GPIOG_PCLK_EN();
			break;
		case 7:
			GPIOH_PCLK_EN();
			break;
		case 8:
			GPIOI_PCLK_EN();
			break;
		case 9:
			GPIOJ_PCLK_EN();
			break;
		case 10:
			GPIOK_PCLK_EN();
			break;
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
		pGPIOHandle->pGPIOx->MODER &= ~(0x3U << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		// Set moder
		aux = pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		pGPIOHandle->pGPIOx->MODER |= aux;
	} else
	{
		// interrupt mode
		// 1. configure FTSR or RTSR
		if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT)
		{
			// configure FTSR
			EXTI->FTSR |= 0x1U << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;
			// clear RTSR (for safety)
			EXTI->RTSR &= ~(0x1U << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		} else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT)
		{
			// configure RTSR
			EXTI->RTSR |= 0x1U << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;
			// clear FTSR (for safety)
			EXTI->FTSR &= ~(0x1U << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		} else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT)
		{
			// configure FTSR and RTSR
			EXTI->FTSR |= 0x1U << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;
			EXTI->RTSR |= 0x1U << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;
		}

		// 2. configure the GPIO port in SYSCFG_EXTICR
		uint8_t group = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
		uint8_t offset = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
		uint8_t portCode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		// enable clock
		SYSCFG_PCLK_EN();
		// clear
		SYSCFG->EXTICR[group] &= ~(0xFU << (4 * offset));
		// set
		SYSCFG->EXTICR[group] |= portCode << (4 * offset);

		// 3. enable EXTI interrupt with IMR
		EXTI->IMR |= 0x1U << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;
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
	uint8_t gpio = GPIO_BASEADDR_TO_CODE(pGPIOx);
	switch (gpio)
	{
	case 0:
		GPIOA_REG_RESET();
		break;
	case 1:
		GPIOB_REG_RESET();
		break;
	case 2:
		GPIOC_REG_RESET();
		break;
	case 3:
		GPIOD_REG_RESET();
		break;
	case 4:
		GPIOE_REG_RESET();
		break;
	case 5:
		GPIOF_REG_RESET();
		break;
	case 6:
		GPIOG_REG_RESET();
		break;
	case 7:
		GPIOH_REG_RESET();
		break;
	case 8:
		GPIOI_REG_RESET();
		break;
	case 9:
		GPIOJ_REG_RESET();
		break;
	case 10:
		GPIOK_REG_RESET();
		break;
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
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi)
{
	if (EnOrDi == ENABLE)
	{
		/*
		 * 1. Configure ISERx. Interrupt set-enable register x
		 */
		switch (IRQNumber/32)
		{
		// IRQ from 0 to 31
		case 0:
			// configure ISER0
			*NVIC_ISER0 |= (0x1U << IRQNumber);
			break;
		// IRQ from 32 to 63
		case 1:
			// configure ISER1
			*NVIC_ISER1 |= (0x1U << (IRQNumber-32));
			break;
		// IRQ from 64 to 95
		case 2:
			// configure ISER2
			*NVIC_ISER2 |= (0x1U << (IRQNumber-64));
			break;
		// enough because MCU only uses up to 90 interruptions
		}
	} else
	{
		/*
		 * 1. Configure ICERx. Interrupt clear-enable register x
		 */
		switch (IRQNumber/32)
		{
		// IRQ from 0 to 31
		case 0:
			// configure ISER0
			*NVIC_ICER0 |= (0x1U << IRQNumber);
			break;
		// IRQ from 32 to 63
		case 1:
			// configure ISER1
			*NVIC_ICER1 |= (0x1U << (IRQNumber-32));
			break;
		// IRQ from 64 to 95
		case 2:
			// configure ISER2
			*NVIC_ICER2 |= (0x1U << (IRQNumber-64));
			break;
		// enough because MCU only uses up to 90 interruptions
		}
	}
}


void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority)
{
	// 1. Calculate IPR register
	uint8_t iprx = IRQNumber/4;
	uint8_t offset = IRQNumber%4;

	// clear
	*(NVIC_IPR + iprx*4) &= ~(0xFFU << (offset*8));
	// set
	uint8_t shift_amount = (offset*8) + (8-NO_PR_BITS_IMPLEMENTED);
	*(NVIC_IPR + iprx*4) |= IRQPriority << shift_amount;
}

// Clear pending register to avoid infinite interrupts
void GPIO_IRQHandling(uint8_t PinNumber)
{
	// Clear exti pr register corresponding to the pin
	if (EXTI->PR & (0x1U << PinNumber))
	{
		EXTI->PR |= (0x1U << PinNumber);
	}
}
