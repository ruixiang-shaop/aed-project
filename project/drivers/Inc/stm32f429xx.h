/*
 * stm32f429xx.h
 *
 *  Created on: Jun 19, 2021
 *      Author: ruixi
 */

#ifndef INC_STM32F429XX_H_
#define INC_STM32F429XX_H_

#include <stdint.h>
#include <stddef.h>

/*
 * ARM Cortex Mx Processor NVIC register Addresses
 */

#define NVIC_BASEADDR			0xE000E000
#define NVIC_ISER_BASEADDR		(NVIC_BASEADDR + 0x100U)
#define NVIC_ISER0				((volatile uint32_t*)(NVIC_ISER_BASEADDR + 0x00U))
#define NVIC_ISER1				((volatile uint32_t*)(NVIC_ISER_BASEADDR + 0x04U))
#define NVIC_ISER2				((volatile uint32_t*)(NVIC_ISER_BASEADDR + 0x08U))

#define NVIC_ICER_BASEADDR		(NVIC_BASEADDR + 0x180U)
#define NVIC_ICER0				((volatile uint32_t*)(NVIC_ICER_BASEADDR + 0x00U))
#define NVIC_ICER1				((volatile uint32_t*)(NVIC_ICER_BASEADDR + 0x04U))
#define NVIC_ICER2				((volatile uint32_t*)(NVIC_ICER_BASEADDR + 0x08U))

#define NVIC_IPR				((volatile uint32_t*)(NVIC_BASEADDR + 0x400U))

/*
 * ARM Cortex Mx Processor number of priority bits implemented in Priority Register
 */
#define NO_PR_BITS_IMPLEMENTED	4

/*
 * Base addresses of Flash and SRAM memories
 */

#define FLASH_BASEADDR			0x08000000U
#define SRAM1_BASEADDR			0x20000000U
#define SRAM2_BASEADDR			0x2001C000U
#define ROM_BASEADDR			0x1FFF0000U
#define SRAM					SRAM1_BASEADDR

/*
 * AHBx and APBx Bus Peripheral base addresses
 */

#define APB1PERIPH_BASEADDR		0x40000000U
#define APB2PERIPH_BASEADDR		0x40010000U
#define AHB1PERIPH_BASEADDR		0x40020000U
#define AHB2PERIPH_BASEADDR		0x50000000U

/*
 * Base addresses for GPIO
 */

#define GPIOA_BASEADDR			(AHB1PERIPH_BASEADDR + 0x0000)
#define GPIOB_BASEADDR			(AHB1PERIPH_BASEADDR + 0x0400)
#define GPIOC_BASEADDR			(AHB1PERIPH_BASEADDR + 0x0800)
#define GPIOD_BASEADDR			(AHB1PERIPH_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR			(AHB1PERIPH_BASEADDR + 0x1000)
#define GPIOF_BASEADDR			(AHB1PERIPH_BASEADDR + 0x1400)
#define GPIOG_BASEADDR			(AHB1PERIPH_BASEADDR + 0x1800)
#define GPIOH_BASEADDR			(AHB1PERIPH_BASEADDR + 0x1C00)
#define GPIOI_BASEADDR			(AHB1PERIPH_BASEADDR + 0x2000)
#define GPIOJ_BASEADDR			(AHB1PERIPH_BASEADDR + 0x2400)
#define GPIOK_BASEADDR			(AHB1PERIPH_BASEADDR + 0x2800)
#define RCC_BASEADDR			(AHB1PERIPH_BASEADDR + 0x3800)

/*
 * Base addresses for I2Cx
 */

#define I2C1_BASEADDR			(APB1PERIPH_BASEADDR + 0x5400)
#define I2C2_BASEADDR			(APB1PERIPH_BASEADDR + 0x5800)
#define I2C3_BASEADDR			(APB1PERIPH_BASEADDR + 0x5C00)

/*
 * Base addresses for CANx
 */

#define CAN1_BASEADDR			(APB1PERIPH_BASEADDR + 0x6400)
#define CAN2_BASEADDR			(APB1PERIPH_BASEADDR + 0x6800)

/*
 * Base address for other peripherals
 */
#define EXTI_BASEADDR			(APB2PERIPH_BASEADDR + 0x3C00)
#define SYSCFG_BASEADDR			(APB2PERIPH_BASEADDR + 0x3800)

/******************** Peripheral register definition structures ********************/

typedef struct
{								// Offset	Definition
	volatile uint32_t MODER;	// 0x00		GPIO port mode register
	volatile uint32_t OTYPER;	// 0x04		GPIO port output type register
	volatile uint32_t OSPEEDR;	// 0x08		GPIO port output speed register
	volatile uint32_t PUPDR;	// 0x0C		GPIO port pull-up/pull-down register
	volatile uint32_t IDR;		// 0x10		GPIO port input data register
	volatile uint32_t ODR;		// 0x14		GPIO port output data register
	volatile uint32_t BSRR;		// 0x18		GPIO port bit set/reset register
	volatile uint32_t LCKR;		// 0x1C		GPIO port configuration lock register
	volatile uint32_t AFR[2];	// 0x20		GPIO alternate function low register
								// 0x24		GPIO alternate function high register
} GPIO_RegDef_t;

typedef struct
{									// Offset	Definition
	volatile uint32_t CR;			// 0x00		RCC clock control register
	volatile uint32_t PLLCFGR;		// 0x04		RCC PLL configuration register
	volatile uint32_t CFGR;			// 0x08		RCC clock configuration register
	volatile uint32_t CIR;			// 0x0C		RCC clock interrupt register
	volatile uint32_t AHB1RSTR;		// 0x10		RCC AHB1 peripheral reset register
	volatile uint32_t AHB2RSTR;		// 0x14		RCC AHB2 peripheral reset register
	volatile uint32_t AHB3RSTR;		// 0x18		RCC AHB3 peripheral reset register
	uint32_t RESERVED1;				// 0x1C		Reserved
	volatile uint32_t APB1RSTR;		// 0x20		RCC APB1 peripheral reset register
	volatile uint32_t APB2RSTR;		// 0x24		RCC APB2 peripheral reset register
	uint32_t RESERVED2[2];			// 0x28		Reserved
	volatile uint32_t AHB1ENR;		// 0x30		RCC AHB1 peripheral clock enable register
	volatile uint32_t AHB2ENR;		// 0x34		RCC AHB2 peripheral clock enable register
	volatile uint32_t AHB3ENR;		// 0x38		RCC AHB3 peripheral clock enable register
	uint32_t RESERVED3;				// 0x3C		Reserved
	volatile uint32_t APB1ENR;		// 0x40		RCC APB1 peripheral clock enable register
	volatile uint32_t APB2ENR;		// 0x44		RCC APB2 peripheral clock enable register
	uint32_t RESERVED4[2];			// 0x48		Reserved
	volatile uint32_t AHB1LPENR;	// 0x50		RCC AHB1 peripheral clock enable in low power mode register
	volatile uint32_t AHB2LPENR;	// 0x54		RCC AHB2 peripheral clock enable in low power mode register
	volatile uint32_t AHB3LPENR;	// 0x58		RCC AHB3 peripheral clock enable in low power mode register
	uint32_t RESERVED5;				// 0x5C		Reserved
	volatile uint32_t APB1LPENR;	// 0x60		RCC APB1 peripheral clock enable in low power mode register
	volatile uint32_t APB2LPENR;	// 0x64		RCC APB2 peripheral clock enable in low power mode register
	uint32_t RESERVED6[2];			// 0x68		Reserved
	volatile uint32_t BDCR;			// 0x70		RCC Backup domain control register
	volatile uint32_t CSR;			// 0x74		RCC clock control & status register
	uint32_t RESERVED7[2];			// 0x78		Reserved
	volatile uint32_t SSCGR;		// 0x80		RCC spread spectrum clock generation register
	volatile uint32_t PLLI2SCFGR;	// 0x84		RCC PLLI2S configuration register
	volatile uint32_t PLLSAICFGR;	// 0x88		RCC PLL configuration register
	volatile uint32_t DCKCFGR;		// 0x8C		RCC Dedicated Clock Configuration Register
} RCC_RegDef_t;

typedef struct
{									// Offset	Definition
	volatile uint32_t IMR;			// 0x00		Interrupt mask register
	volatile uint32_t EMR;			// 0x04		Event mask register
	volatile uint32_t RTSR;			// 0x08		Rising trigger selection register
	volatile uint32_t FTSR;			// 0x0C		Falling trigger selection register
	volatile uint32_t SWIER;		// 0x10		Software interrupt event register
	volatile uint32_t PR;			// 0x14		Pending register
} EXTI_RegDef_t;

typedef struct
{									// Offset	Definition
	volatile uint32_t MEMRMP;		// 0x00		SYSCFG memory remap register
	volatile uint32_t PMC;			// 0x04		SYSCFG peripheral mode configuration register
	volatile uint32_t EXTICR[4];	// 0x08		SYSCFG external interrupt configuration register 1-4
	uint32_t RESERVED1[2];			// 0x10		Reserved
	volatile uint32_t CMPCR;		// 0x1C		Compensation cell control register
} SYSCFG_RegDef_t;

typedef struct
{									// Offset	Definition
	volatile uint32_t CR1;	 		// 0x00		I2C Control register 1
	volatile uint32_t CR2;			// 0x04		I2C Control register 2
	volatile uint32_t OAR1;			// 0x08		I2C Own address register 1
	volatile uint32_t OAR2;			// 0x0C		I2C Own address register 2
	volatile uint32_t DR;			// 0x10		I2C Data register
	volatile uint32_t SR1;			// 0x14		I2C Status register 1
	volatile uint32_t SR2;			// 0x18		I2C Status register 2
	volatile uint32_t CCR;			// 0x1C		I2C Clock control register
	volatile uint32_t TRISE;		// 0x20		I2C TRISE register
	volatile uint32_t FLTR;			// 0x24		I2C FLTR register
} I2C_RegDef_t;

/***************************** Peripheral definitions *****************************/

#define GPIOA		((GPIO_RegDef_t*) GPIOA_BASEADDR)
#define GPIOB		((GPIO_RegDef_t*) GPIOB_BASEADDR)
#define GPIOC		((GPIO_RegDef_t*) GPIOC_BASEADDR)
#define GPIOD		((GPIO_RegDef_t*) GPIOD_BASEADDR)
#define GPIOE		((GPIO_RegDef_t*) GPIOE_BASEADDR)
#define GPIOF		((GPIO_RegDef_t*) GPIOF_BASEADDR)
#define GPIOG 		((GPIO_RegDef_t*) GPIOG_BASEADDR)
#define GPIOH 		((GPIO_RegDef_t*) GPIOH_BASEADDR)
#define GPIOI 		((GPIO_RegDef_t*) GPIOI_BASEADDR)
#define GPIOJ 		((GPIO_RegDef_t*) GPIOJ_BASEADDR)
#define GPIOK 		((GPIO_RegDef_t*) GPIOK_BASEADDR)

#define RCC 		((RCC_RegDef_t*) RCC_BASEADDR)

#define EXTI		((EXTI_RegDef_t*) EXTI_BASEADDR)

#define SYSCFG		((SYSCFG_RegDef_t*) SYSCFG_BASEADDR)

#define I2C1		((I2C_RegDef_t*) I2C1_BASEADDR)
#define I2C2		((I2C_RegDef_t*) I2C2_BASEADDR)
#define I2C3		((I2C_RegDef_t*) I2C3_BASEADDR)

/*
 * Clock enable macros for GPIOx
 */

#define GPIOA_PCLK_EN()	(RCC->AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN()	(RCC->AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN()	(RCC->AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN()	(RCC->AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN()	(RCC->AHB1ENR |= (1 << 4))
#define GPIOF_PCLK_EN()	(RCC->AHB1ENR |= (1 << 5))
#define GPIOG_PCLK_EN()	(RCC->AHB1ENR |= (1 << 6))
#define GPIOH_PCLK_EN()	(RCC->AHB1ENR |= (1 << 7))
#define GPIOI_PCLK_EN()	(RCC->AHB1ENR |= (1 << 8))
#define GPIOJ_PCLK_EN()	(RCC->AHB1ENR |= (1 << 9))
#define GPIOK_PCLK_EN()	(RCC->AHB1ENR |= (1 << 10))

/*
 * Reset GPIOx peripherals
 */

#define GPIOA_REG_RESET()	{RCC->AHB1RSTR |= (1 << 0); RCC->AHB1RSTR &= ~(1 << 0);}
#define GPIOB_REG_RESET()	{RCC->AHB1RSTR |= (1 << 1); RCC->AHB1RSTR &= ~(1 << 1);}
#define GPIOC_REG_RESET()	{RCC->AHB1RSTR |= (1 << 2); RCC->AHB1RSTR &= ~(1 << 2);}
#define GPIOD_REG_RESET()	{RCC->AHB1RSTR |= (1 << 3); RCC->AHB1RSTR &= ~(1 << 3);}
#define GPIOE_REG_RESET()	{RCC->AHB1RSTR |= (1 << 4); RCC->AHB1RSTR &= ~(1 << 4);}
#define GPIOF_REG_RESET()	{RCC->AHB1RSTR |= (1 << 5); RCC->AHB1RSTR &= ~(1 << 5);}
#define GPIOG_REG_RESET()	{RCC->AHB1RSTR |= (1 << 6); RCC->AHB1RSTR &= ~(1 << 6);}
#define GPIOH_REG_RESET()	{RCC->AHB1RSTR |= (1 << 7); RCC->AHB1RSTR &= ~(1 << 7);}
#define GPIOI_REG_RESET()	{RCC->AHB1RSTR |= (1 << 8); RCC->AHB1RSTR &= ~(1 << 8);}
#define GPIOJ_REG_RESET()	{RCC->AHB1RSTR |= (1 << 9); RCC->AHB1RSTR &= ~(1 << 9);}
#define GPIOK_REG_RESET()	{RCC->AHB1RSTR |= (1 << 10); RCC->AHB1RSTR &= ~(1 << 10);}


/*
 * Clock enable macros for I2Cx
 */

#define I2C1_PCLK_EN()	(RCC->APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN()	(RCC->APB1ENR |= (1 << 22))
#define I2C3_PCLK_EN()	(RCC->APB1ENR |= (1 << 23))

/*
 * Reset I2Cx peripherals
 */

#define I2C1_REG_RESET()	{RCC->APB1RSTR |= (1 << 21); RCC->APB1RSTR &= ~(1 << 21);}
#define I2C2_REG_RESET()	{RCC->APB1RSTR |= (1 << 22); RCC->APB1RSTR &= ~(1 << 22);}
#define I2C3_REG_RESET()	{RCC->APB1RSTR |= (1 << 23); RCC->APB1RSTR &= ~(1 << 23);}

/*
 * Clock enable macros for CANx
 */

#define CAN1_PCLK_EN()	(RCC->APB1ENR |= (1 << 25))
#define CAN2_PCLK_EN()	(RCC->APB1ENR |= (1 << 26))

/*
 * Clock enable macros for SYSCFG
 */
#define SYSCFG_PCLK_EN()	(RCC->APB2ENR |= (1 << 14))
#define GPIO_BASEADDR_TO_CODE(x)	(((void*)x-(void*)GPIOA)/0x0400U)

/*
 * Interrupt request number
 */
#define IRQ_NO_EXTI0		6
#define IRQ_NO_EXTI1		7
#define IRQ_NO_EXTI2		8
#define IRQ_NO_EXTI3		9
#define IRQ_NO_EXTI4		10
#define IRQ_NO_EXTI9_5		23
#define IRQ_NO_EXTI15_10	40
#define IRQ_NO_I2C1_EV		31
#define IRQ_NO_I2C1_ER		32
#define IRQ_NO_I2C2_EV		33
#define IRQ_NO_I2C2_ER		34
#define IRQ_NO_I2C3_EV		72
#define IRQ_NO_I2C3_ER		73

/*
 * Interrupt priority levels
 */

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

/************* Bit position definitions for I2C peripheral **********************/

/*
 * Bit position definitions I2C_CR1
 */
#define I2C_CR1_PE						0
#define I2C_CR1_NOSTRETCH  				7
#define I2C_CR1_START 					8
#define I2C_CR1_STOP  				 	9
#define I2C_CR1_ACK 				 	10
#define I2C_CR1_SWRST  				 	15

/*
 * Bit position definitions I2C_CR2
 */
#define I2C_CR2_FREQ				 	0
#define I2C_CR2_ITERREN				 	8
#define I2C_CR2_ITEVTEN				 	9
#define I2C_CR2_ITBUFEN 			    10

/*
 * Bit position definitions I2C_OAR1
 */
#define I2C_OAR1_ADD0    				 0
#define I2C_OAR1_ADD71 				 	 1
#define I2C_OAR1_ADD98  			 	 8
#define I2C_OAR1_ADDMODE   			 	15

/*
 * Bit position definitions I2C_SR1
 */

#define I2C_SR1_SB 					 	0
#define I2C_SR1_ADDR 				 	1
#define I2C_SR1_BTF 					2
#define I2C_SR1_ADD10 					3
#define I2C_SR1_STOPF 					4
#define I2C_SR1_RXNE 					6
#define I2C_SR1_TXE 					7
#define I2C_SR1_BERR 					8
#define I2C_SR1_ARLO 					9
#define I2C_SR1_AF 					 	10
#define I2C_SR1_OVR 					11
#define I2C_SR1_TIMEOUT 				14

/*
 * Bit position definitions I2C_SR2
 */
#define I2C_SR2_MSL						0
#define I2C_SR2_BUSY 					1
#define I2C_SR2_TRA 					2
#define I2C_SR2_GENCALL 				4
#define I2C_SR2_DUALF 					7

/*
 * Bit position definitions I2C_CCR
 */
#define I2C_CCR_CCR 					 0
#define I2C_CCR_DUTY 					14
#define I2C_CCR_FS  				 	15



// Generic macros

#define ENABLE				1
#define DISABLE				0
#define SET					ENABLE
#define RESET				DISABLE
#define GPIO_PIN_SET		SET
#define GPIO_PIN_RESET		RESET
#define FLAG_SET			SET
#define FLAG_RESET			RESET

#include "stm32f429xx_gpio_driver.h"
#include "stm32f429xx_i2c_driver.h"

#endif /* INC_STM32F429XX_H_ */
