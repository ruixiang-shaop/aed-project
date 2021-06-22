/*
 * stm32f429xx_gpio_driver.h
 *
 *  Created on: 19 jun. 2021
 *      Author: ruixi
 */

#ifndef INC_STM32F429XX_GPIO_DRIVER_H_
#define INC_STM32F429XX_GPIO_DRIVER_H_

#include "stm32f429xx.h"

/*
 * Handle structure for a GPIO pin
 */

typedef struct
{
	uint8_t GPIO_PinNumber;
	uint8_t GPIO_PinMode;
	uint8_t GPIO_PinSpeed;
	uint8_t GPIO_PinPuPdControl;
	uint8_t GPIO_PinOPType;
	uint8_t GPIO_PinAltFunMode;
} GPIO_PinConfig_t;

typedef struct
{
	GPIO_RegDef_t * pGPIOx;	// base address of the GPIO port
	GPIO_PinConfig_t GPIO_PinConfig;	// GPIO pin settings
} GPIO_Handle_t;


// GPIO pin numbers
#define GPIO_PIN_0  		0
#define GPIO_PIN_1  		1
#define GPIO_PIN_2  		2
#define GPIO_PIN_3  		3
#define GPIO_PIN_4  		4
#define GPIO_PIN_5  		5
#define GPIO_PIN_6  		6
#define GPIO_PIN_7  		7
#define GPIO_PIN_8  		8
#define GPIO_PIN_9  		9
#define GPIO_PIN_10 		10
#define GPIO_PIN_11 		11
#define GPIO_PIN_12 		12
#define GPIO_PIN_13 		13
#define GPIO_PIN_14 		14
#define GPIO_PIN_15 		15


// GPIO Modes
#define GPIO_MODE_IN 		0
#define GPIO_MODE_OUT 		1
#define GPIO_MODE_ALTFN		2
#define GPIO_MODE_ANALOG	3
#define GPIO_MODE_IT_FT		4
#define GPIO_MODE_IT_RT		5
#define GPIO_MODE_IT_RFT	6


// GPIO pin possible output types
#define GPIO_OP_TYPE_PP   0		// push-pull
#define GPIO_OP_TYPE_OD   1		// output drain


// GPIO pin possible output speeds
#define GPIO_SPEED_LOW			0
#define GPIO_SPEED_MEDIUM		1
#define GPIO_SPEED_FAST			2
#define GPOI_SPEED_HIGH			3


// GPIO pin pull up AND pull down
#define GPIO_NO_PUPD   		0
#define GPIO_PIN_PU			1
#define GPIO_PIN_PD			2

/******************* API *******************/

// Peripheral clock setup
void GPIO_PClkControl(GPIO_RegDef_t *pGPIOx, uint8_t EnOrDi);


// Init and de-init
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);


// Data read and write
uint8_t GPIO_ReadFromInputPin(GPIO_Handle_t *pGPIOHandle);
uint16_t GPIO_ReadFromInputPort(GPIO_Handle_t *pGPIOHandle);
void GPIO_WriteToOutputPin(GPIO_Handle_t *pGPIOHandle, uint8_t value);
void GPIO_WriteToOutputPort(GPIO_Handle_t *pGPIOHandle, uint16_t value);
void GPIO_ToggleOutputPort(GPIO_Handle_t *pGPIOHandle);


// IRQ configuration and ISR handling
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnOrDi);
void GPIO_IRQHandling(uint8_t PinNumber);

#endif /* INC_STM32F429XX_GPIO_DRIVER_H_ */
