/*
 * stm32f429xx_i2c.c
 *
 *  Created on: 28 jun. 2021
 *      Author: ruixi
 */

#include "stm32f429xx_i2c_driver.h"


uint16_t AHB_PreScaler[8] = {2,4,8,16,64,128,256,512};
uint8_t APB1_PreScaler[4] = {2,4,8,16};

static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx);
static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t * pI2Cx, uint8_t SlaveAddr);
static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t * pI2Cx, uint8_t SlaveAddr);
static void I2C_ClearADDRFlag(I2C_Handle_t * pI2CHandle);

static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t *pI2CHandle);
static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t *pI2CHandle);




// Initiates Start phase on a Data transfer operation
static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= (0x1U << I2C_CR1_START);
}

// Sends slave address and indicates that it will be a write transmission
static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t * pI2Cx, uint8_t SlaveAddr)
{
	SlaveAddr = SlaveAddr << 1;
	SlaveAddr &= ~(0x1U);		// SlaveAddress + write
	pI2Cx->DR = SlaveAddr;
}

// Sends slave address and indicates that it will be a read transmission
static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t * pI2Cx, uint8_t SlaveAddr)
{
	SlaveAddr = SlaveAddr << 1;
	SlaveAddr |= 0x1U;			// SlaveAddress + read
	pI2Cx->DR = SlaveAddr;
}

// Clears ADDR flag after address is sent (master) or matched (slave)
static void I2C_ClearADDRFlag(I2C_Handle_t * pI2CHandle)
{
	uint32_t dummy;
	if (pI2CHandle->pI2Cx->SR2 & (0x1U << I2C_SR2_MSL))
	{
		// Device is master
		if (pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
		{
			if (pI2CHandle->RxSize == 1)
			{
				// 1. Disable ACK
				I2C_ManageAcking(pI2CHandle->pI2Cx, DISABLE);
				// 2. Clear ADDR flag (Read SR1, READ SR2)
				dummy = pI2CHandle->pI2Cx->SR1;
				dummy = pI2CHandle->pI2Cx->SR2;
				(void)dummy;		// to avoid variable not used compiler warning
			}
		}
	} else
	{
		// Device is slave
		// Clear ADDR flag (Read SR1, READ SR2)
		dummy = pI2CHandle->pI2Cx->SR1;
		dummy = pI2CHandle->pI2Cx->SR2;
		(void)dummy;		// to avoid variable not used compiler warning
	}
}

// Initiates Stop phase on a Data transfer operation
void I2C_GenerateStopCondition(I2C_RegDef_t * pI2Cx)
{
	pI2Cx->CR1 |= (0x1U << I2C_CR1_STOP);
}

void I2C_SlaveEnableDisableCallbackEvents(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi)
{
	if (EnOrDi == ENABLE)
	{
		pI2Cx->CR2 |= 0x1U << I2C_CR2_ITEVTEN;
		pI2Cx->CR2 |= 0x1U << I2C_CR2_ITBUFEN;
		pI2Cx->CR2 |= 0x1U << I2C_CR2_ITERREN;
	} else
	{
		pI2Cx->CR2 &= ~(0x1U << I2C_CR2_ITEVTEN);
		pI2Cx->CR2 &= ~(0x1U << I2C_CR2_ITBUFEN);
		pI2Cx->CR2 &= ~(0x1U << I2C_CR2_ITERREN);
	}
}

void I2C_PControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi)
{
	if (EnOrDi == ENABLE)
	{
		pI2Cx->CR1 |= 0x1U << I2C_CR1_PE;
	} else
	{
		pI2Cx->CR1 &= ~(0x1U << I2C_CR1_PE);
	}
}

// Peripheral clock setup
void I2C_PClkControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi)
{
	if (EnOrDi == ENABLE)
	{
		if (pI2Cx == I2C1)
		{
			I2C1_PCLK_EN();
		} else if (pI2Cx == I2C2)
		{
			I2C2_PCLK_EN();
		} else if (pI2Cx == I2C3)
		{
			I2C3_PCLK_EN();
		}
	} else
	{
		//TODO
	}
}

// Get clock frequency of APB1 bus
uint32_t RCC_GetPCLK1Value(void)
{
	uint32_t pclk1, SystemClk;
	uint8_t aux, ahbp, apb1p;
	uint8_t clkSrc = (RCC->CFGR >> 2) & 0x3U;
	if (clkSrc == 0)
	{
		// HSI
		SystemClk = 16000000; // 16Mhz
	} else if (clkSrc == 1)
	{
		// HSE
		SystemClk = 25000000; // 25Mhz
	} else if (clkSrc == 2)
	{
		// PLL
		//TODO will not be used
	}

	// HPRE: AHB prescaler
	aux = (RCC->CFGR >> 4) & 0xFU;
	if (aux < 8)
	{
		ahbp = 1;
	} else
	{
		ahbp = AHB_PreScaler[aux-8];
	}

	// PPRE1: APB Low speed prescaler
	aux = (RCC->CFGR >> 10) & 0x7U;
	if (aux < 4)
	{
		apb1p = 1;
	} else
	{
		apb1p = APB1_PreScaler[aux-4];
	}

	pclk1 = ((SystemClk / ahbp) / apb1p);

	return pclk1;
}

void I2C_Init(I2C_Handle_t *pI2CHandle)
{
	uint32_t aux = 0;

	// 0. Enable clock for I2Cx peripheral
	I2C_PClkControl(pI2CHandle->pI2Cx, ENABLE);

	// 1. Enable ACK
	aux = pI2CHandle->I2C_Config.I2C_ACKControl << 10;
	pI2CHandle->pI2Cx->CR1 = aux;

	// 2. Configure FREQ field of CR2
	aux = RCC_GetPCLK1Value()/1000000U; // MHz
	pI2CHandle->pI2Cx->CR2 = aux & 0x3FU;

	// 3. Configure device address if slave
	aux = pI2CHandle->I2C_Config.I2C_DeviceAddress << 1;
	aux |= 0x1U << 14;
	pI2CHandle->pI2Cx->OAR1 = aux;

	// 4. CCR calculations
	uint16_t ccr_value = 0;
	if (pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{
		// Standard mode
		ccr_value = RCC_GetPCLK1Value()/(2*pI2CHandle->I2C_Config.I2C_SCLSpeed);
		aux = ccr_value & 0xFFF; // 12 bits
	} else
	{
		// Fast mode
		aux = 0x1U << 15; // Fast mode
		aux |= pI2CHandle->I2C_Config.I2C_FMDutyCycle << 14; // Bit set because of reference manual
		// Check reference manual for I2C CCR formula
		if (pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2)
		{
			ccr_value = RCC_GetPCLK1Value()/(3*pI2CHandle->I2C_Config.I2C_SCLSpeed);
		} else
		{
			ccr_value = RCC_GetPCLK1Value()/(25*pI2CHandle->I2C_Config.I2C_SCLSpeed);
		}
		aux |= ccr_value & 0xFFF; // 12 bits
	}
	pI2CHandle->pI2Cx->CCR = aux;

	// 5. Configure rise time for I2C pins
	// Check reference manual for formulas
	if (pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{
		// Standard mode
		aux = (RCC_GetPCLK1Value() / 1000000U) + 1; // 1000ns max
	} else
	{
		// Fast mode
		aux = (RCC_GetPCLK1Value() * 300 / 1000000000U) + 1;	// 300ns max
	}
	pI2CHandle->pI2Cx->TRISE = aux & 0x3F;	// 5 bits
}

void I2C_DeInit(I2C_RegDef_t *pI2Cx)
{
	if (pI2Cx == I2C1)
	{
		I2C1_REG_RESET();
	} else if (pI2Cx == I2C2)
	{
		I2C2_REG_RESET();
	} else if (pI2Cx == I2C3)
	{
		I2C3_REG_RESET();
	}
}

void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t length, uint8_t SlaveAddr, uint8_t Sr)
{
	// 1. Generate START condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	// 2. Confirm that start generation is completed
	// It completes when the SB flag from SR1 is set
	while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB));

	// 3. Send the address of the slave with r/nw bit
	I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx, SlaveAddr);

	// 4. Confirm that address phase is completed
	// It completes when the ADDR flag from SR1 is set
	while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR));

	// 5. Clear ADDR flag
	I2C_ClearADDRFlag(pI2CHandle);

	// 6. Send all the data
	while (length > 0)
	{
		// wait until TXE is set (we are able to send data)
		while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE));
		// assign data to transfer
		pI2CHandle->pI2Cx->DR = *pTxBuffer;
		pTxBuffer++;
		length--;
		// data will be transfered automatically by hardware
	}

	// 7. Wait until TXE and BTF flags are set, this will mean
	//    that STOP condition can be generated
	while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE));
	while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_BTF));

	// 8. Generate STOP condition
	if (Sr == I2C_DISABLE_SR)
	{
		I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
	}
}

void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t length, uint8_t SlaveAddr, uint8_t Sr)
{
	// 1. Generate START condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	// 2. Confirm that start generation is completed
	// It completes when the SB flag from SR1 is set
	while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB));

	// 3. Send the address of the slave with r/nw bit
	I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx, SlaveAddr);

	// 4. Confirm that address phase is completed
	// It completes when the ADDR flag from SR1 is set
	while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR));

	// Case when length is only 1 byte
	if (length == 1)
	{
		// Disable ACKing
		I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);

		// Clear ADDR flag
		I2C_ClearADDRFlag(pI2CHandle);

		// Wait until TXE is set
		while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE));

		// Generate STOP condition
		if (Sr == I2C_DISABLE_SR)
		{
			I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
		}

		// Read data and write it to the buffer
		*pRxBuffer = pI2CHandle->pI2Cx->DR;
	}

	if (length > 1)
	{
		// Clear ADDR flag
		I2C_ClearADDRFlag(pI2CHandle);

		// Read all the data
		for (uint32_t i = length; i > 0; i--)
		{
			// Wait until TXE is set
			while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE));

			if (i == 2)	// Special case when there are only two bytes remaining
			{
				// Disable ACKing
				I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);

				// Generate STOP condition
				if (Sr == I2C_DISABLE_SR)
				{
					I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
				}
			}

			// Read data and write it to the buffer
			*pRxBuffer = pI2CHandle->pI2Cx->DR;
			pRxBuffer++;
		}
	}

	// Re-enable ACKing
	if (pI2CHandle->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE)
	{
		I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_ENABLE);
	}
}

// IRQ configuration and ISR handling
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi)
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


void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	// 1. Calculate IPR register
	uint8_t iprx = IRQNumber/4;
	uint8_t offset = IRQNumber%4;

	// clear
	*(NVIC_IPR + iprx) &= ~(0xFFU << (offset*8));
	// set
	uint8_t shift_amount = (offset*8) + (8-NO_PR_BITS_IMPLEMENTED);
	*(NVIC_IPR + iprx) |= IRQPriority << shift_amount;
}

uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName)
{
	if (pI2Cx->SR1 & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}

void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi)
{
	if (EnOrDi == I2C_ACK_ENABLE)
	{
		// Enable ACK
		pI2Cx->CR1 |= (0x1U << I2C_CR1_ACK);
	} else
	{
		// Disable ACK
		pI2Cx->CR1 &= ~(0x1U << I2C_CR1_ACK);
	}
}

uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t length, uint8_t SlaveAddr, uint8_t Sr)
{

	uint8_t busystate = pI2CHandle->TxRxState;

	if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
	{
		pI2CHandle->pTxBuffer = pTxBuffer;
		pI2CHandle->TxLen = length;
		pI2CHandle->TxRxState = I2C_BUSY_IN_TX;
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Sr = Sr;

		//Implement code to Generate START Condition
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

		//Implement the code to enable ITBUFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN);

		//Implement the code to enable ITEVFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITEVTEN);

		//Implement the code to enable ITERREN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITERREN);

	}

	return busystate;

}

uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t length, uint8_t SlaveAddr, uint8_t Sr)
{

	uint8_t busystate = pI2CHandle->TxRxState;

	if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
	{
		pI2CHandle->pRxBuffer = pRxBuffer;
		pI2CHandle->RxLen = length;
		pI2CHandle->TxRxState = I2C_BUSY_IN_RX;
		pI2CHandle->RxSize = length; //Rxsize is used in the ISR code to manage the data reception
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Sr = Sr;

		//Implement code to Generate START Condition
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

		//Implement the code to enable ITBUFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN);

		//Implement the code to enable ITEVFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITEVTEN);

		//Implement the code to enable ITERREN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITERREN);

	}

	return busystate;
}

static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t *pI2CHandle)
{
	if (pI2CHandle->TxLen > 0)
	{
		// 1. Write data to DR
		pI2CHandle->pI2Cx->DR = *(pI2CHandle->pTxBuffer);
		// 2. Update conditions
		pI2CHandle->pTxBuffer++;
		pI2CHandle->TxLen--;
	}
}

static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t *pI2CHandle)
{
	if (pI2CHandle->RxSize == 1)
	{
		// 1. Read data from DR
		*(pI2CHandle->pRxBuffer) = pI2CHandle->pI2Cx->DR;
		// 2. Update conditions
		pI2CHandle->RxLen--;
	}

	if (pI2CHandle->RxSize > 1)
	{
		// Special case, disable ACK
		if (pI2CHandle->RxLen == 2)
		{
			I2C_ManageAcking(pI2CHandle->pI2Cx, DISABLE);
		}

		// 1. Read data from DR
		*(pI2CHandle->pRxBuffer) = pI2CHandle->pI2Cx->DR;
		// 2. Update conditions
		pI2CHandle->RxLen--;
		pI2CHandle->pRxBuffer++;
	}

	if (pI2CHandle->RxSize == 0)
	{
		// Close I2C data reception and notify the app
		// 1. Generate Stop condition
		if (pI2CHandle->Sr == I2C_DISABLE_SR)
		{
			I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
		}

		// 2. Close I2C Rx
		I2C_CloseReceiveData(pI2CHandle);

		// 3. Notify app
		I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_RX_CMPLT);
	}
}

void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle)
{
	// Disable ITBUFEN Control Bit
	pI2CHandle->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITBUFEN);
	// Disable ITEVFEN Control Bit
	pI2CHandle->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITEVTEN);

	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pRxBuffer = NULL;
	pI2CHandle->RxLen = 0;
	pI2CHandle->RxSize = 0;

	if(pI2CHandle->I2C_Config.I2C_ACKControl== I2C_ACK_ENABLE)
	{
		I2C_ManageAcking(pI2CHandle->pI2Cx,ENABLE);
	}

}

void I2C_CloseSendData(I2C_Handle_t *pI2CHandle)
{
	// Disable ITBUFEN Control Bit
	pI2CHandle->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITBUFEN);
	// Disable ITEVFEN Control Bit
	pI2CHandle->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITEVTEN);

	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pTxBuffer = NULL;
	pI2CHandle->TxLen = 0;
}


void I2C_SlaveSendData(I2C_RegDef_t *pI2Cx, uint8_t data)
{
	pI2Cx->DR = data;
}

uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2Cx)
{
	return (uint8_t) pI2Cx->DR;
}


void I2C_EV_IRQHandling(I2C_Handle_t * pI2CHandle)
{
	//Interrupt handling for both master and slave mode of a device
	uint32_t aux1, aux2, aux3;

	aux1 = pI2CHandle->pI2Cx->CR2 & (0x1U << I2C_CR2_ITEVTEN);
	aux2 = pI2CHandle->pI2Cx->CR2 & (0x1U << I2C_CR2_ITBUFEN);

	aux3 = pI2CHandle->pI2Cx->SR1 & (0x1U << I2C_SR1_SB);

	//1. Handle For interrupt generated by SB event
	//	Note : SB flag is only applicable in Master mode
	if (aux1 && aux3)
	{
		// SB flag is set
		// Execute address phase
		if (pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
		{
			I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx, pI2CHandle->DevAddr);
		} else
		{
			I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx, pI2CHandle->DevAddr);
		}
	}

	aux3 = pI2CHandle->pI2Cx->SR1 & (0x1U << I2C_SR1_ADDR);

	//2. Handle For interrupt generated by ADDR event
	//Note : When master mode : Address is sent
	//		 When Slave mode   : Address matched with own address
	if (aux1 && aux3)
	{
		// ADDR flag is set
		I2C_ClearADDRFlag(pI2CHandle);
	}

	aux3 = pI2CHandle->pI2Cx->SR1 & (0x1U << I2C_SR1_BTF);

	//3. Handle For interrupt generated by BTF(Byte Transfer Finished) event
	if (aux1 && aux3)
	{
		// BTF flag is set
		if (pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
		{
			// make sure that TXE is also set
			if (pI2CHandle->pI2Cx->SR1 & (0x1U << I2C_SR1_TXE))
			{
				// BTF=1, TXE=1, transmission completed
				if (pI2CHandle->TxLen == 0)
				{
					// 1. Generate STOP condition
					if (pI2CHandle->Sr == I2C_DISABLE_SR)
					{
						I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
					}
					// 2. Reset all members of the handle structure
					I2C_CloseSendData(pI2CHandle);

					// 3. Notify the application about a completed transmission
					I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_TX_CMPLT);
				}
			}
		}
	}

	aux3 = pI2CHandle->pI2Cx->SR1 & (0x1U << I2C_SR1_TXE);

	//4. Handle For interrupt generated by TXE event
	if (aux1 && aux2 && aux3)
	{
		// Only if device is master
		if (pI2CHandle->pI2Cx->SR2 & (0x1U << I2C_SR2_MSL))
		{
			// TXE flag is set
			if (pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
			{
				// Data transmission
				I2C_MasterHandleTXEInterrupt(pI2CHandle);
			}
		} else
		{
			// Slave
			// Make sure slave is in transmitter mode
			if (pI2CHandle->pI2Cx->SR2 & (0x1U << I2C_SR2_TRA))
			{
				I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_REQ);
			}
		}
	}

	aux3 = pI2CHandle->pI2Cx->SR1 & (0x1U << I2C_SR1_RXNE);

	//5. Handle For interrupt generated by RXNE event
	if (aux1 && aux2 && aux3)
	{
		// Only if device is master
		if (pI2CHandle->pI2Cx->SR2 & (0x1U << I2C_SR2_MSL))
		{
			// RXNE flag is set
			if (pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
			{
				// Data reception
				I2C_MasterHandleRXNEInterrupt(pI2CHandle);
			}
		} else
		{
			// Slave
			// Make sure slave is in receiver mode
			if (!(pI2CHandle->pI2Cx->SR2 & (0x1U << I2C_SR2_TRA)))
			{
				I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_RCV);
			}
		}
	}

	aux3 = pI2CHandle->pI2Cx->SR1 & (0x1U << I2C_SR1_STOPF);

	//6. Handle For interrupt generated by STOPF event
	// Note : Stop detection flag is applicable only slave mode . For master this flag will never be set
	if (aux1 && aux3)
	{
		// STOPF flag is set
		// To clear STOPF, we must be read SR1 (already done before) and then write CR1
		pI2CHandle->pI2Cx->CR1 |= 0x0U;	// writes CR1 but doesn't change its value

		I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_STOP);
	}
}

void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle)
{
	uint32_t aux1, aux2;

	//Know the status of  ITERREN control bit in the CR2
	aux2 = pI2CHandle->pI2Cx->CR2 & (0x1U << I2C_CR2_ITERREN);

/***********************Check for Bus error************************************/
	aux1 = pI2CHandle->pI2Cx->SR1 & (0x1U << I2C_SR1_BERR);
	if (aux1 && aux2)
	{
		// Bus error
		// Clear the buss error flag
		pI2CHandle->pI2Cx->SR1 &= ~(0x1U << I2C_SR1_BERR);

		// Notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_BERR);
	}

/***********************Check for arbitration lost error************************************/
	aux1 = pI2CHandle->pI2Cx->SR1 & (0x1U << I2C_SR1_ARLO);
	if (aux1 && aux2)
	{
		// Arbitration lost error
		// Clear the arbitration lost error flag
		pI2CHandle->pI2Cx->SR1 &= ~(0x1U << I2C_SR1_ARLO);
		// Notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_ARLO);
	}

/***********************Check for ACK failure  error************************************/

	aux1 = pI2CHandle->pI2Cx->SR1 & (0x1U << I2C_SR1_AF);
	if (aux1 && aux2)
	{
		// ACK failure error
		// Clear the ACK failure error flag
		pI2CHandle->pI2Cx->SR1 &= ~(0x1U << I2C_SR1_AF);
		// Notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_AF);
	}

/***********************Check for Overrun/underrun error************************************/
	aux1 = pI2CHandle->pI2Cx->SR1 & (0x1U << I2C_SR1_OVR);
	if (aux1 && aux2)
	{
		// Overrun/underrun
		// Clear the Overrun/underrun error flag
		pI2CHandle->pI2Cx->SR1 &= ~(0x1U << I2C_SR1_OVR);
		// Notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_OVR);
	}

/***********************Check for Time out error************************************/
	aux1 = pI2CHandle->pI2Cx->SR1 & (0x1U << I2C_SR1_TIMEOUT);
	if (aux1 && aux2)
	{
		// Time out error
		// Clear the Time out error flag
		pI2CHandle->pI2Cx->SR1 &= ~(0x1U << I2C_SR1_TIMEOUT);
		// Notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_TIMEOUT);
	}

}
