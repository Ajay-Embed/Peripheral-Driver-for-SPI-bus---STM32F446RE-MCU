#include "stm32f446xx_spi_driver.h"


/* stm32f446xx_spi_driver.c
 *
 *  Created on: 18-Sep-2019
 *      Author: blueh
 */
//API definitions of the spi peripheral

//Peripheral clock setup
/********************************************************************
 * @FN 					- SPI_PeriClockControl
 *
 *
 * @BRIEF				-
 *
 *
 *
 *
 * @PARAM[in] 			-
 * @PARAM[in] 			-
 * @PARAM[in] 			-
 *
 *
 *
 * @return				-
 *
 *
 * @note				-
 *
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
		{
			if(pSPIx == SPI1)
					{
				SPI1_PCLK_EN();
					}
	   else if(pSPIx == SPI2)
					{
						SPI2_PCLK_EN();
					}
	   else if(pSPIx == SPI3)
					{
						SPI3_PCLK_EN();
					}
		}
	else
	{
			if(pSPIx == SPI1)
						{
							SPI1_PCLK_DI();
						}
		else if(pSPIx == SPI2)
						{
							SPI2_PCLK_DI();
						}
		else if(pSPIx == SPI2)
						{
							SPI3_PCLK_DI();
						}
	}

}



//SPI initialization
/********************************************************************
 * @FN 					- SPI_Init
 *
 *
 * @BRIEF				-
 *
 *
 *
 *
 * @PARAM[in] 			-
 * @PARAM[in] 			-
 * @PARAM[in] 			-
 *
 *
 *
 * @return				-
 *
 *
 * @note				-
 *
 */
void SPI_Init(SPI_Handle_t *pSPIHandle)
{

//Enabling the peripheral clock so that the user doesn't has to do
SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

//Initialized a variable named tempreg and then changed the bit values of the tempreg and then assigned tempreg to CR1 register.
	uint32_t tempreg = 0;


	//Configuring SPI CR1 register
	tempreg |= pSPIHandle->SPIConfig.SPI_DeviceMode << 2;

	//Configuring the BUS config
	if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD)
	{
		//BIDI MODE SHOULD BE CLEARED
		tempreg &= ~(1<<15);
	}else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD)
	{
		//BIDI MODE SHOULD BE SET
		tempreg |= (1<<15);
	}else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY)
	{
		//BIDI MODE SHOULD BE CLEARED AND RX MODE SHOULD BE SET
		tempreg &= ~(1<<15);
		tempreg |=  (1<<15);
	}

	//Configuring the dff
	tempreg |= pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF;



	//Configuring the SPI SERIAL CLOCK(BAUD RATE)
	tempreg |= pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR;



	//Configuring the SPI CLOCK PHASE
	tempreg |= pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA;



	//Configuring the CLOCK POLARITY
	tempreg |= pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL;




	pSPIHandle->pSPIx->CR1 = tempreg;



}



//SPI De-initialization
/********************************************************************
 * @FN 					- SPI_DeInit
 *
 *
 * @BRIEF				-
 *
 *
 *
 *
 * @PARAM[in] 			-
 * @PARAM[in] 			-
 * @PARAM[in] 			-
 *
 *
 *
 * @return				-
 *
 *
 * @note				-
 *
 */

void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
	if(pSPIx == SPI1)
			{
				SPI1_REG_RESET();
			}
else if(pSPIx == SPI2)
			{
				SPI2_REG_RESET();
			}
else if(pSPIx == SPI3)
			{
				SPI3_REG_RESET();
			}
}





//SPI_SendData
/********************************************************************
 * @FN 					- SPI_SendData
 *
 *
 * @BRIEF				- This is the blocking API, it is called so because the function call will wait until all the bytes are transmitted.
 *
 *
 *
 *
 * @PARAM[in] 			- Base address of the SPI peripheral
 * @PARAM[in] 			- Pointer to the data
 * @PARAM[in] 			- Number of bytes to be transmitted
 *
 *
 *
 * @return				-
 *
 *
 * @note				-   This is a blocking call, if there are atmost a 1000 elements also in the len or the transmit buffer
 * 							then after sending all of them only it will come out of the while loop, else it stays inside
 * 							This is a perfect example of POLLING type firmware that in future might require a Watchdog peripheral to reset if the
 * 							 peripheral gets stuck or it goes into looping state.
 *
 */
uint8_t SPI_GetFlag_Status(SPI_RegDef_t *pSPIx, uint32_t FlagName)
{
	if(pSPIx->SR & FlagName)
	{
		return SET;
	}

return FLAG_RESET;
}

void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len)
{
		while(Len > 0)
			{
				//1.  Wait until Transmit buffer empty (TXE) is set
				//while ( !(pSPIx->SR & (1<<1) ) ); // Instead of this, implement a small function which returns if a requested flag is set or reset
					while(SPI_GetFlag_Status(pSPIx, SPI_TXE_FLAG) == FLAG_RESET);
				//2. Check the DFF bit in CR1
					if(pSPIx->CR1 & ( 1<< SPI_CR1_DFF ))
					{
						//16 bit Data format
						//1. Load data  into the DR
						pSPIx->DR = *((uint16_t*)pTxBuffer); //Typecasted to uint16_t type variable
						Len--;
						Len--;
						(uint16_t*)pTxBuffer++;
					}
					else
					{
						//8 bit data format
						pSPIx->DR = *pTxBuffer; //No typecasting required
						Len--;
						pTxBuffer++;
					}


}
}


//SPI_ReceiveData
/********************************************************************
 * @FN 					- SPI_ReceiveData
 *
 *
 * @BRIEF				-
 *
 *
 *
 *
 * @PARAM[in] 			-
 * @PARAM[in] 			-
 * @PARAM[in] 			-
 *
 *
 *
 * @return				-
 *
 *
 * @note				-
 *
 */

void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len)
{
	while(Len > 0)
		{
			//1.  Wait until Transmit buffer empty (RXNE) is set
			//while ( !(pSPIx->SR & (1<<1) ) ); // Instead of this, implement a small function which returns if a requested flag is set or reset
				while(SPI_GetFlag_Status(pSPIx, SPI_RXNE_FLAG) == FLAG_RESET);
			//2. Check the DFF bit in CR1
				if(pSPIx->CR1 & ( 1<< SPI_CR1_DFF ))
				{
					//16 bit Data format
					//1. Load data  into the DR
					*((uint16_t*)pRxBuffer) = pSPIx->DR  ; //Typecasted to uint16_t type variable
					Len--;
					Len--;
					(uint16_t*)pRxBuffer++;
				}
				else
				{
					//8 bit data format
					*pRxBuffer = pSPIx->DR ; //No typecasting required
					Len--;
					pRxBuffer++;
				}


}
}


//IRQInterruptConfig
/********************************************************************
 * @FN 					- SPI_IRQInterruptConfig
 *
 *
 * @BRIEF				-
 *
 *
 *
 *
 * @PARAM[in] 			-
 * @PARAM[in] 			-
 * @PARAM[in] 			-
 *
 *
 *
 * @return				-
 *
 *
 * @note				-
 *
 */

void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi )
{
	//All the cofig in this API is PROCESSOR SPECIFIC
	//REFER TO REGISTER OF PROCESSOR CORTEX M4 PROCESSOR GENERIC USER GUIDE
	if(EnorDi == ENABLE)
	{
		if(IRQNumber <= 31)
		{
			//program ISER0
			*NVIC_ISER0 |= (1 << IRQNumber);

		}else if(IRQNumber>31 && IRQNumber<=64)
		{
			//program ISER1 register
			*NVIC_ISER1 |= (1 << IRQNumber % 32 );
		}else if(IRQNumber>=64 && IRQNumber<96)
		{
			//program ISER2 register
			*NVIC_ISER2 |= (1 << IRQNumber%64);

		}
	}else
	{
		if(IRQNumber <= 31)
			{
				*NVIC_ICER0 |= (1 << IRQNumber);

			}else if(IRQNumber>31 && IRQNumber<=64)
			{
				//program ISER1 register
				*NVIC_ICER1 |= (1 << IRQNumber % 32 );
			}else if(IRQNumber>=64 && IRQNumber<96)
			{
				//program ISER2 register
				*NVIC_ICER2 |= (1 << IRQNumber%64);
			}
	}

}




//IRQ PRIORITY CONFIGURATION
/********************************************************************
 * @FN 					- SPI_IRQPriorityConfig
 *
 *
 * @BRIEF				-
 *
 *
 *
 *
 * @PARAM[in] 			-
 * @PARAM[in] 			-
 * @PARAM[in] 			-
 *
 *
 *
 * @return				-
 *
 *
 * @note				-
 *
 */

void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;

	uint8_t shiftAmount = (8*iprx_section) + (8 - NO_OF_BITS_IMPLEMENTED);
	*(NVIC_PR_BASEADDR + (iprx*4)) |= IRQPriority << shiftAmount;
}




//IRQ HANDLING
/********************************************************************
 * @FN 					- SPI_IRQHandling
 *
 *
 * @BRIEF				-
 *
 *
 *
 *
 * @PARAM[in] 			-
 * @PARAM[in] 			-
 * @PARAM[in] 			-
 *
 *
 *
 * @return				-
 *
 *
 *
 *
 *
 *
 * @note				-
 *
 */

void SPI_IRQHandling(SPI_Handle_t *pHandle)
{
uint8_t temp1, temp2;
// First let us check for TXE flag
temp1 = pHandle->pSPIx->SR & (SPI_SR_TXE);
temp2 = pHandle->pSPIx->CR2 & (SPI_CR2_TXEIE);

if(temp1 && temp2)
{
	//we are via this function handling the TXE
	spi_txe_interrupt_handle(pHandle);
}


//Now checking for RXNE

temp1 = pHandle->pSPIx->SR & (1<<SPI_SR_RXNE);
temp2 = pHandle->pSPIx->CR2 & (1<< SPI_CR2_RXNEIE);

if(temp1 && temp2)
{
	//Handling the RXNE interrupt
	spi_rxne_interrupt_handle(pHandle);
}


//Now check for OVR flag
temp1 = pHandle->pSPIx->SR & (1<< SPI_SR_OVR);
temp2 = pHandle->pSPIx -> CR2 & (1<<SPI_CR2_ERRIE);

if(temp1 && temp2)
{
	//Handling the OVR ERR  interrupt
	spi_ovr_err_interrupt_handle(pHandle);
}































}



//SPI Peripheral clock control
/********************************************************************
 * @FN 					- //SPI Peripheral clock control
 *
 *
 * @BRIEF				-
 *
 *
 *
 *
 * @PARAM[in] 			-
 * @PARAM[in] 			-
 * @PARAM[in] 			-
 *
 *
 *
 * @return				-
 *
 *
 *
 *
 *
 *
 * @note				-
 *
 */

void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pSPIx->CR1 |= (1<<SPI_CR1_SPE);
	}else	{
		pSPIx->CR1 &= ~(1<<SPI_CR1_SPE);
			}
}




//SPI SSI Config
/********************************************************************
 * @FN 					- //SPI SSI Config
 *
 *
 * @BRIEF				-
 *
 *
 *
 *
 * @PARAM[in] 			-
 * @PARAM[in] 			-
 * @PARAM[in] 			-
 *
 *
 *
 * @return				-
 *
 *
 *
 *
 *
 *
 * @note				-
 *
 */
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
		{
			pSPIx->CR1 |= (1<<SPI_CR1_SPE);
		}else	{
			pSPIx->CR1 &= ~(1<<SPI_CR1_SPE);
				}
}

//SPI SSOE
/********************************************************************
 * @FN 					- SPI_ssoe
 *
 *
 * @BRIEF				-
 *
 *
 *
 *
 * @PARAM[in] 			-
 * @PARAM[in] 			-
 * @PARAM[in] 			-
 *
 *
 *
 * @return				-
 *
 *
 *
 *
 *
 *
 * @note				-
 *
 */
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
			{
				pSPIx->CR2 |= (1<<SPI_CR2_SSOE);
			}else	{
				pSPIx->CR2 &= ~(1<<SPI_CR2_SSOE);
					}

}




//SPI SEND & Receive DATA IT
/********************************************************************
 * @FN 					- SPI_Send data IT
 *
 *
 * @BRIEF				-
 *
 *
 *
 *
 * @PARAM[in] 			-
 * @PARAM[in] 			-
 * @PARAM[in] 			-
 *
 *
 *
 * @return				-
 *
 *
 *
 *
 *
 *
 * @note				-
 *
 */
uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle->TxState;
	if(state != SPI_BUSY_IN_TX)
	{
	//1. Save Tx buffer Address and Len info in some global variables
	pSPIHandle-> pTxBuffer = pTxBuffer;
	pSPIHandle->TxLen   = Len;


	//2. Mark the SPI state BUSY in transmission so that No other peripheral can take over SPI communication until the communication is over
	pSPIHandle->TxState = SPI_BUSY_IN_TX;


	//3. Enable the TXEIE control bit to get interrupt whenever the TXE flag is set in SR
	pSPIHandle->pSPIx->CR2 |= (1<<SPI_CR2_TXEIE);
	}

	//4. Data transmission will be handles by the ISR code - Will be implemented later

	return state;
}




uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle->RxState;
	if(state != SPI_BUSY_IN_RX)
	{
	//1. Save Tx buffer Address and Len info in some global variables
	pSPIHandle-> pRxBuffer = pRxBuffer;
	pSPIHandle->RxLen   = Len;


	//2. Mark the SPI state BUSY in transmission so that No other peripheral can take over SPI communication until the communication is over
	pSPIHandle->RxState = SPI_BUSY_IN_RX;


	//3. Enable the TXEIE control bit to get interrupt whenever the TXE flag is set in SR
	pSPIHandle->pSPIx->CR2 |= (1<<SPI_CR2_RXNEIE);
	}

	//4. Data transmission will be handles by the ISR code - Will be implemented later

	return state;
}










/********************************************HELPER FUNCTION IMPLEMENTATION************************************************************************
 *
 *
 */
void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	//2. Check the DFF bit in CR1
						if(pSPIHandle->pSPIx->CR1 & ( 1<< SPI_CR1_DFF ))
						{
							//16 bit Data format
							//1. Load data  into the DR
							pSPIHandle->pSPIx->DR = *((uint16_t*)pSPIHandle->pTxBuffer); //Typecasted to uint16_t type variable
							pSPIHandle->TxLen--;
							pSPIHandle->TxLen--;
							(uint16_t*)pSPIHandle->pTxBuffer++;
						}
						else
											{
												//8 bit data format
													pSPIHandle->pSPIx->DR = *pSPIHandle->pTxBuffer; //No typecasting
													pSPIHandle->TxLen--;
													pSPIHandle->pTxBuffer++;
											}
						if(!(pSPIHandle->TxLen))
						{
							//If tx length is zero then close the SPI transmition, and inform the application that TX is Over


							//This prevents interrupts from TXE flag
							pSPIHandle->pSPIx->CR2 &= ~(1<<SPI_CR2_TXEIE);
							pSPIHandle->pTxBuffer = NULL;
							pSPIHandle->TxLen = 0;
							pSPIHandle->TxState = SPI_READY;
							SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_TX_CMPLT);
						}


}
 void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	//2. Check the DFF bit in CR1
							if(pSPIHandle->pSPIx->CR1 & ( 1<< 11 ))
							{
								//16 bit Data format
								//1. Load data  into the DR
								*((uint16_t*)pSPIHandle->pTxBuffer) = pSPIHandle->pSPIx->DR;//Typecasted to uint16_t type variable

								pSPIHandle-> RxLen -=2;
								pSPIHandle->RxLen--;
								pSPIHandle->RxLen--;
								(uint16_t*)pSPIHandle->pTxBuffer++;
							}
							else
												{
													//8 bit data format
								*(pSPIHandle->pRxBuffer)= (uint8_t)pSPIHandle->pSPIx->DR; //No typecasting
														pSPIHandle->RxLen--;
														pSPIHandle->pRxBuffer++;
												}
							if(!(pSPIHandle->TxLen))
							{
								//If tx length is zero then close the SPI transmission, and inform the application that TX is Over


								//This prevents interrupts from TXE flag
								pSPIHandle->pSPIx->CR2 &= ~(1<<SPI_CR2_RXNEIE);
								pSPIHandle->pRxBuffer = NULL;
								pSPIHandle->RxLen = 0;
								pSPIHandle->RxState = SPI_READY;
								SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_RX_CMPLT);
							}
}
void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	uint8_t temp;
	//1 Clear the ovr flag
	if(pSPIHandle->TxState != SPI_BUSY_IN_TX)
	{
		temp = pSPIHandle->pSPIx->DR;
		temp = pSPIHandle->pSPIx->SR;
	}

	//2. Inform the application
	SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_OVR_ERR);
	(void)temp;
}


void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx)
{
	uint8_t temp;
	temp = pSPIx->DR;
	temp = pSPIx->SR;

(void)temp;

}
void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~(1<<SPI_CR2_TXEIE);
						pSPIHandle->pTxBuffer = NULL;
						pSPIHandle->TxLen = 0;
						pSPIHandle->TxState = SPI_READY;

}
void SPI_CloseReception(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~(1<<SPI_CR2_RXNEIE);
						pSPIHandle->pRxBuffer = NULL;
						pSPIHandle->RxLen = 0;
						pSPIHandle->RxState = SPI_READY;

}

__attribute__((weak)) void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEv)
{
	//This is a weak implementation and the application may overwrite this function

}

