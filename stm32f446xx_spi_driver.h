/*
 * stm32f446xx_spi_driver.h
 *
 *  Created on: 18-Sep-2019
 *      Author: AJAY
 */

#ifndef INC_STM32F446XX_SPI_DRIVER_H_
#define INC_STM32F446XX_SPI_DRIVER_H_
#include "stm32f446xx.h"



/*
 * Possible SPI states
 */
#define SPI_READY 				0
#define SPI_BUSY_IN_RX			1
#define SPI_BUSY_IN_TX			2


//CONFIGURATION STRUCTURE FOR SPIx PERIPHERAL
typedef struct
{
	uint8_t SPI_DeviceMode;
	uint8_t SPI_BusConfig;
	uint8_t SPI_DFF;
	uint8_t SPI_CPHA;
	uint8_t SPI_CPOL;
	uint8_t SPI_SSM;
	uint8_t SPI_SclkSpeed;
}SPI_Config_t;


//HANDLE STRUCTURE FOR SPIx PERIPHERAL
typedef struct
{
	SPI_RegDef_t 	*pSPIx;   //This holds the base address of spix peripherals ; x(0, 1, 2, 3)
	SPI_Config_t	SPIConfig;

	uint8_t *pTxBuffer;  //to store the app. tx buffer address
	uint8_t *pRxBuffer;  //to store the app. rx buffer address
	uint32_t TxLen;		//To store tx len
	uint32_t RxLen;		//To store rx len
	uint8_t TxState;	//To store Tx State
	uint8_t RxState;	//To store Rx state
}SPI_Handle_t;


/*
 * @spi DEVICE MODES
 */
#define SPI_DEVICE_MODE_MASTER 					1
#define SPI_DEVICE_MODE_SLAVE 					0


/*
 * @SPI DEVICE BUS CONFIG
 */

#define SPI_BUS_CONFIG_FD						1
#define SPI_BUS_CONFIG_HD						2
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY			3


/*
 * @SPI DEVICE sCLKsPEED
 */
#define SPI_SCLK_SPEED_DIV2 					0
#define SPI_SCLK_SPEED_DIV4 					1
#define SPI_SCLK_SPEED_DIV8 					2
#define SPI_SCLK_SPEED_DIV16 					3
#define SPI_SCLK_SPEED_DIV32 					4
#define SPI_SCLK_SPEED_DIV64 					5
#define SPI_SCLK_SPEED_DIV128 					6
#define SPI_SCLK_SPEED_DIV256 					7


/*
 * @SPI DEVICE DEVICE FRAME FORMAT
 */
#define SPI_DFF_8BITS							0
#define SPI_DFF_16BITS							1


/*
 * @SPI DEVICE CPOL
 */
#define SPI_CPOL_HIGH 							1
#define SPI_CPOL_LOW 							0



/*
 * @SPI DEVICE CPHA
 */
#define SPI_CPHA_HIGH 							1
#define SPI_CPHA_LOW 							0



/*
 * @SPI DEVICE SSM
 */

#define SPI_SSM_EN								1
#define SPI_SSM_DI								0


#define SPI_TXE_FLAG 						   (1<< SPI_SR_TXE)
#define SPI_RXNE_FLAG 						   (1<< SPI_SR_RXNE)
#define SPI_BUSY_FLAG 						   (1<< SPI_SR_BUSY)

//************************************************************************************************************************



/*
 * *************************************API'S SUPPORTED BY THIS DRIVER*****************************************************
 */


/*
 * PERIPHERAL COCK CONTROL
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);
/*
 * INIT AND DEINIT
 */
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);
/*
 * DATA SEND AND RECEIVE
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len);


uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len);
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len);
/*
 * IRQ CONFIGURATION AND ISR HANDLING
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi );
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pHandle);

uint8_t SPI_GetFlag_Status(SPI_RegDef_t *pSPIx, uint32_t FlagName);
void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx);
void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle);
void SPI_CloseReception(SPI_Handle_t *pSPIHandle);
/*
 * Other Peripheral control API's
 */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi);
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi);


//APPLICATION CALLBACK
void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEv);

/*
 * SOME PRIVATE HELPER FUNCTIONS THAT MUST NOT BE CALLED OR USED BY THE USER APPLICATION
 */
void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle);
 void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle);
void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle);


#endif /* INC_STM32F446XX_SPI_DRIVER_H_ */
