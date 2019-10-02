/*
 * spi_TxOnly_arduino.c
 *
 *  Created on: 20-Sep-2019
 *      Author: blueh
 */


#include "stm32f446xx.h"
#include <string.h>



void delay(void)
{
for(uint32_t i=0; i<500000/6 ; i++);
}
/*
 * 006_spiTx_testing.c

 *
 *  Created on: 19-Sep-2019
 *      Author: AJAY
 */
/*
 * ******************Documentation of the pins being used***********************
 *
 * PB14 -> SPI2_MISO
 * PB15 -> SPI2_MOSI
 * PB13 -> SCLK
 * PB12 -> NSS
 * ALTERNATE FUNCTIONALITY MODE : 5
 */

void SPI2_GPIOInits(void)
{
	GPIO_Handle_t SPIPins;

	SPIPins.pGPIOx = GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OP;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	//SCLK
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIO_Init(&SPIPins);

	//MOSI
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
	GPIO_Init(&SPIPins);

	//MISO
	//SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	//GPIOInit(&SPIPins);

	//NSS
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIO_Init(&SPIPins);
}


void SPI2_Inits(void)
{
	SPI_Handle_t SPI2Handle;
	SPI2Handle.pSPIx 						= SPI2;
	SPI2Handle.SPIConfig.SPI_BusConfig 		= SPI_BUS_CONFIG_FD;
	SPI2Handle.SPIConfig.SPI_DeviceMode 	= SPI_DEVICE_MODE_MASTER;
	SPI2Handle.SPIConfig.SPI_SclkSpeed 		= SPI_SCLK_SPEED_DIV8;			//Will be set to 8 MHz
	SPI2Handle.SPIConfig.SPI_CPHA 			= SPI_CPHA_LOW;
	SPI2Handle.SPIConfig.SPI_CPOL			= SPI_CPOL_LOW;
	SPI2Handle.SPIConfig.SPI_DFF			= SPI_DFF_8BITS;
	SPI2Handle.SPIConfig.SPI_SSM			= SPI_SSM_DI;  					//Hardware slave management enables because we are not using NSS pin
	SPI_Init(&SPI2Handle);
}

void GPIOButtonInit(void)
{

	GPIO_Handle_t GpioButton;

GpioButton.pGPIOx = GPIOC;
GpioButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
GpioButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
GpioButton.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
GpioButton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;



//First enable the peripheral clock
GPIO_Init(&GpioButton);
}




int main(void)
{
	char user_data[] = "Hello World";


	GPIOButtonInit();

	//FUNCTION TO ENABLE GPIO'S TO BEHAVE AS SPI PINS
	SPI2_GPIOInits();

	//Function to initialize SPI2 peripherals
	SPI2_Inits();

	/*MAKING ssoe 1 Does NSS output enable
	 * The NSS Pin is automatically managed by the hardware
	 * i.e when SPE is = 1, NSS will be pulled to low
	 * and NSS will be high when SPE = 0;
	 */
	SPI_SSOEConfig(SPI2, ENABLE);
while(1)
{

	while(!(GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13)));

	delay();

	//This makes NSS internally to +vcc and SSI bit configures
	//SPI_SSIConfig(SPI2, ENABLE);

	//ENABLE SPI2 Peripheral
	SPI_PeripheralControl(SPI2, ENABLE);

	//First let's send length information
	uint8_t dataLen = strlen(user_data);
	SPI_SendData(SPI2, &dataLen, 1);



	//sending Data via SPI
	SPI_SendData(SPI2, (uint8_t *)user_data, strlen(user_data));


	//Let's confirm SPI is not busy
	while(SPI_GetFlag_Status(SPI2, SPI_BUSY_FLAG)); //If spi is not busy then this loop will break and we can resume the communication.


	//Disabling the peripheral after sending the data
	SPI_PeripheralControl(SPI2, DISABLE);
}



	return 0;
}

