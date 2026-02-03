/*******************************************************************************
 *
 * Copyright (c) 2023
 * Lumi, JSC.
 * All Rights Reserved
 *
 * Description: Include file for application
 *
 * Author: HoangNH
 *
 * Last Changed By:  $Author: ThaiHM $
 * Revision:         $Revision: $
 * Last Changed:     $Date: 15/12/2023 $
 *
*******************************************************************************/
/******************************************************************************/
/*                              INCLUDE FILES                                 */
/******************************************************************************/
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_spi.h"
#include "stm32f10x_dma.h"
#include "lcd.h"
#include "utilities.h"
/******************************************************************************/
/*                     EXPORTED TYPES and DEFINITIONS                         */
/******************************************************************************/
/******************************************************************************/
/*                              PRIVATE DATA                                  */
/******************************************************************************/
_lcd_dev lcddev;
uint16_t POINT_COLOR = 0x0000;
uint16_t BACK_COLOR = 0xFFFF;
extern const unsigned char bmp1[40960];
/******************************************************************************/
/*                              EXPORTED DATA                                 */
/******************************************************************************/
/******************************************************************************/
/*                            PRIVATE FUNCTIONS                               */
/******************************************************************************/
static void LCDGPIO_Config(void);
static void LCDSPI_Config(void);
static void LCD_RESET(void);
static uint8_t LCDSPI_WriteByte(uint8_t data);
/******************************************************************************/
/*                            EXPORTED FUNCTIONS                              */
/******************************************************************************/

/**
 * @func	LCD_WR_REG
 * @brief	Write an 8-bit command to the LCD screen
 * @param	Command value to be written
 * @retval  None
*/
void LCD_WR_REG(uint8_t data)
{
	LCD_SPI_CS_RESET;					//LCD_CS=0
	LCD_SPI_RS_RESET;
	LCDSPI_WriteByte(data);
	LCD_SPI_CS_SET;						//LCD_CS=1
}

/**
 * @func	LCD_WR_DATA
 * @brief	Write an 16-bit data to the LCD screen
 * @param	data value to be written
 * @retval  None
*/
void LCD_WR_DATA(uint16_t data)
{
	LCD_SPI_CS_RESET;					//LCD_CS=0
	LCD_SPI_RS_SET;
	LCDSPI_WriteByte(data>>8);
	LCDSPI_WriteByte(data);
	LCD_SPI_CS_SET;						//LCD_CS=1
}

/**
 * @func	LCD_WR_DATA
 * @brief	Write an 8-bit data to the LCD screen
 * @param	data value to be written
 * @retval  None
*/
void LCD_WR_DATA8(uint8_t data)
{
	LCD_SPI_CS_RESET;					//LCD_CS=0
	LCD_SPI_RS_SET;
	LCDSPI_WriteByte(data);
	LCD_SPI_CS_SET;						//LCD_CS=1
}

/**
 * @func   	LCD_WriteReg
 * @brief	Write data into registers
 * @param 	LCD_Reg:Register address
            LCD_RegValue: Data to be written
 * @retval  None
*/
void LCD_WriteReg(uint8_t LCD_Reg, uint16_t LCD_RegValue)
{
	LCD_WR_REG(LCD_Reg);
	LCD_WR_DATA8(LCD_RegValue);
}

/**
 * @func	LCD_WriteRAM_Prepare
 * @brief	Write GRAM
 * @param	None
 * @retval	None
*/
void LCD_WriteRAM_Prepare(void)
{
	LCD_WR_REG(lcddev.wramcmd);
}

/**
 * @func	LCD_DrawPoint
 * @brief	Write a pixel data at a specified location
 * @param	x:the x coordinate of the pixel
            y:the y coordinate of the pixel
 * @retval	None
*/
void LCD_DrawPoint(uint16_t x, uint16_t y)
{
	LCD_SetCursor(x, y);
	LCD_WR_DATA(POINT_COLOR);
}

/**
 * @func	LCD_Clear
 * @brief	Full screen filled LCD screen
 * @param	color:Filled color
 * @retval	None
*/
void LCD_Clear(uint16_t color)
{
	unsigned int i,m;

	LCD_SetWindows(0,0,lcddev.width-1,lcddev.height-1);
	LCD_SPI_CS_RESET;
	LCD_SPI_RS_SET;

	for(i=0;i<lcddev.height;i++) {
		for(m=0;m<lcddev.width;m++) {
			LCD_WR_DATA(color);
		}
	}

	LCD_SPI_CS_SET;
}

/**
 * @func	LCD_Clear_DMA
 * @brief	Full screen filled LCD screen mode DMA
 * @param	color:Filled color
 * @retval	None
*/
void LCD_Clear_DMA(
		uint16_t color
) {
	uint16_t u16Tmp;
	DMA_InitTypeDef 	dmaInit;

	uint32_t totalpoint = lcddev.width;
	totalpoint *= lcddev.height;
	LCD_SetCursor(0x00,0x0000);
	LCD_WriteRAM_Prepare();

	u16Tmp = color;

	SPI2->CR1 &= ~(1 << 6);
	SPI2->CR1 |= (1 << 11);
	SPI2->CR1 |= (1 << 6);

	LCD_SPI_CS_RESET;  //LCD_CS=0
	LCD_SPI_RS_SET;

	DMA_DeInit(DMA1_Channel5);

	dmaInit.DMA_BufferSize = 128 * 160;
	dmaInit.DMA_DIR = DMA_DIR_PeripheralDST;
	dmaInit.DMA_M2M = DMA_M2M_Disable;
	dmaInit.DMA_MemoryBaseAddr = (uint32_t)&u16Tmp;
	dmaInit.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	dmaInit.DMA_MemoryInc = DMA_MemoryInc_Disable;
	dmaInit.DMA_Mode = DMA_Mode_Normal;
	dmaInit.DMA_PeripheralBaseAddr = (uint32_t)&SPI2->DR;
	dmaInit.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	dmaInit.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	dmaInit.DMA_Priority = DMA_Priority_Medium;

	DMA_Init(DMA1_Channel5, &dmaInit);
	DMA_Cmd(DMA1_Channel5, ENABLE);
	SPI_I2S_DMACmd(LCD_SPI, SPI_I2S_DMAReq_Tx, ENABLE);

	while (DMA_GetFlagStatus(DMA1_FLAG_TC5) == RESET) {
	}

	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_BSY) == SET) {
	}

	SPI_I2S_DMACmd(LCD_SPI, SPI_I2S_DMAReq_Tx, DISABLE);

	LCD_SPI_CS_SET;   //LCD_CS=1

	SPI2->CR1 &= ~(1 << 6);
	SPI2->CR1 &= ~(1 << 11);
	SPI2->CR1 |= (1 << 6);
}

/**
 * @func	LCDGPIO_Config
 * @brief	Initialization LCD screen GPIO
 * @param	None
 * @retval	None
*/
static
void LCDGPIO_Config(void)
{
	GPIO_InitTypeDef 	GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(LCD_GPIO_RCC, ENABLE);
	RCC_APB2PeriphClockCmd(LCD_SPI_GPIO_RCC, ENABLE);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

	GPIO_InitStructure.GPIO_Pin = LCD_RST_PIN | LCD_AO_PIN | LCD_LED_PIN;
	GPIO_Init(LCD_GPIO_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = LCD_CS_PIN;
	GPIO_Init(LCD_SPI_PORT, &GPIO_InitStructure);

	/* Turn on Led Background of LCD */
	GPIO_SetBits(LCD_GPIO_PORT, LCD_LED_PIN);
}

/**
 * @func   LCDSPI_Config
 * @brief  Initializes peripheral SPI
 * @param  None
 * @retval None
 */
static
void LCDSPI_Config(void)
{
	GPIO_InitTypeDef 	GPIO_InitStructure;
	SPI_InitTypeDef 	SPI_InitStructure;

	/* Enable SCK, MOSI and NSS GPIO clocks */
	RCC_APB2PeriphClockCmd(LCD_SPI_GPIO_RCC, ENABLE);

	/* Enable the SPI periph */
	RCC_APB1PeriphClockCmd(LCD_SPI_RCC, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

	/* Initializes GPIO Use For SPI */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = LCD_SPI_GPIO_SCK | LCD_SPI_GPIO_MOSI;
	GPIO_Init(LCD_SPI_PORT, &GPIO_InitStructure);

	/* Set to full duplex mode, separate MOSI lines */
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;

	/* Transmit in master mode */
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;

	/* One packet of data is 8 bits wide */
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;

	/* Clock is high when idle */
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;

	/* Data sampled at second edge */
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;

	/* Set NSS us software */
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;

	/* SPI frequency is APB2 frequency / 1 */
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;

	/* Data is transmitted MSB first */
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;

	SPI_InitStructure.SPI_CRCPolynomial = 7;

	SPI_Init(LCD_SPI, &SPI_InitStructure);

	/* Enable LCD_SPI */
	SPI_Cmd(LCD_SPI, ENABLE);
}

/**
 * @func	LCD_RESET
 * @brief	Reset LCD screen
 * @param	None
 * @retval	None
*/
static
void LCD_RESET(void)
{
	LCD_SPI_RST_RESET;
	delay_ms(100);
	LCD_SPI_RST_SET;
	delay_ms(50);
}

/**
 * @func	LCD_Init
 * @brief	Initialization LCD screen
 * @param	None
 * @retval	None
*/
void LCD_Init(void)
{
	LCDSPI_Config();
	LCDGPIO_Config();
	LCD_RESET();

	lcddev.width=128;
	lcddev.height=160;
	lcddev.wramcmd=0X2C;
	lcddev.setxcmd=0X2A;
	lcddev.setycmd=0X2B;

	LCD_WR_REG(0x11); //Sleep out
	delay_ms(120); //Delay 120ms
	//------------------------------------ST7735S Frame Rate-----------------------------------------//
	LCD_WR_REG(0xB1);
	LCD_WR_DATA8(0x05);
	LCD_WR_DATA8(0x3C);
	LCD_WR_DATA8(0x3C);
	LCD_WR_REG(0xB2);
	LCD_WR_DATA8(0x05);
	LCD_WR_DATA8(0x3C);
	LCD_WR_DATA8(0x3C);
	LCD_WR_REG(0xB3);
	LCD_WR_DATA8(0x05);
	LCD_WR_DATA8(0x3C);
	LCD_WR_DATA8(0x3C);
	LCD_WR_DATA8(0x05);
	LCD_WR_DATA8(0x3C);
	LCD_WR_DATA8(0x3C);
	//------------------------------------End ST7735S Frame Rate-----------------------------------------//
	LCD_WR_REG(0xB4); //Dot inversion
	LCD_WR_DATA8(0x03);
	LCD_WR_REG(0xC0);
	LCD_WR_DATA8(0x28);
	LCD_WR_DATA8(0x08);
	LCD_WR_DATA8(0x04);
	LCD_WR_REG(0xC1);
	LCD_WR_DATA8(0XC0);
	LCD_WR_REG(0xC2);
	LCD_WR_DATA8(0x0D);
	LCD_WR_DATA8(0x00);
	LCD_WR_REG(0xC3);
	LCD_WR_DATA8(0x8D);
	LCD_WR_DATA8(0x2A);
	LCD_WR_REG(0xC4);
	LCD_WR_DATA8(0x8D);
	LCD_WR_DATA8(0xEE);
	//---------------------------------End ST7735S Power Sequence-------------------------------------//
	LCD_WR_REG(0xC5); //VCOM
	LCD_WR_DATA8(0x1A);
	LCD_WR_REG(0x36); //MX, MY, RGB mode
	LCD_WR_DATA8(0xC0);
	//------------------------------------ST7735S Gamma Sequence-----------------------------------------//
	LCD_WR_REG(0xE0);
	LCD_WR_DATA8(0x04);
	LCD_WR_DATA8(0x22);
	LCD_WR_DATA8(0x07);
	LCD_WR_DATA8(0x0A);
	LCD_WR_DATA8(0x2E);
	LCD_WR_DATA8(0x30);
	LCD_WR_DATA8(0x25);
	LCD_WR_DATA8(0x2A);
	LCD_WR_DATA8(0x28);
	LCD_WR_DATA8(0x26);
	LCD_WR_DATA8(0x2E);
	LCD_WR_DATA8(0x3A);
	LCD_WR_DATA8(0x00);
	LCD_WR_DATA8(0x01);
	LCD_WR_DATA8(0x03);
	LCD_WR_DATA8(0x13);
	LCD_WR_REG(0xE1);
	LCD_WR_DATA8(0x04);
	LCD_WR_DATA8(0x16);
	LCD_WR_DATA8(0x06);
	LCD_WR_DATA8(0x0D);
	LCD_WR_DATA8(0x2D);
	LCD_WR_DATA8(0x26);
	LCD_WR_DATA8(0x23);
	LCD_WR_DATA8(0x27);
	LCD_WR_DATA8(0x27);
	LCD_WR_DATA8(0x25);
	LCD_WR_DATA8(0x2D);
	LCD_WR_DATA8(0x3B);
	LCD_WR_DATA8(0x00);
	LCD_WR_DATA8(0x01);
	LCD_WR_DATA8(0x04);
	LCD_WR_DATA8(0x13);
	//------------------------------------End ST7735S Gamma Sequence-----------------------------------------//
	LCD_WR_REG(0x3A); //65k mode
	LCD_WR_DATA8(0x05);
	LCD_WR_REG(0x29); //Display on

	LCD_Clear(WHITE);
}

/**
 * @func	LCD_SetWindows
 * @brief	Setting LCD display window
 * @param	xStar:the bebinning x coordinate of the LCD display window
			yStar:the bebinning y coordinate of the LCD display window
			xEnd:the endning x coordinate of the LCD display window
			yEnd:the endning y coordinate of the LCD display window
 * @retval	None
*/
void LCD_SetWindows(
		uint16_t xStar, uint16_t yStar,
		uint16_t xEnd ,uint16_t yEnd
) {
	LCD_WR_REG(lcddev.setxcmd);
	LCD_WR_DATA8(0x00);
	LCD_WR_DATA8(xStar);
	LCD_WR_DATA8(0x00);
	LCD_WR_DATA8(xEnd);

	LCD_WR_REG(lcddev.setycmd);
	LCD_WR_DATA8(0x00);
	LCD_WR_DATA8(yStar);
	LCD_WR_DATA8(0x00);
	LCD_WR_DATA8(yEnd);

	LCD_WriteRAM_Prepare();
}

/**
 * @func	LCD_SetCursor
 * @brief	Set coordinate value
 * @param	Xpos:the  x coordinate of the pixel
			Ypos:the  y coordinate of the pixel
 * @retval	None
*/
void LCD_SetCursor(uint16_t Xpos, uint16_t Ypos)
{
	LCD_SetWindows(Xpos,Ypos,Xpos,Ypos);
}

/**
 * @func	LCD_direction
 * @brief	Setting the display direction of LCD screen
 * @param	direction: 0-0 degree
                       1-90 degree
					   2-180 degree
					   3-270 degree
 * @retval	None
*/
void LCD_direction(uint8_t direction)
{
	lcddev.setxcmd=0x2A;
	lcddev.setycmd=0x2B;
	lcddev.wramcmd=0x2C;

	switch(direction){
		case 0:
			lcddev.width=LCD_W;
			lcddev.height=LCD_H;
			LCD_WriteReg(0x36,(0<<3)|(1<<6)|(1<<7));//BGR==1,MY==0,MX==0,MV==0
			break;
		case 1:
			lcddev.width=LCD_H;
			lcddev.height=LCD_W;
			LCD_WriteReg(0x36,(0<<3)|(1<<7)|(1<<5));//BGR==1,MY==1,MX==0,MV==1
			break;
		case 2:
			lcddev.width=LCD_W;
			lcddev.height=LCD_H;
			LCD_WriteReg(0x36,(0<<3)|(0<<6)|(0<<7));//BGR==1,MY==0,MX==0,MV==0
			break;
		case 3:
			lcddev.width=LCD_H;
			lcddev.height=LCD_W;
			LCD_WriteReg(0x36,(0<<3)|(0<<7)|(1<<6)|(1<<5));//BGR==1,MY==1,MX==0,MV==1
			break;

		default:
			break;
	}
}

/**
 * @func	LCDSPI_WriteByte
 * @brief	Write a byte of data using STM32's Software SPI
 * @param	data:Data to be written
 * @retval	None
*/
static
uint8_t LCDSPI_WriteByte(
		uint8_t data
) {
	SPI_I2S_SendData(LCD_SPI, data);
	while (SPI_I2S_GetFlagStatus(LCD_SPI, SPI_I2S_FLAG_BSY) == SET) {
	}
	return SPI_I2S_ReceiveData(LCD_SPI);
}

/* END FILE */
