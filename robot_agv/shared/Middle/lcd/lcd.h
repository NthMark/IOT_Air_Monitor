/*******************************************************************************
 *
 * Copyright (c) 2023
 * Lumi, JSC.
 * All Rights Reserved
 *
 *
 * Description:
 *
 * Author: ThaiHM
 *
 * Last Changed By:  $Author: ThaiHM $
 * Revision:         $Revision: 2.0 $
 * Last Changed:     $Date:  28/11/23 $
 *
*******************************************************************************/
#ifndef _LCD_H_
#define _LCD_H_
/******************************************************************************/
/*                              INCLUDE FILES                                 */
/******************************************************************************/
#include <stdint.h>
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
/******************************************************************************/
/*                     EXPORTED TYPES and DEFINITIONS                         */
/******************************************************************************/
/*! @brief LCD Pins */
#define LCD_SPI					SPI2
#define LCD_SPI_RCC				RCC_APB1Periph_SPI2
#define LCD_SPI_GPIO_RCC		RCC_APB2Periph_GPIOB
#define LCD_SPI_PORT			GPIOB

#define LCD_CS_PIN				GPIO_Pin_12
#define LCD_SPI_GPIO_SCK		GPIO_Pin_13
#define LCD_SPI_GPIO_MOSI		GPIO_Pin_15

#define LCD_GPIO_RCC			RCC_APB2Periph_GPIOA
#define LCD_GPIO_PORT			GPIOA

#define LCD_RST_PIN 			GPIO_Pin_8
#define LCD_AO_PIN 			    GPIO_Pin_11
#define LCD_LED_PIN 			GPIO_Pin_12

/*! @brief LCD_RST Pin */
#define LCD_SPI_RST_SET			GPIO_SetBits(LCD_GPIO_PORT, LCD_RST_PIN)
#define LCD_SPI_RST_RESET		GPIO_ResetBits(LCD_GPIO_PORT, LCD_RST_PIN)

/*! @brief LCD_RS Pin */
#define LCD_SPI_RS_SET			GPIO_SetBits(LCD_GPIO_PORT, LCD_AO_PIN)
#define LCD_SPI_RS_RESET		GPIO_ResetBits(LCD_GPIO_PORT, LCD_AO_PIN)

/*! @brief LCD_CS Pin */
#define LCD_SPI_CS_SET			GPIO_SetBits(LCD_SPI_PORT, LCD_CS_PIN)
#define LCD_SPI_CS_RESET		GPIO_ResetBits(LCD_SPI_PORT, LCD_CS_PIN)

/*! @brief LCD_MOSI Pin */
#define LCD_SPI_MOSI_SET		GPIO_SetBits(LCD_SPI_PORT, LCD_SPI_GPIO_MOSI)
#define LCD_SPI_MOSI_RESET		GPIO_ResetBits(LCD_SPI_PORT, LCD_SPI_GPIO_MOSI)

/*! @brief LCD_SCK Pin */
#define LCD_SPI_SCK_SET			GPIO_SetBits(LCD_SPI_PORT, LCD_SPI_GPIO_SCK)
#define LCD_SPI_SCK_RESET		GPIO_ResetBits(LCD_SPI_PORT, LCD_SPI_GPIO_SCK)

#define USE_HORIZONTAL  		0

/*! @brief LCD pixels */
#define LCD_W 					128
#define LCD_H 					160

/*! @brief Color */
#define WHITE       			0xFFFF
#define BLACK      				0x0000
#define BLUE       				0x001F
#define BRED        			0XF81F
#define GRED 					0XFFE0
#define GBLUE					0X07FF
#define RED         			0xF800
#define MAGENTA     			0xF81F
#define GREEN       			0x07E0
#define CYAN        			0x7FFF
#define YELLOW      			0xFFE0
#define BROWN 					0XBC40
#define BRRED 					0XFC07
#define GRAY  					0X8430

//GUI
#define DARKBLUE      	 		0X01CF
#define LIGHTBLUE      	 		0X7D7C
#define GRAYBLUE       	 		0X5458

#define LIGHTGREEN     			0X841F
#define LIGHTGRAY     			0XEF5B
#define LGRAY 					0XC618

#define LGRAYBLUE      			0XA651
#define LBBLUE          		0X2B12

#define LCD_BASE        		((uint32_t)(0x60000000 | 0x0007FFFE))
#define LCD             		((LCD_TypeDef *) LCD_BASE)

/*! @brief structure lcd */
typedef struct
{
	uint16_t width;
	uint16_t height;
	uint16_t id;
	uint16_t wramcmd;
	uint16_t setxcmd;
	uint16_t setycmd;
}_lcd_dev;

typedef struct
{
	uint16_t LCD_REG;
	uint16_t LCD_RAM;
} LCD_TypeDef;

/******************************************************************************/
/*                              PRIVATE DATA                                  */
/******************************************************************************/
/******************************************************************************/
/*                              EXPORTED DATA                                 */
/******************************************************************************/
extern _lcd_dev lcddev;
extern uint16_t POINT_COLOR;
extern uint16_t BACK_COLOR;
/******************************************************************************/
/*                            EXPORTED FUNCTIONS                              */
/******************************************************************************/

/**
 * @func	LCD_Init
 * @brief	Initialization LCD screen
 * @param	None
 * @retval	None
*/
void LCD_Init(void);

/**
 * @func	LCD_Clear
 * @brief	Full screen filled LCD screen
 * @param	color:Filled color
 * @retval	None
*/
void LCD_Clear(uint16_t color);

/**
 * @func	LCD_SetCursor
 * @brief	Set coordinate value
 * @param	Xpos:the  x coordinate of the pixel
			Ypos:the  y coordinate of the pixel
 * @retval	None
*/
void LCD_SetCursor(uint16_t Xpos, uint16_t Ypos);

/**
 * @func	LCD_DrawPoint
 * @brief	Write a pixel data at a specified location
 * @param	x:the x coordinate of the pixel
            y:the y coordinate of the pixel
 * @retval	None
*/
void LCD_DrawPoint(uint16_t x, uint16_t y);

/**
 * @func	LCD_DrawLine
 * @brief	Draw a line between two points
 * @param	x1:the bebinning x coordinate of the line
			y1:the bebinning y coordinate of the line
			x2:the ending x coordinate of the line
			y2:the ending y coordinate of the line
 * @retval	None
*/
void LCD_DrawLine(uint16_t x1, uint16_t y1, \
				  uint16_t x2, uint16_t y2);

/**
 * @func	LCD_DrawRectangle
 * @brief	Draw a rectangle
 * @param	x1:the bebinning x coordinate of the rectangle
			y1:the bebinning y coordinate of the rectangle
			x2:the ending x coordinate of the rectangle
			y2:the ending y coordinate of the rectangle
 * @retval	None
*/
void LCD_DrawRectangle(uint16_t x1, uint16_t y1, \
					   uint16_t x2, uint16_t y2);

/**
 * @func	LCD_SetWindows
 * @brief	Setting LCD display window
 * @param	xStar:the bebinning x coordinate of the LCD display window
			yStar:the bebinning y coordinate of the LCD display window
			xEnd:the endning x coordinate of the LCD display window
			yEnd:the endning y coordinate of the LCD display window
 * @retval	None
*/
void LCD_SetWindows(uint16_t xStar, uint16_t yStar, \
					uint16_t xEnd, uint16_t yEnd);

/**
 * @func   	LCD_WriteReg
 * @brief	Write data into registers
 * @param 	LCD_Reg:Register address
            LCD_RegValue: Data to be written
 * @retval  None
*/
void LCD_WriteReg(uint8_t LCD_Reg, uint16_t LCD_RegValue);

/**
 * @func	LCD_WR_DATA
 * @brief	Write an 8-bit data to the LCD screen
 * @param	data value to be written
 * @retval  None
*/
void LCD_WR_DATA8(uint8_t data);

/**
 * @func   	LCD_WriteReg
 * @brief	Write data into registers
 * @param 	LCD_Reg:Register address
            LCD_RegValue: Data to be written
 * @retval  None
*/
void LCD_WriteReg(uint8_t LCD_Reg, uint16_t LCD_RegValue);

/**
 * @func	LCD_WriteRAM_Prepare
 * @brief	Write GRAM
 * @param	None
 * @retval	None
*/
void LCD_WriteRAM_Prepare(void);

/**
 * @func	LCD_WR_DATA
 * @brief	Write an 16-bit data to the LCD screen
 * @param	data value to be written
 * @retval  None
*/
void LCD_WR_DATA(uint16_t data);

/**
 * @func	LCD_direction
 * @brief	Setting the display direction of LCD screen
 * @param	direction: 0-0 degree
                       1-90 degree
					   2-180 degree
					   3-270 degree
 * @retval	None
*/
void LCD_direction(uint8_t direction);

#endif

/* END FILE */
