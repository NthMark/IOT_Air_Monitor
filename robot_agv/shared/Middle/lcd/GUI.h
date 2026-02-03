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
 * Last Changed:     $Date:  15/12/23 $
 *
*******************************************************************************/
#ifndef _GUI_H_
#define _GUI_H_
/******************************************************************************/
/*                              INCLUDE FILES                                 */
/******************************************************************************/
/******************************************************************************/
/*                     EXPORTED TYPES and DEFINITIONS                         */
/******************************************************************************/
/******************************************************************************/
/*                              PRIVATE DATA                                  */
/******************************************************************************/
/******************************************************************************/
/*                              EXPORTED DATA                                 */
/******************************************************************************/
/******************************************************************************/
/*                            EXPORTED FUNCTIONS                              */
/******************************************************************************/

/**
 * @func	GUI_DrawPoint
 * @brief	draw a point in LCD screen
 * @param	x:the x coordinate of the point
			y:the y coordinate of the point
			color:the color value of the point
 * @retval	None
*/
void GUI_DrawPoint(uint16_t x, uint16_t y, uint16_t color);

/**
 * @func	LCD_Fill
 * @brief	fill the specified area
 * @param	sx:the bebinning x coordinate of the specified area
			sy:the bebinning y coordinate of the specified area
			ex:the ending x coordinate of the specified area
			ey:the ending y coordinate of the specified area
			color:the filled color value
 * @retval	None
*/

void LCD_DrawFillBox(
        uint16_t x, uint16_t y,
        uint16_t w, uint16_t h,
        uint16_t color
);

void LCD_Fill(uint16_t sx, uint16_t sy,
			  uint16_t ex, uint16_t ey, uint16_t color);

/**
 * @func	LCD_DrawLine
 * @brief	Draw a line between two points
 * @param	x1:the bebinning x coordinate of the line
			y1:the bebinning y coordinate of the line
			x2:the ending x coordinate of the line
			y2:the ending y coordinate of the line
 * @retval	None
*/
void LCD_DrawLine(uint16_t x1, uint16_t y1,
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
void LCD_DrawRectangle(uint16_t x1, uint16_t y1,
					   uint16_t x2, uint16_t y2);

/* */
void Draw_Circle(uint16_t x0, uint16_t y0,
				 uint16_t fc, uint8_t r);

/**
 * @func	Draw_Triangel
 * @brief	Draw a triangle at a specified position
 * @param	x0:the bebinning x coordinate of the triangular edge
			y0:the bebinning y coordinate of the triangular edge
			x1:the vertex x coordinate of the triangular
			y1:the vertex y coordinate of the triangular
			x2:the ending x coordinate of the triangular edge
			y2:the ending y coordinate of the triangular edge
 * @retval	None
*/
void Draw_Triangel(uint16_t x0, uint16_t y0,
				   uint16_t x1, uint16_t y1,
				   uint16_t x2, uint16_t y2);

/**
 * @func	Fill_Triangel
 * @brief	filling a triangle at a specified position
 * @param	x0:the bebinning x coordinate of the triangular edge
			y0:the bebinning y coordinate of the triangular edge
			x1:the vertex x coordinate of the triangular
			y1:the vertex y coordinate of the triangular
			x2:the ending x coordinate of the triangular edge
			y2:the ending y coordinate of the triangular edge
 * @retval	None
*/
void Fill_Triangel(uint16_t x0, uint16_t y0,
				   uint16_t x1, uint16_t y1,
				   uint16_t x2, uint16_t y2);

/**
 * @func	LCD_ShowChar
 * @brief	Display a single English character
 * @param	x:the bebinning x coordinate of the Character display position
			y:the bebinning y coordinate of the Character display position
			fc:the color value of display character
			bc:the background color of display character
			num:the ascii code of display character(0~94)
			size:the size of display character
			mode:0-no overlying,1-overlying
 * @retval	None
*/
void LCD_ShowChar(uint16_t x, uint16_t y,
				  uint16_t fc, uint16_t bc,
				  uint8_t num, uint8_t size, uint8_t mode);

/**
 * @func	LCD_ShowNum
 * @brief	Display number
 * @param	x:the bebinning x coordinate of the number
			y:the bebinning y coordinate of the number
			num:the number(0~4294967295)
			len:the length of the display number
			size:the size of display number
 * @retval	None
*/
void LCD_ShowNum(uint16_t x, uint16_t y,
				 uint32_t num, uint8_t len, uint8_t size);

/* */
void LCD_Show2Num(uint16_t x, uint16_t y, uint16_t num,
				  uint8_t len, uint8_t size, uint8_t mode);

/**
 * @func	LCD_ShowString
 * @brief	Display English string
 * @param	x:the bebinning x coordinate of the English string
			y:the bebinning y coordinate of the English string
			p:the start address of the English string
			size:the size of display character
			mode:0-no overlying,1-overlying
 * @retval	None
*/
void LCD_ShowString(uint16_t x, uint16_t y,
					uint8_t size, uint8_t *p, uint8_t mode);

/**
 * @func	GUI_DrawFont16
 * @brief	Display a single 16x16 Chinese character
 * @param	x:the bebinning x coordinate of the Chinese character
			y:the bebinning y coordinate of the Chinese character
			fc:the color value of Chinese character
			bc:the background color of Chinese character
			s:the start address of the Chinese character
			mode:0-no overlying,1-overlying
 * @retval	None
*/
void GUI_DrawFont16(uint16_t x, uint16_t y,
					uint16_t fc, uint16_t bc,
					uint8_t *s, uint8_t mode);

/**
 * @func	GUI_DrawFont24
 * @brief	Display a single 24x24 Chinese character
 * @param	x:the bebinning x coordinate of the Chinese character
			y:the bebinning y coordinate of the Chinese character
			fc:the color value of Chinese character
			bc:the background color of Chinese character
			s:the start address of the Chinese character
			mode:0-no overlying,1-overlying
 * @retval	None
*/
void GUI_DrawFont24(uint16_t x, uint16_t y,
					uint16_t fc, uint16_t bc,
					uint8_t *s, uint8_t mode);

/**
 * @func	GUI_DrawFont32
 * @brief	Display a single 32x32 Chinese character
 * @param	x:the bebinning x coordinate of the Chinese character
			y:the bebinning y coordinate of the Chinese character
			fc:the color value of Chinese character
			bc:the background color of Chinese character
			s:the start address of the Chinese character
			mode:0-no overlying,1-overlying
 * @retval	None
*/
void GUI_DrawFont32(uint16_t x, uint16_t y,
					uint16_t fc, uint16_t bc,
					uint8_t *s, uint8_t mode);

/**
 * @func	Show_Str
 * @brief	Display Chinese and English strings
 * @param	x:the bebinning x coordinate of the Chinese and English strings
			y:the bebinning y coordinate of the Chinese and English strings
			fc:the color value of Chinese and English strings
			bc:the background color of Chinese and English strings
			str:the start address of the Chinese and English strings
			size:the size of Chinese and English strings
			mode:0-no overlying,1-overlying
 * @retval	None
*/
void Show_Str(uint16_t x, uint16_t y,
			  uint16_t fc, uint16_t bc,
			  uint8_t *str, uint8_t size, uint8_t mode);

/**
 * @func	Gui_Drawbmp16
 * @brief	Display a 16-bit BMP image
 * @param	x:the bebinning x coordinate of the BMP image
			y:the bebinning y coordinate of the BMP image
			p:the start address of image array
 * @retval	None
*/
void Gui_Drawbmp16(uint16_t x, uint16_t y,
				   const unsigned char *p);

/**
 * @func	gui_circle
 * @brief	Draw a circle of specified size at a specified location
 * @param	xc:the x coordinate of the Circular center
			yc:the y coordinate of the Circular center
			r:Circular radius
			fill:1-filling,0-no filling
 * @retval	None
*/
void gui_circle(int xc, int yc,
				uint16_t c,
				int r, int fill);

/**
 * @func	Gui_StrCenter
 * @brief	Centered display of English and Chinese strings
 * @param	x:the bebinning x coordinate of the Chinese and English strings
			y:the bebinning y coordinate of the Chinese and English strings
			fc:the color value of Chinese and English strings
			bc:the background color of Chinese and English strings
			str:the start address of the Chinese and English strings
			size:the size of Chinese and English strings
			mode:0-no overlying,1-overlying
 * @retval	None
*/
void Gui_StrCenter(uint16_t x, uint16_t y,
				   uint16_t fc, uint16_t bc,
				   uint8_t *str, uint8_t size, uint8_t mode);

/**
 * @func	LCD_DrawFillRectangle
 * @brief	Filled a rectangle
 * @param	x1:the bebinning x coordinate of the filled rectangle
			y1:the bebinning y coordinate of the filled rectangle
			x2:the ending x coordinate of the filled rectangle
			y2:the ending y coordinate of the filled rectangle
 * @retval	None
*/
void LCD_DrawFillRectangle(uint16_t x1, uint16_t y1,
						   uint16_t x2, uint16_t y2);

void LCD_Clear_DMA(uint16_t color);
//void showimage(uint16_t x, uint16_t y);
void showimage(void);
void show_img1(void);
void showimage_DMA(uint16_t x, uint16_t y);

#endif

/* END FILE */
