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
/******************************************************************************/
/*                              INCLUDE FILES                                 */
/******************************************************************************/
#include "stm32f10x_dma.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_spi.h"
#include "lcd.h"
#include "string.h"
#include "font.h"
#include "GUI.h"
#include "utilities.h"
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
 * @func	LCD_DrawFillBox
 * @brief	Draw a filled box
 * @param	x: the beginning x coordinate of the filled box
 *          y: the beginning y coordinate of the filled box
 *          w: the width of the filled box
 *          h: the height of the filled box
 *          color: the filled color value
 * @retval	None
 */
void LCD_DrawFillBox(
        uint16_t x, uint16_t y,
        uint16_t w, uint16_t h,
        uint16_t color
) {
    LCD_Fill(x, y, x + w - 1, y + h - 1, color);
}

/**
 * @func	GUI_DrawPoint
 * @brief	draw a point in LCD screen
 * @param	x:the x coordinate of the point
			y:the y coordinate of the point
			color:the color value of the point
 * @retval	None
*/
void GUI_DrawPoint(
		uint16_t x, uint16_t y,
		uint16_t color
) {
	LCD_SetCursor(x,y);
	LCD_WR_DATA(color);
}



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
void LCD_Fill(
		uint16_t sx, uint16_t sy,
		uint16_t ex, uint16_t ey,
		uint16_t color
) {
	uint16_t i,j;
	uint16_t width=ex-sx+1;
	uint16_t height=ey-sy+1;

	LCD_SetWindows(sx,sy,ex,ey);

	for(i=0;i<height;i++) {
		for(j=0;j<width;j++) {
			LCD_WR_DATA(color);
		}
	}

	LCD_SetWindows(0,0,lcddev.width-1,lcddev.height-1);
}

/**
 * @func	LCD_DrawLine
 * @brief	Draw a line between two points
 * @param	x1:the bebinning x coordinate of the line
			y1:the bebinning y coordinate of the line
			x2:the ending x coordinate of the line
			y2:the ending y coordinate of the line
 * @retval	None
*/
void LCD_DrawLine(
		uint16_t x1, uint16_t y1,
		uint16_t x2, uint16_t y2
) {
	uint16_t t;
	int xerr=0,yerr=0,delta_x,delta_y,distance;
	int incx,incy,uRow,uCol;

	delta_x=x2-x1;
	delta_y=y2-y1;
	uRow=x1;
	uCol=y1;
	if(delta_x>0)incx=1;
	else if(delta_x==0)incx=0;
	else {incx=-1;delta_x=-delta_x;}
	if(delta_y>0)incy=1;
	else if(delta_y==0)incy=0;
	else{incy=-1;delta_y=-delta_y;}
	if( delta_x>delta_y)distance=delta_x;
	else distance=delta_y;
	for(t=0;t<=distance+1;t++ )
	{
		LCD_DrawPoint(uRow,uCol);
		xerr+=delta_x ;
		yerr+=delta_y ;
		if(xerr>distance)
		{
			xerr-=distance;
			uRow+=incx;
		}
		if(yerr>distance)
		{
			yerr-=distance;
			uCol+=incy;
		}
	}
}

/**
 * @func	LCD_DrawRectangle
 * @brief	Draw a rectangle
 * @param	x1:the bebinning x coordinate of the rectangle
			y1:the bebinning y coordinate of the rectangle
			x2:the ending x coordinate of the rectangle
			y2:the ending y coordinate of the rectangle
 * @retval	None
*/
void LCD_DrawRectangle(
		uint16_t x1, uint16_t y1,
		uint16_t x2, uint16_t y2
) {
	LCD_DrawLine(x1,y1,x2,y1);
	LCD_DrawLine(x1,y1,x1,y2);
	LCD_DrawLine(x1,y2,x2,y2);
	LCD_DrawLine(x2,y1,x2,y2);
}

/**
 * @func	LCD_DrawFillRectangle
 * @brief	Filled a rectangle
 * @param	x1:the bebinning x coordinate of the filled rectangle
			y1:the bebinning y coordinate of the filled rectangle
			x2:the ending x coordinate of the filled rectangle
			y2:the ending y coordinate of the filled rectangle
 * @retval	None
*/
void LCD_DrawFillRectangle(
		uint16_t x1, uint16_t y1,
		uint16_t x2, uint16_t y2
) {
	LCD_Fill(x1,y1,x2,y2,POINT_COLOR);
}

/**
 * @func	_draw_circle_8
 * @brief	8 symmetry circle drawing algorithm (internal call)
 * @param	xc:the x coordinate of the Circular center
			yc:the y coordinate of the Circular center
			x:the x coordinate relative to the Circular center
			y:the y coordinate relative to the Circular center
			c:the color value of the circle
 * @retval	None
*/
void _draw_circle_8(
		int xc, int yc,
		int x, int y,
		uint16_t c
) {
	GUI_DrawPoint(xc + x, yc + y, c);
	GUI_DrawPoint(xc - x, yc + y, c);
	GUI_DrawPoint(xc + x, yc - y, c);
	GUI_DrawPoint(xc - x, yc - y, c);
	GUI_DrawPoint(xc + y, yc + x, c);
	GUI_DrawPoint(xc - y, yc + x, c);
	GUI_DrawPoint(xc + y, yc - x, c);
	GUI_DrawPoint(xc - y, yc - x, c);
}

/**
 * @func	gui_circle
 * @brief	Draw a circle of specified size at a specified location
 * @param	xc:the x coordinate of the Circular center
			yc:the y coordinate of the Circular center
			r:Circular radius
			fill:1-filling,0-no filling
 * @retval	None
*/
void gui_circle(
		int xc, int yc,
		uint16_t c,
		int r, int fill

) {
	int x = 0, y = r, yi, d;

	d = 3 - 2 * r;


	if (fill)
	{
		while (x <= y) {
			for (yi = x; yi <= y; yi++)
				_draw_circle_8(xc, yc, x, yi, c);

			if (d < 0) {
				d = d + 4 * x + 6;
			} else {
				d = d + 4 * (x - y) + 10;
				y--;
			}
			x++;
		}
	} else
	{
		while (x <= y) {
			_draw_circle_8(xc, yc, x, y, c);
			if (d < 0) {
				d = d + 4 * x + 6;
			} else {
				d = d + 4 * (x - y) + 10;
				y--;
			}
			x++;
		}
	}
}

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
void Draw_Triangel(
		uint16_t x0, uint16_t y0,
		uint16_t x1, uint16_t y1,
		uint16_t x2, uint16_t y2
) {
	LCD_DrawLine(x0,y0,x1,y1);
	LCD_DrawLine(x1,y1,x2,y2);
	LCD_DrawLine(x2,y2,x0,y0);
}

/**
 * @func	_swap
 * @brief
 * @param
 * @retval	None
*/
static
void _swap(
		uint16_t *a, uint16_t *b
) {
	u16 tmp;
	tmp = *a;
	*a = *b;
	*b = tmp;
}

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
void Fill_Triangel(
		uint16_t x0, uint16_t y0,
		uint16_t x1, uint16_t y1,
		uint16_t x2, uint16_t y2
) {
	uint16_t a, b, y, last;
	int dx01, dy01, dx02, dy02, dx12, dy12;
	long sa = 0;
	long sb = 0;

	if (y0 > y1)
	{
	_swap(&y0,&y1);
		_swap(&x0,&x1);
	}
	if (y1 > y2)
	{
	_swap(&y2,&y1);
		_swap(&x2,&x1);
	}
	if (y0 > y1)
	{
	_swap(&y0,&y1);
		_swap(&x0,&x1);
	}
	if(y0 == y2)
	{
		a = b = x0;
		if(x1 < a)
	{
			a = x1;
	}
	else if(x1 > b)
	{
			b = x1;
	}
	if(x2 < a)
	{
			a = x2;
	}
		else if(x2 > b)
	{
			b = x2;
	}
		LCD_Fill(a,y0,b,y0,POINT_COLOR);
	return;
	}
	dx01 = x1 - x0;
	dy01 = y1 - y0;
	dx02 = x2 - x0;
	dy02 = y2 - y0;
	dx12 = x2 - x1;
	dy12 = y2 - y1;

	if(y1 == y2)
	{
		last = y1;
	}
	else
	{
		last = y1-1;
	}
	for(y=y0; y<=last; y++)
	{
		a = x0 + sa / dy01;
		b = x0 + sb / dy02;
		sa += dx01;
	sb += dx02;
	if(a > b)
	{
			_swap(&a,&b);
		}
		LCD_Fill(a,y,b,y,POINT_COLOR);
	}
	sa = dx12 * (y - y1);
	sb = dx02 * (y - y0);
	for(; y<=y2; y++)
	{
		a = x1 + sa / dy12;
		b = x0 + sb / dy02;
		sa += dx12;
		sb += dx02;
		if(a > b)
		{
			_swap(&a,&b);
		}
		LCD_Fill(a,y,b,y,POINT_COLOR);
	}
}

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
void LCD_ShowChar(
		uint16_t x,  uint16_t y,
		uint16_t fc, uint16_t bc,
		uint8_t num, uint8_t size, uint8_t mode
) {
	uint8_t temp;
	uint8_t pos,t;
	uint16_t colortemp=POINT_COLOR;

	num=num-' ';
	LCD_SetWindows(x,y,x+size/2-1,y+size-1);
	if(!mode)
	{
		for(pos=0;pos<size;pos++)
		{
			if(size==12)temp=asc2_1206[num][pos];
			else temp=asc2_1608[num][pos];
			for(t=0;t<size/2;t++)
		    {
		        if(temp&0x01)LCD_WR_DATA(fc);
				else LCD_WR_DATA(bc);
				temp>>=1;

		    }

		}
	}else
	{
		for(pos=0;pos<size;pos++)
		{
			if(size==12)temp=asc2_1206[num][pos];
			else temp=asc2_1608[num][pos];
			for(t=0;t<size/2;t++)
		    {
				POINT_COLOR=fc;
		        if(temp&0x01)LCD_DrawPoint(x+t,y+pos);
		        temp>>=1;
		    }
		}
	}
	POINT_COLOR=colortemp;
	LCD_SetWindows(0,0,lcddev.width-1,lcddev.height-1);
}

/**
 * @func	LCD_ShowString
 * @brief	Display English string
 * @param	x:the bebinning x coordinate of the English string
			y:the bebinning y coordinate of the English string
			p:the start address of the English string
			size:the size of display character (>= 12 to display)
			mode:0-no overlying,1-overlying
 * @retval	None
*/
void LCD_ShowString(
		uint16_t x,  uint16_t y,
		uint8_t size, uint8_t *p, uint8_t mode
) {
    while((*p<='~')&&(*p>=' '))
    {
		if(x>(lcddev.width-1)||y>(lcddev.height-1))
		return;
        LCD_ShowChar(x,y,POINT_COLOR,BACK_COLOR,*p,size,mode);
        x+=size/2;
        p++;
    }
}

/**
 * @func	mypow
 * @brief	get the nth power of m (internal call)
 * @param	m:the multiplier
			n:the power
 * @retval	the nth power of m
*/
uint32_t mypow(
		uint8_t m, uint8_t n
) {
	uint32_t result=1;
	while(n--)result*=m;
	return result;
}

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
void LCD_ShowNum(
		uint16_t x, uint16_t y,
		uint32_t num,
		uint8_t len, uint8_t size
) {
	uint8_t t,temp;
	uint8_t enshow=0;
	for(t=0;t<len;t++)
	{
		temp=(num/mypow(10,len-t-1))%10;
		if(enshow==0&&t<(len-1))
		{
			if(temp==0)
			{
				LCD_ShowChar(x+(size/2)*t,y,POINT_COLOR,BACK_COLOR,' ',size,0);
				continue;
			}else enshow=1;

		}
	 	LCD_ShowChar(x+(size/2)*t,y,POINT_COLOR,BACK_COLOR,temp+'0',size,0);
	}
}

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
void GUI_DrawFont16(
		uint16_t x,  uint16_t y,
		uint16_t fc, uint16_t bc,
		uint8_t *s,  uint8_t mode
) {
	uint8_t i,j;
	uint16_t k;
	uint16_t HZnum;
	uint16_t x0=x;

	HZnum=sizeof(tfont16)/sizeof(typFNT_GB16);


	for (k=0;k<HZnum;k++)
	{
	  if ((tfont16[k].Index[0]==*(s))&&(tfont16[k].Index[1]==*(s+1)))
	  { 	LCD_SetWindows(x,y,x+16-1,y+16-1);
		    for(i=0;i<16*2;i++)
		    {
				for(j=0;j<8;j++)
		    	{
					if(!mode)
					{
						if(tfont16[k].Msk[i]&(0x80>>j))	LCD_WR_DATA(fc);
						else LCD_WR_DATA(bc);
					}
					else
					{
						POINT_COLOR=fc;
						if(tfont16[k].Msk[i]&(0x80>>j))	LCD_DrawPoint(x,y);
						x++;
						if((x-x0)==16)
						{
							x=x0;
							y++;
							break;
						}
					}
				}
			}
		}
		continue;
	}

	LCD_SetWindows(0,0,lcddev.width-1,lcddev.height-1);
}

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
void GUI_DrawFont24(
		uint16_t x,  uint16_t y,
		uint16_t fc, uint16_t bc,
		uint8_t *s,  uint8_t mode
) {
	uint8_t i,j;
	uint16_t k;
	uint16_t HZnum;
	uint16_t x0=x;

	HZnum=sizeof(tfont24)/sizeof(typFNT_GB24);

	for (k=0;k<HZnum;k++)
	{
	  if ((tfont24[k].Index[0]==*(s))&&(tfont24[k].Index[1]==*(s+1)))
	  { 	LCD_SetWindows(x,y,x+24-1,y+24-1);
			for(i=0;i<24*3;i++)
			{
					for(j=0;j<8;j++)
					{
						if(!mode)
						{
							if(tfont24[k].Msk[i]&(0x80>>j))	LCD_WR_DATA(fc);
							else LCD_WR_DATA(bc);
						}
					else
					{
						POINT_COLOR=fc;
						if(tfont24[k].Msk[i]&(0x80>>j))	LCD_DrawPoint(x,y);
						x++;
						if((x-x0)==24)
						{
							x=x0;
							y++;
							break;
						}
					}
				}
			}


		}
		continue;
	}

	LCD_SetWindows(0,0,lcddev.width-1,lcddev.height-1);
}

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
void GUI_DrawFont32(
		uint16_t x,  uint16_t y,
		uint16_t fc, uint16_t bc,
		uint8_t *s,  uint8_t mode
) {
	uint8_t i,j;
	uint16_t k;
	uint16_t HZnum;
	uint16_t x0=x;

	HZnum=sizeof(tfont32)/sizeof(typFNT_GB32);
	for (k=0;k<HZnum;k++)
			{
			  if ((tfont32[k].Index[0]==*(s))&&(tfont32[k].Index[1]==*(s+1)))
			  { 	LCD_SetWindows(x,y,x+32-1,y+32-1);
				    for(i=0;i<32*4;i++)
				    {
						for(j=0;j<8;j++)
				    	{
							if(!mode)
							{
								if(tfont32[k].Msk[i]&(0x80>>j))	LCD_WR_DATA(fc);
								else LCD_WR_DATA(bc);
							}
							else
							{
								POINT_COLOR=fc;
								if(tfont32[k].Msk[i]&(0x80>>j))	LCD_DrawPoint(x,y);
								x++;
								if((x-x0)==32)
								{
									x=x0;
									y++;
									break;
								}
							}
						}
					}


				}
				continue;
			}

	LCD_SetWindows(0,0,lcddev.width-1,lcddev.height-1);
}

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
void Show_Str(
		uint16_t x,  uint16_t y,
		uint16_t fc, uint16_t bc,
		uint8_t *str, uint8_t size, uint8_t mode
) {
	uint16_t x0=x;
	uint8_t bHz=0;

    while(*str!=0)
    {
        if(!bHz)
        {
			if(x>(lcddev.width-size/2)||y>(lcddev.height-size))
			return;
	        if(*str>0x80)bHz=1;
	        else
	        {
		        if(*str==0x0D)
		        {
		            y+=size;
					x=x0;
		            str++;
		        }
		        else
				{
					if(size>16)
					{
					LCD_ShowChar(x,y,fc,bc,*str,16,mode);
					x+=8;
					}
					else
					{
					LCD_ShowChar(x,y,fc,bc,*str,size,mode);
					x+=size/2;
					}
				}
				str++;

	        }
        }else
        {
			if(x>(lcddev.width-size)||y>(lcddev.height-size))
			return;
            bHz=0;
			if(size==32)
			GUI_DrawFont32(x,y,fc,bc,str,mode);
			else if(size==24)
			GUI_DrawFont24(x,y,fc,bc,str,mode);
			else
			GUI_DrawFont16(x,y,fc,bc,str,mode);

	        str+=2;
	        x+=size;
        }
    }
}

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
void Gui_StrCenter(
		uint16_t x,  uint16_t y,
		uint16_t fc, uint16_t bc,
		uint8_t *str, uint8_t size, uint8_t mode
) {
	uint16_t len=strlen((const char *)str);
	uint16_t x1=(lcddev.width-len*8)/2;
	Show_Str(x1,y,fc,bc,str,size,mode);
}

/**
 * @func	Gui_Drawbmp16
 * @brief	Display a 16-bit BMP image
 * @param	x:the bebinning x coordinate of the BMP image
			y:the bebinning y coordinate of the BMP image
			p:the start address of image array
 * @retval	None
*/
void Gui_Drawbmp16(
		uint16_t x, uint16_t y,
		const unsigned char *p
) {
  	int i;
	unsigned char picH,picL;
	LCD_SetWindows(x,y,x+40-1,y+40-1);

    for(i=0;i<40*40;i++) {
	 	picL=*(p+i*2);
		picH=*(p+i*2+1);
		LCD_WR_DATA(picH<<8|picL);
	}

	LCD_SetWindows(0,0,lcddev.width-1,lcddev.height-1);
}

/**
 * @func	showimage
 * @brief
 * @param
 * @retval	None
*/
void showimage(
//		uint16_t x,
//		uint16_t y
		void
) {
//	uint16_t i,j,k;
//	uint16_t da;
//	k=0;
//	for(i=0;i<40;i++)
//	{
//		LCD_SetCursor(x,y+i);
//		LCD_WriteRAM_Prepare();
//		for(j=0;j<500;j++)
//		{
//			da=qqimage[k*2+1];
////			da<<=8;
////			da|=qqimage[k*2];
//			LCD_WR_DATA8(da);
//			k++;
//		}
//	}
//	Gui_Drawbmp16(25, 45, &qqimage);
}

/**
 * @func	show_img1
 * @brief
 * @param
 * @retval	None
*/
void show_img1(void)
{
	DMA_InitTypeDef dmaInit;
	uint32_t i;

	for(i=0;i<160;i++)
	{
		LCD_SetCursor(0,i);
		LCD_WriteRAM_Prepare();

		/* cho nay chuyen thanh gui du lieu bang DMA */
		SPI2->CR1 &= ~(1 << 6);
		SPI2->CR1 |= (1 << 11);
		SPI2->CR1 |= (1 << 6);

		LCD_SPI_CS_RESET;  //LCD_CS=0
		LCD_SPI_RS_SET;

		DMA_DeInit(DMA1_Channel5);

		dmaInit.DMA_BufferSize = 128;
		dmaInit.DMA_DIR = DMA_DIR_PeripheralDST;
		dmaInit.DMA_M2M = DMA_M2M_Disable;
		dmaInit.DMA_MemoryBaseAddr = (uint32_t)&bmp1[i * 256];
		dmaInit.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
		dmaInit.DMA_MemoryInc = DMA_MemoryInc_Enable;
		dmaInit.DMA_Mode = DMA_Mode_Normal;
		dmaInit.DMA_PeripheralBaseAddr = (uint32_t)&SPI2->DR;
		dmaInit.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
		dmaInit.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
		dmaInit.DMA_Priority = DMA_Priority_Medium;

		DMA_Init(DMA1_Channel5, &dmaInit);
		DMA_Cmd(DMA1_Channel5, ENABLE);
		SPI_I2S_DMACmd(SPI2, SPI_I2S_DMAReq_Tx, ENABLE);

		while (DMA_GetFlagStatus(DMA1_FLAG_TC5) == RESET) {
		}

		while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_BSY) == SET) {
		}

		SPI_I2S_DMACmd(SPI2, SPI_I2S_DMAReq_Tx, DISABLE);

		LCD_SPI_CS_SET;  //LCD_CS=1

		SPI2->CR1 &= ~(1 << 6);
		SPI2->CR1 &= ~(1 << 11);
		SPI2->CR1 |= (1 << 6);
	}
}

/**
 * @func	showimage_DMA
 * @brief
 * @param
 * @retval	None
*/void showimage_DMA(
		uint16_t x, uint16_t y
) {
	uint16_t i,k;
	DMA_InitTypeDef dmaInit;

	k=0;
	for(i=0;i<40;i++)
	{
		LCD_SetCursor(x,y+i);
		LCD_WriteRAM_Prepare();

		/* cho nay chuyen thanh gui du lieu bang DMA */
		SPI2->CR1 &= ~(1 << 6);
		SPI2->CR1 |= (1 << 11);
		SPI2->CR1 |= (1 << 6);

		LCD_SPI_CS_RESET;  //LCD_CS=0
		LCD_SPI_RS_RESET;

		DMA_DeInit(DMA1_Channel5);

		dmaInit.DMA_BufferSize = 40;
		dmaInit.DMA_DIR = DMA_DIR_PeripheralDST;
		dmaInit.DMA_M2M = DMA_M2M_Disable;
		dmaInit.DMA_MemoryBaseAddr = (uint32_t)&qqimage[k * 2];
		k += 40;
		dmaInit.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
		dmaInit.DMA_MemoryInc = DMA_MemoryInc_Enable;
		dmaInit.DMA_Mode = DMA_Mode_Normal;
		dmaInit.DMA_PeripheralBaseAddr = (uint32_t)&SPI2->DR;
		dmaInit.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
		dmaInit.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
		dmaInit.DMA_Priority = DMA_Priority_Medium;

		DMA_Init(DMA1_Channel5, &dmaInit);
		DMA_Cmd(DMA1_Channel5, ENABLE);
		SPI_I2S_DMACmd(SPI2, SPI_I2S_DMAReq_Tx, ENABLE);

		while (DMA_GetFlagStatus(DMA1_FLAG_TC5) == RESET) {
		}

		while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_BSY) == SET) {
		}

		SPI_I2S_DMACmd(SPI2, SPI_I2S_DMAReq_Tx, DISABLE);

		LCD_SPI_CS_SET;  //LCD_CS=1

		SPI2->CR1 &= ~(1 << 6);
		SPI2->CR1 &= ~(1 << 11);
		SPI2->CR1 |= (1 << 6);
	}
}


/* END FILE */
