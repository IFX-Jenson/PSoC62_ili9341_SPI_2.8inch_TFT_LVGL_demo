/*
 * Copyright (c) 2006-2023,
 *
 * based on freeRTOS to drive the LCD_ili9341
 * Change Logs:
 * Date           Author           Notes
 * 2023-12-21     Xiao mile        First version
 */
#ifndef __LCD_ILI9341_H__
#define __LCD_ILI9341_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include "cyhal.h"

typedef struct
{
    uint16_t width;   /* LCD width */
    uint16_t height;  /* LCD high */
    uint32_t id;      /* LCD ID */
    uint8_t dir;      /* 0:Vertical | 1:Vertical */
    uint16_t wramcmd; /* gram cmd */
    uint16_t setxcmd; /* set x cmd */
    uint16_t setycmd; /* set y cmd */
} _lcd_dev;

/* LCD param */
extern _lcd_dev lcddev;

/* 0-0 angle|1-90 angle|2-180 angle|-270 angle */
#define USE_DIRECTION   0

/* lcd size */
#define LCD_W 240
#define LCD_H 320

#define PKG_ILI_9341_DC_PIN P7_6   // refer to SCH
#define PKG_ILI_9341_RES_PIN P0_5  // refer to SCH
#define PKG_ILI_9341_BLK_PIN NULL // BLK always on
#define PIN_LOW 0
#define PIN_HIGH 1

//#define LCD_DC_CLR  cyhal_gpio_write(PKG_ILI_9341_DC_PIN, PIN_LOW)
//#define LCD_DC_SET  cyhal_gpio_write(PKG_ILI_9341_DC_PIN, PIN_HIGH)
//#define LCD_RES_CLR cyhal_gpio_write(PKG_ILI_9341_RES_PIN, PIN_LOW)
//#define LCD_RES_SET cyhal_gpio_write(PKG_ILI_9341_RES_PIN, PIN_HIGH)
//#define LCD_BLK_CLR cyhal_gpio_write(PKG_ILI_9341_BLK_PIN, PIN_HIGH)

#define LCD_DC_CLR  Cy_GPIO_Write(CYHAL_GET_PORTADDR(PKG_ILI_9341_DC_PIN),CYHAL_GET_PIN(PKG_ILI_9341_DC_PIN), PIN_LOW)
#define LCD_DC_SET  Cy_GPIO_Write(CYHAL_GET_PORTADDR(PKG_ILI_9341_DC_PIN),CYHAL_GET_PIN(PKG_ILI_9341_DC_PIN),PIN_HIGH)
#define LCD_RES_CLR Cy_GPIO_Write(CYHAL_GET_PORTADDR(PKG_ILI_9341_RES_PIN),CYHAL_GET_PIN(PKG_ILI_9341_RES_PIN),PIN_LOW)
#define LCD_RES_SET Cy_GPIO_Write(CYHAL_GET_PORTADDR(PKG_ILI_9341_RES_PIN),CYHAL_GET_PIN(PKG_ILI_9341_RES_PIN), PIN_HIGH)
#define LCD_BLK_CLR Cy_GPIO_Write(PKG_ILI_9341_BLK_PIN, PIN_HIGH)

#define WHITE 0xFFFF
#define BLACK 0x0000
#define BLUE 0x001F
#define BRED 0XF81F
#define GRED 0XFFE0
#define GBLUE 0X07FF
#define RED 0xF800
#define MAGENTA 0xF81F
#define GREEN 0x07E0
#define CYAN 0x7FFF
#define YELLOW 0xFFE0
#define BROWN 0XBC40
#define BRRED 0XFC07
#define GRAY 0X8430

void LCD_Clear(uint16_t Color);
void LCD_direction(uint8_t direction);
void LCD_SetCursor(uint16_t Xpos, uint16_t Ypos);
void LCD_SetWindows(uint16_t xStar, uint16_t yStar, uint16_t xEnd, uint16_t yEnd);
void LCD_Fill(uint16_t xsta, uint16_t ysta, uint16_t xend, uint16_t yend, uint16_t color);
void lcd_fill_array_spi(uint16_t x_start, uint16_t y_start, uint16_t x_end, uint16_t y_end, void *pcolor);
void ili9341_lcd_init(void);

cy_rslt_t spi_lcd_init(uint32_t freq);

#ifdef __cplusplus
}
#endif
#endif
