/*
 * Copyright (c) 2006-2023,
 *
 * Change Logs:
 * Date           Author           Notes
 * 2023-12-21     Xiao mile        First version
 */
#include "lcd_ili9341.h"
#include "cybsp.h"
#include "FreeRTOS.h"

#define DBG_TAG "ili9341"
#define DBG_LVL DBG_INFO

/* SPI transfer bits per frame */
#define BITS_PER_FRAME             (8)

#define LCD_SPI_MOSI P12_0
#define LCD_SPI_MISO P12_1
#define LCD_SPI_CLK  P12_2
#define LCD_SPI_CS   P12_3

_lcd_dev lcddev;
cyhal_spi_t lcd_spi;

void spi_send(cyhal_spi_t *obj, const uint8_t *tx, size_t tx_length)
{
	cyhal_spi_transfer(obj, tx, tx_length, NULL, 0x00,0x00);
}

void spi_read(cyhal_spi_t *obj, const uint8_t *rx, size_t rx_length)
{
	cyhal_spi_transfer(obj, NULL, 0x00, rx, rx_length,0x00);
}

void LCD_RESET(void)
{
	LCD_RES_SET; // for debug, per logic capture, the pin's default is low, so trigger a high to monitor the 100ms LCR's time
    LCD_RES_CLR;
    vTaskDelay(100); //delay 100ms
    LCD_RES_SET;
    vTaskDelay(100); // delay 100ms
}

static void LCD_WR_REG(uint8_t reg)
{
    LCD_DC_CLR;
    spi_send(&lcd_spi, &reg, 1);
    LCD_DC_SET;
}

static void LCD_WR_DATA(uint8_t data)
{
    LCD_DC_SET;
    spi_send(&lcd_spi, &data, 1);
}

static void LCD_ReadData(uint8_t *data, uint16_t length)
{
    LCD_DC_SET;
    spi_read(&lcd_spi, &data, length);
}

static void LCD_WriteReg(uint8_t reg, uint16_t regdata)
{
    LCD_WR_REG(reg);
    LCD_WR_DATA(regdata);
}

static void LCD_WriteRAM_Prepare(void)
{
    LCD_WR_REG(lcddev.wramcmd);
}

static void LCD_WriteData_16Bit(uint16_t Data)
{
    uint8_t buf[2];
    LCD_DC_SET;
    buf[0] = Data >> 8;
    buf[1] = Data & 0xff;
    spi_send(&lcd_spi, buf, 2);
}

void LCD_direction(uint8_t direction)
{
    lcddev.setxcmd = 0x2A;
    lcddev.setycmd = 0x2B;
    lcddev.wramcmd = 0x2C;
    switch (direction)
    {
    case 0:
        lcddev.width = LCD_W;
        lcddev.height = LCD_H;
        LCD_WriteReg(0x36, (1 << 3) | (0 << 6) | (0 << 7)); /* BGR==1,MY==0,MX==0,MV==0 */
        break;
    case 1:
        lcddev.width = LCD_H;
        lcddev.height = LCD_W;
        LCD_WriteReg(0x36, (1 << 3) | (0 << 7) | (1 << 6) | (1 << 5)); /* BGR==1,MY==1,MX==0,MV==1 */
        break;
    case 2:
        lcddev.width = LCD_W;
        lcddev.height = LCD_H;
        LCD_WriteReg(0x36, (1 << 3) | (1 << 6) | (1 << 7)); /* BGR==1,MY==0,MX==0,MV==0 */
        break;
    case 3:
        lcddev.width = LCD_H;
        lcddev.height = LCD_W;
        LCD_WriteReg(0x36, (1 << 3) | (1 << 7) | (1 << 5)); /* BGR==1,MY==1,MX==0,MV==1 */
        break;
    default:
        break;
    }
}

void LCD_SetWindows(uint16_t xStar, uint16_t yStar, uint16_t xEnd, uint16_t yEnd)
{
    LCD_WR_REG(lcddev.setxcmd);
    LCD_WR_DATA(xStar >> 8);
    LCD_WR_DATA(0x00FF & xStar);
    LCD_WR_DATA(xEnd >> 8);
    LCD_WR_DATA(0x00FF & xEnd);

    LCD_WR_REG(lcddev.setycmd);
    LCD_WR_DATA(yStar >> 8);
    LCD_WR_DATA(0x00FF & yStar);
    LCD_WR_DATA(yEnd >> 8);
    LCD_WR_DATA(0x00FF & yEnd);

    LCD_WriteRAM_Prepare();
}

void LCD_SetCursor(uint16_t Xpos, uint16_t Ypos)
{
    LCD_SetWindows(Xpos, Ypos, Xpos, Ypos);
}

void LCD_Clear(uint16_t Color)
{
    unsigned int i, m;
    uint8_t buf[80];

    for (i = 0; i < 40; i++)
    {
        buf[2 * i] = Color >> 8;
        buf[2 * i + 1] = Color & 0xff;
    }

    LCD_SetWindows(0, 0, lcddev.width - 1, lcddev.height - 1);

    LCD_DC_SET;
    for (i = 0; i < lcddev.height; i++)
    {
        for (m = 0; m < lcddev.width;)
        {
            m += 40;
            spi_send(&lcd_spi, buf, 80);
        }
    }
}

void LCD_Fill(uint16_t xsta, uint16_t ysta, uint16_t xend, uint16_t yend, uint16_t color)
{
    uint16_t i, j;
    LCD_SetWindows(xsta, ysta, xend - 1, yend - 1);
    for (i = ysta; i < yend; i++)
    {
        for (j = xsta; j < xend; j++)
        {
            LCD_WriteData_16Bit(color);
        }
    }
}

void lcd_fill_array_spi(uint16_t Xstart, uint16_t Ystart, uint16_t Xend, uint16_t Yend, void *Image)
{
    uint32_t size = 0;

    size = (Xend - Xstart + 1) * (Yend - Ystart + 1) * 2;/*16bit*/
    LCD_SetWindows(Xstart, Ystart, Xend, Yend);
    LCD_DC_SET;

    spi_send(&lcd_spi, Image, size);
}

static void _ili9341_init(void)
{
    LCD_WR_REG(0xCF);
    LCD_WR_DATA(0x00);
    LCD_WR_DATA(0X83);
    LCD_WR_DATA(0X30);

    LCD_WR_REG(0xED);
    LCD_WR_DATA(0x64);
    LCD_WR_DATA(0x03);
    LCD_WR_DATA(0X12);
    LCD_WR_DATA(0X81);

    LCD_WR_REG(0xE8);
    LCD_WR_DATA(0x85);
    LCD_WR_DATA(0x00);
    LCD_WR_DATA(0x79);

    LCD_WR_REG(0xCB);
    LCD_WR_DATA(0x39);
    LCD_WR_DATA(0x2C);
    LCD_WR_DATA(0x00);
    LCD_WR_DATA(0x34);
    LCD_WR_DATA(0x02);

    LCD_WR_REG(0xF7);
    LCD_WR_DATA(0x20);

    LCD_WR_REG(0xEA);
    LCD_WR_DATA(0x00);
    LCD_WR_DATA(0x00);

    LCD_WR_REG(0xC0);   /* Power control */
    LCD_WR_DATA(0x26);  /* VRH[5:0] */

    LCD_WR_REG(0xC1);   /* Power control */
    LCD_WR_DATA(0x11);  /* SAP[2:0];BT[3:0] */

    LCD_WR_REG(0xC5);   /* VCM control */
    LCD_WR_DATA(0x35);
    LCD_WR_DATA(0x3E);

    LCD_WR_REG(0xC7);   /* VCM control2 */
    LCD_WR_DATA(0XBE);

    LCD_WR_REG(0x36);   /* Memory Access Control */
    LCD_WR_DATA(0x28);

    LCD_WR_REG(0x3A);
    LCD_WR_DATA(0x55);

    LCD_WR_REG(0xB1);
    LCD_WR_DATA(0x00);
    LCD_WR_DATA(0x1B);

    LCD_WR_REG(0xB6);   /* Display Function Control */
    LCD_WR_DATA(0x0A);
    LCD_WR_DATA(0xA2);

    LCD_WR_REG(0xF2);   /* 3Gamma Function Disable */
    LCD_WR_DATA(0x08);

    LCD_WR_REG(0x26);   /* Gamma curve selected */
    LCD_WR_DATA(0x01);

    LCD_WR_REG(0xE0);   /* set Gamma */
    LCD_WR_DATA(0X1F);
    LCD_WR_DATA(0X1A);
    LCD_WR_DATA(0X18);
    LCD_WR_DATA(0X0A);
    LCD_WR_DATA(0X0F);
    LCD_WR_DATA(0X06);
    LCD_WR_DATA(0X45);
    LCD_WR_DATA(0X87);
    LCD_WR_DATA(0X32);
    LCD_WR_DATA(0X0A);
    LCD_WR_DATA(0X07);
    LCD_WR_DATA(0X02);
    LCD_WR_DATA(0X07);
    LCD_WR_DATA(0X05);
    LCD_WR_DATA(0X00);

    LCD_WR_REG(0xE1);   /* set Gamma */
    LCD_WR_DATA(0X00);
    LCD_WR_DATA(0X25);
    LCD_WR_DATA(0X27);
    LCD_WR_DATA(0X05);
    LCD_WR_DATA(0X10);
    LCD_WR_DATA(0X09);
    LCD_WR_DATA(0X3A);
    LCD_WR_DATA(0X78);
    LCD_WR_DATA(0X4D);
    LCD_WR_DATA(0X05);
    LCD_WR_DATA(0X18);
    LCD_WR_DATA(0X0D);
    LCD_WR_DATA(0X38);
    LCD_WR_DATA(0X3A);
    LCD_WR_DATA(0X2F);

    LCD_WR_REG(0x29);
}

static void Lcd_pin_init(void)
{
    cyhal_gpio_configure(PKG_ILI_9341_DC_PIN,CYHAL_GPIO_DIR_OUTPUT,CYHAL_GPIO_DRIVE_STRONG);
    cyhal_gpio_configure(PKG_ILI_9341_RES_PIN,CYHAL_GPIO_DIR_OUTPUT,CYHAL_GPIO_DRIVE_STRONG);
    //cyhal_gpio_configure(PKG_ILI_9341_BLK_PIN,CYHAL_GPIO_DIR_OUTPUT,CYHAL_GPIO_DRIVE_STRONG);  // no need to set due to connect BLK to V3.3 in HW
}

void ili9341_lcd_init(void)
{
	Lcd_pin_init();
	LCD_RESET();
    LCD_WR_REG(0x11);   /* Sleep out */
    LCD_DC_CLR;
    vTaskDelay(120);         /* Delay 120ms */
    _ili9341_init();    /* IlI9341 init */
   // LCD_BLK_CLR;        /* Open Backlight */ // no need to call due to BLK is connect to V3.3 in HW

    LCD_direction(USE_DIRECTION);
}

cy_rslt_t spi_lcd_init(uint32_t freq)
{
	cy_rslt_t res = 0;
	uint32_t spi_max_freq;

    if (&lcd_spi != NULL)
    {
        spi_max_freq = freq * 1000 * 1000;

        res = cyhal_spi_init(&lcd_spi,LCD_SPI_MOSI,LCD_SPI_MISO,LCD_SPI_CLK,
                                LCD_SPI_CS,NULL,BITS_PER_FRAME,
                                CYHAL_SPI_MODE_00_MSB,false);

        /* Set the SPI baud rate */
        res = cyhal_spi_set_frequency(&lcd_spi, spi_max_freq);
    }
    else
    {
        res = -1;
    }
    return res;
}

static uint16_t color_array[] =
{
    WHITE, BLACK, BLUE, BRED,
    GRED, GBLUE, RED, YELLOW
};

void lcd_spi_test(void)
{
    uint8_t index = 0;
    for (index = 0; index < sizeof(color_array) / sizeof(color_array[0]); index++)
    {
        LCD_Clear(color_array[index]);
        //LOG_I("lcd clear color: %#x", color_array[index]);
        vTaskDelay(200);
    }
}
