/******************************************************************************
*
* File Name: tft_task.c
*
* Description: This file contains task and functions related to the tft-task
* that demonstrates controlling a tft display using the LVGL Graphics Library.
*
* The project displays a start up screen with text "LVGL Demo Music player" and
* LVGL logo.
*
* The project then displays the Demo music player application.
*
*******************************************************************************
* Copyright 2023, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*******************************************************************************/

/*******************************************************************************
 * Header file includes
 ******************************************************************************/
#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"
#include "lcd_ili9341.h"
#include "tft_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "lvgl.h"
#include "lvgl_support.h"
#include "demos/widgets/lv_demo_widgets.h"
#include "examples/lv_examples.h"
#include "examples/anim/lv_example_anim.h"

/*******************************************************************************
* Macros
*******************************************************************************/
#define DELAY_PARAM       (5u)
#define DELAY_300_MS      (300)   /* milliseconds */
#define DELAY_10_MS       (10)    /* milliseconds */


/*******************************************************************************
* Function Name: void tft_task(void *arg)
********************************************************************************
*
* Summary: tft_task is a handler for LVGL. It is a task function that handles 
*          LVGL-related tasks and operations. The LVGL music player demo gets
*          called inside this function after all initialization are done.
*
* Parameters:
*  arg: task argument
*
* Return:
*  None
*
*******************************************************************************/
void tft_task(void *arg)
{
    cy_rslt_t result;

    /* Initialize the graphical unit */
    result = graphics_init();
    CY_ASSERT(result == CY_RSLT_SUCCESS);

    vTaskDelay(1000);
        
    /* Display LVGL demo music player*/
    lv_demo_music();
    //lv_demo_widgets();
    printf("lv_demo_music called\r\n");

    /* To avoid compiler warning */
    (void)result;
    
    /* Repeatedly running part of the task */
    for (;;)
    {
        lv_task_handler();
        vTaskDelay(DELAY_PARAM);
        lv_tick_inc(DELAY_PARAM);
    }

}

/*******************************************************************************
* Function Name: cy_rslt_t graphics_init(void)
********************************************************************************
*
* Summary: This function performs all graphics related initializations. 
*
* Parameters:
*  None
*
* Return:
*  None
*
*
*******************************************************************************/
cy_rslt_t  graphics_init()
{
    cy_rslt_t result;

    /* Initialize the spi to driver ili9341 */
    result = spi_lcd_init(25);  //50M Hz
    printf("spi lcd init result:%d\r\n", result);
    CY_ASSERT(result == CY_RSLT_SUCCESS);
    
    /* Perform initialization specific to the ili9341 display controller. */
    ili9341_lcd_init();

    /* Initialize LVGL and set up the essential components required for LVGL. */
    lv_init();

    printf("lv init\r\n");

    /*Initialize display driver. */
    lv_port_disp_init();

    printf("lv_port_disp_init\r\n");

    return result;
}

/* END OF FILE */
