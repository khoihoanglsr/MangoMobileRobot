/*
 * touch_event.c
 *
 *  Created on: Jan 13, 2026
 *      Author: khoi2
 */

#include "touch_event.h"
#include "xpt2046.h"
#include "main.h"
#include "spi.h"

volatile TouchEvent_t g_touchState = {0,0,0};

// SPI1 bạn đã có: extern SPI_HandleTypeDef hspi1;

void Touch_Init_Lib(void)
{
	xpt2046_spi(&hspi1);
	xpt2046_cs(GPIOG, GPIO_PIN_2);
	xpt2046_penirq(GPIOG, GPIO_PIN_3);
	xpt2046_set_size(240, 320);
	xpt2046_orientation(XPT2046_ORIENTATION_LANDSCAPE);
	xpt2046_set_raw_range(0, 4095);   // mặc định
	xpt2046_set_offset(0, 0);
	xpt2046_init();
}
