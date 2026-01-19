/*
 * xpt2046.h
 *
 *  Created on: Jan 12, 2026
 *      Author: khoi2
 */

#ifndef INC_XPT2046_H_
#define INC_XPT2046_H_

#include "main.h"
#include "stm32f7xx_hal_spi.h"
#include "stm32f7xx_hal_gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

// ====== User tuning ======
#ifndef XPT2046_AVG_SAMPLES
#define XPT2046_AVG_SAMPLES 5       // 3..10, càng lớn càng mượt càng chậm
#endif

#ifndef XPT2046_SPI_TIMEOUT_MS
#define XPT2046_SPI_TIMEOUT_MS 5    // tránh HAL_MAX_DELAY trong RTOS
#endif

#ifndef XPT2046_Z_THRESHOLD
#define XPT2046_Z_THRESHOLD  50   // bạn có thể thử 20..150 tuỳ màn
#endif

// ====== XPT2046 commands (12-bit) ======
// Start=1, Mode=12bit(0), SER/DFR=0(DFR), PD bits tuỳ (ở đây luôn ON khi đọc)
#define XPT2046_CMD_X   0xD0  // 1101 0000
#define XPT2046_CMD_Y   0x90  // 1001 0000
#define XPT2046_CMD_Z1  0xB0
#define XPT2046_CMD_Z2  0xC0

typedef enum {
    XPT2046_ORIENTATION_PORTRAIT = 0,
    XPT2046_ORIENTATION_LANDSCAPE = 1,
    XPT2046_ORIENTATION_PORTRAIT_MIRROR = 2,
    XPT2046_ORIENTATION_LANDSCAPE_MIRROR = 3
} XPT2046_Orientation_t;

typedef struct {
    uint16_t x;
    uint16_t y;
    uint16_t z1;
    uint16_t z2;
} XPT2046_Raw_t;

typedef struct {
    uint16_t x;
    uint16_t y;
    uint16_t z;     // dùng z1 làm z đơn giản
} XPT2046_Point_t;

// ====== API ======
void xpt2046_spi(SPI_HandleTypeDef* spi);

void xpt2046_cs(GPIO_TypeDef* port, uint16_t pin);
void xpt2046_penirq(GPIO_TypeDef* port, uint16_t pin);

void xpt2046_set_size(uint16_t w, uint16_t h);
void xpt2046_orientation(XPT2046_Orientation_t o);

// optional calibration/tuning
void xpt2046_set_raw_range(uint16_t raw_min, uint16_t raw_max); // default 0..4095
void xpt2046_set_offset(int16_t x_off, int16_t y_off);          // default 0,0

void xpt2046_init(void);

uint8_t xpt2046_pressed(void);          // 1=pressed, 0=released (dựa PENIRQ)
void xpt2046_update(void);              // cập nhật raw + mapped point
void xpt2046_read_position(uint16_t* x, uint16_t* y);  // pressed->xy, else 0,0

// expose latest data if needed
XPT2046_Raw_t   xpt2046_get_raw(void);
XPT2046_Point_t xpt2046_get_point(void);

#ifdef __cplusplus
}
#endif

#endif /* INC_XPT2046_H_ */
