#include <stdlib.h>
#include "touch_rtos.h"
#include "xpt2046.h"
#include "spi.h"     // để có hspi1
#include "main.h"

QueueHandle_t g_touchQueue = NULL;
TaskHandle_t  g_touchTaskHandle = NULL;

// Lọc đơn giản: median-of-5 (rất hiệu quả chống nhảy)
//static uint16_t median5(uint16_t a, uint16_t b, uint16_t c, uint16_t d, uint16_t e)
//{
//    uint16_t v[5] = {a,b,c,d,e};
//    // sort 5 phần tử (bubble nhỏ)
//    for (int i=0;i<5;i++){
//        for (int j=i+1;j<5;j++){
//            if (v[j] < v[i]) { uint16_t t=v[i]; v[i]=v[j]; v[j]=t; }
//        }
//    }
//    return v[2];
//}

void TouchTask_Start(void *argument)
{
    (void)argument;

    // Lưu handle để IRQ notify
    g_touchTaskHandle = xTaskGetCurrentTaskHandle();
    // ===== INIT TOUCH LIB (SPI1 + PG2/PG3) =====
    xpt2046_spi(&hspi1);
    xpt2046_cs(GPIOG, GPIO_PIN_2);
    xpt2046_penirq(GPIOG, GPIO_PIN_3);
    xpt2046_set_size(240, 320);
    xpt2046_orientation(XPT2046_ORIENTATION_PORTRAIT_MIRROR);
    xpt2046_set_raw_range(0, 4095);   // mặc định
    xpt2046_set_offset(0, 0);
    xpt2046_init();


    for (;;)
    {
        uint16_t x,y;
        xpt2046_read_position(&x,&y);

        if (x || y) {
                    g_touchState.x = x;
                    g_touchState.y = y;
                    g_touchState.pressed = 1;
                } else {
                    g_touchState.x = 0;
                    g_touchState.y = 0;
                    g_touchState.pressed = 0;
                }

        vTaskDelay(pdMS_TO_TICKS(10)); // 10ms là đủ mượt
    }
}
