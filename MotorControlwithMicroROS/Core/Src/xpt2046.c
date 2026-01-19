#include "xpt2046.h"

// ====== internal state ======
static SPI_HandleTypeDef* s_spi = NULL;

static GPIO_TypeDef* s_cs_port = NULL;
static uint16_t      s_cs_pin  = 0;

static GPIO_TypeDef* s_irq_port = NULL;
static uint16_t      s_irq_pin  = 0;

static XPT2046_Orientation_t s_ori = XPT2046_ORIENTATION_LANDSCAPE;
static uint16_t s_w = 240, s_h = 320;

// raw range -> map to pixels (mặc định 12-bit)
static uint16_t s_raw_min = 0;
static uint16_t s_raw_max = 4095;

// pixel offsets
static int16_t s_x_off = 0;
static int16_t s_y_off = 0;

static XPT2046_Raw_t   s_raw = {0};
static XPT2046_Point_t s_pt  = {0};

// ====== helpers ======
static uint8_t pressed_by_z(uint16_t z1, uint16_t z2)
{
    // Cách đơn giản: z1 đủ lớn coi như nhấn
    // (z2 có thể dùng để tính pressure chính xác hơn, nhưng thường không cần)
    return (z1 > XPT2046_Z_THRESHOLD);
}

static inline void cs_low(void)
{
    if (s_cs_port) HAL_GPIO_WritePin(s_cs_port, s_cs_pin, GPIO_PIN_RESET);
}

static inline void cs_high(void)
{
    if (s_cs_port) HAL_GPIO_WritePin(s_cs_port, s_cs_pin, GPIO_PIN_SET);
}

// đọc 12-bit: gửi 1 byte cmd, nhận 2 byte data, lấy 12-bit (MSB first)
static uint16_t read12(uint8_t cmd)
{
    uint8_t tx[3] = { cmd, 0x00, 0x00 };
    uint8_t rx[3] = { 0, 0, 0 };

    HAL_SPI_TransmitReceive(s_spi, tx, rx, 3, XPT2046_SPI_TIMEOUT_MS);

    // rx[1] và rx[2] mới chứa ADC
    uint16_t v = ((uint16_t)rx[1] << 8) | rx[2];
    v >>= 3;
    return (v & 0x0FFF);
}

// clamp int -> [0..max]
static inline uint16_t clamp_u16(int32_t v, uint16_t maxv)
{
    if (v < 0) return 0;
    if (v > (int32_t)maxv) return maxv;
    return (uint16_t)v;
}

// map raw [raw_min..raw_max] -> [0..pix_max]
static uint16_t map_raw(uint16_t raw, uint16_t pix_max)
{
    if (s_raw_max <= s_raw_min) return 0;
    if (raw < s_raw_min) raw = s_raw_min;
    if (raw > s_raw_max) raw = s_raw_max;

    uint32_t num = (uint32_t)(raw - s_raw_min) * (uint32_t)pix_max;
    uint32_t den = (uint32_t)(s_raw_max - s_raw_min);
    return (uint16_t)(num / den);
}

// ====== API impl ======
void xpt2046_spi(SPI_HandleTypeDef* spi)
{
    s_spi = spi;
}

void xpt2046_cs(GPIO_TypeDef* port, uint16_t pin)
{
    s_cs_port = port;
    s_cs_pin  = pin;
}

void xpt2046_penirq(GPIO_TypeDef* port, uint16_t pin)
{
    s_irq_port = port;
    s_irq_pin  = pin;
}

void xpt2046_set_size(uint16_t w, uint16_t h)
{
    s_w = w;
    s_h = h;
}

void xpt2046_orientation(XPT2046_Orientation_t o)
{
    s_ori = o;
}

void xpt2046_set_raw_range(uint16_t raw_min, uint16_t raw_max)
{
    s_raw_min = raw_min;
    s_raw_max = raw_max;
}

void xpt2046_set_offset(int16_t x_off, int16_t y_off)
{
    s_x_off = x_off;
    s_y_off = y_off;
}

void xpt2046_init(void)
{
    // sanity checks
    // (bạn có thể đặt breakpoint nếu NULL)
    cs_high();
}

uint8_t xpt2046_pressed(void)
{
    // PENIRQ active-low: LOW = pressed
    if (!s_irq_port) return 0;
    return (HAL_GPIO_ReadPin(s_irq_port, s_irq_pin) == GPIO_PIN_RESET);
}

void xpt2046_update(void)
{
    if (!s_spi || !s_cs_port) return;

    uint32_t ax=0, ay=0, az1=0, az2=0;

    cs_low();

    for (int i = 0; i < XPT2046_AVG_SAMPLES; i++)
    {
        uint16_t rx  = read12(XPT2046_CMD_X);
        uint16_t ry  = read12(XPT2046_CMD_Y);
        uint16_t rz1 = read12(XPT2046_CMD_Z1);
        uint16_t rz2 = read12(XPT2046_CMD_Z2);

        ax  += rx;
        ay  += ry;
        az1 += rz1;
        az2 += rz2;
    }

    cs_high();

    s_raw.x  = (uint16_t)(ax  / XPT2046_AVG_SAMPLES);
    s_raw.y  = (uint16_t)(ay  / XPT2046_AVG_SAMPLES);
    s_raw.z1 = (uint16_t)(az1 / XPT2046_AVG_SAMPLES);
    s_raw.z2 = (uint16_t)(az2 / XPT2046_AVG_SAMPLES);

    // Nếu không nhấn -> clear và return
    if (!pressed_by_z(s_raw.z1, s_raw.z2)) {
        s_pt.x = 0; s_pt.y = 0; s_pt.z = 0;
        return;
    }

    // ---- map raw->pixel như phần bạn đang dùng ----
    uint16_t px = map_raw(s_raw.x, (s_w > 0) ? (s_w - 1) : 0);
    uint16_t py = map_raw(s_raw.y, (s_h > 0) ? (s_h - 1) : 0);

    int32_t x = 0, y = 0;
    switch (s_ori)
    {
        case XPT2046_ORIENTATION_PORTRAIT:
            x = (int32_t)px; y = (int32_t)py; break;
        case XPT2046_ORIENTATION_LANDSCAPE:
            x = (int32_t)py;
            y = (int32_t)((s_h>0)?((s_h-1)-px):0);
            break;
        case XPT2046_ORIENTATION_PORTRAIT_MIRROR:
            x = (int32_t)((s_w>0)?((s_w-1)-px):0);
            y = (int32_t)py;
            break;
        case XPT2046_ORIENTATION_LANDSCAPE_MIRROR:
            x = (int32_t)((s_w>0)?((s_w-1)-py):0);
            y = (int32_t)px;
            break;
        default:
            x = (int32_t)px; y = (int32_t)py; break;
    }

    x -= s_x_off;
    y -= s_y_off;

    s_pt.x = clamp_u16(x, (s_w>0) ? (s_w - 1) : 0);
    s_pt.y = clamp_u16(y, (s_h>0) ? (s_h - 1) : 0);
    s_pt.z = s_raw.z1;
}


void xpt2046_read_position(uint16_t* x, uint16_t* y)
{
    if (!x || !y) return;

    xpt2046_update();  // sẽ tự clear 0,0 nếu không nhấn
    *x = s_pt.x;
    *y = s_pt.y;
}
//    if (HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_3) == GPIO_PIN_RESET)
//    {
//    	int aa=0;
//    }
//    if (HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_3) == GPIO_PIN_RESET)  // active-low
//    {
//    	xpt2046_update();
//
//    	// update() đã clear về 0 nếu không pressed
//    	*x = s_pt.x;
//    	*y = s_pt.y;
//    }
//    else
//    {
//    	*x = 0; *y = 0;
//    }

//    if (HAL_GPIO_ReadPin(s_irq_port, s_irq_pin) != GPIO_PIN_RESET) {
//            *x = 0; *y = 0;
//            return;
//        }



XPT2046_Raw_t xpt2046_get_raw(void)
{
    return s_raw;
}

XPT2046_Point_t xpt2046_get_point(void)
{
    return s_pt;
}
