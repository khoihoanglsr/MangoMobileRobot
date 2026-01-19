#include "ps2_pad.h"
#include "usb_host.h"
#include "usbh_core.h"
#include "usbh_hid.h"
#include <string.h>

volatile PS2Signals_t g_ps2 = {0};

/* ====== Cấu hình FIFO cho HID reports ======
   Size FIFO: chọn 256..1024 tùy report len và tần số.
*/
#define HID_FIFO_SIZE 512
static uint8_t hid_fifo_mem[HID_FIFO_SIZE];
static FIFO_TypeDef hid_fifo;

/* Nhiều thiết bị gamepad có report dài 8~64 bytes.
   Ta đọc tối đa 64 bytes mỗi report (đủ cho đa số).
*/
#define HID_REPORT_MAX 64

static void ps2_clear(void)
{
  PS2Signals_t z;
  memset(&z, 0, sizeof(z));
  g_ps2 = z;
}

static void hat_to_dpad(uint8_t hat)
{
  // 0: up, 1: up-right, 2: right, 3: down-right,
  // 4: down, 5: down-left, 6: left, 7: up-left, 0xF: none
  g_ps2.dpad_up    = (hat == 0 || hat == 1 || hat == 7);
  g_ps2.dpad_right = (hat == 1 || hat == 2 || hat == 3);
  g_ps2.dpad_down  = (hat == 3 || hat == 4 || hat == 5);
  g_ps2.dpad_left  = (hat == 5 || hat == 6 || hat == 7);
}

// Parse theo layout “generic gamepad” thường gặp:
// report[0..1] buttons bitmask, report[2..5] sticks (0..255),
// report[6] hat. Nếu receiver bạn khác, ta sẽ chỉnh sau dựa trên log.
static void parse_ps2_report(const uint8_t *r, uint16_t len)
{
  if (len < 6) return;

  g_ps2.connected = 1;

  uint16_t buttons = (uint16_t)r[0] | ((uint16_t)r[1] << 8);

  // mapping phổ biến (có thể phải đổi bit sau khi log)
  g_ps2.square   = (buttons & (1u << 0)) ? 1 : 0;
  g_ps2.cross    = (buttons & (1u << 1)) ? 1 : 0;
  g_ps2.circle   = (buttons & (1u << 2)) ? 1 : 0;
  g_ps2.triangle = (buttons & (1u << 3)) ? 1 : 0;

  g_ps2.l1 = (buttons & (1u << 4)) ? 1 : 0;
  g_ps2.r1 = (buttons & (1u << 5)) ? 1 : 0;
  g_ps2.l2 = (buttons & (1u << 6)) ? 1 : 0;
  g_ps2.r2 = (buttons & (1u << 7)) ? 1 : 0;

  g_ps2.select = (buttons & (1u << 8)) ? 1 : 0;
  g_ps2.start  = (buttons & (1u << 9)) ? 1 : 0;
  g_ps2.l3     = (buttons & (1u << 10)) ? 1 : 0;
  g_ps2.r3     = (buttons & (1u << 11)) ? 1 : 0;

  // sticks: 0..255 -> -128..127
  g_ps2.lx = (int16_t)r[2] - 128;
  g_ps2.ly = (int16_t)r[3] - 128;
  g_ps2.rx = (int16_t)r[4] - 128;
  g_ps2.ry = (int16_t)r[5] - 128;

  if (len > 6)
    hat_to_dpad(r[6]);
}

void PS2_HID_Init(void)
{
  USBH_HID_FifoInit(&hid_fifo, hid_fifo_mem, HID_FIFO_SIZE);
  ps2_clear();
}

/* Gọi hàm này trong 1 task (ví dụ 100~1000 Hz) để lấy report từ FIFO và parse */
void PS2_HID_ProcessReports(void)
{
  if (Appli_state != APPLICATION_READY)
  {
    ps2_clear();
    return;
  }

  // Đọc hết các report hiện có trong FIFO
  // Lưu ý: FIFO trong USBH_HID thường chứa raw bytes liên tiếp.
  // Một số stack sẽ đẩy đúng từng report-length. Nếu không chắc,
  // ta sẽ log len bằng cách cố định reportLen theo device.
  uint8_t buf[HID_REPORT_MAX];

  // Cách đọc an toàn: đọc “từng gói” theo report length bạn biết.
  // Nếu chưa biết report length -> ta đọc thử 8 bytes trước để log.
  const uint16_t guess_len = 8;  // PS2 receiver thường 8 bytes; sẽ chỉnh theo log.

  while (USBH_HID_FifoRead(&hid_fifo, buf, guess_len) == guess_len)
  {
    parse_ps2_report(buf, guess_len);
  }
}

void PS2_HID_PushReport(const uint8_t *data, uint16_t len)
{
  if (data == NULL || len == 0) return;
  USBH_HID_FifoWrite(&hid_fifo, (void*)data, len);
}
