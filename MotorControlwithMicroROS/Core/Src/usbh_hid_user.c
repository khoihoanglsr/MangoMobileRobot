#include "usbh_hid.h"
#include "usb_host.h"     // để dùng hUsbHostFS nếu cần
#include "ps2_pad.h"
#include <string.h>

void USBH_HID_EventCallback(USBH_HandleTypeDef *phost)
{
  uint8_t rep[64];
  memset(rep, 0, sizeof(rep));

  uint8_t len = 8;

  if (USBH_HID_GetReport(phost, 1 /*Input*/, 0 /*ID*/, rep, len) == USBH_OK)
  {
    PS2_HID_PushReport(rep, len);
  }
}
