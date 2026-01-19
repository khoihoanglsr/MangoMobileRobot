#include "stm32f7xx_hal.h"
#include "main.h"

HCD_HandleTypeDef hhcd_USB_OTG_FS;

void MX_USB_OTG_FS_HCD_Init(void)
{
  hhcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hhcd_USB_OTG_FS.Init.Host_channels = 8;
  hhcd_USB_OTG_FS.Init.speed = HCD_SPEED_FULL;
  hhcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hhcd_USB_OTG_FS.Init.phy_itface = HCD_PHY_EMBEDDED;
  hhcd_USB_OTG_FS.Init.Sof_enable = ENABLE;
  hhcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hhcd_USB_OTG_FS.Init.use_external_vbus = ENABLE;

  if (HAL_HCD_Init(&hhcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
}
