#include "ps2xlib.h"

// ====== USER PIN MAP (as you specified) ======
// CS/ATT = PG0
#define PS2_CS_GPIO_Port   GPIOG
#define PS2_CS_Pin         GPIO_PIN_0

static inline void PS2_Select(void)   { HAL_GPIO_WritePin(PS2_CS_GPIO_Port, PS2_CS_Pin, GPIO_PIN_RESET); }
static inline void PS2_Deselect(void) { HAL_GPIO_WritePin(PS2_CS_GPIO_Port, PS2_CS_Pin, GPIO_PIN_SET);   }

// ====== Microsecond delay (DWT with safe fallback, never deadlocks) ======
static void delay_us(uint32_t us)
{
  // Try to enable DWT CYCCNT
  if ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) == 0) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  }

  uint32_t cycles = (HAL_RCC_GetHCLKFreq() / 1000000U) * us;
  uint32_t start  = DWT->CYCCNT;

  // If CYCCNT is NOT running → bounded NOP fallback
  if (DWT->CYCCNT == start) {
    for (volatile uint32_t i = 0; i < (cycles / 6U + 1U); i++) {
      __NOP();
    }
    return;
  }

  while ((uint32_t)(DWT->CYCCNT - start) < cycles) {
    __NOP();
  }
}

// ====== Low-level transfer: keep CS low for entire frame ======
static bool ps2_transfer(PS2X_Handle *ps2, const uint8_t *tx, uint8_t *rx, uint8_t len)
{
  PS2_Select();
  delay_us(10);

  for (uint8_t i = 0; i < len; i++) {
    uint8_t t = tx ? tx[i] : 0x00;
    uint8_t r = 0xFF;

    HAL_StatusTypeDef st = HAL_SPI_TransmitReceive(ps2->hspi, &t, &r, 1, 5);
    ps2->last_spi_status = st;

    if (st != HAL_OK) {
      ps2->last_err = PS2_FAIL_SPI;
      PS2_Deselect();
      delay_us(50);
      return false;
    }

    if (rx) rx[i] = r;

    // PS2 likes small inter-byte gap
    delay_us(10);
  }

  delay_us(10);
  PS2_Deselect();
  delay_us(50);
  return true;
}

// Signature check: rx[2] is commonly 0x5A when aligned
static bool ps2_signature_ok(const uint8_t *rx, uint8_t len)
{
  if (len < 3) return false;
  return (rx[2] == 0x5A);
}

void PS2X_Init(PS2X_Handle *ps2, SPI_HandleTypeDef *hspi)
{
  ps2->hspi = hspi;

  ps2->rx_len = 9;
  ps2->buttons_raw = 0xFFFF;
  ps2->buttons = 0;
  ps2->last_buttons = 0;
  ps2->id = 0x00;
  ps2->analog_enabled = false;

  for (int i = 0; i < 4; i++) ps2->analog[i] = 0x80;

  // Debug fields init
  ps2->last_err = PS2_OK;
  ps2->last_spi_status = HAL_OK;
  ps2->last_rx0 = 0x00;
  ps2->last_rx1 = 0x00;
  ps2->last_rx2 = 0x00;

  // Ensure CS idle high
  PS2_Deselect();
}

// Common PS2 config frames (similar to Arduino libs)
static bool ps2_enter_config(PS2X_Handle *ps2)
{
  uint8_t tx[] = {0x01, 0x43, 0x00, 0x01, 0x00};
  uint8_t rx[sizeof(tx)] = {0};

  if (!ps2_transfer(ps2, tx, rx, (uint8_t)sizeof(tx))) return false;
  return true;
}

static bool ps2_set_analog(PS2X_Handle *ps2, bool lock_analog)
{
  uint8_t tx[] = {0x01, 0x44, 0x00, 0x01,
                  (uint8_t)(lock_analog ? 0x03 : 0x00),
                  0x00, 0x00, 0x00, 0x00};
  uint8_t rx[sizeof(tx)] = {0};

  if (!ps2_transfer(ps2, tx, rx, (uint8_t)sizeof(tx))) return false;
  return true;
}

static bool ps2_enable_rumble(PS2X_Handle *ps2)
{
  uint8_t tx[] = {0x01, 0x4D, 0x00, 0x00, 0x01};
  uint8_t rx[sizeof(tx)] = {0};

  if (!ps2_transfer(ps2, tx, rx, (uint8_t)sizeof(tx))) return false;
  return true;
}

static bool ps2_exit_config(PS2X_Handle *ps2)
{
  uint8_t tx[] = {0x01, 0x43, 0x00, 0x00, 0x5A, 0x5A, 0x5A, 0x5A, 0x5A};
  uint8_t rx[sizeof(tx)] = {0};

  if (!ps2_transfer(ps2, tx, rx, (uint8_t)sizeof(tx))) return false;
  return true;
}

static bool ps2_poll(PS2X_Handle *ps2, uint8_t motor_small, uint8_t motor_large, bool rumble)
{
  // Poll frame length: 9 bytes is enough for basic + analog sticks
  const uint8_t len = 9;

  for (uint8_t i = 0; i < len; i++) ps2->tx[i] = 0x00;

  ps2->tx[0] = 0x01;
  ps2->tx[1] = 0x42;
  ps2->tx[2] = 0x00;

  if (rumble) {
    ps2->tx[3] = (motor_small ? 0x01 : 0x00);
    ps2->tx[4] = motor_large;
  }

  if (!ps2_transfer(ps2, ps2->tx, ps2->rx, len)) {
    // last_err already set to SPI fail inside transfer
    return false;
  }

  ps2->rx_len = len;

  // Record first bytes for debugging
  ps2->last_rx0 = ps2->rx[0];
  ps2->last_rx1 = ps2->rx[1];
  ps2->last_rx2 = ps2->rx[2];

  // Signature check
  if (!ps2_signature_ok(ps2->rx, len)) {
    ps2->last_err = PS2_FAIL_SIGNATURE;
    return false;
  }

  ps2->last_err = PS2_OK;
  return true;
}

bool PS2X_ConfigGamepad(PS2X_Handle *ps2, bool enable_analog, bool enable_rumble)
{
  for (int attempt = 0; attempt < 5; attempt++) {

    // 1) Basic poll
    if (!ps2_poll(ps2, 0, 0, false)) {
      ps2->last_err = (ps2->last_err == PS2_FAIL_SPI) ? PS2_FAIL_SPI : PS2_FAIL_POLL1;
      HAL_Delay(20);
      continue;
    }

    // 2) Enter config
    if (!ps2_enter_config(ps2)) {
      ps2->last_err = (ps2->last_err == PS2_FAIL_SPI) ? PS2_FAIL_SPI : PS2_FAIL_ENTER_CFG;
      HAL_Delay(20);
      continue;
    }
    HAL_Delay(10);

    // 3) Set analog mode
    if (enable_analog) {
      if (!ps2_set_analog(ps2, true)) {
        ps2->last_err = (ps2->last_err == PS2_FAIL_SPI) ? PS2_FAIL_SPI : PS2_FAIL_SET_ANALOG;
        HAL_Delay(20);
        continue;
      }
      HAL_Delay(10);
    }

    // 4) Enable rumble
    if (enable_rumble) {
      if (!ps2_enable_rumble(ps2)) {
        ps2->last_err = (ps2->last_err == PS2_FAIL_SPI) ? PS2_FAIL_SPI : PS2_FAIL_RUMBLE;
        HAL_Delay(20);
        continue;
      }
      HAL_Delay(10);
    }

    // 5) Exit config
    if (!ps2_exit_config(ps2)) {
      ps2->last_err = (ps2->last_err == PS2_FAIL_SPI) ? PS2_FAIL_SPI : PS2_FAIL_EXIT_CFG;
      HAL_Delay(20);
      continue;
    }
    HAL_Delay(20);

    // 6) Verify poll again
    if (!ps2_poll(ps2, 0, 0, enable_rumble)) {
      ps2->last_err = (ps2->last_err == PS2_FAIL_SPI) ? PS2_FAIL_SPI : PS2_FAIL_POLL2;
      HAL_Delay(20);
      continue;
    }

    ps2->id = ps2->rx[1];
    ps2->analog_enabled = (ps2->id == 0x73); // common analog ID (not universal)

    ps2->last_err = PS2_OK;
    return true;
  }

  return false;
}

bool PS2X_ReadGamepad(PS2X_Handle *ps2, uint8_t motor_small, uint8_t motor_large)
{
  ps2->last_buttons = ps2->buttons;

  bool ok = ps2_poll(ps2, motor_small, motor_large, true);
  if (!ok) return false;

  ps2->id = ps2->rx[1];
  ps2->analog_enabled = (ps2->id == 0x73);

  // Buttons: rx[3]=low byte, rx[4]=high byte, active LOW
  ps2->buttons_raw = ((uint16_t)ps2->rx[4] << 8) | ps2->rx[3];
  ps2->buttons = (uint16_t)(~ps2->buttons_raw); // active HIGH pressed

  // Analog sticks (common layout)
  ps2->analog[PSS_RX] = ps2->rx[5];
  ps2->analog[PSS_RY] = ps2->rx[6];
  ps2->analog[PSS_LX] = ps2->rx[7];
  ps2->analog[PSS_LY] = ps2->rx[8];

  return true;
}
