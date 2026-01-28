#include "pid_params_flash.h"
#include "stm32f7xx_hal.h"
#include <string.h>

// ----------------- USER CONFIG -----------------
// Recommended: last sector of 1MB flash (commonly sector 7) base.
#define PID_FLASH_BASE_ADDR   (0x080C0000u)
#define PID_FLASH_END_ADDR    (0x08100000u)   // end-exclusive (0x080FFFFF last byte)
#define PID_FLASH_SECTOR      FLASH_SECTOR_7

// If your VDD is 2.7–3.6V, use VOLTAGE_RANGE_3 (typical for Nucleo).
#define PID_FLASH_VRANGE      FLASH_VOLTAGE_RANGE_3
// ------------------------------------------------

#define PID_MAGIC   (0x50494447u)  // 'PIDG'
#define PID_VER     (1u)

// Record stored in flash (append-only log)
typedef struct __attribute__((packed)) {
  uint32_t magic;
  uint16_t ver;
  uint16_t reserved;
  uint32_t seq;

  PIDGains_t gains;

  uint32_t crc32;   // CRC over everything except crc32 field
} PIDRecord_t;

static uint32_t crc32_sw(const void *data, uint32_t len_bytes)
{
  // Standard CRC-32 (Ethernet, ZIP) polynomial 0x04C11DB7, reflected form 0xEDB88320
  uint32_t crc = 0xFFFFFFFFu;
  const uint8_t *p = (const uint8_t*)data;

  for (uint32_t i = 0; i < len_bytes; i++) {
    crc ^= p[i];
    for (uint32_t b = 0; b < 8; b++) {
      uint32_t mask = (uint32_t)-(int32_t)(crc & 1u);
      crc = (crc >> 1) ^ (0xEDB88320u & mask);
    }
  }
  return ~crc;
}

static bool record_is_valid(const PIDRecord_t *r)
{
  if (r->magic != PID_MAGIC) return false;
  if (r->ver != PID_VER) return false;

  uint32_t calc = crc32_sw(r, (uint32_t)offsetof(PIDRecord_t, crc32));
  return (calc == r->crc32);
}

static const PIDRecord_t* find_latest_record(void)
{
  const uint8_t *addr = (const uint8_t*)PID_FLASH_BASE_ADDR;
  const PIDRecord_t *best = NULL;

  // Scan sequentially until we hit erased area (0xFF...)
  while ((uint32_t)addr + sizeof(PIDRecord_t) <= PID_FLASH_END_ADDR) {
    const PIDRecord_t *r = (const PIDRecord_t*)addr;

    // erased flash usually reads as all 0xFF
    if (r->magic == 0xFFFFFFFFu) break;

    if (record_is_valid(r)) {
      if (!best || r->seq > best->seq) best = r;
    }

    addr += sizeof(PIDRecord_t);
  }
  return best;
}

static uint32_t find_next_write_addr(void)
{
  const uint8_t *addr = (const uint8_t*)PID_FLASH_BASE_ADDR;
  while ((uint32_t)addr + sizeof(PIDRecord_t) <= PID_FLASH_END_ADDR) {
    const PIDRecord_t *r = (const PIDRecord_t*)addr;
    if (r->magic == 0xFFFFFFFFu) {
      return (uint32_t)addr;
    }
    addr += sizeof(PIDRecord_t);
  }
  return 0u; // full
}

static bool flash_erase_sector(void)
{
  FLASH_EraseInitTypeDef erase = {0};
  uint32_t sector_error = 0;

  erase.TypeErase    = FLASH_TYPEERASE_SECTORS;
  erase.VoltageRange = PID_FLASH_VRANGE;
  erase.Sector       = PID_FLASH_SECTOR;
  erase.NbSectors    = 1;

  HAL_StatusTypeDef st = HAL_FLASHEx_Erase(&erase, &sector_error);
  return (st == HAL_OK);
}

static bool flash_program_record(uint32_t dst_addr, const PIDRecord_t *rec)
{
  // STM32F7 supports programming by word/dword depending on settings.
  // We'll use 64-bit "doubleword" programming for alignment/robustness.
  // Ensure dst_addr is 8-byte aligned:
  if (dst_addr & 0x7u) return false;

  const uint64_t *src64 = (const uint64_t*)rec;
  uint32_t bytes = sizeof(PIDRecord_t);
  uint32_t n_qwords = (bytes + 7u) / 8u;

  for (uint32_t i = 0; i < n_qwords; i++) {
    HAL_StatusTypeDef st = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,
                                            dst_addr + (i * 8u),
                                            src64[i]);
    if (st != HAL_OK) return false;
  }
  return true;
}

bool PIDFlash_Load(PIDGains_t *out)
{
  if (!out) return false;

  const PIDRecord_t *best = find_latest_record();
  if (!best) return false;

  *out = best->gains;
  return true;
}

bool PIDFlash_Save(const PIDGains_t *in)
{
  if (!in) return false;

  // Build next record
  PIDRecord_t rec;
  memset(&rec, 0, sizeof(rec));
  rec.magic = PID_MAGIC;
  rec.ver   = PID_VER;

  const PIDRecord_t *best = find_latest_record();
  rec.seq = best ? (best->seq + 1u) : 1u;
  rec.gains = *in;

  rec.crc32 = crc32_sw(&rec, (uint32_t)offsetof(PIDRecord_t, crc32));

  __disable_irq();               // avoid timing/flash conflicts
  HAL_FLASH_Unlock();

  uint32_t addr = find_next_write_addr();
  if (addr == 0u) {
    // sector full -> erase then start fresh
    if (!flash_erase_sector()) {
      HAL_FLASH_Lock();
      __enable_irq();
      return false;
    }
    addr = PID_FLASH_BASE_ADDR;
  }

  bool ok = flash_program_record(addr, &rec);

  HAL_FLASH_Lock();
  __enable_irq();
  return ok;
}
