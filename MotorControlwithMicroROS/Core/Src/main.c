/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "fmc.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "robot_params.h"
#include "encoder.h"
#include "motor.h"
#include "encoder_speed.h"
#include "pid.h"
#include "odometry.h"
#include "ili9341.h"
#include "xpt2046.h"
#include "touch_event.h"
#include "ps2xlib.h"
#include <stdarg.h>
#include <stdio.h>
#include <stdbool.h>

//#include "usb_otg_host.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define min(a,b) (((a)<(b))?(a):(b))
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
void demoLCD(int i);
void locLCD(void);
unsigned long testFillScreen();
unsigned long testText();
unsigned long testLines(uint16_t color);
unsigned long testFastLines(uint16_t color1, uint16_t color2);
unsigned long testRects(uint16_t color);
unsigned long testFilledRects(uint16_t color1, uint16_t color2);
unsigned long testFilledCircles(uint8_t radius, uint16_t color);
unsigned long testCircles(uint8_t radius, uint16_t color);
unsigned long testTriangles();
unsigned long testFilledTriangles();
unsigned long testRoundRects();
unsigned long testFilledRoundRects();
unsigned long testDrawImage();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_USART3_UART_Init();
  MX_TIM3_Init();
  MX_TIM9_Init();
  MX_FMC_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  //HAL_NVIC_DisableIRQ(TIM6_DAC_IRQn);  //tam dung ngat timer 6 de setup
  //MX_USB_OTG_FS_HCD_Init();
  //MX_USB_HOST_Init();
  //HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);   //bat lai ngat timer 6

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET); // RESET = 0
  HAL_Delay(10);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);   // RESET = 1
  HAL_Delay(120);

  //Initialize touch event
  //Touch_Init_Lib();

  Encoder_Reset(ENCODER_1);
  Encoder_Reset(ENCODER_2);
  Encoder_Init();
  EncoderSpeed_Init();
  Motor_Init();

  //int32_t PID_sampletime = 5;

  PID(&PosPID1, &CurPos1, &PosPIDOut1, &DesiredPos1, Kp1, Ki1, Kd1, _PID_P_ON_E, _PID_CD_DIRECT);
  PID_SetMode(&PosPID1, _PID_MODE_AUTOMATIC);
  PID_SetSampleTime(&PosPID1, MOTORCONTROL_PERIOD_MS);
  PID_SetOutputLimits(&PosPID1, -MOTOR_SPEED_MAX, MOTOR_SPEED_MAX);

  PID(&PosPID2, &CurPos2, &PosPIDOut2, &DesiredPos2, Kp2, Ki2, Kd2, _PID_P_ON_E, _PID_CD_DIRECT);
  PID_SetMode(&PosPID2, _PID_MODE_AUTOMATIC);
  PID_SetSampleTime(&PosPID2, MOTORCONTROL_PERIOD_MS);
  PID_SetOutputLimits(&PosPID2, -MOTOR_SPEED_MAX, MOTOR_SPEED_MAX);

  PID(&SpeedPID1, &CurSpeed1, &SpeedPIDOut1, &DesiredSpeed1, Kp1, Ki1, Kd1, _PID_P_ON_E, _PID_CD_DIRECT);
  PID_SetMode(&SpeedPID1, _PID_MODE_AUTOMATIC);
  PID_SetSampleTime(&SpeedPID1, MOTORCONTROL_PERIOD_MS);
  PID_SetOutputLimits(&SpeedPID1, -MOTOR_SPEED_MAX, MOTOR_SPEED_MAX);

  PID(&SpeedPID2, &CurSpeed2, &SpeedPIDOut2, &DesiredSpeed2, Kp2, Ki2, Kd2, _PID_P_ON_E, _PID_CD_DIRECT);
  PID_SetMode(&SpeedPID2, _PID_MODE_AUTOMATIC);
  PID_SetSampleTime(&SpeedPID2, MOTORCONTROL_PERIOD_MS);
  PID_SetOutputLimits(&SpeedPID2, -MOTOR_SPEED_MAX, MOTOR_SPEED_MAX);

  PID_Init(&PosPID1); PID_Init(&PosPID2);
  PID_Init(&SpeedPID1); PID_Init(&SpeedPID2);

  //Here start to display LCD
  lcdInit();
  int i = 1;
  for (i=0;i<=0;i++)
  {
	  //demoLCD(i);
	  locLCD();

  }

//  const char test_msg[] = "USART2 VCP OK\r\n";
//  while (1)
//  {
//	  HAL_StatusTypeDef st = HAL_UART_Transmit(&huart2,
//	                                           (uint8_t*)test_msg,
//	                                           sizeof(test_msg)-1,
//	                                           10);
//
//    HAL_Delay(500);   // gửi mỗi 500 ms
//  }
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  HAL_MPU_Disable();

  // -------- Region0: default deny (giữ giống bạn) --------
  MPU_InitStruct.Enable           = MPU_REGION_ENABLE;
  MPU_InitStruct.Number           = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress      = 0x00000000;
  MPU_InitStruct.Size             = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87; // như bạn
  MPU_InitStruct.TypeExtField     = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec      = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable      = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable      = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable     = MPU_ACCESS_NOT_BUFFERABLE;
  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  // -------- Region1: allow FMC registers @0xA0000000 --------
  MPU_InitStruct.Enable           = MPU_REGION_ENABLE;
  MPU_InitStruct.Number           = MPU_REGION_NUMBER1;
  MPU_InitStruct.BaseAddress      = 0xA0000000;
  MPU_InitStruct.Size             = MPU_REGION_SIZE_1MB;   // đủ cho FMC regs
  MPU_InitStruct.SubRegionDisable = 0x00;
  MPU_InitStruct.TypeExtField     = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.DisableExec      = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable      = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable      = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable     = MPU_ACCESS_BUFFERABLE; // peripheral/device
  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  // -------- Region2: allow FMC Bank1 mapped @0x60000000 (LCD) --------
  MPU_InitStruct.Enable           = MPU_REGION_ENABLE;
  MPU_InitStruct.Number           = MPU_REGION_NUMBER2;
  MPU_InitStruct.BaseAddress      = 0x60000000;
  MPU_InitStruct.Size             = MPU_REGION_SIZE_64MB;  // Bank1 window (thường dùng)
  MPU_InitStruct.SubRegionDisable = 0x00;
  MPU_InitStruct.TypeExtField     = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.DisableExec      = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable      = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable      = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable     = MPU_ACCESS_BUFFERABLE;
  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}



void demoLCD(int i)
{
	lcdSetOrientation(i % 4);

	uint32_t t = testFillScreen();
	lcdSetTextFont(&Font16);
	lcdSetCursor(0, lcdGetHeight() - lcdGetTextFont()->Height - 1);
	lcdSetTextColor(COLOR_WHITE, COLOR_BLACK);
	lcdPrintf("Time: %4lu ms", t);
	HAL_Delay(2000);

	t = HAL_GetTick();
	lcdTest();
	lcdSetTextFont(&Font16);
	lcdSetCursor(0, lcdGetHeight() - lcdGetTextFont()->Height - 1);
	lcdSetTextColor(COLOR_WHITE, COLOR_BLACK);
	lcdPrintf("Time: %4lu ms", HAL_GetTick() - t);
	HAL_Delay(2000);

	t = testText();
	lcdSetTextFont(&Font16);
	lcdSetCursor(0, lcdGetHeight() - lcdGetTextFont()->Height - 1);
	lcdSetTextColor(COLOR_WHITE, COLOR_BLACK);
	lcdPrintf("Time: %4lu ms", t);
	HAL_Delay(2000);

	lcdSetTextFont(&Font16);
	lcdSetCursor(0, lcdGetHeight() - lcdGetTextFont()->Height - 1);
	lcdSetTextColor(COLOR_WHITE, COLOR_BLACK);
	lcdPrintf("Time: %4lu ms", testLines(COLOR_CYAN));
	HAL_Delay(2000);

	lcdSetTextFont(&Font16);
	lcdSetCursor(0, lcdGetHeight() - lcdGetTextFont()->Height - 1);
	lcdSetTextColor(COLOR_WHITE, COLOR_BLACK);
	lcdPrintf("Time: %4lu ms", testFastLines(COLOR_RED, COLOR_BLUE));
	HAL_Delay(2000);

	lcdSetTextFont(&Font16);
	lcdSetCursor(0, lcdGetHeight() - lcdGetTextFont()->Height - 1);
	lcdSetTextColor(COLOR_WHITE, COLOR_BLACK);
	lcdPrintf("Time: %4lu ms", testRects(COLOR_GREEN));
	HAL_Delay(2000);

	lcdSetTextFont(&Font16);
	lcdSetCursor(0, lcdGetHeight() - lcdGetTextFont()->Height - 1);
	lcdSetTextColor(COLOR_WHITE, COLOR_BLACK);
	lcdPrintf("Time: %4lu ms", testFilledRects(COLOR_YELLOW, COLOR_MAGENTA));
	HAL_Delay(2000);

	lcdSetTextFont(&Font16);
	lcdSetCursor(0, lcdGetHeight() - lcdGetTextFont()->Height - 1);
	lcdSetTextColor(COLOR_WHITE, COLOR_BLACK);
	lcdPrintf("Time: %4lu ms", testFilledCircles(10, COLOR_MAGENTA));
	HAL_Delay(2000);

	lcdSetTextFont(&Font16);
	lcdSetCursor(0, lcdGetHeight() - lcdGetTextFont()->Height - 1);
	lcdSetTextColor(COLOR_WHITE, COLOR_BLACK);
	lcdPrintf("Time: %4lu ms", testCircles(10, COLOR_WHITE));
	HAL_Delay(2000);

	lcdSetTextFont(&Font16);
	lcdSetCursor(0, lcdGetHeight() - lcdGetTextFont()->Height - 1);
	lcdSetTextColor(COLOR_WHITE, COLOR_BLACK);
	lcdPrintf("Time: %4lu ms", testTriangles());
	HAL_Delay(2000);

	lcdSetTextFont(&Font16);
	lcdSetCursor(0, lcdGetHeight() - lcdGetTextFont()->Height - 1);
	lcdSetTextColor(COLOR_WHITE, COLOR_BLACK);
	lcdPrintf("Time: %4lu ms", testFilledTriangles());
	HAL_Delay(2000);

	lcdSetTextFont(&Font16);
	lcdSetCursor(0, lcdGetHeight() - lcdGetTextFont()->Height - 1);
	lcdSetTextColor(COLOR_WHITE, COLOR_BLACK);
	lcdPrintf("Time: %4lu ms", testRoundRects());
	HAL_Delay(2000);

	lcdSetTextFont(&Font16);
	lcdSetCursor(0, lcdGetHeight() - lcdGetTextFont()->Height - 1);
	lcdSetTextColor(COLOR_WHITE, COLOR_BLACK);
	lcdPrintf("Time: %4lu ms", testFilledRoundRects());
	HAL_Delay(2000);

	lcdSetTextFont(&Font16);
	lcdSetCursor(0, lcdGetHeight() - lcdGetTextFont()->Height - 1);
	lcdSetTextColor(COLOR_WHITE, COLOR_BLACK);
	lcdPrintf("Time: %4lu ms", testDrawImage());
	HAL_Delay(2000);
}

typedef struct { uint16_t x,y,w,h; } Rect_t;

static Rect_t g_val_pwm1, g_val_pwm2, g_val_enc1, g_val_enc2;
static Rect_t g_val_v1, g_val_v2;
static Rect_t g_val_x, g_val_y, g_val_yaw;

static uint16_t g_charW = 8, g_fontH = 12, g_pad = 6;

void locLCD(void)
{
    lcdSetOrientation(LCD_ORIENTATION_LANDSCAPE);
    lcdFillRGB(COLOR_BLACK);
    lcdSetTextFont(&Font12);

    uint16_t W = lcdGetWidth();
    uint16_t H = lcdGetHeight();

    uint16_t fontH = lcdGetTextFont()->Height;
    uint16_t charW = lcdGetTextFont()->Width;   // nếu lỗi -> đổi thành 8

    // ===== Grid =====
    uint16_t M  = 6;
    uint16_t GX = M;
    uint16_t GY = 22;
    uint16_t GW = W - 2*M;
    uint16_t GH = H - GY - M;

    uint16_t CW = GW / 3;
    uint16_t RH = GH / 4;
    uint16_t P  = 6;
    g_fontH = fontH;
    g_charW = (charW == 0) ? 8 : charW;
    g_pad   = P;

    #define STORE_VALUE_RECT(_rect, cx, cy, cw, ch) do {                          \
        int16_t _y0 = (int16_t)(cy) + (int16_t)(((ch) - 2*(int16_t)fontH)/2);     \
        if (_y0 < (int16_t)(cy)+P) _y0 = (int16_t)(cy)+P;                         \
        (_rect).x = (uint16_t)((cx) + 1);                                         \
        (_rect).y = (uint16_t)(_y0 + (int16_t)fontH);                             \
        (_rect).w = (uint16_t)((cw) - 2);                                         \
        (_rect).h = (uint16_t)(fontH);                                            \
    } while(0)
    // ===== Header =====
    lcdSetTextColor(COLOR_YELLOW, COLOR_BLACK);
    lcdSetCursor(M, 4);
    lcdPrintf("Robomango Zackon");

    // ===== Draw grid =====
    lcdSetTextColor(COLOR_WHITE, COLOR_BLACK);
    lcdDrawRect(GX, GY, GW, GH, COLOR_WHITE);
    lcdDrawVLine(GX + CW,   GY, GY + GH, COLOR_WHITE);
    lcdDrawVLine(GX + 2*CW, GY, GY + GH, COLOR_WHITE);
    for (int r = 1; r < 4; r++)
        lcdDrawHLine(GX, GX + GW, GY + r*RH, COLOR_WHITE);

    #define CELL_X(c) (GX + (c)*CW)
    #define CELL_Y(r) (GY + (r)*RH)
    #define MAX_CHARS(_w)  ((int)(((_w) - 2*P) / charW))

    // ===== In 2 dòng, CẢ 2 dòng đều căn giữa + clip không tràn =====
    #define PRINT_2LINES_CENTER(cx,cy,cw,ch,l1,l2,color,bg) do {           \
        int m = MAX_CHARS(cw); if (m < 1) m = 1;                           \
        int len1 = (int)strlen(l1); if (len1 > m) len1 = m;                \
        int len2 = (int)strlen(l2); if (len2 > m) len2 = m;                \
        int w1 = len1 * (int)charW;                                        \
        int w2 = len2 * (int)charW;                                        \
        int wMax = (w1 > w2) ? w1 : w2;                                    \
        int16_t x = (int16_t)(cx) + (int16_t)((cw) - wMax)/2;              \
        int16_t y = (int16_t)(cy) + (int16_t)((ch) - 2*(int16_t)fontH)/2;  \
        if (x < (int16_t)(cx)+P) x = (int16_t)(cx)+P;                      \
        if (y < (int16_t)(cy)+P) y = (int16_t)(cy)+P;                      \
        lcdSetTextColor(color, bg);                                        \
        lcdSetCursor((unsigned short)x, (unsigned short)y);                \
        lcdPrintf("%.*s", len1, l1);                                       \
        lcdSetCursor((unsigned short)x, (unsigned short)(y + fontH));      \
        lcdPrintf("%.*s", len2, l2);                                       \
    } while(0)

    // ===== Button: 1 dòng căn giữa + clip =====
    #define PRINT_CENTER_1LINE(cx,cy,cw,ch,text,color,bg) do {             \
        int m = MAX_CHARS(cw); if (m < 1) m = 1;                           \
        int len = (int)strlen(text); if (len > m) len = m;                 \
        int16_t x = (int16_t)(cx) + (int16_t)((cw) - len*(int)charW)/2;    \
        int16_t y = (int16_t)(cy) + (int16_t)((ch) - (int16_t)fontH)/2 + 1;\
        if (x < (int16_t)(cx)+P) x = (int16_t)(cx)+P;                      \
        if (y < (int16_t)(cy)+P) y = (int16_t)(cy)+P;                      \
        lcdSetTextColor(color, bg);                                        \
        lcdSetCursor((unsigned short)x, (unsigned short)y);                \
        lcdPrintf("%.*s", len, text);                                      \
    } while(0)

    // ===== Cột 1 (tất cả CENTER) =====
    PRINT_2LINES_CENTER(CELL_X(0), CELL_Y(0), CW, RH, "PWM1", "0.0",  COLOR_CYAN, COLOR_BLACK);
    STORE_VALUE_RECT(g_val_pwm1, CELL_X(0), CELL_Y(0), CW, RH);

    PRINT_2LINES_CENTER(CELL_X(0), CELL_Y(1), CW, RH, "PWM2", "0.0",  COLOR_CYAN, COLOR_BLACK);
    STORE_VALUE_RECT(g_val_pwm2, CELL_X(0), CELL_Y(1), CW, RH);

    PRINT_2LINES_CENTER(CELL_X(0), CELL_Y(2), CW, RH, "Enc1", "0.0", COLOR_CYAN, COLOR_BLACK);
    STORE_VALUE_RECT(g_val_enc1, CELL_X(0), CELL_Y(2), CW, RH);

    PRINT_2LINES_CENTER(CELL_X(0), CELL_Y(3), CW, RH, "Enc2", "0.0", COLOR_CYAN, COLOR_BLACK);
    STORE_VALUE_RECT(g_val_enc2, CELL_X(0), CELL_Y(3), CW, RH);


    // ===== Cột 2 =====
    PRINT_2LINES_CENTER(CELL_X(1), CELL_Y(0), CW, RH, "V1", "0.0", COLOR_CYAN, COLOR_BLACK);
    STORE_VALUE_RECT(g_val_v1, CELL_X(1), CELL_Y(0), CW, RH);

    PRINT_2LINES_CENTER(CELL_X(1), CELL_Y(1), CW, RH, "V2", "0.0", COLOR_CYAN, COLOR_BLACK);
    STORE_VALUE_RECT(g_val_v2, CELL_X(1), CELL_Y(1), CW, RH);


    // Button Figure
    {
        uint16_t bx = CELL_X(1)+4, by = CELL_Y(2)+4;
        uint16_t bw = CW-8, bh = RH-8;
        lcdFillRect(bx, by, bw, bh, COLOR_DARKGREY);
        lcdDrawRect(bx, by, bw, bh, COLOR_WHITE);
        PRINT_CENTER_1LINE(bx, by, bw, bh, "Figure", COLOR_DARKGREY, COLOR_DARKGREY);
    }

    // Button Controller
    {
        uint16_t bx = CELL_X(1)+4, by = CELL_Y(3)+4;
        uint16_t bw = CW-8, bh = RH-8;
        lcdFillRect(bx, by, bw, bh, COLOR_DARKGREY);
        lcdDrawRect(bx, by, bw, bh, COLOR_WHITE);
        PRINT_CENTER_1LINE(bx, by, bw, bh, "Controller", COLOR_WHITE, COLOR_DARKGREY);
    }

    // ===== Cột 3 =====
    PRINT_2LINES_CENTER(CELL_X(2), CELL_Y(0), CW, RH, "Odom X", "0.0", COLOR_CYAN, COLOR_BLACK);
    STORE_VALUE_RECT(g_val_x, CELL_X(2), CELL_Y(0), CW, RH);

    PRINT_2LINES_CENTER(CELL_X(2), CELL_Y(1), CW, RH, "Odom Y", "0.0", COLOR_CYAN, COLOR_BLACK);
    STORE_VALUE_RECT(g_val_y, CELL_X(2), CELL_Y(1), CW, RH);

    PRINT_2LINES_CENTER(CELL_X(2), CELL_Y(2), CW, RH, "Yaw",   "0.0", COLOR_CYAN, COLOR_BLACK);
    STORE_VALUE_RECT(g_val_yaw, CELL_X(2), CELL_Y(2), CW, RH);


    // Button Setting PID
    {
        uint16_t bx = CELL_X(2)+4, by = CELL_Y(3)+4;
        uint16_t bw = CW-8, bh = RH-8;
        lcdFillRect(bx, by, bw, bh, COLOR_DARKGREY);
        lcdDrawRect(bx, by, bw, bh, COLOR_WHITE);
        PRINT_CENTER_1LINE(bx, by, bw, bh, "Setting PID", COLOR_WHITE, COLOR_DARKGREY);
    }

    #undef PRINT_CENTER_1LINE
    #undef PRINT_2LINES_CENTER
    #undef MAX_CHARS
    #undef CELL_X
    #undef CELL_Y
	#undef STORE_VALUE_RECT

}

static void drawValueInRect(const Rect_t* r, const char* val, uint16_t color, uint16_t bg)
{
    // clear đúng 1 dòng value
    lcdFillRect(r->x, r->y, r->w, r->h, bg);

    int maxChars = (int)((r->w - 2*g_pad) / g_charW);
    if (maxChars < 1) maxChars = 1;

    int len = (int)strlen(val);
    if (len > maxChars) len = maxChars;

    int16_t x = (int16_t)r->x + (int16_t)((r->w - len*(int)g_charW)/2);
    int16_t y = (int16_t)r->y;

    lcdSetTextColor(color, bg);
    lcdSetCursor((unsigned short)x, (unsigned short)y);
    lcdPrintf("%.*s", len, val);
}




unsigned long testFillScreen()
{
	unsigned long start = HAL_GetTick(), t = 0;
	lcdFillRGB(COLOR_BLACK);
	t += HAL_GetTick() - start;
	lcdSetCursor(0, 0);
	lcdSetTextFont(&Font24);
	lcdSetTextColor(COLOR_WHITE, COLOR_BLACK);
	lcdPrintf("BLACK");
	HAL_Delay(1000);
	start = HAL_GetTick();
	lcdFillRGB(COLOR_RED);
	t += HAL_GetTick() - start;
	lcdSetCursor(0, 0);
	lcdSetTextFont(&Font24);
	lcdSetTextColor(COLOR_WHITE, COLOR_BLACK);
	lcdPrintf("RED");
	HAL_Delay(1000);
	start = HAL_GetTick();
	lcdFillRGB(COLOR_GREEN);
	t += HAL_GetTick() - start;
	lcdSetCursor(0, 0);
	lcdSetTextFont(&Font24);
	lcdSetTextColor(COLOR_WHITE, COLOR_BLACK);
	lcdPrintf("GREEN");
	HAL_Delay(1000);
	start = HAL_GetTick();
	lcdFillRGB(COLOR_BLUE);
	t += HAL_GetTick() - start;
	lcdSetCursor(0, 0);
	lcdSetTextFont(&Font24);
	lcdSetTextColor(COLOR_WHITE, COLOR_BLACK);
	lcdPrintf("BLUE");
	HAL_Delay(1000);
	start = HAL_GetTick();
	lcdFillRGB(COLOR_BLACK);
	return t += HAL_GetTick() - start;
}

	unsigned long testText()
	{
		lcdFillRGB(COLOR_BLACK);
		unsigned long start = HAL_GetTick();
		lcdSetCursor(0, 0);
		lcdSetTextColor(COLOR_WHITE, COLOR_BLACK);
		lcdSetTextFont(&Font8);
		lcdPrintf("Hello World!\r\n");
		lcdSetTextColor(COLOR_YELLOW, COLOR_BLACK);
		lcdSetTextFont(&Font12);
		lcdPrintf("%i\r\n", 1234567890);
		lcdSetTextColor(COLOR_RED, COLOR_BLACK);
		lcdSetTextFont(&Font12);
		lcdPrintf("%#X\r\n", 0xDEADBEEF);
		lcdPrintf("\r\n");
		lcdSetTextColor(COLOR_GREEN, COLOR_BLACK);
		lcdSetTextFont(&Font20);
		lcdPrintf("Groop\r\n");
		lcdSetTextFont(&Font12);
		lcdPrintf("I implore thee,\r\n");
		lcdSetTextFont(&Font12);
		lcdPrintf("my foonting turlingdromes.\r\n");
		lcdPrintf("And hooptiously drangle me\r\n");
		lcdPrintf("with crinkly bindlewurdles,\r\n");
		lcdPrintf("Or I will rend thee\r\n");
		lcdPrintf("in the gobberwarts\r\n");
		lcdPrintf("with my blurglecruncheon,\r\n");
		lcdPrintf("see if I don't!\r\n");
		return HAL_GetTick() - start;
	}

unsigned long testLines(uint16_t color)
{
  unsigned long start, t;
  int           x1, y1, x2, y2,
                w = lcdGetWidth(),
                h = lcdGetHeight();

  lcdFillRGB(COLOR_BLACK);

  x1 = y1 = 0;
  y2    = h - 1;
  start = HAL_GetTick();
  for(x2 = 0; x2 < w; x2 += 6) lcdDrawLine(x1, y1, x2, y2, color);
  x2 = w - 1;
  for(y2 = 0; y2 < h; y2 += 6) lcdDrawLine(x1, y1, x2, y2, color);
  t = HAL_GetTick() - start; // fillScreen doesn't count against timing

  HAL_Delay(1000);
  lcdFillRGB(COLOR_BLACK);

  x1 = w - 1;
  y1 = 0;
  y2 = h - 1;

  start = HAL_GetTick();

  for(x2 = 0; x2 < w; x2 += 6) lcdDrawLine(x1, y1, x2, y2, color);
  x2 = 0;
  for(y2 = 0; y2 < h; y2 += 6) lcdDrawLine(x1, y1, x2, y2, color);
  t += HAL_GetTick() - start;

  HAL_Delay(1000);
  lcdFillRGB(COLOR_BLACK);

  x1 = 0;
  y1 = h - 1;
  y2 = 0;
  start = HAL_GetTick();

  for(x2 = 0; x2 < w; x2 += 6) lcdDrawLine(x1, y1, x2, y2, color);
  x2 = w - 1;
  for(y2 = 0; y2 < h; y2 += 6) lcdDrawLine(x1, y1, x2, y2, color);
  t += HAL_GetTick() - start;

  HAL_Delay(1000);
  lcdFillRGB(COLOR_BLACK);

  x1 = w - 1;
  y1 = h - 1;
  y2 = 0;

  start = HAL_GetTick();

  for(x2 = 0; x2 < w; x2 += 6) lcdDrawLine(x1, y1, x2, y2, color);
  x2 = 0;
  for(y2 = 0; y2 < h; y2 += 6) lcdDrawLine(x1, y1, x2, y2, color);

  return t += HAL_GetTick() - start;
}

unsigned long testFastLines(uint16_t color1, uint16_t color2)
{
  unsigned long start;
  int x, y, w = lcdGetWidth(), h = lcdGetHeight();

  lcdFillRGB(COLOR_BLACK);
  start = HAL_GetTick();
  for(y = 0; y < h; y += 5) lcdDrawHLine(0, w, y, color1);
  for(x = 0; x < w; x += 5) lcdDrawVLine(x, 0, h, color2);

  return HAL_GetTick() - start;
}

unsigned long testRects(uint16_t color)
{
  unsigned long start;
  int n, i, i2,
      cx = lcdGetWidth()  / 2,
      cy = lcdGetHeight() / 2;

  lcdFillRGB(COLOR_BLACK);
  n = min(lcdGetWidth(), lcdGetHeight());
  start = HAL_GetTick();
  for(i = 2; i < n; i += 6)
  {
    i2 = i / 2;
    lcdDrawRect(cx - i2, cy - i2, i, i, color);
  }

  return HAL_GetTick() - start;
}

unsigned long testFilledRects(uint16_t color1, uint16_t color2)
{
  unsigned long start, t = 0;
  int n, i, i2,
      cx = lcdGetWidth() / 2 - 1,
      cy = lcdGetHeight() / 2 - 1;

  lcdFillRGB(COLOR_BLACK);
  n = min(lcdGetWidth(), lcdGetHeight());

  for(i = n; i > 0; i -= 6)
  {
    i2 = i / 2;
    start = HAL_GetTick();
    lcdFillRect(cx-i2, cy-i2, i, i, color1);
    t    += HAL_GetTick() - start;
    // Outlines are not included in timing results
    lcdDrawRect(cx-i2, cy-i2, i, i, color1);
  }

  return t;
}

unsigned long testFilledCircles(uint8_t radius, uint16_t color)
{
  unsigned long start;
  int x, y, w = lcdGetWidth(), h = lcdGetHeight(), r2 = radius * 2;

  lcdFillRGB(COLOR_BLACK);
  start = HAL_GetTick();
  for(x = radius; x < w; x += r2)
  {
    for(y = radius; y < h; y += r2)
    {
      lcdFillCircle(x, y, radius, color);
    }
  }

  return HAL_GetTick() - start;
}

unsigned long testCircles(uint8_t radius, uint16_t color)
{
  unsigned long start;
  int x, y, r2 = radius * 2,
      w = lcdGetWidth()  + radius,
      h = lcdGetHeight() + radius;

  // Screen is not cleared for this one -- this is
  // intentional and does not affect the reported time.
  start = HAL_GetTick();
  for(x = 0; x < w; x += r2)
  {
    for(y = 0; y < h; y += r2)
    {
      lcdDrawCircle(x, y, radius, color);
    }
  }

  return HAL_GetTick() - start;
}

unsigned long testTriangles()
{
  unsigned long start;
  int n, i, cx = lcdGetWidth() / 2 - 1,
            cy = lcdGetHeight() / 2 - 1;

  lcdFillRGB(COLOR_BLACK);
  n = min(cx, cy);
  start = HAL_GetTick();
  for(i = 0; i < n; i += 5)
  {
    lcdDrawTriangle(
      cx    , cy - i, // peak
      cx - i, cy + i, // bottom left
      cx + i, cy + i, // bottom right
      lcdColor565(i, i, i));
  }

  return HAL_GetTick() - start;
}

unsigned long testFilledTriangles()
{
  unsigned long start, t = 0;
  int i, cx = lcdGetWidth() / 2 - 1,
         cy = lcdGetHeight() / 2 - 1;

  lcdFillRGB(COLOR_BLACK);
  for(i = min(cx, cy); i > 10; i -= 5)
  {
    start = HAL_GetTick();
    lcdFillTriangle(cx, cy - i, cx - i, cy + i, cx + i, cy + i, lcdColor565(0, i*10, i*10));
    t += HAL_GetTick() - start;
    lcdFillTriangle(cx, cy - i, cx - i, cy + i, cx + i, cy + i, lcdColor565(i*10, i*10, 0));
  }

  return t;
}

unsigned long testRoundRects()
{
  unsigned long start;
  int w, i, i2,
      cx = lcdGetWidth() / 2 - 1,
      cy = lcdGetHeight() / 2 - 1;

  lcdFillRGB(COLOR_BLACK);
  w = lcdGetWidth(), lcdGetHeight();
  start = HAL_GetTick();
  for(i = 0; i < w; i += 6)
  {
    i2 = i / 2;
    lcdDrawRoundRect(cx-i2, cy-i2, i, i, i/8, lcdColor565(i, 0, 0));
  }

  return HAL_GetTick() - start;
}

unsigned long testFilledRoundRects()
{
  unsigned long start;
  int i, i2,
      cx = lcdGetWidth()  / 2 - 1,
      cy = lcdGetHeight() / 2 - 1;

  lcdFillRGB(COLOR_BLACK);
  start = HAL_GetTick();
  for(i = min(lcdGetWidth(), lcdGetHeight()); i > 20; i -=6 )
  {
    i2 = i / 2;
    lcdFillRoundRect(cx - i2, cy - i2, i, i, i / 8, lcdColor565(0, i, 0));
  }

  return HAL_GetTick() - start;
}

unsigned long testDrawImage()
{
	unsigned long start;

	lcdFillRGB(COLOR_BLACK);
	start = HAL_GetTick();
	if (lcdGetOrientation() == LCD_ORIENTATION_LANDSCAPE || lcdGetOrientation() == LCD_ORIENTATION_LANDSCAPE_MIRROR)
	{
		lcdDrawImage((lcdGetWidth() - bmSTLogo.xSize) / 2, 0, &bmSTLogo);
	}
	else
	{
		lcdDrawImage(0, (lcdGetHeight() - bmSTLogo.ySize) / 2, &bmSTLogo);
	}
	return HAL_GetTick() - start;
}
/* USER CODE END 4 */


/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
