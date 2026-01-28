/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <math.h>
#include "usart.h"
#include "robot_params.h"
#include "encoder.h"
#include "motor.h"
#include "motion_profile.h"
#include "encoder_speed.h"
#include "pid.h"
#include "odometry.h"
#include "semphr.h"
#include "touch_rtos.h"
#include "spi.h"
#include "touch_event.h"
#include "xpt2046.h"
#include "ps2xlib.h"
#include "ili9341.h"
//#include "usb_host.h"
//#include "usbh_core.h"
//#include "ps2_pad.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <uxr/client/transport.h>
#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>

#include <std_msgs/msg/int32.h>

#include <nav_msgs/msg/odometry.h>
#include <geometry_msgs/msg/quaternion.h>
#include <geometry_msgs/msg/vector3.h>
#include <geometry_msgs/msg/pose.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/float64_multi_array.h>
#include <rosidl_runtime_c/string_functions.h>


static rcl_publisher_t odom_pub;
static nav_msgs__msg__Odometry odom_msg;

static float odom_vx  = 0.0f;
static float odom_vy  = 0.0f;
static float odom_vth = 0.0f;
volatile uint32_t last_cmdvel_ms = 0;

//this variables are the set pos or speed from the high level controller

volatile MP_Mode_t modCtrl = MP_MODE_VELOCITY; //default mode
#define POS_CMD_RESYNC_TH    (0.01f)   // position command step that triggers resync (units of position)
#define SPD_CMD_RESYNC_TH    (0.05f)   // speed command step that triggers resync (units of speed)
static inline float fabsf_fast(double x){ return (x < 0.0f) ? -x : x; }

volatile double SetPos1 = 0.0f;
volatile double SetPos2 = 0.0f;
volatile double SetSpeed1 = 0.0f;
volatile double SetSpeed2 = 0.0f;

volatile int g_activeCell = -1;   // biến dùng cho update lcd
static uint8_t s_prevPressed = 0;


static volatile uint8_t pid_update_pending = 0;  //biến dùng để update PID từ ROS
static SemaphoreHandle_t pid_mutex;  //kỹ thuật cho phép tạm thời lock trong quá trình thay đổi biến PID, tránh bị cập nhật nửa chừng

static uint16_t lastX=0, lastY=0;

volatile uint16_t touch_x = 0;
volatile uint16_t touch_y = 0;
volatile uint8_t  touch_pressed = 0;

typedef struct {
    uint16_t x, y, w, h;
    const char* label;
    uint8_t id;
} UICell_t;

static inline int pointInRect(uint16_t px, uint16_t py, const UICell_t* r)
{
    return (px >= r->x) && (px < r->x + r->w) &&
           (py >= r->y) && (py < r->y + r->h);
}
enum {
    CELL_PWM1 = 0,
    CELL_PWM2,
    CELL_ENC1,
    CELL_ENC2,
    CELL_V1,
    CELL_V2,
    CELL_FIGURE,
    CELL_CONTROLLER,
    CELL_ODOMX,
    CELL_ODOMY,
    CELL_YAW,
    CELL_SETTING_PID,
    CELL_COUNT
};
static void uiBuildCellsFromLocLCD(void)
{
	static UICell_t g_cells[CELL_COUNT];
    uint16_t W = lcdGetWidth();
    uint16_t H = lcdGetHeight();

    // giống hệt locLCD()
    uint16_t M  = 6;
    uint16_t GX = M;
    uint16_t GY = 22;
    uint16_t GW = W - 2*M;
    uint16_t GH = H - GY - M;

    uint16_t CW = GW / 3;
    uint16_t RH = GH / 4;

    #define CELL_X(c) (GX + (c)*CW)
    #define CELL_Y(r) (GY + (r)*RH)

    // Cột 1
    g_cells[CELL_PWM1] = (UICell_t){ CELL_X(0), CELL_Y(0), CW, RH, "PWM1", CELL_PWM1 };
    g_cells[CELL_PWM2] = (UICell_t){ CELL_X(0), CELL_Y(1), CW, RH, "PWM2", CELL_PWM2 };
    g_cells[CELL_ENC1] = (UICell_t){ CELL_X(0), CELL_Y(2), CW, RH, "Enc1", CELL_ENC1 };
    g_cells[CELL_ENC2] = (UICell_t){ CELL_X(0), CELL_Y(3), CW, RH, "Enc2", CELL_ENC2 };

    // Cột 2
    g_cells[CELL_V1] = (UICell_t){ CELL_X(1), CELL_Y(0), CW, RH, "V1", CELL_V1 };
    g_cells[CELL_V2] = (UICell_t){ CELL_X(1), CELL_Y(1), CW, RH, "V2", CELL_V2 };
    g_cells[CELL_FIGURE]     = (UICell_t){ CELL_X(1), CELL_Y(2), CW, RH, "Figure", CELL_FIGURE };
    g_cells[CELL_CONTROLLER] = (UICell_t){ CELL_X(1), CELL_Y(3), CW, RH, "Controller", CELL_CONTROLLER };

    // Cột 3
    g_cells[CELL_ODOMX] = (UICell_t){ CELL_X(2), CELL_Y(0), CW, RH, "Odom X", CELL_ODOMX };
    g_cells[CELL_ODOMY] = (UICell_t){ CELL_X(2), CELL_Y(1), CW, RH, "Odom Y", CELL_ODOMY };
    g_cells[CELL_YAW]   = (UICell_t){ CELL_X(2), CELL_Y(2), CW, RH, "Yaw",   CELL_YAW };
    g_cells[CELL_SETTING_PID] = (UICell_t){ CELL_X(2), CELL_Y(3), CW, RH, "Setting PID", CELL_SETTING_PID };

    #undef CELL_X
    #undef CELL_Y
}

static const UICell_t g_cells[CELL_COUNT] = {
    { 0,   0, 160, 120, "ENC1", CELL_ENC1 },
    {160,  0, 160, 120, "ENC2", CELL_ENC2 },
};



typedef struct {
  uint32_t tick;
  uint8_t  id;
  uint16_t buttons;
  uint8_t  lx, ly, rx, ry;
  uint8_t  ok;
  uint8_t  analog;
} PS2X_State;

static PS2X_Handle ps2;

// Shared snapshot for other tasks
static PS2X_State g_ps2_state;
static osMutexId_t g_ps2_mutex;

static inline float clampf(float x, float lo, float hi) {
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

static inline int is_pressed(uint16_t buttons, uint16_t mask) {
    return (buttons & mask) ? 1 : 0;
}

//bien dung de cap nhat du lieu len locLCD
typedef struct {
  int32_t enc1, enc2;
  float   v1, v2;
} EncData_t;

typedef struct {
  float x, y, yaw;
} OdomData_t;

typedef struct {
  float pwm1, pwm2;
} MotorData_t;

// dữ liệu hiện tại
static EncData_t   g_encData;
static OdomData_t  g_odomData;
static MotorData_t g_motorData;

// sequence numbers (odd = đang ghi, even = xong)
static volatile uint32_t g_encSeq   = 0;
static volatile uint32_t g_odomSeq  = 0;
static volatile uint32_t g_motorSeq = 0;

// memory barrier (CMSIS)
#ifndef __DMB
#define __DMB() __asm volatile ("dmb 0xF":::"memory")
#endif

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
static inline void EncData_Publish(int32_t enc1, int32_t enc2, float v1, float v2);
static inline void OdomData_Publish(float x, float y, float yaw);
static inline void MotorData_Publish(float pwm1, float pwm2);


bool cubemx_transport_open(struct uxrCustomTransport * transport);
bool cubemx_transport_close(struct uxrCustomTransport * transport);
size_t cubemx_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err);
size_t cubemx_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err);

void * microros_allocate(size_t size, void * state);
void microros_deallocate(void * pointer, void * state);
void * microros_reallocate(void * pointer, size_t size, void * state);
void * microros_zero_allocate(size_t number_of_elements, size_t size_of_element, void * state);
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
static void microros_odom_init(rcl_node_t * node);
static void microros_publish_odom(void);
static void cmdvel_cb(const void * msgin);
static void pid_cb(const void * msgin);
static inline uint32_t millis_void(void);
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 3000 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
osThreadId_t encoderTaskHandle;
osThreadId_t motorTaskHandle;
//osThreadId_t uartTaskHandle
osThreadId_t odometryTaskHandle;
osThreadId_t touchTaskHandle;
//osThreadId_t UsbHostTaskHandle;
osThreadId_t PS2TaskHandle;
osThreadId_t LCDTaskHandle;
osThreadId_t teleopTaskHandle;
osThreadId_t motionTaskHandle;

const osThreadAttr_t encoderTask_attributes = {
  .name       = "EncoderTask",
  .priority   = (osPriority_t) osPriorityHigh,
  .stack_size = 256 * 4
};

const osThreadAttr_t motorTask_attributes = {
  .name       = "MotorTask",
  .priority   = (osPriority_t) osPriorityHigh,
  .stack_size = 256 * 4
};

//const osThreadAttr_t uartTask_attributes = {
//  .name       = "UartTask",
//  .priority   = (osPriority_t) osPriorityLow,
//  .stack_size = 256 * 4
//};

const osThreadAttr_t odometryTask_attributes = {
  .name       = "OdometryTask",
  .priority   = (osPriority_t) osPriorityHigh,
  .stack_size = 256 * 4
};

static const osThreadAttr_t touchTask_attributes = {
      .name = "TouchTask",
      .stack_size = 512 * 4,              // bytes
      .priority = (osPriority_t) osPriorityBelowNormal
};


//const osThreadAttr_t UsbHostTask_attributes = {
//  .name = "UsbHostTask",
//  .stack_size = 1024 * 4,     // 4KB stack (đủ an toàn)
//  .priority = (osPriority_t) osPriorityNormal
//};
//
const osThreadAttr_t PS2Task_attributes = {
  .name       = "PS2Task",
  .stack_size = 512 * 4,   // 2KB stack (đủ cho parse HID)
  .priority   = (osPriority_t) osPriorityNormal
};

const osThreadAttr_t teleopTask_attributes = {
  .name = "TeleopTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 512 * 4   // bytes (512 words if you think in words; here it's bytes)
};

const osThreadAttr_t lcdTask_attributes = {
  .name = "lcdTask",
  .stack_size = 1024,
  .priority = (osPriority_t) osPriorityLow  // UI để Low là hợp lý
};

const osThreadAttr_t motionTask_attributes = {
  .name = "motionTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal  // UI để Low là hợp lý
};

void StartUITask(void *argument);


void EncoderTask(void *argument);
void MotorControlTask(void *argument);
//void UartTask(void *argument);
void OdometryTask(void *argument);
void MotionTask(void *argument);
void TouchTask(void *argument);
//void StartUsbHostTask(void *argument);
void StartPS2Task(void *argument);
void StartLCDTask(void *argument);
void StartTeleopTask(void *argument);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  pid_mutex = xSemaphoreCreateMutex();
  g_ps2_mutex = osMutexNew(NULL);


  encoderTaskHandle = osThreadNew(EncoderTask, NULL, &encoderTask_attributes);
  motorTaskHandle   = osThreadNew(MotorControlTask, NULL, &motorTask_attributes);
//  uartTaskHandle    = osThreadNew(UartTask, NULL, &uartTask_attributes);
  odometryTaskHandle = osThreadNew(OdometryTask, NULL, &odometryTask_attributes);

  touchTaskHandle = osThreadNew(TouchTask_Start, NULL, &touchTask_attributes);
//  UsbHostTaskHandle = osThreadNew(StartUsbHostTask, NULL, &UsbHostTask_attributes);
  PS2TaskHandle = osThreadNew(StartPS2Task, NULL, &PS2Task_attributes);
  teleopTaskHandle = osThreadNew(StartTeleopTask, NULL, &teleopTask_attributes);
  LCDTaskHandle = osThreadNew(StartLCDTask, NULL, &lcdTask_attributes);
  teleopTaskHandle = osThreadNew(StartTeleopTask, NULL, &teleopTask_attributes);
  motionTaskHandle = osThreadNew(MotionTask, NULL, &motionTask_attributes);
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void StartDefaultTask(void *argument)
{

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET); // LED1 sáng luôn
  // ==== micro-ROS transport config ====
  rmw_uros_set_custom_transport(
    true,
    (void *) &huart3,
    cubemx_transport_open,
    cubemx_transport_close,
    cubemx_transport_write,
    cubemx_transport_read);


  // ==== Allocator FreeRTOS ====
  rcl_allocator_t freeRTOS_allocator = rcutils_get_zero_initialized_allocator();
  freeRTOS_allocator.allocate      = microros_allocate;
  freeRTOS_allocator.deallocate    = microros_deallocate;
  freeRTOS_allocator.reallocate    = microros_reallocate;
  freeRTOS_allocator.zero_allocate = microros_zero_allocate;

  if (!rcutils_set_default_allocator(&freeRTOS_allocator)) {
      printf("Error on default allocators (line %d)\n", __LINE__);
  }

  // ==== micro-ROS core ====
  rclc_support_t support;
  rcl_allocator_t allocator;
  rcl_node_t node;
  rcl_subscription_t sub_cmdvel;
  rcl_subscription_t sub_pid;
  geometry_msgs__msg__Twist msg_cmdvel;
  std_msgs__msg__Float64MultiArray msg_pid;
  rclc_executor_t executor;

  allocator = rcl_get_default_allocator();

  // init_options + context
  rclc_support_init(&support, 0, NULL, &allocator);

  // create node
  rclc_node_init_default(&node, "stm32_odom_node", "", &support);

  // init odom publisher
  microros_odom_init(&node);

  // ==== create subscription /cmd_vel ====
  rclc_subscription_init_default(
      &sub_cmdvel,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
      "cmd_vel"
  );

  // ==== create subscription /pid_tunning ====
  rclc_subscription_init_default(
      &sub_pid,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray),
      "pid_tuning"
  );

  // ==== init executor và add sub_cmdvel vào ====
  rclc_executor_init(&executor, &support.context, 2, &allocator);
  rclc_executor_add_subscription(
      &executor,
      &sub_cmdvel,
      &msg_cmdvel,
      &cmdvel_cb,
      ON_NEW_DATA
  );

  rclc_executor_add_subscription(
      &executor,
      &sub_pid,
      &msg_pid,
      &pid_cb,
      ON_NEW_DATA
  );

  // ====== Vòng lặp publish /odom mỗi 10 ms + xử lý /cmd_vel ======
  float last_x     = odom_x;
  float last_y     = odom_y;
  float last_theta = odom_theta;

  const uint32_t period_ms = 10;
  const float    dt        = 0.010f;   // 10 ms
  const uint64_t spin_timeout_ns = 2ULL * 1000000ULL; // 2 ms

  for(;;)
  {
	//Đọc trạng thái touch
	touch_x = g_touchState.x;
	touch_y = g_touchState.y;
	touch_pressed = g_touchState.pressed;

    // Đọc odometry (do task khác đã update odom_x/y/theta)
    float curr_x     = odom_x;
    float curr_y     = odom_y;
    float curr_theta = odom_theta;

    float dx  = curr_x     - last_x;
    float dy  = curr_y     - last_y;
    float dth = curr_theta - last_theta;

    odom_vx  = dx  / dt;
    odom_vy  = dy  / dt;
    odom_vth = dth / dt;

    last_x     = curr_x;
    last_y     = curr_y;
    last_theta = curr_theta;

    // Publish /odom
    microros_publish_odom();

    // Xử lý /cmd_vel (gọi callback nếu có data mới)
    rclc_executor_spin_some(&executor, spin_timeout_ns);

    osDelay(period_ms);
  }
}



void EncoderTask(void *argument)
{
	const TickType_t period = pdMS_TO_TICKS(ENCODER_PERIOD_MS);
	TickType_t lastWake = xTaskGetTickCount();

  for(;;)
  {
    // Đọc count mở rộng
	g_encCount1 = Encoder_GetCount(ENCODER_1);
    g_encCount2 = Encoder_GetCount(ENCODER_2);

    g_speed1_mps = Encoder_GetSpeed_mps(ENCODER_1,DT_TIME);
    g_speed2_mps = Encoder_GetSpeed_mps(ENCODER_2,DT_TIME);

    CurSpeed1 = g_speed1_mps;
    CurSpeed2 = g_speed2_mps;

    EncData_Publish(g_encCount1, g_encCount2, g_speed1_mps, g_speed2_mps);

    //CurPos1 = Encoder_GetPosition_m(ENCODER_1);
    //CurPos2 = Encoder_GetPosition_m(ENCODER_2);

    // Tính số vòng quay của motor (vòng)
    //g_revCount1 = (float)g_encCount1 / TICKS_PER_REV;
    //g_revCount2 = (float)g_encCount2 / TICKS_PER_REV;

    vTaskDelayUntil(&lastWake, period);
  }
}

void MotorControlTask(void *argument)
{
	const TickType_t period = pdMS_TO_TICKS(MOTORCONTROL_PERIOD_MS);
	TickType_t lastWake = xTaskGetTickCount();

	for (;;)
	    {
			//update thông số PID nếu có thay đổi
			if (pid_update_pending)
			{
				if (pid_mutex) xSemaphoreTake(pid_mutex, pdMS_TO_TICKS(2));

				PID_SetTunings(&SpeedPID1, Kp1, Ki1, Kd1);
				PID_SetTunings(&SpeedPID2, Kp2, Ki2, Kd2);
				PID_SetTunings(&PosPID1, Kp1, Ki1, Kd1);
				PID_SetTunings(&PosPID2, Kp2, Ki2, Kd2);

				pid_update_pending = 0;

				if (pid_mutex) xSemaphoreGive(pid_mutex);
			}

			if (modCtrl == MP_MODE_POSITION) {
				PID_Compute(&PosPID1);
				PID_Compute(&PosPID2);
				MotorData_Publish(PosPIDOut1, PosPIDOut2);  //update PWM data for LCD display
				Motor_SetSpeed(MOTOR_1,PosPIDOut1);
				Motor_SetSpeed(MOTOR_2,PosPIDOut2);
			} else {
				PID_Compute(&SpeedPID1);
				PID_Compute(&SpeedPID2);
				MotorData_Publish(SpeedPIDOut1, SpeedPIDOut2);  //update PWM data for LCD display
				Motor_SetSpeed(MOTOR_1,SpeedPIDOut1);
				Motor_SetSpeed(MOTOR_2,SpeedPIDOut2);
			}

//			if (millis_void() - last_cmdvel_ms > 500) {
//			    // hơn 500ms không nhận được lệnh -> dừng robot
//				PosPIDOut1 = 0;
//				PosPIDOut2 = 0;
//				SpeedPIDOut1 = 0;
//				SpeedPIDOut2 = 0;
//				Motor_SetSpeed(MOTOR_1,SpeedPIDOut1);
//				Motor_SetSpeed(MOTOR_2,SpeedPIDOut2);
//			}

			vTaskDelayUntil(&lastWake, period);
	    }
}

//void UartTask(void *argument)
//{
//	const TickType_t period = pdMS_TO_TICKS(UART_PERIOD_MS);
//	TickType_t lastWake = xTaskGetTickCount();
//
//    char buf[64];
//
//    for(;;)
//    {
//        int len = snprintf(buf, sizeof(buf),
//                          "%.3f %.3f\r\n",
//                          g_speed1_mps,
//                          g_speed2_mps);
//
//        HAL_UART_Transmit(&huart3, (uint8_t*)buf, len, 10);
//
//        vTaskDelayUntil(&lastWake, period);
//    }
//}

void OdometryTask(void *argument)
{
	const TickType_t period = pdMS_TO_TICKS(ODOMETRY_PERIOD_MS);
	TickType_t lastWake = xTaskGetTickCount();

    for (;;)
    {
        Odometry_Update();
        OdomData_Publish(odom_x, odom_y, odom_theta);

        vTaskDelayUntil(&lastWake, period);
    }
}

void yaw_to_quaternion(float yaw, geometry_msgs__msg__Quaternion *q)
{
    q->x = 0.0;
    q->y = 0.0;
    q->z = sinf(yaw * 0.5f);
    q->w = cosf(yaw * 0.5f);
}

static void microros_odom_init(rcl_node_t * node)
{
    nav_msgs__msg__Odometry__init(&odom_msg);

    rclc_publisher_init_default(
        &odom_pub,
        node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
        "odomfromSTM32"
    );

    rosidl_runtime_c__String__assign(&odom_msg.header.frame_id, "odom");
    rosidl_runtime_c__String__assign(&odom_msg.child_frame_id, "base_footprint");
}

static void microros_publish_odom(void)
{
    uint64_t now_ms = rmw_uros_epoch_millis();
    int32_t  sec    = (int32_t)(now_ms / 1000ULL);
    uint32_t nsec   = (uint32_t)((now_ms % 1000ULL) * 1000000ULL);

    // header
    odom_msg.header.stamp.sec     = sec;
    odom_msg.header.stamp.nanosec = nsec;

    // pose
    odom_msg.pose.pose.position.x = odom_x;
    odom_msg.pose.pose.position.y = odom_y;
    odom_msg.pose.pose.position.z = 0.0f;

    yaw_to_quaternion(odom_theta, &odom_msg.pose.pose.orientation);

    // twist
    odom_msg.twist.twist.linear.x  = odom_vx;
    odom_msg.twist.twist.linear.y  = odom_vy;
    odom_msg.twist.twist.angular.z = odom_vth;

    rcl_ret_t rc = rcl_publish(&odom_pub, &odom_msg, NULL);
    (void)rc;  // để compiler khỏi báo unused-variable
}

static inline uint32_t millis_void(void)
{
    return HAL_GetTick();
}

// ===== Callback /cmd_vel: tính DesiredSpeed1/2 cho PID =====
static void cmdvel_cb(const void * msgin)
{
    const geometry_msgs__msg__Twist *m = (const geometry_msgs__msg__Twist *) msgin;

    float v = m->linear.x;   // m/s
    float w = m->angular.z;  // rad/s

    // diff-drive: v_r = v + w*L/2 ; v_l = v - w*L/2
    float v_r = v + (w * WHEEL_BASE_M * 0.5f);
    float v_l = v - (w * WHEEL_BASE_M * 0.5f);

    DesiredSpeed1 = v_r;
    DesiredSpeed2 = v_l;

    last_cmdvel_ms = millis_void();
}

//hàm callback khi nhận được data PID từ ROS
static void pid_cb(const void * msgin)
{
    const std_msgs__msg__Float64MultiArray *m =
        (const std_msgs__msg__Float64MultiArray *)msgin;

    // cần đủ 6 phần tử
    if (m->data.size < 6) return;

    if (pid_mutex) xSemaphoreTake(pid_mutex, pdMS_TO_TICKS(2));

    Kp1 = m->data.data[0];
    Ki1 = m->data.data[1];
    Kd1 = m->data.data[2];
    Kp2 = m->data.data[3];
    Ki2 = m->data.data[4];
    Kd2 = m->data.data[5];

    pid_update_pending = 1;

    if (pid_mutex) xSemaphoreGive(pid_mutex);
}

void MotionTask(void *argument)
{
	const TickType_t period = pdMS_TO_TICKS(MOTORCONTROL_PERIOD_MS);
	TickType_t lastWake = xTaskGetTickCount();

	MP_Config_t mp_cfg1, mp_cfg2;
	MP_State_t  mp_state1, mp_state2;

    mp_cfg1.dt      = DT_TIME;     // 5 ms
    mp_cfg1.vmax    = MAXSPEED;   // your units
    mp_cfg1.amax    = MAXACC;
    mp_cfg1.vel_eps = 0.01f * MAXSPEED; // tune
    mp_cfg1.pos_eps = 0.001f;          // tune (depends on units)

    mp_cfg2.dt      = DT_TIME;     // 5 ms
    mp_cfg2.vmax    = MAXSPEED;   // your units
    mp_cfg2.amax    = MAXACC;
    mp_cfg2.vel_eps = 0.01f * MAXSPEED; // tune
    mp_cfg2.pos_eps = 0.001f;          // tune (depends on units)

    // Initialize using current measured state:
    MP_Init(&mp_state1, CurPos1, CurSpeed1);
    MP_Init(&mp_state2, CurPos2, CurSpeed2);

    int   prevMode = modCtrl;

    float prevSetPos1 = SetPos1, prevSetPos2 = SetPos2;
    float prevSetSpd1 = SetSpeed1, prevSetSpd2 = SetSpeed2;

    for (;;)
    {
    	 // ---- Snapshot inputs (avoid reading changing globals mid-loop) ----
    	int   mode = modCtrl;
        float curPos1 = CurPos1, curPos2 = CurPos2;
        float curSpd1 = CurSpeed1, curSpd2 = CurSpeed2;

        float setPos1 = SetPos1, setPos2 = SetPos2;
        float setSpd1 = SetSpeed1, setSpd2 = SetSpeed2;

        // ---- Determine whether to re-seed planner state (ONE-TIME) ----
        uint8_t modeChanged = (mode != prevMode);

        uint8_t posCmdJump =
            (fabsf_fast(setPos1 - prevSetPos1) > POS_CMD_RESYNC_TH) ||
            (fabsf_fast(setPos2 - prevSetPos2) > POS_CMD_RESYNC_TH);

        uint8_t spdCmdJump =
            (fabsf_fast(setSpd1 - prevSetSpd1) > SPD_CMD_RESYNC_TH) ||
            (fabsf_fast(setSpd2 - prevSetSpd2) > SPD_CMD_RESYNC_TH);

        // Choose resync conditions based on mode
        uint8_t needResync = modeChanged;

        if (mode == MP_MODE_POSITION) {
            needResync = needResync || posCmdJump;
        } else {
            needResync = needResync || spdCmdJump;
        }

        if (needResync) {
            // Re-seed once from measured state so profile starts from reality
            mp_state1.pos_ref = curPos1;
            mp_state1.vel_ref = curSpd1;   // set to measured if your speed is OK; otherwise set to mp_state1.vel_ref
            mp_state2.pos_ref = curPos2;
            mp_state2.vel_ref = curSpd2;
        }

        // ---- Step profile ----
        MP_Output_t ref1, ref2;

        if (modCtrl == MP_MODE_POSITION) {
        	ref1 = MP_StepPosition(&mp_cfg1, &mp_state1, setPos1);
        	ref2 = MP_StepPosition(&mp_cfg2, &mp_state2, setPos2);
            DesiredPos1 = ref1.pos_next;
            DesiredPos2 = ref2.pos_next;
        } else {
        	ref1 = MP_StepVelocity(&mp_cfg1, &mp_state1, setSpd1);
        	ref2 = MP_StepVelocity(&mp_cfg2, &mp_state2, setSpd2);
            DesiredSpeed1 = ref1.vel_next;
            DesiredSpeed2 = ref2.vel_next;
        }

        // ---- Save previous command/mode for next loop ----
		prevMode = mode;
		prevSetPos1 = setPos1;  prevSetPos2 = setPos2;
		prevSetSpd1 = setSpd1;  prevSetSpd2 = setSpd2;

        // v_ref goes into your PID controller later
        // PID_Update(v_ref, measured_speed);

        vTaskDelayUntil(&lastWake, period);
    }
}

//void StartUsbHostTask(void *argument)
//{
//  for(;;)
//  {
//    USBH_Process(&hUsbHostFS);
//    osDelay(1);
//  }
//}
//
void StartPS2Task(void *argument)
{
  (void)argument;

  // Init library (HAL-based, not RTOS-aware)
  PS2X_Init(&ps2, &hspi2);

  // Configure controller: analog + rumble enabled
  bool cfg_ok = PS2X_ConfigGamepad(&ps2, true, true);

  for (;;)
  {
    // Poll controller
    bool ok = cfg_ok && PS2X_ReadGamepad(&ps2, 0, 0);

    // Build snapshot
    PS2X_State st;
    st.tick   = osKernelGetTickCount();
    st.ok     = ok ? 1 : 0;
    st.id     = ps2.id;
    st.buttons= ps2.buttons;
    st.analog = ps2.analog_enabled ? 1 : 0;
    st.rx     = ps2.analog[PSS_RX];
    st.ry     = ps2.analog[PSS_RY];
    st.lx     = ps2.analog[PSS_LX];
    st.ly     = ps2.analog[PSS_LY];

    // Publish snapshot for other tasks
    if (osMutexAcquire(g_ps2_mutex, 2) == osOK) {
      g_ps2_state = st;
      osMutexRelease(g_ps2_mutex);
    }

    // Poll period
    osDelay(10);   // 10ms is typical; adjust if you want
  }
}

void StartTeleopTask(void *argument)
{
    (void)argument;

    for (;;)
    {
        // ---- snapshot controller state ----
        PS2X_State st;
        bool got = false;
        if (osMutexAcquire(g_ps2_mutex, 2) == osOK) {
            st = g_ps2_state;
            osMutexRelease(g_ps2_mutex);
            got = true;
        }

        uint32_t now = osKernelGetTickCount();

        // Default lock
        float v_cmd = 0.0f;
        float w_cmd = 0.0f;
        bool enabled = false;

        if (got && st.ok) {
            // timeout check
            uint32_t age = now - st.tick;
            if (age <= PS2_TIMEOUT_MS) {

                // L1 enable
                if (is_pressed(st.buttons, PSB_L1)) {
                    enabled = true;

                    // ---- decide analog vs d-pad ----
                    int16_t rx = (int16_t)st.rx - 128;
                    int16_t ry = (int16_t)st.ry - 128;

                    if (abs(rx) < STICK_DEADBAND) rx = 0;
                    if (abs(ry) < STICK_DEADBAND) ry = 0;

                    bool analog_active = (rx != 0) || (ry != 0);

                    if (analog_active) {
                        // Analog mode: proportional
                        float rx_n = (float)rx / 127.0f;      // -1..1
                        float ry_n = (float)ry / 127.0f;      // -1..1

                        // Forward: usually pushing up gives ry negative => invert
                        v_cmd = (-ry_n) * V_MAX;
                        w_cmd = ( rx_n) * W_MAX;

                    } else {
                        // D-pad mode: only one effective
                        bool up    = is_pressed(st.buttons, PSB_UP);
                        bool down  = is_pressed(st.buttons, PSB_DOWN);
                        bool left  = is_pressed(st.buttons, PSB_LEFT);
                        bool right = is_pressed(st.buttons, PSB_RIGHT);

                        // Priority choice (simple):
                        if (up && !down && !left && !right) {
                            v_cmd = +V_STEP;  w_cmd = 0.0f;
                        } else if (down && !up && !left && !right) {
                            v_cmd = -V_STEP;  w_cmd = 0.0f;
                        } else if (left && !up && !down && !right) {
                            v_cmd = 0.0f;     w_cmd = -W_STEP;
                        } else if (right && !up && !down && !left) {
                            v_cmd = 0.0f;     w_cmd = +W_STEP;
                        } else {
                            // none or multiple => stop (safer)
                            v_cmd = 0.0f;     w_cmd = 0.0f;
                        }
                    }
                }
            }
        }

        // If not enabled or timeout => lock (0,0)
        if (!enabled) {
            v_cmd = 0.0f;
            w_cmd = 0.0f;
        }

        // ---- convert cmd_vel to wheel linear speeds ----
        float v_r = v_cmd + (w_cmd * WHEEL_BASE_M * 0.5f);
        float v_l = v_cmd - (w_cmd * WHEEL_BASE_M * 0.5f);

        // Optional clamp per wheel to V_MAX
        v_r = clampf(v_r, -V_MAX, +V_MAX);
        v_l = clampf(v_l, -V_MAX, +V_MAX);

        // Publish to planner
        SetSpeed1 = v_l;   // left
        SetSpeed2 = v_r;   // right

        osDelay(10); // teleop update at 10ms is fine
    }
}

static inline void EncData_Publish(int32_t enc1, int32_t enc2, float v1, float v2)
{
  g_encSeq++; __DMB();
  g_encData.enc1 = enc1;
  g_encData.enc2 = enc2;
  g_encData.v1   = v1;
  g_encData.v2   = v2;
  __DMB(); g_encSeq++;
}

static inline void OdomData_Publish(float x, float y, float yaw)
{
  g_odomSeq++; __DMB();
  g_odomData.x   = x;
  g_odomData.y   = y;
  g_odomData.yaw = yaw;
  __DMB(); g_odomSeq++;
}

static inline void MotorData_Publish(float pwm1, float pwm2)
{
  g_motorSeq++; __DMB();
  g_motorData.pwm1 = pwm1;
  g_motorData.pwm2 = pwm2;
  __DMB(); g_motorSeq++;
}

static inline void EncData_Read(EncData_t* out)
{
  uint32_t s1, s2;
  do {
    s1 = g_encSeq; __DMB();
    *out = g_encData; __DMB();
    s2 = g_encSeq;
  } while ((s1 != s2) || (s1 & 1u));
}

static inline void OdomData_Read(OdomData_t* out)
{
  uint32_t s1, s2;
  do {
    s1 = g_odomSeq; __DMB();
    *out = g_odomData; __DMB();
    s2 = g_odomSeq;
  } while ((s1 != s2) || (s1 & 1u));
}

static inline void MotorData_Read(MotorData_t* out)
{
  uint32_t s1, s2;
  do {
    s1 = g_motorSeq; __DMB();
    *out = g_motorData; __DMB();
    s2 = g_motorSeq;
  } while ((s1 != s2) || (s1 & 1u));
}

static void lcdUpdateCell2Lines(uint16_t cx, uint16_t cy, uint16_t cw, uint16_t ch,
                                const char* l1, const char* l2,
                                uint16_t color, uint16_t bg,
                                uint16_t charW, uint16_t fontH, uint16_t P)
{
    int m = (int)((cw - 2*P) / charW); if (m < 1) m = 1;

    int len1 = (int)strlen(l1); if (len1 > m) len1 = m;
    int len2 = (int)strlen(l2); if (len2 > m) len2 = m;

    int w1 = len1 * (int)charW;
    int w2 = len2 * (int)charW;
    int wMax = (w1 > w2) ? w1 : w2;

    int16_t x = (int16_t)cx + (int16_t)((cw - wMax)/2);
    int16_t y = (int16_t)cy + (int16_t)((ch - 2*(int16_t)fontH)/2);

    if (x < (int16_t)cx + (int16_t)P) x = (int16_t)cx + (int16_t)P;
    if (y < (int16_t)cy + (int16_t)P) y = (int16_t)cy + (int16_t)P;

    // clear vùng 2 dòng
    lcdFillRect(cx + 1, (uint16_t)y, cw - 2, 2*fontH, bg);

    lcdSetTextColor(color, bg);
    lcdSetCursor((unsigned short)x, (unsigned short)y);
    lcdPrintf("%.*s", len1, l1);

    lcdSetCursor((unsigned short)x, (unsigned short)(y + fontH));
    lcdPrintf("%.*s", len2, l2);
}

static void lcdUpdateAllValues_FromSnapshots(void)
{
    // --- đọc snapshot (nhanh, không block) ---
    EncData_t   enc;
    OdomData_t  od;
    MotorData_t mot;

    EncData_Read(&enc);
    OdomData_Read(&od);
    MotorData_Read(&mot);

    // --- tính grid y hệt locLCD() ---
    uint16_t W = lcdGetWidth();
    uint16_t H = lcdGetHeight();

    uint16_t fontH = lcdGetTextFont()->Height;
    uint16_t charW = lcdGetTextFont()->Width;
    if (charW == 0) charW = 8;

    uint16_t M  = 6;
    uint16_t GX = M;
    uint16_t GY = 22;
    uint16_t GW = W - 2*M;
    uint16_t GH = H - GY - M;

    uint16_t CW = GW / 3;
    uint16_t RH = GH / 4;
    uint16_t P  = 6;

    #define CELL_X(c) (GX + (c)*CW)
    #define CELL_Y(r) (GY + (r)*RH)

    // --- format string ---
    char s_pwm1[16], s_pwm2[16];
    char s_enc1[16], s_enc2[16];
    char s_v1[16],   s_v2[16];
    char s_x[16],    s_y[16], s_yaw[16];

    snprintf(s_pwm1, sizeof(s_pwm1), "%.2f", mot.pwm1);
    snprintf(s_pwm2, sizeof(s_pwm2), "%.2f", mot.pwm2);
    snprintf(s_enc1, sizeof(s_enc1), "%ld", (long)enc.enc1);
    snprintf(s_enc2, sizeof(s_enc2), "%ld", (long)enc.enc2);
    snprintf(s_v1,   sizeof(s_v1),   "%.2f", enc.v1);
    snprintf(s_v2,   sizeof(s_v2),   "%.2f", enc.v2);
    snprintf(s_x,    sizeof(s_x),    "%.2f", od.x);
    snprintf(s_y,    sizeof(s_y),    "%.2f", od.y);
    snprintf(s_yaw,  sizeof(s_yaw),  "%.2f", od.yaw);

    // --- update các ô (đúng như UI của bạn) ---
    lcdUpdateCell2Lines(CELL_X(0), CELL_Y(0), CW, RH, "PWM1",  s_pwm1, COLOR_CYAN, COLOR_BLACK, charW, fontH, P);
    lcdUpdateCell2Lines(CELL_X(0), CELL_Y(1), CW, RH, "PWM2",  s_pwm2, COLOR_CYAN, COLOR_BLACK, charW, fontH, P);
    lcdUpdateCell2Lines(CELL_X(0), CELL_Y(2), CW, RH, "Enc1",  s_enc1, COLOR_CYAN, COLOR_BLACK, charW, fontH, P);
    lcdUpdateCell2Lines(CELL_X(0), CELL_Y(3), CW, RH, "Enc2",  s_enc2, COLOR_CYAN, COLOR_BLACK, charW, fontH, P);

    lcdUpdateCell2Lines(CELL_X(1), CELL_Y(0), CW, RH, "V1",    s_v1,   COLOR_CYAN, COLOR_BLACK, charW, fontH, P);
    lcdUpdateCell2Lines(CELL_X(1), CELL_Y(1), CW, RH, "V2",    s_v2,   COLOR_CYAN, COLOR_BLACK, charW, fontH, P);

    lcdUpdateCell2Lines(CELL_X(2), CELL_Y(0), CW, RH, "Odom X", s_x,    COLOR_CYAN, COLOR_BLACK, charW, fontH, P);
    lcdUpdateCell2Lines(CELL_X(2), CELL_Y(1), CW, RH, "Odom Y", s_y,    COLOR_CYAN, COLOR_BLACK, charW, fontH, P);
    lcdUpdateCell2Lines(CELL_X(2), CELL_Y(2), CW, RH, "Yaw",    s_yaw,  COLOR_CYAN, COLOR_BLACK, charW, fontH, P);

    #undef CELL_X
    #undef CELL_Y
}

void StartLCDTask(void *argument)
{
    (void)argument;

    //locLCD(); // vẽ UI tĩnh 1 lần

    const uint32_t period_ms = 50; // 40Hz
    uint32_t tick = osKernelGetTickCount();

// for (;;)
//    {
//        tick += period_ms;
//        uint32_t t0 = osKernelGetTickCount();
//        lcdUpdateAllValues_FromSnapshots();
//        uint32_t t1 = osKernelGetTickCount();
//        uint32_t dt_ticks = t1 - t0;
//        osDelayUntil(tick);
//    }
    for (;;)
    {
        osDelay(500);   // tạm thời không update gì
    }

}
/* USER CODE END Application */

