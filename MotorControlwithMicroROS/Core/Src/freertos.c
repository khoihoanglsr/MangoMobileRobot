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
#include "encoder_speed.h"
#include "pid.h"
#include "odometry.h"
#include "semphr.h"
#include "touch_rtos.h"
#include "spi.h"
#include "touch_event.h"
#include "xpt2046.h"
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

static volatile uint8_t pid_update_pending = 0;  //biến dùng để update PID từ ROS
static SemaphoreHandle_t pid_mutex;  //kỹ thuật cho phép tạm thời lock trong quá trình thay đổi biến PID, tránh bị cập nhật nửa chừng

volatile uint16_t touch_x = 0;
volatile uint16_t touch_y = 0;
volatile uint8_t  touch_pressed = 0;

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
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
//osThreadId_t PS2TaskHandle;

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
//const osThreadAttr_t PS2Task_attributes = {
//  .name       = "PS2Task",
//  .stack_size = 512 * 4,   // 2KB stack (đủ cho parse HID)
//  .priority   = (osPriority_t) osPriorityNormal
//};


void EncoderTask(void *argument);
void MotorControlTask(void *argument);
//void UartTask(void *argument);
void OdometryTask(void *argument);

void TouchTask(void *argument);
//void StartUsbHostTask(void *argument);
//void StartPS2Task(void *argument);

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

  encoderTaskHandle = osThreadNew(EncoderTask, NULL, &encoderTask_attributes);
  motorTaskHandle   = osThreadNew(MotorControlTask, NULL, &motorTask_attributes);
//  uartTaskHandle    = osThreadNew(UartTask, NULL, &uartTask_attributes);
  odometryTaskHandle = osThreadNew(OdometryTask, NULL, &odometryTask_attributes);

  touchTaskHandle = osThreadNew(TouchTask_Start, NULL, &touchTask_attributes);
//  UsbHostTaskHandle = osThreadNew(StartUsbHostTask, NULL, &UsbHostTask_attributes);
//  PS2TaskHandle = osThreadNew(StartPS2Task, NULL, &PS2Task_attributes);

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
//void StartDefaultTask(void *argument)  ///this is for debugging
//{
//	// Đợi hệ thống ổn 1 chút
//	  osDelay(1000);
//
//	  // Báo qua UART là default task đã start
//	  uint8_t msg[] = "DEFAULT TASK START\r\n";
//	  HAL_UART_Transmit(&huart3, msg, sizeof(msg)-1, 100);
//
//	  for (;;)
//	  {
//	    // Toggle cả 3 LED trên NUCLEO-F756ZG
//	    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);   // LD1 - Green
//	    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_7);   // LD2 - Blue
//	    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);  // LD3 - Red
//
//	    osDelay(500); // 500 ms
//	  }
//}
void StartDefaultTask(void *argument)
{

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET); // LED1 sáng luôn
  // ==== micro-ROS transport config ====
  rmw_uros_set_custom_transport(
    true,
    (void *) &huart2,
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
/* USER CODE END Header_StartDefaultTask */

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
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

    CurPos1 = Encoder_GetPosition_m(ENCODER_1);
    CurPos2 = Encoder_GetPosition_m(ENCODER_2);

    // Tính số vòng quay của motor (vòng)
    g_revCount1 = (float)g_encCount1 / TICKS_PER_REV;
    g_revCount2 = (float)g_encCount2 / TICKS_PER_REV;

    vTaskDelayUntil(&lastWake, period);
  }
}

void MotorControlTask(void *argument)
{
	const TickType_t period = pdMS_TO_TICKS(MOTORCONTROL_PERIOD_MS);
	TickType_t lastWake = xTaskGetTickCount();

	DesiredSpeed1 = 0;
	DesiredSpeed2 = 0;

	for (;;)
	    {
			//update thông số PID nếu có thay đổi
			if (pid_update_pending)
			{
				if (pid_mutex) xSemaphoreTake(pid_mutex, pdMS_TO_TICKS(2));

				PID_SetTunings(&SpeedPID1, Kp1, Ki1, Kd1);
				PID_SetTunings(&SpeedPID2, Kp2, Ki2, Kd2);

				pid_update_pending = 0;

				if (pid_mutex) xSemaphoreGive(pid_mutex);
			}
			PID_Compute(&SpeedPID1);
			PID_Compute(&SpeedPID2);

			Motor_SetSpeed(MOTOR_1,SpeedPIDOut1);
			Motor_SetSpeed(MOTOR_2,SpeedPIDOut2);

			if (millis_void() - last_cmdvel_ms > 500) {
			    // hơn 500ms không nhận được lệnh -> dừng robot
			    DesiredSpeed1 = 0;
			    DesiredSpeed2 = 0;
			}

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

//void StartUsbHostTask(void *argument)
//{
//  for(;;)
//  {
//    USBH_Process(&hUsbHostFS);
//    osDelay(1);
//  }
//}
//
//void StartPS2Task(void *argument)
//{
//  PS2_HID_Init();
//
//  for(;;)
//  {
//    PS2_HID_ProcessReports();
//    osDelay(2); // 2ms là đủ mượt
//  }
//}
/* USER CODE END Application */

