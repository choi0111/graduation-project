/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.cpp
  * @brief          : ROS rosserial Open-Loop + Limited P Speed Correction
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"

/* USER CODE BEGIN Includes */
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point32.h>
#include <stdint.h>
/* USER CODE END Includes */

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* ================= Robot constants ================= */

#define WHEEL_BASE_M          0.695f

#define WHEEL_DIAMETER_M      0.125f
#define WHEEL_CIRCUM_M        (3.1415926f * WHEEL_DIAMETER_M)

#define LEFT_COUNTS_PER_REV   3788.0f
#define RIGHT_COUNTS_PER_REV  3691.0f

#define MAX_LINEAR_X_MPS      0.10f
#define MAX_ANGULAR_Z_RADPS   0.20f

#define PWM_MAX_CCR           49

/*
 * STM 단독 테스트 기준
 * 전진 최대: 25
 * 후진 최대: 15
 */
#define FORWARD_PWM_LIMIT      25
#define BACKWARD_PWM_LIMIT     15

#define PWM_OUTPUT_LIMIT       25

/*
 * 기본 최소 구동 PWM
 */
#define PWM_MIN_MOVE           12

/*
 * 후진 전용 최소 PWM
 */
#define LEFT_BWD_MIN_PWM       12
#define RIGHT_BWD_MIN_PWM      12

#define CONTROL_PERIOD_MS      50
#define CONTROL_DT_SEC         0.05f

/*
 * 50ms마다 PWM 1씩 변화
 * 급격한 전류 상승 방지
 */
#define PWM_STEP               1
#define CMD_TIMEOUT_MS         500

/*
 * 전진/후진 출력 보정
 *
 * 기존 후진 TRIM 0.75는 후진 PWM 차이를 죽였기 때문에 제거.
 * 후진에서도 12~15 범위가 살아있도록 1.00으로 변경.
 */
#define LEFT_FWD_TRIM          1.00f
#define RIGHT_FWD_TRIM         1.00f
#define LEFT_BWD_TRIM          1.00f
#define RIGHT_BWD_TRIM         1.00f

/*
 * 전진 + 좌회전 보정
 *
 * STM 단독 테스트에서 전진 좌회전은 LEFT 12 / RIGHT 25가 적절했음.
 * ROS 계산상 왼쪽이 약 15 정도 나올 수 있으므로 0.80을 곱해 약 12로 맞춤.
 */
#define FWD_LEFT_TURN_LEFT_TRIM      0.80f
#define FWD_LEFT_TURN_LEFT_MIN_PWM   12

/*
 * 후진 중 오른쪽 모터가 더 빨라져야 하는 상황 보정
 *
 * STM 단독 테스트에서 후진 좌회전은 LEFT 12 / RIGHT 14가 만족스러웠음.
 * 따라서 오른쪽 후진 fast 상황은 14까지만 제한.
 */
#define REV_RIGHT_FAST_LIMIT         14

#define PID_KP                 60.0f
#define PID_KI                 0.0f
#define PID_KD                 0.0f

#define PID_CORRECTION_LIMIT   4
#define PID_INTEGRAL_LIMIT     3.0f

#define LEFT_ENCODER_SIGN      1
#define RIGHT_ENCODER_SIGN    -1

#define RIGHT_PWM_CHANNEL      TIM_CHANNEL_1   // PA6
#define LEFT_PWM_CHANNEL       TIM_CHANNEL_2   // PA7

#define RIGHT_DIR_GPIO_PORT    GPIOB
#define RIGHT_DIR_PIN          GPIO_PIN_0

#define LEFT_DIR_GPIO_PORT     GPIOB
#define LEFT_DIR_PIN           GPIO_PIN_1

ros::NodeHandle nh;

/*
 * x = left ticks
 * y = right ticks
 * z = 0
 */
geometry_msgs::Point32 wheel_ticks_msg;
ros::Publisher ticks_pub("wheel_ticks", &wheel_ticks_msg);

static int16_t prevL = 0;
static int16_t prevR = 0;

static int32_t posL = 0;
static int32_t posR = 0;

static float measured_left_mps = 0.0f;
static float measured_right_mps = 0.0f;

static volatile float cmd_linear_x = 0.0f;
static volatile float cmd_angular_z = 0.0f;

static volatile float target_left_mps = 0.0f;
static volatile float target_right_mps = 0.0f;

static volatile uint32_t last_cmd_time = 0;

static int base_left_pwm = 0;
static int base_right_pwm = 0;

static int desired_left_pwm = 0;
static int desired_right_pwm = 0;

static int current_left_pwm = 0;
static int current_right_pwm = 0;

static float left_integral = 0.0f;
static float right_integral = 0.0f;

static float left_prev_error = 0.0f;
static float right_prev_error = 0.0f;

/* USER CODE END PV */

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);

/* USER CODE BEGIN PFP */
static float clamp_float(float value, float min_value, float max_value);
static float abs_float(float value);
static int abs_int(int value);
static int clamp_int(int value, int min_value, int max_value);
static int approach_int(int current, int target, int step);

static uint32_t pct_to_ccr(uint8_t pct);

static void set_right_pwm_pct(uint8_t pct);
static void set_left_pwm_pct(uint8_t pct);

static void right_dir(uint8_t rev);
static void left_dir(uint8_t rev);

static int speed_to_signed_pwm(float target_mps);
static int limited_pid_correction(float target_mps,
                                  float measured_mps,
                                  float *integral,
                                  float *prev_error);

static void set_left_motor_signed(int signed_pwm);
static void set_right_motor_signed(int signed_pwm);
static void motors_stop(void);
static void apply_open_loop_limited_pid_outputs(void);

static void UpdateEncoderTicksAndSpeed(void);
static void PublishEncoderTicks(void);
static void reset_pid(void);

void cmdVelCallback(const geometry_msgs::Twist& cmd_msg);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

static float clamp_float(float value, float min_value, float max_value)
{
  if (value > max_value) return max_value;
  if (value < min_value) return min_value;
  return value;
}

static float abs_float(float value)
{
  return (value < 0.0f) ? -value : value;
}

static int abs_int(int value)
{
  return (value < 0) ? -value : value;
}

static int clamp_int(int value, int min_value, int max_value)
{
  if (value > max_value) return max_value;
  if (value < min_value) return min_value;
  return value;
}

static int approach_int(int current, int target, int step)
{
  if (target > current + step)
  {
    return current + step;
  }

  if (target < current - step)
  {
    return current - step;
  }

  return target;
}

/*
 * TIM3 ARR 기준 percent -> CCR 변환
 */
static uint32_t pct_to_ccr(uint8_t pct)
{
  if (pct > 100) pct = 100;

  uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim3);
  return (arr + 1U) * pct / 100U;
}

static void set_right_pwm_pct(uint8_t pct)
{
  __HAL_TIM_SET_COMPARE(&htim3, RIGHT_PWM_CHANNEL, pct_to_ccr(pct));
}

static void set_left_pwm_pct(uint8_t pct)
{
  __HAL_TIM_SET_COMPARE(&htim3, LEFT_PWM_CHANNEL, pct_to_ccr(pct));
}

/*
 * rev = 0 -> forward
 * rev = 1 -> reverse
 */
static void right_dir(uint8_t rev)
{
  HAL_GPIO_WritePin(RIGHT_DIR_GPIO_PORT,
                    RIGHT_DIR_PIN,
                    rev ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void left_dir(uint8_t rev)
{
  HAL_GPIO_WritePin(LEFT_DIR_GPIO_PORT,
                    LEFT_DIR_PIN,
                    rev ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/*
 * target_mps -> signed PWM
 *
 * 전진: 12 ~ 25
 * 후진: 12 ~ 15
 */
static int speed_to_signed_pwm(float target_mps)
{
  float speed_abs = abs_float(target_mps);

  if (speed_abs < 0.002f)
  {
    return 0;
  }

  int limit = (target_mps >= 0.0f) ? FORWARD_PWM_LIMIT : BACKWARD_PWM_LIMIT;

  if (limit < PWM_MIN_MOVE)
  {
    limit = PWM_MIN_MOVE;
  }

  int pwm_abs = PWM_MIN_MOVE
              + (int)((speed_abs / MAX_LINEAR_X_MPS)
              * (float)(limit - PWM_MIN_MOVE));

  pwm_abs = clamp_int(pwm_abs, PWM_MIN_MOVE, limit);

  if (target_mps >= 0.0f)
  {
    return pwm_abs;
  }
  else
  {
    return -pwm_abs;
  }
}

/*
 * 제한 PID 보정
 * 현재는 P만 사용하고 보정량은 -1 ~ +1로 제한.
 */
static int limited_pid_correction(float target_mps,
                                  float measured_mps,
                                  float *integral,
                                  float *prev_error)
{
  if (abs_float(target_mps) < 0.002f)
  {
    *integral = 0.0f;
    *prev_error = 0.0f;
    return 0;
  }

  float error = target_mps - measured_mps;

  *integral += error * CONTROL_DT_SEC;
  *integral = clamp_float(*integral,
                          -PID_INTEGRAL_LIMIT,
                          PID_INTEGRAL_LIMIT);

  float derivative = (error - *prev_error) / CONTROL_DT_SEC;
  *prev_error = error;

  float correction_f = PID_KP * error
                     + PID_KI * (*integral)
                     + PID_KD * derivative;

  int correction = (int)correction_f;

  correction = clamp_int(correction,
                         -PID_CORRECTION_LIMIT,
                         PID_CORRECTION_LIMIT);

  return correction;
}

/*
 * 왼쪽 모터 출력
 */
static void set_left_motor_signed(int signed_pwm)
{
  int pwm_abs = abs_int(signed_pwm);

  if (signed_pwm > 0)
  {
    pwm_abs = (int)((float)pwm_abs * LEFT_FWD_TRIM);

    /*
     * 전진+좌회전에서는 왼쪽을 약 12 근처로 유지하기 위해
     * 좌회전 전용 최소 PWM을 사용.
     */
    if ((cmd_linear_x > 0.002f) && (cmd_angular_z > 0.002f))
    {
      if (pwm_abs > 0 && pwm_abs < FWD_LEFT_TURN_LEFT_MIN_PWM)
      {
        pwm_abs = FWD_LEFT_TURN_LEFT_MIN_PWM;
      }
    }
    else
    {
      if (pwm_abs > 0 && pwm_abs < PWM_MIN_MOVE)
      {
        pwm_abs = PWM_MIN_MOVE;
      }
    }

    pwm_abs = clamp_int(pwm_abs, 0, FORWARD_PWM_LIMIT);
    left_dir(0);
  }
  else if (signed_pwm < 0)
  {
    pwm_abs = (int)((float)pwm_abs * LEFT_BWD_TRIM);

    if (pwm_abs > 0 && pwm_abs < LEFT_BWD_MIN_PWM)
    {
      pwm_abs = LEFT_BWD_MIN_PWM;
    }

    pwm_abs = clamp_int(pwm_abs, 0, BACKWARD_PWM_LIMIT);
    left_dir(1);
  }
  else
  {
    pwm_abs = 0;
  }

  set_left_pwm_pct((uint8_t)pwm_abs);
}

/*
 * 오른쪽 모터 출력
 */
static void set_right_motor_signed(int signed_pwm)
{
  int pwm_abs = abs_int(signed_pwm);

  if (signed_pwm > 0)
  {
    pwm_abs = (int)((float)pwm_abs * RIGHT_FWD_TRIM);

    if (pwm_abs > 0 && pwm_abs < PWM_MIN_MOVE)
    {
      pwm_abs = PWM_MIN_MOVE;
    }

    pwm_abs = clamp_int(pwm_abs, 0, FORWARD_PWM_LIMIT);
    right_dir(0);
  }
  else if (signed_pwm < 0)
  {
    pwm_abs = (int)((float)pwm_abs * RIGHT_BWD_TRIM);

    if (pwm_abs > 0 && pwm_abs < RIGHT_BWD_MIN_PWM)
    {
      pwm_abs = RIGHT_BWD_MIN_PWM;
    }

    pwm_abs = clamp_int(pwm_abs, 0, BACKWARD_PWM_LIMIT);
    right_dir(1);
  }
  else
  {
    pwm_abs = 0;
  }

  set_right_pwm_pct((uint8_t)pwm_abs);
}

static void reset_pid(void)
{
  left_integral = 0.0f;
  right_integral = 0.0f;

  left_prev_error = 0.0f;
  right_prev_error = 0.0f;
}

static void motors_stop(void)
{
  base_left_pwm = 0;
  base_right_pwm = 0;

  desired_left_pwm = 0;
  desired_right_pwm = 0;

  current_left_pwm = 0;
  current_right_pwm = 0;

  reset_pid();

  set_left_pwm_pct(0);
  set_right_pwm_pct(0);
}

static void apply_open_loop_limited_pid_outputs(void)
{
  if (HAL_GetTick() - last_cmd_time > CMD_TIMEOUT_MS)
  {
    motors_stop();
    return;
  }

  base_left_pwm = speed_to_signed_pwm(target_left_mps);
  base_right_pwm = speed_to_signed_pwm(target_right_mps);

  int left_corr = limited_pid_correction(target_left_mps,
                                         measured_left_mps,
                                         &left_integral,
                                         &left_prev_error);

  int right_corr = limited_pid_correction(target_right_mps,
                                          measured_right_mps,
                                          &right_integral,
                                          &right_prev_error);

  desired_left_pwm = base_left_pwm + left_corr;
  desired_right_pwm = base_right_pwm + right_corr;

  /*
   * 전진 + 좌회전 보정
   *
   * STM 단독 테스트의 LEFT 12 / RIGHT 25 느낌을 맞추기 위해
   * 오른쪽은 직진 기준 PWM까지만 유지하고,
   * 왼쪽은 trim을 적용해 낮춘다.
   */
  /* Keep cmd_vel -> wheel PWM symmetric for DWA odometry consistency. */

  /*
   * 후진 중 오른쪽 모터가 더 빨라지는 상황 보정
   *
   * 예: 후진 중 좌회전처럼 오른쪽 모터가 더 많이 돌아야 하는 경우
   * 기존 ROS 계산은 오른쪽이 15까지 갈 수 있음.
   * STM 단독 테스트에서 만족스러웠던 RIGHT 14에 맞춰 제한.
   *
   * 조건:
   * - 왼쪽, 오른쪽 모두 후진
   * - 오른쪽의 절댓값 PWM이 왼쪽보다 큼
   */
  /* Do not add direction-specific clamps here. */

  if (desired_left_pwm > 0)
  {
    desired_left_pwm = clamp_int(desired_left_pwm,
                                 -FORWARD_PWM_LIMIT,
                                  FORWARD_PWM_LIMIT);
  }
  else if (desired_left_pwm < 0)
  {
    desired_left_pwm = clamp_int(desired_left_pwm,
                                 -BACKWARD_PWM_LIMIT,
                                  BACKWARD_PWM_LIMIT);
  }

  if (desired_right_pwm > 0)
  {
    desired_right_pwm = clamp_int(desired_right_pwm,
                                  -FORWARD_PWM_LIMIT,
                                   FORWARD_PWM_LIMIT);
  }
  else if (desired_right_pwm < 0)
  {
    desired_right_pwm = clamp_int(desired_right_pwm,
                                  -BACKWARD_PWM_LIMIT,
                                   BACKWARD_PWM_LIMIT);
  }

  current_left_pwm = approach_int(current_left_pwm,
                                  desired_left_pwm,
                                  PWM_STEP);

  current_right_pwm = approach_int(current_right_pwm,
                                   desired_right_pwm,
                                   PWM_STEP);

  set_left_motor_signed(current_left_pwm);
  set_right_motor_signed(current_right_pwm);
}

static void UpdateEncoderTicksAndSpeed(void)
{
  int16_t rawL = (int16_t)__HAL_TIM_GET_COUNTER(&htim2);
  int16_t rawR = (int16_t)__HAL_TIM_GET_COUNTER(&htim4);

  int16_t nowL = (int16_t)(LEFT_ENCODER_SIGN * rawL);
  int16_t nowR = (int16_t)(RIGHT_ENCODER_SIGN * rawR);

  int16_t dL = (int16_t)(nowL - prevL);
  int16_t dR = (int16_t)(nowR - prevR);

  prevL = nowL;
  prevR = nowR;

  posL += dL;
  posR += dR;

  float left_rev = ((float)dL) / LEFT_COUNTS_PER_REV;
  float right_rev = ((float)dR) / RIGHT_COUNTS_PER_REV;

  measured_left_mps = (left_rev * WHEEL_CIRCUM_M) / CONTROL_DT_SEC;
  measured_right_mps = (right_rev * WHEEL_CIRCUM_M) / CONTROL_DT_SEC;
}

static void PublishEncoderTicks(void)
{
  wheel_ticks_msg.x = (float)posL;
  wheel_ticks_msg.y = (float)posR;
  wheel_ticks_msg.z = 0.0f;

  ticks_pub.publish(&wheel_ticks_msg);
}

void cmdVelCallback(const geometry_msgs::Twist& cmd_msg)
{
  float linear_x = (float)cmd_msg.linear.x;
  float angular_z = (float)cmd_msg.angular.z;

  linear_x = clamp_float(linear_x, -MAX_LINEAR_X_MPS, MAX_LINEAR_X_MPS);
  angular_z = clamp_float(angular_z, -MAX_ANGULAR_Z_RADPS, MAX_ANGULAR_Z_RADPS);

  cmd_linear_x = linear_x;
  cmd_angular_z = angular_z;

  target_left_mps  = linear_x - angular_z * WHEEL_BASE_M / 2.0f;
  target_right_mps = linear_x + angular_z * WHEEL_BASE_M / 2.0f;

  target_left_mps =
      clamp_float(target_left_mps, -MAX_LINEAR_X_MPS, MAX_LINEAR_X_MPS);

  target_right_mps =
      clamp_float(target_right_mps, -MAX_LINEAR_X_MPS, MAX_LINEAR_X_MPS);

  last_cmd_time = HAL_GetTick();
}

ros::Subscriber<geometry_msgs::Twist> cmd_sub("cmd_vel", cmdVelCallback);

/* USER CODE END 0 */

int main(void)
{
  HAL_Init();

  SystemClock_Config();

  MX_GPIO_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();

  /* USER CODE BEGIN 2 */

  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);

  __HAL_TIM_SET_COUNTER(&htim2, 0);
  __HAL_TIM_SET_COUNTER(&htim4, 0);

  prevL = 0;
  prevR = 0;
  posL = 0;
  posR = 0;

  measured_left_mps = 0.0f;
  measured_right_mps = 0.0f;

  right_dir(0);
  left_dir(0);
  motors_stop();

  HAL_TIM_PWM_Start(&htim3, RIGHT_PWM_CHANNEL);
  HAL_TIM_PWM_Start(&htim3, LEFT_PWM_CHANNEL);

  set_right_pwm_pct(0);
  set_left_pwm_pct(0);

  HAL_Delay(500);

  cmd_linear_x = 0.0f;
  cmd_angular_z = 0.0f;
  target_left_mps = 0.0f;
  target_right_mps = 0.0f;
  last_cmd_time = HAL_GetTick();

  reset_pid();

  nh.initNode();

  nh.advertise(ticks_pub);
  nh.subscribe(cmd_sub);

  nh.loginfo("STM32 ROS Motor Control Ready - STM PWM Tuned");

  uint32_t last_control_time = 0;

  /* USER CODE END 2 */

  while (1)
  {
    nh.spinOnce();

    if (HAL_GetTick() - last_control_time >= CONTROL_PERIOD_MS)
    {
      last_control_time = HAL_GetTick();

      UpdateEncoderTicksAndSpeed();
      PublishEncoderTicks();
      apply_open_loop_limited_pid_outputs();
    }
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;

  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                              | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_TIM2_Init(void)
{
  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 4;

  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 4;

  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;

  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_TIM3_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 83;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 49;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;

  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_TIM_MspPostInit(&htim3);
}

static void MX_TIM4_Init(void)
{
  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 4;

  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 4;

  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;

  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;

  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0 | GPIO_PIN_1, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

void Error_Handler(void)
{
  __disable_irq();

  while (1)
  {
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif
