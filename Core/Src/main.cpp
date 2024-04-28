/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "SCServo.h"
#include "Stepper.hpp"
// #include "STM32Step/src/STM32Step.hpp"
#include <stdlib.h>
#include <string.h>
#include "stdio.h"
#include "VL53L4CD_api.h"
#include <vector>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
FDCAN_HandleTypeDef hfdcan1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
int value_adc = 0;
int i_mot=0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
extern "C"
{
int _write(int file, char *ptr, int len)
{
   for (int DataIdx = 0; DataIdx < len; DataIdx++)
//        ITM_SendChar(*ptr++);
   	HAL_UART_Transmit(&huart2, (uint8_t*)ptr++, 1, HAL_MAX_DELAY);
   return len;
}

}

unsigned long seconds_elapsed = 0;
// TIM2 interrupt callback (reaches ARR every second
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM2) {
    seconds_elapsed++;
  }
}

unsigned long get_time_us() {
  unsigned long time_us = htim2.Instance->CNT;

  return seconds_elapsed * 1000000 + time_us;
}

void list_servos_ids(uint8_t id_start,  uint8_t id_stop, SCServo servos) {
	for(uint8_t id=id_start; id<id_stop; id++) {
		if(servos.ReadPos(id)!=-1) {
			printf("Found ID %d\n", id);
		}
	}
}



#define SENSOR_LEFT_ADDRESS 0x01
#define SENSOR_RIGHT_ADDRESS 0x05
#define SENSOR_LEFT_OFFSET -10
#define SENSOR_RIGHT_OFFSET -8

struct LASER_SENSOR
{
  // gpio_pin, address, results
  uint16_t pin;
  GPIO_TypeDef *port;
  Dev_t address;
  VL53L4CD_ResultsData_t results = {};
};


LASER_SENSOR sensor_left{};
LASER_SENSOR sensor_right{};


int setup_laser(LASER_SENSOR sensor)
{
  uint16_t sensor_id;
  uint8_t status;
  printf("SENSOR_PIN: %d\n", sensor.pin);

  HAL_Delay(5);
  // set the pin to high to enable the sensor
  HAL_GPIO_WritePin(sensor.port, sensor.pin, GPIO_PIN_SET);
  HAL_Delay(5);

  // set I2C address (other unset addresses XSHUT have to be pull to low before)
  status = VL53L4CD_SetI2CAddress(0x52, sensor.address); // 0x52 is the default address
  if (status)
  {
    printf("VL53L4CD_SetI2CAddress failed with status %u\n", status);
    return status;
  }

  /* (Optional) Check if there is a VL53L4CD sensor connected */
  printf("Checking for laser sensor at address %x\n", sensor.address);
  status = VL53L4CD_GetSensorId(sensor.address, &sensor_id);

  if (status || (sensor_id != 0xEBAA))
  {
    printf("VL53L4CD not detected at requested address\n");
    return status;
  }
  printf("VL53L4CD detected at address %x\n", sensor.address);

  /* (Mandatory) Init VL53L4CD sensor */
  printf("Initializing laser sensor\n");
  status = VL53L4CD_SensorInit(sensor.address);
  if (status)
  {
    printf("VL53L4CD ULD Loading failed\n");
    return status;
  }

  // set the offsets
  if (sensor.address == SENSOR_LEFT_ADDRESS)
  {
    status = VL53L4CD_SetOffset(sensor.address, SENSOR_LEFT_OFFSET);
  }
  else if (sensor.address == SENSOR_RIGHT_ADDRESS)
  {
    status = VL53L4CD_SetOffset(sensor.address, SENSOR_RIGHT_OFFSET);
  }
  if (status)
  {
    printf("VL53L4CD_SetOffset failed with status %u\n", status);
    return status;
  }

  status = VL53L4CD_StartRanging(sensor.address);
  if (status)
  {
    printf("VL53L4CD_StartRanging failed with status %u\n", status);
    return status;
  }
  printf("VL53L4CD ULD ready at address %x ready\n", sensor.address);

  return 0;
}

void scan()
{
  HAL_Delay(5);

  /*I2C Bus Scanning*/
  uint16_t sensor_id;
  uint8_t status;

  for (int i = 1; i < 128; i++)
  {
    status = VL53L4CD_GetSensorId(i, &sensor_id);
    if (status || (sensor_id != 0xEBAA))
    {
      // printf("VL53L4CD not detected at address %x\n", i);
    }
    else
    {
      printf("VL53L4CD detected at address %x\n", i);
    }
    HAL_Delay(5);
  }
  printf("end of scan\n\n");
}

int update_distance(LASER_SENSOR &sensor)
{
  // We don't want to read data at too high frequency, so we store previous time and check against HAL_GetTick(). (5ms min)
  static uint32_t last_read_time = 0;
  if (HAL_GetTick() - last_read_time < 5)
  {
    return 0;
  }

  /* Use polling function to know when a new measurement is ready.
   * Another way can be to wait for HW interrupt raised on PIN 7
   * (GPIO 1) when a new measurement is ready */

  uint8_t isReady;

  uint8_t status = VL53L4CD_CheckForDataReady(sensor.address, &isReady);

  if (isReady)
  {
    /* (Mandatory) Clear HW interrupt to restart measurements */
    VL53L4CD_ClearInterrupt(sensor.address);

    /* Read measured distance. RangeStatus = 0 means valid data */
    VL53L4CD_GetResult(sensor.address, &sensor.results);
  }

  return status;
}


int setup_lasers()
{

  sensor_left.port = XSHUT_LEFT_GPIO_Port;
  sensor_left.pin = XSHUT_LEFT_Pin;
  sensor_left.address = SENSOR_LEFT_ADDRESS;
  sensor_right.port = XSHUT_RIGHT_GPIO_Port;
  sensor_right.pin = XSHUT_RIGHT_Pin;
  sensor_right.address = SENSOR_RIGHT_ADDRESS;

  /* Toggle Xshut pin to reset the sensors so that their addresses can be set individually*/
  HAL_GPIO_WritePin(XSHUT_LEFT_GPIO_Port, XSHUT_LEFT_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(XSHUT_RIGHT_GPIO_Port, XSHUT_RIGHT_Pin, GPIO_PIN_RESET);

  /* Setup the first laser sensor */
  printf("SETUP LASER LEFT\n");
  int status = setup_laser(sensor_left);
  if (status)
  {
    printf("setup_laser at address %x failed with status %u\n", sensor_left.address, status);
    return -1;
  }

  /* Setup the second laser sensor */
  printf("\n\nSETUP LASER RIGHT\n");
  status = setup_laser(sensor_right);
  if (status)
  {
    printf("setup_laser at address %x failed with status %u\n", sensor_right.address, status);
    return -1;
  }

  // AFTER ALL SETUPS WE PULL TO HIGH THE SHUTPINS to enable the sensors
  HAL_GPIO_WritePin(XSHUT_LEFT_GPIO_Port, XSHUT_LEFT_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(XSHUT_RIGHT_GPIO_Port, XSHUT_RIGHT_Pin, GPIO_PIN_SET);

  return 0;
}

void test_lasers()
{
  uint8_t loop, status;

  /*********************************/
  /*         Ranging loop          */
  /*********************************/
  loop = 0;
  while (loop < 200)
  {
    status = update_distance(sensor_left);
    if (status)
    {
      printf("get_distance at address %x failed with status %u\n", sensor_left.address, status);
      return;
    }
    status = update_distance(sensor_right);
    if (status)
    {
      printf("get_distance at address %x failed with status %u\n", sensor_right.address, status);
      return;
    }
    printf("Left: %d mm, Right: %d mm\n", sensor_left.results.distance_mm, sensor_right.results.distance_mm);
    loop++;
  }

  status = VL53L4CD_StopRanging(sensor_left.address);
  status = VL53L4CD_StopRanging(sensor_right.address);

  printf("End of ULD demo\n");
}





#define SERVO_HORIZ_ID 8

#define SERVO_HORIZ_POS_RETRACT 280
#define SERVO_HORIZ_POS_EXTEND 1023

#define LIFT_POS_UP 500
#define LIFT_POS_MIDDLE 5000
#define LIFT_POS_DOWN 14000

#define HOPPER_LEFT_OPEN 1023
#define HOPPER_LEFT_CLOSE 500
#define HOPPER_RIGHT_OPEN 0
#define HOPPER_RIGHT_CLOSE 500


Stepper stepper_lift = Stepper(get_time_us, STEP_LIFT_GPIO_Port, STEP_LIFT_Pin, DIR_LIFT_GPIO_Port, DIR_LIFT_Pin);
Stepper stepper_res = Stepper(get_time_us, STEP_RES_GPIO_Port, STEP_RES_Pin, DIR_RES_GPIO_Port, DIR_RES_Pin);

SCServo servos = SCServo(&huart1);

void lift_go_down()
{
  stepper_lift.set_goal(LIFT_POS_DOWN);
  while(!stepper_lift.is_stopped())
  {
    stepper_lift.spin_once();
  }
}

void lift_go_up()
{
  stepper_lift.set_goal(LIFT_POS_UP);
  while(!stepper_lift.is_stopped())
  {
    stepper_lift.spin_once();
  }
}

void lift_go_middle()
{
  stepper_lift.set_goal(LIFT_POS_MIDDLE);
  while(!stepper_lift.is_stopped())
  {
    stepper_lift.spin_once();
  }
}

void grabber_extend()
{
  servos.WritePos(SERVO_HORIZ_ID ,SERVO_HORIZ_POS_EXTEND, 500);
  HAL_Delay(500);
}


void grabber_retract(bool block=true)
{
  servos.WritePos(SERVO_HORIZ_ID ,SERVO_HORIZ_POS_RETRACT, 500);
  if(block) HAL_Delay(500);
}


void reservoir_go_to_init_pos()
{
  // Set big goal. When button is pressed, reset current pos to 0 and stop the motor
  stepper_res.set_goal(100000);

  while(HAL_GPIO_ReadPin(FIN_COURSE_RES_GPIO_Port, FIN_COURSE_RES_Pin) == GPIO_PIN_SET)
  {
    stepper_res.spin_once();
  }

  stepper_res.set_pos(0);
  stepper_res.set_goal(0);

}

void lift_go_to_init_pos()
{
  // Here we don't have a sensor. So we just turn the motor for a certain distance. (5 spins at 3200 steps per spin)

  stepper_lift.set_goal(-5*3200);
  while(!stepper_lift.is_stopped())
  {
    stepper_lift.spin_once();
  }
  stepper_lift.set_pos(0);

  grabber_extend();
  grabber_retract();

  lift_go_down();
}

void servo_horiz_go_to_init_pos()
{
  // servos.EnableTorque(SERVO_HORIZ_ID, 1);
  servos.WriteLimitTroque(SERVO_HORIZ_ID, 1023);
  servos.WritePos(SERVO_HORIZ_ID ,SERVO_HORIZ_POS_RETRACT, 200);
  HAL_Delay(200);
}

void actuators_go_to_init_poses()
{
  servo_horiz_go_to_init_pos();
  // reservoir_go_to_init_pos();
  lift_go_to_init_pos();
}




void setup_hoppers()
{
  // Hoppers are servos 7 and 14

  // Close then open
  servos.WritePos(7, HOPPER_LEFT_CLOSE, 500);
  servos.WritePos(14, HOPPER_RIGHT_CLOSE, 500);
  HAL_Delay(1000);
  servos.WritePos(7, HOPPER_LEFT_OPEN, 500);
  servos.WritePos(14, HOPPER_RIGHT_OPEN, 500);
  HAL_Delay(1000);


}

bool hopper_left_wait_and_close_spin_once()
{
  // Check if distance < 50mm for left plant
  if(sensor_left.results.distance_mm < 50)
  {
    // Close the hopper
    servos.WritePos(7, HOPPER_LEFT_CLOSE, 200);
    return true;
  }
  return false;
}

bool hopper_right_wait_and_close_spin_once()
{
  // Check if distance < 50mm for left plant
  if(sensor_right.results.distance_mm < 50)
  {
    // Close the hopper
    servos.WritePos(14, HOPPER_RIGHT_CLOSE, 200);
    return true;
  }
  return false;
}




// bool wait_and_lock_plant_spin_once()
// {
//   // Check if distance < 50mm for left plant
//   if(sensor_left.results.distance_mm < 50)
//   {
//     // Lock the plant
//     grabber_extend();
//     return true;
//   }
// }




/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

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
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  MX_FDCAN1_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

    // Start the timer
    HAL_TIM_Base_Start_IT(&htim2);

  actuators_go_to_init_poses();

  setup_lasers();
  setup_hoppers();



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  bool left_closed = false;
  bool right_closed = false;
  bool stored = false;

    while (1)
    {
      update_distance(sensor_left);
      update_distance(sensor_right);

      // Print the distances
      // printf("Left: %d mm, Right: %d mm\n", sensor_left.results.distance_mm, sensor_right.results.distance_mm);

      if(!left_closed)
      {
        left_closed = hopper_left_wait_and_close_spin_once();

      }
      if(!right_closed)
      {
        right_closed = hopper_right_wait_and_close_spin_once();
      }
      if(left_closed && right_closed && !stored)
      {
        HAL_Delay(500); // Bc hoppers functions are not blocking
        stored = true;
        lift_go_up();
        grabber_extend();
        lift_go_middle();
        grabber_retract(false);
      }

      //  if (stepper_lift.is_stopped()) {
    //    HAL_Delay(1000);
    //     current_goal = current_goal == pos_up ? pos_down : pos_up;
    //     stepper_lift.set_goal(current_goal);
    //  }
    //
    // stepper_lift.spin_once();
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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = DISABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 16;
  hfdcan1.Init.NominalSyncJumpWidth = 1;
  hfdcan1.Init.NominalTimeSeg1 = 2;
  hfdcan1.Init.NominalTimeSeg2 = 2;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 1;
  hfdcan1.Init.DataTimeSeg2 = 1;
  hfdcan1.Init.StdFiltersNbr = 0;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

  /* USER CODE END FDCAN1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x30A0A7FB;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 16;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 1000000;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_HalfDuplex_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_EnableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, STEP_RES_Pin|ENABLE_PIN_Pin|DIR_LIFT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, DIR_RES_Pin|XSHUT_LEFT_Pin|XSHUT_RIGHT_Pin|STEP_LIFT_Pin
                          |LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : STEP_RES_Pin ENABLE_PIN_Pin DIR_LIFT_Pin */
  GPIO_InitStruct.Pin = STEP_RES_Pin|ENABLE_PIN_Pin|DIR_LIFT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : DIR_RES_Pin XSHUT_LEFT_Pin XSHUT_RIGHT_Pin STEP_LIFT_Pin
                           LD2_Pin */
  GPIO_InitStruct.Pin = DIR_RES_Pin|XSHUT_LEFT_Pin|XSHUT_RIGHT_Pin|STEP_LIFT_Pin
                          |LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : FIN_COURSE_RES_Pin */
  GPIO_InitStruct.Pin = FIN_COURSE_RES_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(FIN_COURSE_RES_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

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

#ifdef  USE_FULL_ASSERT
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
