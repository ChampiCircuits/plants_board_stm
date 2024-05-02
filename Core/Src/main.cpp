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
#include "LaserSensor.hpp"
#include <stdlib.h>
#include <string.h>
#include "stdio.h"
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

void grabber_extend();
void grabber_retract(bool block=true);
void lift_go_down();
void lift_go_up();
void lift_go_middle();
void hopper_close(int side);
void hopper_open(int side);
bool hopper_wait_and_close_spin_once(int side);
void reservoir_initialize_and_test();
void lift_initialize_and_test();
void grabber_initialize_and_test();
void hoppers_initialize_and_test();
int setup_lasers();

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

// ===================================== DEFINES DISTANCE SENSORS =====================================

#define SENSOR_LEFT_ADDRESS 0x01
#define SENSOR_RIGHT_ADDRESS 0x05
#define SENSOR_LEFT_OFFSET -10
#define SENSOR_RIGHT_OFFSET -8



// ============================================= DEFINES GRABBER =============================================

#define SERVO_GRABBER_ID 8

#define SERVO_GRABBER_POS_RETRACT 280
#define SERVO_GRABBER_POS_EXTEND 1023




// ============================================= DEFINES LIFT =============================================

#define LIFT_POS_UP 500
#define LIFT_POS_MIDDLE 5000
#define LIFT_POS_DOWN 14000



// ================================================ DEFINES HOPPERS ====================================================

#define LEFT 0
#define RIGHT 1

std::vector<int> hoppers_ids = {7, 14};
std::vector<int> hoppers_pos_open = {1023, 0};
std::vector<int> hoppers_pos_close = {600, 430};




// ================================ DECLARE / INITIALIZE ACTUATORS / SENSORS OBJECTS ===================================

// Distance sensors
auto sensors = std::vector<LaserSensor>({
  LaserSensor(XSHUT_LEFT_GPIO_Port, XSHUT_LEFT_Pin,  SENSOR_LEFT_ADDRESS, SENSOR_LEFT_OFFSET),
  LaserSensor(XSHUT_RIGHT_GPIO_Port, XSHUT_RIGHT_Pin,  SENSOR_RIGHT_ADDRESS, SENSOR_RIGHT_OFFSET)
});

// Steppers
Stepper stepper_lift = Stepper(get_time_us, STEP_LIFT_GPIO_Port, STEP_LIFT_Pin, DIR_LIFT_GPIO_Port, DIR_LIFT_Pin);
Stepper stepper_res = Stepper(get_time_us, STEP_RES_GPIO_Port, STEP_RES_Pin, DIR_RES_GPIO_Port, DIR_RES_Pin);

// Servos
SCServo servos = SCServo(&huart1);




// =============================================== SYSTEM STATE RELATED VARIABLES ======================================

std::vector<int> servo_ids_to_check = {
  SERVO_GRABBER_ID,
  hoppers_ids[LEFT],
  hoppers_ids[RIGHT]
};


struct SystemState
{
  bool hopper_left_closed = false;
  bool hopper_right_closed = false;
  bool storing = false;
  std::vector<bool> servos_ok = {false, false, false};

} system_state;




// ================================================ DIAGNOSTIC FUNCTIONS ===============================================

int ping_servos()
{
  int res = 0;
  for(int i = 0; i < servo_ids_to_check.size(); i++)
  {
    int id = servo_ids_to_check[i];
    int pos = servos.ReadPos(id);

    if(pos == -1)
    {
      printf("Error reading servo %d\n", id);
      res = -1;
    }
    else
    {
      printf("Servo %d was ping successfully\n", id);
    }
  }
  return res;
}



// ================================ SETUP / INITIALIZE POSES (SENSORS, ACTUATORS) ======================================

/**
 * @brief Setup the distance sensors
 *
 * @return int 0 if all sensors are setup correctly, otherwise the error code
 */
int setup_lasers()
{
  /* Toggle Xshut pin to reset the sensors so that their addresses can be set individually*/
  HAL_GPIO_WritePin(XSHUT_LEFT_GPIO_Port, XSHUT_LEFT_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(XSHUT_RIGHT_GPIO_Port, XSHUT_RIGHT_Pin, GPIO_PIN_RESET);

  /* Setup the first laser sensor */
  int status = sensors[LEFT].setup();

  if(status)
  {
    return status;
  }

  /* Setup the second laser sensor */
  status = sensors[RIGHT].setup();

  if(status)
  {
    return status;
  }

  // AFTER ALL SETUPS WE PULL TO HIGH THE SHUTPINS to enable the sensors
  HAL_GPIO_WritePin(XSHUT_LEFT_GPIO_Port, XSHUT_LEFT_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(XSHUT_RIGHT_GPIO_Port, XSHUT_RIGHT_Pin, GPIO_PIN_SET);

  return 0;
}


void reservoir_initialize_and_test()
{
  // Set big goal. When button is pressed, reset current pos to 0 and stop the motor
  stepper_res.set_goal(-100000);

  // Turn untill the button is released
  while(HAL_GPIO_ReadPin(FIN_COURSE_RES_GPIO_Port, FIN_COURSE_RES_Pin) == GPIO_PIN_RESET)
  {
    stepper_res.spin_once();
  }

  // Debouncing: spin for 200ms
  unsigned long start = HAL_GetTick();
  while(HAL_GetTick() - start < 200)
  {
    stepper_res.spin_once();
  }

  // Turn untill the button is pressed (reservoir in position)
  while(HAL_GPIO_ReadPin(FIN_COURSE_RES_GPIO_Port, FIN_COURSE_RES_Pin) == GPIO_PIN_SET)
  {
    stepper_res.spin_once();
  }

  stepper_res.set_pos(0);
  stepper_res.set_goal(0);

}

void lift_initialize_and_test()
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


void grabber_initialize_and_test()
{
  // servos.EnableTorque(SERVO_GRABBER_ID, 1);
  servos.WriteLimitTroque(SERVO_GRABBER_ID, 1023);
  servos.WritePos(SERVO_GRABBER_ID ,SERVO_GRABBER_POS_RETRACT, 200);
  HAL_Delay(200);
}


/**
 * Close then open the hoppers
 *
 */
void hoppers_initialize_and_test()
{
  hopper_close(LEFT);
  hopper_close(RIGHT);

  HAL_Delay(1000); // Because hoppers functions are not blocking

  hopper_open(LEFT);
  hopper_open(RIGHT);
}



// ============================================ ACTIONS FUNCTIONS ===========================================


// ----------------------------------------- LIFT -----------------------------------------

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

// ----------------------------------------- GRABBER -----------------------------------------

void grabber_extend()
{
  servos.WritePos(SERVO_GRABBER_ID ,SERVO_GRABBER_POS_EXTEND, 1000);
  HAL_Delay(1000);
}


void grabber_retract(bool block)
{
  servos.WritePos(SERVO_GRABBER_ID ,SERVO_GRABBER_POS_RETRACT, 500);
  if(block) HAL_Delay(500);
}


// ----------------------------------------- HOPPERS -----------------------------------------

void hopper_close(int side)
{
  servos.WritePos(hoppers_ids[side], hoppers_pos_close[side], 1000);
}


void hopper_open(int side)
{
  servos.WritePos(hoppers_ids[side], hoppers_pos_open[side], 500);
}



// ---------------------------------------- RESERVOIR ----------------------------------------

void reservoir_rotate()
{
  // Set big goal. When button is pressed, reset current pos to 0 and stop the motor
  stepper_res.set_pos(0);
  stepper_res.set_goal(-100000);

  // Turn untill the button is released
  while(HAL_GPIO_ReadPin(FIN_COURSE_RES_GPIO_Port, FIN_COURSE_RES_Pin) == GPIO_PIN_RESET)
  {
    stepper_res.spin_once();
  }

  // Debouncing: spin for 200ms
  unsigned long start = HAL_GetTick();
  while(HAL_GetTick() - start < 200)
  {
    stepper_res.spin_once();
  }

  // Turn untill the button is pressed (reservoir in position)
  while(HAL_GPIO_ReadPin(FIN_COURSE_RES_GPIO_Port, FIN_COURSE_RES_Pin) == GPIO_PIN_SET)
  {
    stepper_res.spin_once();
  }

  stepper_res.set_pos(0);
  stepper_res.set_goal(0);

}


void reservoir_rotate(int n_slots)
{
  for(int i = 0; i < n_slots; i++)
  {
    reservoir_rotate();
  }
}




// =============================================== HIGH LEVEL ACTIONS =================================================


bool hopper_wait_and_close_spin_once(int side)
{
  // Check if distance < 50mm for left plant
  int dist = sensors[side].get_dist_mm();
  if(dist < 50)
  {
    // Close the hopper
    hopper_close(side);
    return true;
  }
  return false;
}


void request_store_plants()
{
  system_state.storing = true;

  // Clear distance sensors buffers (?)
  sensors[LEFT].clear_interrupt();
  sensors[RIGHT].clear_interrupt();
  HAL_Delay(100);
}


// /!\ DELAY IN THIS FUNCTION
void store_plants_spin_once()
{

  if(!system_state.storing)
  {
    return;
  }

  if(!system_state.hopper_left_closed)
  {
    system_state.hopper_left_closed = hopper_wait_and_close_spin_once(LEFT);
  }

  if(!system_state.hopper_right_closed)
  {
    system_state.hopper_right_closed = hopper_wait_and_close_spin_once(RIGHT);
  }

  if(system_state.hopper_left_closed && system_state.hopper_right_closed)
  {
    HAL_Delay(1500); // Because hoppers functions are not blocking // TODO ADD NON BLOCKING DELAY
    lift_go_up();
    grabber_extend();
    HAL_Delay(500);
    lift_go_middle();
    grabber_retract(false);
    lift_go_down();
    hopper_open(LEFT);
    hopper_open(RIGHT);
    HAL_Delay(500);
    system_state.storing = false;
    system_state.hopper_left_closed = false;
    system_state.hopper_right_closed = false;
    reservoir_rotate(3);
  }
}


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


  stepper_lift.set_speed(5000);

    // Initialize the sensors
  if(setup_lasers() != 0)
  {
    printf("Error setting up the sensors\n");
    Error_Handler();
  }

  if(ping_servos() != 0)
  {
    printf("Error pinging servos\n");
    Error_Handler();
  }

    // Initialize / move actuators

    reservoir_initialize_and_test();

    hoppers_initialize_and_test();
    HAL_Delay(200);
    grabber_retract();
    lift_initialize_and_test();


    request_store_plants();



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */



    while (1)
    {

      // Print distances
      printf("Left: %d mm, Right: %d mm\n", sensors[LEFT].get_dist_mm(), sensors[RIGHT].get_dist_mm());

      store_plants_spin_once();
      if(!system_state.storing)
      {
        request_store_plants();
      }

      // Print the distances
      // printf("Left: %d mm, Right: %d mm\n", sensor_left.results.distance_mm, sensor_right.results.distance_mm);




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
