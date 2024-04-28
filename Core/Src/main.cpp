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
// #include "STM32Step/src/STM32Step.hpp"
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
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc2;

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
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC2_Init(void);
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

void list_servos_ids(uint8_t id_start,  uint8_t id_stop, SCServo servos) {
	for(uint8_t id=id_start; id<id_stop; id++) {
		if(servos.ReadPos(id)!=-1) {
			printf("Found ID %d\n", id);
		}
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

  int s = sizeof(time_us);

  return seconds_elapsed * 1000000 + time_us;
}

int ids_servos[3] = {8, 17, 18};
int lower_limit[3] = {353, 100, 0};
int upper_limit[3] = {740, 900, 1023};

void manual_control() {

    HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
    HAL_ADC_Start_DMA(&hadc2, (uint32_t *) &value_adc, 1);
    HAL_ADC_Start(&hadc2);

    SCServo servos = SCServo(&huart1);

    int pos_servos[3];

    for (int i = 0; i < 3; i++) {
        servos.WriteLimitAngle(ids_servos[i], lower_limit[i], upper_limit[i]);
    }

    for (int i = 0; i < 3; i++) {
        pos_servos[i] = servos.ReadPos(ids_servos[i]);
        printf("init pos_servo[%d]=%d\n", ids_servos[i], pos_servos[i]);
    }

    // Read the current position of the servo
    int pos = servos.ReadPos(ids_servos[i_mot]);
    printf("Waiting for the user to move the potentiometer to the current position of the servo\n");
    // tolerance 10 units for the potentiometer and the servo
    while (abs(value_adc - pos) > 10) {
        HAL_Delay(20);
//        printf("value_adc = %d\n", value_adc);
    }
    printf("OK\n");
    
    while (1) {


        if (!HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8)) {

            // Change ID Servo
            i_mot = (i_mot + 1) % 3;
            printf("button pushed, i_mot=%d\n", i_mot);

            HAL_Delay(500);

            // The user will move the servo to a position using a potentiometer. But because the
            // potentiometer is absolute, we dont want the servo to move right away to the new position. Instead,
            // we wait for the user to move the potentiometer to the current position of the servo, and then we
            // let him move the servo to the new position. The value of the potentiometer is read by the ADC and
            // stored automatically in the variable value_adc.

            // Read the current position of the servo
            int pos = servos.ReadPos(ids_servos[i_mot]);
            printf("Waiting for the user to move the potentiometer to the current position of the servo\n");
            // tolerance 10 units for the potentiometer and the servo
            while (abs(value_adc - pos) > 10) {
                if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8) == RESET) {
                    printf("button pushed\n");
                    HAL_Delay(500);

                    // Change ID Servo
                    i_mot = (i_mot + 1) % 3;
                    printf("i_mot=%d\n", i_mot);
                }
                HAL_Delay(20);
//                printf("value_adc = %d\n", value_adc);
            }
            printf("OK\n");

        }


        // Move the servo to the pos from the adc
        int val = value_adc;
//      printf("value_adc = %d\n", val);
        servos.WritePos(ids_servos[i_mot], val, 600);
        pos_servos[i_mot] = val;

        // Print the values of the 3 servos in 1 line
        for (int i = 0; i < 3; i++) {
            printf("%d: %d ", i, pos_servos[i]);
        }
        printf("\n");
        HAL_Delay(50);

        //	  printf("value ADC = %d\n", value_adc);
        //	  HAL_Delay(10);

    }
}

void go_to_waypoints() {

    SCServo servos = SCServo(&huart1);

    int pos_servos[3];

    for (int i = 0; i < 3; i++) {
        servos.WriteLimitAngle(ids_servos[i], lower_limit[i], upper_limit[i]);
    }

    HAL_Delay(200);

    // Vector of waypoints. each waypoint is a vector of 4 integers: the 3 positions of the servos and the time to reach the waypoint
    std::vector<std::vector<int>> waypoints = {
            std::vector<int>{500, 215, 232, 500},
//            std::vector<int>{184, 202, 27, 2000},
            std::vector<int>{635, 202, 298, 500},
            std::vector<int>{754, 453, 581, 500},
            std::vector<int>{441, 719, 695, 500}, // POSE
            std::vector<int>{441, 719, 780, 200}, // DEPOSE
            std::vector<int>{649, 719, 762, 500}, // DRESSE
            std::vector<int>{649, 202, 206, 500}, // PRE HOME
    };

    // TTB
//    std::vector<std::vector<int>> waypoints = {
//            std::vector<int>{500, 195, 232, 2000},
////            std::vector<int>{184, 202, 27, 2000},
//            std::vector<int>{635, 202, 298, 2000},
//            std::vector<int>{754, 453, 581, 1000},
//            std::vector<int>{441, 719, 695, 1000}, // POSE
//            std::vector<int>{441, 719, 780, 200}, // DEPOSE
//            std::vector<int>{649, 719, 762, 1000}, // DRESSE
//            std::vector<int>{649, 202, 206, 1000}, // PRE HOME
//    };

    // TB
//    std::vector<std::vector<int>> waypoints = {
//            std::vector<int>{515, 190, 232, 2000},
////            std::vector<int>{184, 202, 27, 2000},
//            std::vector<int>{635, 202, 298, 2000},
//            std::vector<int>{754, 453, 581, 1000},
//            std::vector<int>{441, 719, 695, 1000}, // POSE
//            std::vector<int>{441, 719, 762, 200}, // DEPOSE
//            std::vector<int>{649, 719, 762, 1000}, // DRESSE
//            std::vector<int>{649, 202, 206, 1000}, // PRE HOME
//    };


    // BIEN:
//    std::vector<int>{521, 204, 232, 2000},
////            std::vector<int>{184, 202, 27, 2000},
//            std::vector<int>{635, 202, 298, 2000},
//            std::vector<int>{754, 453, 581, 2000},
//            std::vector<int>{441, 719, 695, 2000}, // POSE
//            std::vector<int>{441, 719, 762, 2000}, // DEPOSE
//            std::vector<int>{649, 719, 762, 2000}, // DRESSE
//            std::vector<int>{649, 202, 206, 2000}, // PRE HOME

    // Limit torque to 200
    for (int i = 0; i < 3; i++) {
        servos.WriteLimitTroque(ids_servos[i], 600);
    }

    HAL_Delay(500);


    while(1) {

        for (int i = 0; i < waypoints.size(); i++) {
            std::vector<int> waypoint = waypoints[i];

            // send goal repeatedly for the amount of time specified in the waypoint
            int time = waypoint[3];
            int start_time = HAL_GetTick();
            while (HAL_GetTick() - start_time < time) {
                for (int i = 0; i < 3; i++) {
                    servos.WritePos(ids_servos[i], waypoint[i], time);
                }
                HAL_Delay(50);
            }
            HAL_Delay(1000);

            printf("\n");
        }
    }
}

void lift_up_no_lib()
{
  HAL_GPIO_WritePin(DIR_LIFT_GPIO_Port, DIR_LIFT_Pin, GPIO_PIN_SET);
  for (int i = 0; i < 500; i++) {
    HAL_GPIO_WritePin(STEP_LIFT_GPIO_Port, STEP_LIFT_Pin, GPIO_PIN_SET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(STEP_LIFT_GPIO_Port, STEP_LIFT_Pin, GPIO_PIN_RESET);
    HAL_Delay(1);
  }
}

void lift_down_no_lib()
{
  HAL_GPIO_WritePin(DIR_LIFT_GPIO_Port, DIR_LIFT_Pin, GPIO_PIN_RESET);
  for (int i = 0; i < 500; i++) {
    HAL_GPIO_WritePin(STEP_LIFT_GPIO_Port, STEP_LIFT_Pin, GPIO_PIN_SET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(STEP_LIFT_GPIO_Port, STEP_LIFT_Pin, GPIO_PIN_RESET);
    HAL_Delay(1);
  }
}



class Stepper {
public:
  Stepper(GPIO_TypeDef *gpio_port_step, uint16_t gpio_pin_step, GPIO_TypeDef *gpio_port_dir, uint16_t gpio_pin_dir) {
    this->gpio_port_step = gpio_port_step;
    this->gpio_pin_step = gpio_pin_step;
    this->gpio_port_dir = gpio_port_dir;
    this->gpio_pin_dir = gpio_pin_dir;

    HAL_GPIO_WritePin(gpio_port_step, gpio_pin_step, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(gpio_port_dir, gpio_pin_dir, GPIO_PIN_RESET);

  }

  void set_goal(int goal) {
    this->goal = goal;
    state.state = State::HIGH;
    state.direction = goal > state.pos ? 1 : -1;
    time_start_step = get_time_us();
    time_start_high = time_start_step;
    HAL_GPIO_WritePin(gpio_port_dir, gpio_pin_dir, state.direction == 1 ? GPIO_PIN_SET : GPIO_PIN_RESET);
  }

  void spin_once() {

    if (state.state == State::STOPPED) {
      return;
    }

    if (state.state == State::HIGH) {
      if (get_time_us() - time_start_high > time_high) {
        // Set low
        HAL_GPIO_WritePin(gpio_port_step, gpio_pin_step, GPIO_PIN_RESET);
        state.state = State::LOW;
      }
    } else if (state.state == State::LOW) {
      if (get_time_us() - time_start_step > time_step) {
        // Set high
        HAL_GPIO_WritePin(gpio_port_step, gpio_pin_step, GPIO_PIN_SET);
        state.state = State::HIGH;
        time_start_high = get_time_us();
        time_start_step = get_time_us();
        state.pos += state.direction;
      }
    }

    if (state.pos == goal) {
      state.state = State::STOPPED;
    }
  }

  bool is_stopped() {
    return state.state == State::STOPPED;
  }

  private:
    GPIO_TypeDef *gpio_port_step;
    uint16_t gpio_pin_step;
    GPIO_TypeDef *gpio_port_dir;
    uint16_t gpio_pin_dir;

    struct State {
      int pos;
      int direction; // 1 or -1
      enum {STOPPED, HIGH, LOW} state;
    } state = {0, 1, State::STOPPED};

    int goal = 0; // steps

    unsigned long speed = 10000; // step/s
    unsigned long time_step = 10000000 / speed; // us
    unsigned long time_high = 10; // us


    unsigned long time_start_step = 0;
    unsigned long time_start_high = 0;






 };

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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  MX_ADC2_Init();
  MX_FDCAN1_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

    // Start the timer
    HAL_TIM_Base_Start_IT(&htim2);

    Stepper stepper_lift = Stepper(STEP_LIFT_GPIO_Port, STEP_LIFT_Pin, DIR_LIFT_GPIO_Port, DIR_LIFT_Pin);

    int pos_up = 100000;
    int pos_down = 0;
    int current_goal = pos_up;

    stepper_lift.set_goal(pos_up);


 
    // go_to_waypoints();
    // manual_control();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1)
    {

     if (stepper_lift.is_stopped()) {
       HAL_Delay(1000);
        current_goal = current_goal == pos_up ? pos_down : pos_up;
        stepper_lift.set_goal(current_goal);
     }

    stepper_lift.spin_once();
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
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_10B;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.GainCompensation = 0;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.ContinuousConvMode = ENABLE;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.DMAContinuousRequests = ENABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc2.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_92CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  HAL_GPIO_WritePin(GPIOB, DIR_RES_Pin|STEP_LIFT_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : STEP_RES_Pin ENABLE_PIN_Pin DIR_LIFT_Pin */
  GPIO_InitStruct.Pin = STEP_RES_Pin|ENABLE_PIN_Pin|DIR_LIFT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : DIR_RES_Pin STEP_LIFT_Pin LD2_Pin */
  GPIO_InitStruct.Pin = DIR_RES_Pin|STEP_LIFT_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : BUTTON_Pin */
  GPIO_InitStruct.Pin = BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BUTTON_GPIO_Port, &GPIO_InitStruct);

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
