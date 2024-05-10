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

#include "MessageRecomposer.h"
#include "ChampiCan.h"
#include "ChampiState.h"

#include <pb_encode.h>
#include <pb_decode.h>
#include "msgs_can.pb.h"


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
void on_receive_action(const std::string& proto_msg);
void reservoir_rotate(bool keep_speed);
void reservoir_rotate(int n_slots);

void close_fildefer();
void hide_fildefer();
void fildefer_initialize();

void push_plant();
void retract_servo_plant();
void open_circle_plant();
void close_circle_plant();
void pusher_and_circle_initialize();
void push_one_plant_out();
void reservoir_align_with_output();
void reservoir_realign_back();

void request_store_plants();
void request_stop_storing();
void request_plant_out();


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


// ================================================= GENRAL VARIABLES =================================================

ChampiCan champi_can;
MessageRecomposer msg_recomposer_action;

ChampiState champi_state;

msgs_can_ActStatus status_msg = msgs_can_ActStatus_init_zero;


// ================================================== SYSTEM STATE ====================================================


/**
 * @brief Error handler we call when CAN might still work.
 * It blinks the built-in LED at 1Hz AND sends status on CAN bus.
 */
void Error_Handler_CAN_ok() {

  // Blink the built-in LED at 1Hz
  uint32_t last_time = HAL_GetTick();
  while (true) {
    champi_state.spin_once();
    HAL_Delay(10); // 10ms required to match the main loop frequency (for control)

    if (HAL_GetTick() - last_time > 500) {
      last_time = HAL_GetTick();
      HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    }
  }
}

/**
 * @brief Fonction qui attend que le l'envoi de données sur le CAN fonctionne. Ca envoie un message de test
 * à répétition jusqu'à ce que ça fonctionne.
 * Also blinks the built-in LED at 5 Hz.
 */
void tx_ok_or_reset() {
  uint8_t buff[20] = {0}; // We need a big message to fill the FIFO

  // Send a message to test if the can bus works (at least 1 node up)
  uint32_t ret = champi_can.send_msg(CAN_ID_ACT_TEST, (uint8_t *) buff, 20);

  if(ret==0){
    return;
  }

  // If we get an error, retry doesn't work sometimes. So we reset the stm to try again. Also blink the led 10Hz

  // blink the built-in LED for 1s
  for (int i = 0; i < 10; i++) {
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    HAL_Delay(100);
  }

  // Then reset the stm
  NVIC_SystemReset();

}

// ==================================================== CAN COMMUNICATION ==============================================


/**
  * @brief  Rx FIFO 0 callback.
  * @param  hfdcan: pointer to an FDCAN_HandleTypeDef structure that contains
  *         the configuration information for the specified FDCAN.
  * @param  RxFifo0ITs: indicates which Rx FIFO 0 interrupts are signalled.
  *         This parameter can be any combination of @arg FDCAN_Rx_Fifo0_Interrupts.
  * @retval None
  */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs) {

    // Attention !! Quand on met un breakpoint dans cette fonction, on ne reçoit plus que 2 messages au lieu du
    // bon nombre.

    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET) {
        /* Retrieve Rx messages from RX FIFO0 */
        FDCAN_RxHeaderTypeDef RxHeader;
        uint8_t RxData[8];

        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK) {
            status_msg.status.status = msgs_can_Status_StatusType_ERROR;
            status_msg.status.error = msgs_can_Status_ErrorType_CAN_RX;
            champi_state.report_status(status_msg);
            Error_Handler_CAN_ok();
        }
        /* Handle Interesting messages
         * Pour le moment, on n'utilise pas de mutex ou de choses comme ça, donc il faut faire attention
         * à ne pas modifier trop de variables partagées, et de priviligier la modifcation de variables
         * de 32 bits ou moins (pour que leur modification soit une opération atomique)
         * */

        if (RxHeader.Identifier == CAN_ID_ACT_ACTION) {
            msg_recomposer_action.add_frame(RxData, RxHeader.DataLength);

            if (msg_recomposer_action.check_if_new_full_msg()) {
                std::string proto_msg = msg_recomposer_action.get_full_msg();
                on_receive_action(proto_msg);

            }
        }

      if (RxHeader.Identifier == CAN_ID_ACT_RESET) {
        NVIC_SystemReset();
      }

    }
}


// ------------------------------------------ ON RECEIVE FUNCTIONS -----------------------------------------------

void on_receive_action(const std::string& proto_msg)
{
  // Allocate space for the decoded message.
  msgs_can_ActCmd ret_action = msgs_can_ActCmd_init_zero;
  // Create a stream that reads from the buffer.
  pb_istream_t stream_ret = pb_istream_from_buffer((const unsigned char*)proto_msg.c_str(), proto_msg.size());
  // Now we are ready to decode the message.
  if (!pb_decode(&stream_ret, msgs_can_ActCmd_fields, &ret_action)) {
    // Decoding failed
    status_msg.status.status = msgs_can_Status_StatusType_ERROR;
    status_msg.status.error = msgs_can_Status_ErrorType_PROTO_DECODE;
    champi_state.report_status(status_msg);
    Error_Handler_CAN_ok();
  }

  // Use message

  switch (ret_action.action)
  {
    case msgs_can_ActActions_START_GRAB_PLANTS:
      request_store_plants();
      break;
    case msgs_can_ActActions_STOP_GRAB_PLANTS:
      request_stop_storing();
      break;
    case msgs_can_ActActions_RELEASE_PLANT:
      request_plant_out();
      break;
    default:
      break;
  }
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

#define LIFT_POS_UP 4*3200
#define LIFT_POS_MIDDLE 3*3200

#define LIFT_OFFSET_FROM_BUTTON -200




// =========================================== DEFINES RESERVOIR ===========================================

#define RES_OFFSET_FROM_BUTTON 100
#define RES_OFFSET_FOR_OUTPUT 800

// ================================================ DEFINES HOPPERS ====================================================

#define LEFT 0
#define RIGHT 1

std::vector<int> hoppers_ids = {7, 14};
std::vector<int> hoppers_pos_open = {1023, 0};
std::vector<int> hoppers_pos_close = {600, 430};


// =============================================== DEFINES FILDEFER ===============================================

#define FILDEFER_ID 18
#define FILDEFER_POS_CLOSED 450
#define FILDEFER_POS_HIDDEN 200

// =============================================== DEFINES PLANT OUTPUT ===============================================

#define SERVO_PUSH_PLANT_ID 9
#define SERVO_CIRCLE_PLANT_ID 16
#define SERVO_CIRCLE_POS_CLOSE 540
#define SERVO_CIRCLE_POS_OPEN 950
#define SERVO_CIRCLE_POS_OPEN_MORE 1023
#define SERVO_PUSH_POSE_PUSHED 200
#define SERVO_PUSH_POSE_RETRACTED 420


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
  hoppers_ids[RIGHT],
  FILDEFER_ID,
  SERVO_CIRCLE_PLANT_ID,
  SERVO_PUSH_PLANT_ID
};

enum Action {INITIALIZING, FREE, STORING, PUTTING_OUT, FREE_CIRCLE_OUT};

struct SystemState
{
  bool hopper_left_closed = false;
  bool hopper_right_closed = false;
  Action current_action = INITIALIZING;
} system_state;

struct ReservoirState
{
  // array containing the state of each slot (in the clockwise order)
  std::vector<bool> slots = {false, false, false, false, false, false, false, false, false, false, false, false, false, false, false};
  // head. At initialization, it is pointing to the first slot (which is the left slot of the grabber)
  int head = 0;
  int nb_stored = 0;
} reservoir_state;

void print_reservoir()
{
  // print reservoir state
  printf("\n");
  for (int i = 0; i < reservoir_state.slots.size(); i++)
  {
    printf("%s", reservoir_state.slots[i] ? "1" : "0");

  }
  printf("nb stored: %d\n", reservoir_state.nb_stored);
  printf("head: %d\n", reservoir_state.head);
  printf("\n");
}


void request_store_plants()
{

  printf("Storing requested\n");
  system_state.current_action = STORING;

  // Clear distance sensors buffers (?)
  sensors[LEFT].clear_interrupt();
  sensors[RIGHT].clear_interrupt();
}


void request_stop_storing()
{
  printf("Stop storing requested\n");
  if(system_state.current_action == FREE_CIRCLE_OUT)
  {
    printf("Interpreted as circle close\n");
    close_circle_plant();
  }

  system_state.current_action = FREE;
}

void request_plant_out()
{
  printf("Plant out requested\n");
  system_state.current_action = PUTTING_OUT;
}

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
  reservoir_rotate(2);
}

void lift_initialize_and_test()
{
  // Initialization steps:
  // 1. If button pressed, move up a little
  // 2. Move down until button is pressed

  stepper_lift.set_pos(0);

  // 1
  if(HAL_GPIO_ReadPin(F_COURSE_LIFT_GPIO_Port, F_COURSE_LIFT_Pin) == GPIO_PIN_RESET)
  {
	printf("Lift Goes Up...\n");
    stepper_lift.set_goal(3200);
    while(!stepper_lift.is_stopped())
    {
      stepper_lift.spin_once();
    }
  }

  printf("Lift Goes Down...\n");

  // 2
  lift_go_down();
}


void grabber_initialize_and_test()
{
  grabber_extend();
  HAL_Delay(1000);
  grabber_retract();
  HAL_Delay(1000);
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
  stepper_lift.set_goal(-100000);
  // Turn untill the button is pressed (reservoir in position)
  while(HAL_GPIO_ReadPin(F_COURSE_LIFT_GPIO_Port, F_COURSE_LIFT_Pin) == GPIO_PIN_SET)
  {
    stepper_lift.spin_once();
  }


  // Turn (offset)
  stepper_lift.set_pos(0);
  stepper_lift.set_goal(LIFT_OFFSET_FROM_BUTTON, true);
  while(!stepper_lift.is_stopped())
  {
    stepper_lift.spin_once();
  }

  stepper_lift.set_pos(0);
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

void reservoir_rotate(bool keep_speed=false)
{
  // Set big goal. When button is pressed, reset current pos to 0 and stop the motor
  stepper_res.set_pos(0);
  stepper_res.set_goal(100000, keep_speed);

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

  // Turn (offset)
  stepper_res.set_pos(0);
  stepper_res.set_goal(RES_OFFSET_FROM_BUTTON, true);
  while(!stepper_res.is_stopped())
  {
    stepper_res.spin_once();
  }

}

void reservoir_align_with_output()
{
  // Turn (offset)
  stepper_res.set_pos(0);
  stepper_res.set_goal(RES_OFFSET_FOR_OUTPUT);
  while(!stepper_res.is_stopped())
  {
    stepper_res.spin_once();
  }
}

void reservoir_realign_back()
{
  // Turn (offset)
  stepper_res.set_pos(0);
  stepper_res.set_goal(-RES_OFFSET_FOR_OUTPUT);
  while(!stepper_res.is_stopped())
  {
    stepper_res.spin_once();
  }
}


void reservoir_rotate(int n_slots)
{
  for(int i = 0; i < n_slots; i++)
  {
    reservoir_rotate(i>0);
  }
}

void close_fildefer()
{
  servos.WritePos(FILDEFER_ID, FILDEFER_POS_CLOSED, 1000);
}

void hide_fildefer()
{
  servos.WritePos(FILDEFER_ID, FILDEFER_POS_HIDDEN, 1000);
}

void fildefer_initialize()
{
  // servos.WritePos(FILDEFER_ID, FILDEFER_POS_HIDDEN, 1000);
  hide_fildefer();
  HAL_Delay(1000);
  close_fildefer();
  HAL_Delay(1000);
  hide_fildefer();
  HAL_Delay(1000);
}

void push_plant()
{
  servos.WritePos(SERVO_PUSH_PLANT_ID, SERVO_PUSH_POSE_PUSHED, 500);
}
void retract_servo_plant()
{
  servos.WritePos(SERVO_PUSH_PLANT_ID, SERVO_PUSH_POSE_RETRACTED, 200);
}

void open_circle_plant()
{
  servos.WritePos(SERVO_CIRCLE_PLANT_ID, SERVO_CIRCLE_POS_OPEN, 200);
}
void open_circle_plant_more()
{
  servos.WritePos(SERVO_CIRCLE_PLANT_ID, SERVO_CIRCLE_POS_OPEN_MORE, 200);
}
void close_circle_plant()
{
  servos.WritePos(SERVO_CIRCLE_PLANT_ID, SERVO_CIRCLE_POS_CLOSE, 200);
}

void pusher_and_circle_initialize()
{
  // init circle
  servos.WritePos(SERVO_CIRCLE_PLANT_ID, SERVO_CIRCLE_POS_CLOSE+50, 200);
  HAL_Delay(200);
  close_circle_plant();

  //init pusher
  retract_servo_plant();
  HAL_Delay(500);
  servos.WritePos(SERVO_PUSH_PLANT_ID, SERVO_PUSH_POSE_RETRACTED-50, 200);
  HAL_Delay(200);
  retract_servo_plant();
  HAL_Delay(500);
}

int search_next_plant_to_push()
{
  // the output is aligned with the head + 5
  // we search in reverse order till we find a plant
  for (int i = 0; i < reservoir_state.slots.size(); i++)
  {
    int index = (reservoir_state.head + 5 - i) % reservoir_state.slots.size();
    if (index < 0)
    {
      index += reservoir_state.slots.size();
    }

    if (reservoir_state.slots[index])
    {
      return i;
    }
  }
  return -1;
}

void push_one_plant_out()
{
  // search for a plant to push
  int nb_slot_till_next_plant = search_next_plant_to_push();
  printf("\n\n");
  printf("next plant to push: %d\n", nb_slot_till_next_plant);
  if (nb_slot_till_next_plant == -1)
  {
    system_state.current_action = FREE;
    return;
  }
  // HAL_Delay(2000);
  // rotate the reservoir to the right position
  printf("rotate the reservoir to the right position\n");
  reservoir_rotate(nb_slot_till_next_plant );
  // HAL_Delay(2000);


  // rotate the reservoir by the right offset
  printf("align reservoir \n");
  reservoir_align_with_output();
  // HAL_Delay(2000);
  printf("open circle \n");
  open_circle_plant();
  printf("push plant \n");
  push_plant();
  HAL_Delay(500);
  printf("retract servo plant \n");
  retract_servo_plant();
  printf("retract circle\n");
  open_circle_plant_more();
  HAL_Delay(200);
  reservoir_realign_back();

  // update reservoir state at the head + 5
  int slot = reservoir_state.head+5-nb_slot_till_next_plant;
  if (slot <0) {
    slot += reservoir_state.slots.size();
  } else {
    slot = slot % reservoir_state.slots.size();
  }
  printf("on veut enlever à %d \n",slot);
  reservoir_state.slots[(slot)] = false;
  reservoir_state.nb_stored--;
  reservoir_state.head = reservoir_state.head - nb_slot_till_next_plant;
  if (reservoir_state.head < 0)
  {
    reservoir_state.head += reservoir_state.slots.size();
  }
  else
  {
    reservoir_state.head = reservoir_state.head % reservoir_state.slots.size();
  }
  print_reservoir();
}



// =============================================== HIGH LEVEL ACTIONS =================================================


bool hopper_wait_and_close_spin_once(int side)
{
  // Check if distance < 50mm for left plant
  int dist = sensors[side].get_dist_mm();
  if(dist < 70)
  {
    // Close the hopper
    hopper_close(side);
    return true;
  }
  return false;
}



// /!\ DELAY IN THIS FUNCTION
void store_plants_spin_once()
{

  if(system_state.current_action != STORING)
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
    hide_fildefer();
    lift_go_up();
    grabber_extend();
    HAL_Delay(500);
    lift_go_middle();
    close_fildefer();
    grabber_retract(false);
    lift_go_down();
    // HAL_Delay(2000);
    hopper_open(LEFT);
    hopper_open(RIGHT);
    // HAL_Delay(1000);
    hide_fildefer();
    system_state.hopper_left_closed = false;
    system_state.hopper_right_closed = false;

    // Clear distance sensors buffers (?)
    sensors[LEFT].clear_interrupt();
    sensors[RIGHT].clear_interrupt();

    int nb_to_turn = 3;
    reservoir_rotate(nb_to_turn);
    print_reservoir();

    // fill the reservoir state. 2 have to be set to true, and one skipped
    reservoir_state.slots[reservoir_state.head] = true;
    reservoir_state.slots[(reservoir_state.head + 1) % reservoir_state.slots.size()] = true;
    reservoir_state.nb_stored += 2;

    reservoir_state.head = reservoir_state.head - nb_to_turn;
    printf("head:%d \n", reservoir_state.head);
    if (reservoir_state.head < 0)
    {
      printf("head += %d \n", reservoir_state.slots.size());
      reservoir_state.head += reservoir_state.slots.size();
    }
    else
    {
      reservoir_state.head = reservoir_state.head % reservoir_state.slots.size();
    }
    print_reservoir();
  }
}




void setup()
{

  printf("Setup Begins...\n");

  // Initialize the status message / set has_... fields to true
  status_msg = msgs_can_ActStatus_init_zero;
  status_msg.has_status = true;
  status_msg.status.has_status = true;
  status_msg.status.has_error = true;
  status_msg.has_plant_count = true;
  status_msg.has_action = true;

  status_msg.status.status = msgs_can_Status_StatusType_INIT;
  status_msg.status.error = msgs_can_Status_ErrorType_NONE;
  status_msg.action = msgs_can_ActActions_INITIALIZING;
  status_msg.plant_count = 0;



  // Init Steppers (and start the timer for the time_us function)
  HAL_TIM_Base_Start_IT(&htim2);
  stepper_lift.set_speed(5000);

  printf("Setup lasers...\n");

  // Initialize the sensors
  if(setup_lasers() != 0)
  {
    printf("Error setting up the sensors\n");
    Error_Handler();
  }

  printf("Setup servos...\n");

  if(ping_servos() != 0)
  {
    printf("Error pinging servos\n");
    Error_Handler();
  }

  champi_can = ChampiCan(&hfdcan1);


  if (champi_can.start() != 0) {
    // TODO: On n'a jamais rencontré cette erreur.
    Error_Handler();
  }

  // This is required: when the Raspberry Pi starts up, transmit CAN frames returns error.

  tx_ok_or_reset();

  printf("Setup CAN Done.\n");

  champi_state = ChampiState(&champi_can, 500);

  status_msg.status.status = msgs_can_Status_StatusType_OK;
  champi_state.report_status(status_msg);


  printf("Begin actuators initialization...\n");

  // Initialize / move actuators
  printf("Test pusher and circle...\n");
  pusher_and_circle_initialize();
  printf("Test fildefer...\n");
  fildefer_initialize();
  printf("Test reservoir...\n");
  reservoir_initialize_and_test();
  printf("Test hoppers...\n");
  hoppers_initialize_and_test();
  HAL_Delay(200);
  printf("Test grabber...\n");
  grabber_retract();
  printf("Test lift...\n");
  lift_initialize_and_test();

  printf("Initialization Done\n");


  // Switch led ON to indicate that we're running
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);

  status_msg.action = msgs_can_ActActions_FREE;
  champi_state.report_status(status_msg);

  print_reservoir();
}



void loop()
{
  // Print distances
  // printf("Left: %d mm, Right: %d mm\n", sensors[LEFT].get_dist_mm(), sensors[RIGHT].get_dist_mm());



  // Update champi_state

  if(system_state.current_action==FREE)
  {
    status_msg.action = msgs_can_ActActions_FREE;
  }
  else if(system_state.current_action==STORING)
  {
    status_msg.action = msgs_can_ActActions_START_GRAB_PLANTS;
  }
  else if(system_state.current_action==PUTTING_OUT)
  {
    status_msg.action = msgs_can_ActActions_RELEASE_PLANT;
  }

  status_msg.plant_count = reservoir_state.nb_stored;

  champi_state.report_status(status_msg);

  champi_state.spin_once(); // Send status on CAN bus




  store_plants_spin_once();

  if (system_state.current_action==PUTTING_OUT)
  {

    push_one_plant_out();
    system_state.current_action = FREE_CIRCLE_OUT;
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

  setup();



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */



    while (1)
    {

      loop();

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
  hfdcan1.Init.AutoRetransmission = ENABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 10;
  hfdcan1.Init.NominalSyncJumpWidth = 1;
  hfdcan1.Init.NominalTimeSeg1 = 14;
  hfdcan1.Init.NominalTimeSeg2 = 2;
  hfdcan1.Init.DataPrescaler = 10;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 14;
  hfdcan1.Init.DataTimeSeg2 = 2;
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

  /*Configure GPIO pins : F_COURSE_LIFT_Pin FIN_COURSE_RES_Pin */
  GPIO_InitStruct.Pin = F_COURSE_LIFT_Pin|FIN_COURSE_RES_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
