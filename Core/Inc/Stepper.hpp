/*
 * Stepper.hpp
 *
 *  Created on: Apr 28, 2024
 *      Author: andre
 */

#ifndef INC_STEPPER_HPP_
#define INC_STEPPER_HPP_



#include "stm32g4xx_hal.h"



class Stepper {
public:
  Stepper(unsigned long (*get_time_us)(), GPIO_TypeDef *gpio_port_step, uint16_t gpio_pin_step, GPIO_TypeDef *gpio_port_dir, uint16_t gpio_pin_dir) {
    this->gpio_port_step = gpio_port_step;
    this->gpio_pin_step = gpio_pin_step;
    this->gpio_port_dir = gpio_port_dir;
    this->gpio_pin_dir = gpio_pin_dir;

    this->get_time_us = get_time_us;

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

  int get_pos() {
    return state.pos;
  }

  void set_pos(int pos) {
    state.pos = pos;
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

  unsigned long (*get_time_us)();



 };

#endif /* INC_STEPPER_HPP_ */
