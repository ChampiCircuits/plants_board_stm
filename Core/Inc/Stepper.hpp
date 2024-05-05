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

  void set_speed(unsigned long speed) {
    this->speed = speed;
    time_step = 10000000 / speed;
  }

  void set_goal(int goal, bool keep_previous_speed = false) {

    this->goal = goal;
    state.state = State::HIGH;
    state.direction = goal > state.pos ? 1 : -1;
    time_start_step = get_time_us();
    time_start_high = time_start_step;
    HAL_GPIO_WritePin(gpio_port_dir, gpio_pin_dir, state.direction == 1 ? GPIO_PIN_RESET : GPIO_PIN_SET);

    if (keep_previous_speed) {
      current_speed = speed_when_stopped;
    } else {
      current_speed = 0;
    }
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

    compute_time_step();

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
      speed_when_stopped = current_speed;
      current_speed = 0;
    }
  }

  bool is_stopped() {
    return state.state == State::STOPPED;
  }

  void compute_time_step() {

    // We compute the new speed every 5ms (return if we are not there yet)
    static unsigned long last_time = 0;
    if (get_time_us() - last_time < 5000 && last_time != 0) {
      return;
    }
    last_time = get_time_us();


    if (current_speed == speed) {
      return;
    }

    if (current_speed < speed) {
      current_speed += (long) (((double) max_acceleration) * 0.005); // 5ms (0.005s
      if (current_speed > speed) {
        current_speed = speed;
      }
    } else {
      current_speed -= max_acceleration;
      if (current_speed < speed) {
        current_speed = speed;
      }
    }

    time_step = 10000000 / current_speed;
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
    unsigned long time_step;
    unsigned long time_high = 10; // us


    unsigned long time_start_step = 0;
    unsigned long time_start_high = 0;

    long current_speed = 0;
    long speed_when_stopped = 0;
  long max_acceleration = 500; // step/s^2

  unsigned long (*get_time_us)();



 };

#endif /* INC_STEPPER_HPP_ */
