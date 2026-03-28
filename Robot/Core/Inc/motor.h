#ifndef __MOTOR_H
#define __MOTOR_H

#include "stm32f4xx_hal.h"
#include "main.h"

/* Motor control definitions */
typedef enum {
  MOTOR_STOP = 0,
  MOTOR_FORWARD = 1,
  MOTOR_BACKWARD = 2
} MotorDirection_t;

typedef struct {
  GPIO_TypeDef* direction_port_a;
  uint16_t direction_pin_a;
  GPIO_TypeDef* direction_port_b;
  uint16_t direction_pin_b;
  TIM_HandleTypeDef* timer;
  uint32_t channel;
} Motor_t;

/* Initialize motor control module */
void motor_init(TIM_HandleTypeDef* tim3);

/* Set individual motor direction and speed */
void motor_set_m1(MotorDirection_t direction, uint16_t speed);
void motor_set_m2(MotorDirection_t direction, uint16_t speed);

/* High-level continuous control functions */
void motor_forward(uint16_t speed);
void motor_backward(uint16_t speed);
void motor_turn_left(uint16_t speed);
void motor_turn_right(uint16_t speed);
void motor_stop(void);

/* Timed move functions (non-blocking) */
void motor_forward_1cell(uint16_t speed);
void motor_backward_1cell(uint16_t speed);
void motor_turn_left_90(uint16_t speed);
void motor_turn_right_90(uint16_t speed);

/* Call both of these every iteration of your main loop */
void motor_tick(void);
void motor_auto_tick(void);   /* autonomous logic, uses sonar readings */

uint8_t motor_is_busy(void);
uint8_t motor_auto_is_on(void);

/* USART remote control — 'p'/'P' toggles autonomous mode */
void motor_usart_command(uint8_t cmd);

#endif /* __MOTOR_H */
