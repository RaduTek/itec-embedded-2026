#include "motor.h"

#define MOTOR_PWM_MAX  65535U

/* Timed move durations - tune these to your robot */
#define MOTOR_1CELL_MS    150U   /* ms to travel one cell forward/backward */
#define MOTOR_90DEG_MS    100U   /* ms to rotate 90 degrees */


static inline uint16_t percent_speed(uint8_t percent)
{
    return (uint16_t)((uint32_t)percent * MOTOR_PWM_MAX / 100U);
}

static Motor_t m1, m2;
static TIM_HandleTypeDef* tim_ptr;

/* --- Timed move state --- */
static uint8_t  timed_move_active = 0;
static uint32_t timed_move_start  = 0;
static uint32_t timed_move_dur    = 0;

void motor_init(TIM_HandleTypeDef* tim3)
{
  tim_ptr = tim3;

  /* Motor 1 configuration */
  m1.direction_port_a = GPIOB;
  m1.direction_pin_a = M1_A_Pin;  // M1_A
  m1.direction_port_b = GPIOB;
  m1.direction_pin_b = M1_B_Pin;  // M1_B
  m1.timer = tim3;
  m1.channel = TIM_CHANNEL_3;  // PC8

  /* Motor 2 configuration */
  m2.direction_port_a = GPIOB;
  m2.direction_pin_a = M2_A_Pin;  // M2_A
  m2.direction_port_b = GPIOB;
  m2.direction_pin_b = M2_B_Pin;  // M2_B (changed from PB14 to PB15)
  m2.timer = tim3;
  m2.channel = TIM_CHANNEL_4;  // PC9

  /* Start PWM outputs for both motors */
  HAL_TIM_PWM_Start(tim3, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(tim3, TIM_CHANNEL_4);

  /* Initialize all direction pins to LOW */
  HAL_GPIO_WritePin(m1.direction_port_a, m1.direction_pin_a, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(m1.direction_port_b, m1.direction_pin_b, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(m2.direction_port_a, m2.direction_pin_a, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(m2.direction_port_b, m2.direction_pin_b, GPIO_PIN_RESET);

  /* Set initial speed to 0 */
  __HAL_TIM_SET_COMPARE(tim3, TIM_CHANNEL_3, 0);
  __HAL_TIM_SET_COMPARE(tim3, TIM_CHANNEL_4, 0);
}

void motor_set_m1(MotorDirection_t direction, uint16_t speed)
{
  switch(direction) {
    case MOTOR_FORWARD:
      HAL_GPIO_WritePin(m1.direction_port_a, m1.direction_pin_a, GPIO_PIN_SET);
      HAL_GPIO_WritePin(m1.direction_port_b, m1.direction_pin_b, GPIO_PIN_RESET);
      break;
    case MOTOR_BACKWARD:
      HAL_GPIO_WritePin(m1.direction_port_a, m1.direction_pin_a, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(m1.direction_port_b, m1.direction_pin_b, GPIO_PIN_SET);
      break;
    case MOTOR_STOP:
    default:
      HAL_GPIO_WritePin(m1.direction_port_a, m1.direction_pin_a, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(m1.direction_port_b, m1.direction_pin_b, GPIO_PIN_RESET);
      break;
  }
  __HAL_TIM_SET_COMPARE(m1.timer, m1.channel, speed);
}

void motor_set_m2(MotorDirection_t direction, uint16_t speed)
{
  switch(direction) {
    case MOTOR_FORWARD:
      HAL_GPIO_WritePin(m2.direction_port_a, m2.direction_pin_a, GPIO_PIN_SET);
      HAL_GPIO_WritePin(m2.direction_port_b, m2.direction_pin_b, GPIO_PIN_RESET);
      break;
    case MOTOR_BACKWARD:
      HAL_GPIO_WritePin(m2.direction_port_a, m2.direction_pin_a, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(m2.direction_port_b, m2.direction_pin_b, GPIO_PIN_SET);
      break;
    case MOTOR_STOP:
    default:
      HAL_GPIO_WritePin(m2.direction_port_a, m2.direction_pin_a, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(m2.direction_port_b, m2.direction_pin_b, GPIO_PIN_RESET);
      break;
  }
  __HAL_TIM_SET_COMPARE(m2.timer, m2.channel, speed);
}

static void timed_move_start_fn(uint32_t duration_ms)
{
  timed_move_active = 1;
  timed_move_start  = HAL_GetTick();
  timed_move_dur    = duration_ms;
}

void motor_forward_1cell(uint16_t speed)
{
  motor_set_m1(MOTOR_FORWARD, speed);
  motor_set_m2(MOTOR_FORWARD, speed);
  timed_move_start_fn(MOTOR_1CELL_MS);
}

void motor_backward_1cell(uint16_t speed)
{
  motor_set_m1(MOTOR_BACKWARD, speed);
  motor_set_m2(MOTOR_BACKWARD, speed);
  timed_move_start_fn(MOTOR_1CELL_MS);
}

void motor_turn_left_90(uint16_t speed)
{
  motor_set_m1(MOTOR_BACKWARD, speed);
  motor_set_m2(MOTOR_FORWARD, speed);
  timed_move_start_fn(MOTOR_90DEG_MS);
}

void motor_turn_right_90(uint16_t speed)
{
  motor_set_m1(MOTOR_FORWARD, speed);
  motor_set_m2(MOTOR_BACKWARD, speed);
  timed_move_start_fn(MOTOR_90DEG_MS);
}

void motor_tick(void)
{
  if (!timed_move_active) return;

  if ((HAL_GetTick() - timed_move_start) >= timed_move_dur) {
    motor_stop();
    timed_move_active = 0;
  }
}

uint8_t motor_is_busy(void)
{
  return timed_move_active;
}


void motor_forward(uint16_t speed)
{
  motor_set_m1(MOTOR_FORWARD, speed);
  motor_set_m2(MOTOR_FORWARD, speed);
}

void motor_backward(uint16_t speed)
{
  motor_set_m1(MOTOR_BACKWARD, speed);
  motor_set_m2(MOTOR_BACKWARD, speed);
}

void motor_turn_left(uint16_t speed)
{
  /* Left wheel backward, right wheel forward for CCW rotation */
  motor_set_m1(MOTOR_BACKWARD, speed);
  motor_set_m2(MOTOR_FORWARD, speed);
}

void motor_turn_right(uint16_t speed)
{
  /* Left wheel forward, right wheel backward for CW rotation */
  motor_set_m1(MOTOR_FORWARD, speed);
  motor_set_m2(MOTOR_BACKWARD, speed);
}

void motor_stop(void)
{
  motor_set_m1(MOTOR_STOP, 0);
  motor_set_m2(MOTOR_STOP, 0);
}


void motor_usart_command(uint8_t cmd)
{
  uint16_t speed      = percent_speed(55);  /* ~60% speed for forward/backward */
  uint16_t turn_speed = percent_speed(40);  /* ~38% speed for turns (softer cornering) */

 
  switch(cmd) {
  case 'w': case 'W': motor_forward(speed);                          break;
  case 's': case 'S': motor_backward(speed);                         break;
  case 'a': case 'A': motor_turn_left(turn_speed);                   break;
  case 'd': case 'D': motor_turn_right(turn_speed);                  break;
  case 'x': case 'X': motor_stop(); timed_move_active = 0;           break;
  case 't': case 'T':
    if (!timed_move_active) motor_forward_1cell(speed);
    break;
  case 'g': case 'G':
    if (!timed_move_active) motor_backward_1cell(speed);
    break;
  case 'f': case 'F':
    if (!timed_move_active) motor_turn_left_90(turn_speed);
    break;
  case 'h': case 'H':
    if (!timed_move_active) motor_turn_right_90(turn_speed);
    break;
  default: break;
}
}
