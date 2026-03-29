#include "motor.h"
#include "sonar.h"

#define MOTOR_PWM_MAX  65535U

/* Timed move durations */
#define MOTOR_1CELL_MS      150U   /* ms to travel one cell forward/backward */
#define MOTOR_90DEG_MS      100U   /* ms to rotate 90 degrees */
#define MOTOR_NUDGE_MS       60U   /* ms for a small correction nudge */

/* Autonomous mode thresholds (cm) */
#define AUTO_FRONT_WALL_CM   20U   /* stop and decide if front closer than this */
#define AUTO_SIDE_WALL_CM     8U   /* nudge away if side closer than this */
#define AUTO_BOTH_BLOCKED_CM 15U   /* consider a side "blocked" for path selection */

/* Sensor indices */
#define SONAR_FRONT  1
#define SONAR_BACK   0
#define SONAR_LEFT   3
#define SONAR_RIGHT  2


static inline uint16_t percent_speed(uint8_t percent)
{
    return (uint16_t)((uint32_t)percent * MOTOR_PWM_MAX / 100U);
}

static Motor_t m1, m2;
static TIM_HandleTypeDef* tim_ptr;

/* --- Motion LEDs (1-hot encoding on STM32F4-Discovery LEDs) ---
 * Forward  -> RED   (LD5)
 * Right    -> BLUE  (LD6)
 * Left     -> GREEN (LD4)
 * Backward -> ORANGE (LD3, used as "white" fallback)
 */
typedef enum {
  MOTOR_LED_IDLE = 0,
  MOTOR_LED_FORWARD,
  MOTOR_LED_RIGHT,
  MOTOR_LED_LEFT,
  MOTOR_LED_BACKWARD
} MotorLedState_t;

static void motor_led_set(MotorLedState_t state)
{
  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LD6_GPIO_Port, LD6_Pin, GPIO_PIN_RESET);

  switch (state) {
    case MOTOR_LED_FORWARD:
      HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, GPIO_PIN_SET); /* red */
      break;
    case MOTOR_LED_RIGHT:
      HAL_GPIO_WritePin(LD6_GPIO_Port, LD6_Pin, GPIO_PIN_SET); /* blue */
      break;
    case MOTOR_LED_LEFT:
      HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_SET); /* green */
      break;
    case MOTOR_LED_BACKWARD:
      HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET); /* orange fallback */
      break;
    case MOTOR_LED_IDLE:
    default:
      break;
  }
}

/* --- Timed move state --- */
static uint8_t  timed_move_active = 0;
static uint32_t timed_move_start  = 0;
static uint32_t timed_move_dur    = 0;

/* --- Autonomous mode state --- */
static uint8_t  auto_mode = 0;


void motor_init(TIM_HandleTypeDef* tim3)
{
  tim_ptr = tim3;

  m1.direction_port_a = GPIOB;
  m1.direction_pin_a  = M1_A_Pin;
  m1.direction_port_b = GPIOB;
  m1.direction_pin_b  = M1_B_Pin;
  m1.timer            = tim3;
  m1.channel          = TIM_CHANNEL_3;

  m2.direction_port_a = GPIOB;
  m2.direction_pin_a  = M2_A_Pin;
  m2.direction_port_b = GPIOB;
  m2.direction_pin_b  = M2_B_Pin;
  m2.timer            = tim3;
  m2.channel          = TIM_CHANNEL_4;

  HAL_TIM_PWM_Start(tim3, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(tim3, TIM_CHANNEL_4);

  HAL_GPIO_WritePin(m1.direction_port_a, m1.direction_pin_a, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(m1.direction_port_b, m1.direction_pin_b, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(m2.direction_port_a, m2.direction_pin_a, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(m2.direction_port_b, m2.direction_pin_b, GPIO_PIN_RESET);

  __HAL_TIM_SET_COMPARE(tim3, TIM_CHANNEL_3, 0);
  __HAL_TIM_SET_COMPARE(tim3, TIM_CHANNEL_4, 0);

  motor_led_set(MOTOR_LED_IDLE);
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

/* --- Timed move helpers --- */

static void timed_move_start_fn(uint32_t duration_ms)
{
  timed_move_active = 1;
  timed_move_start  = HAL_GetTick();
  timed_move_dur    = duration_ms;
}

void motor_forward_1cell(uint16_t speed)
{
  motor_led_set(MOTOR_LED_FORWARD);
  motor_set_m1(MOTOR_FORWARD, speed);
  motor_set_m2(MOTOR_FORWARD, speed);
  timed_move_start_fn(MOTOR_1CELL_MS);
}

void motor_backward_1cell(uint16_t speed)
{
  motor_led_set(MOTOR_LED_BACKWARD);
  motor_set_m1(MOTOR_BACKWARD, speed);
  motor_set_m2(MOTOR_BACKWARD, speed);
  timed_move_start_fn(MOTOR_1CELL_MS);
}

void motor_turn_left_90(uint16_t speed)
{
  motor_led_set(MOTOR_LED_LEFT);
  motor_set_m1(MOTOR_BACKWARD, speed);
  motor_set_m2(MOTOR_FORWARD, speed);
  timed_move_start_fn(MOTOR_90DEG_MS);
}

void motor_turn_right_90(uint16_t speed)
{
  motor_led_set(MOTOR_LED_RIGHT);
  motor_set_m1(MOTOR_FORWARD, speed);
  motor_set_m2(MOTOR_BACKWARD, speed);
  timed_move_start_fn(MOTOR_90DEG_MS);
}

static void motor_nudge_left(uint16_t speed)
{
  /* Brief turn left to move away from right wall */
  motor_set_m1(MOTOR_BACKWARD, speed);
  motor_set_m2(MOTOR_FORWARD, speed);
  timed_move_start_fn(MOTOR_NUDGE_MS);
}

static void motor_nudge_right(uint16_t speed)
{
  /* Brief turn right to move away from left wall */
  motor_set_m1(MOTOR_FORWARD, speed);
  motor_set_m2(MOTOR_BACKWARD, speed);
  timed_move_start_fn(MOTOR_NUDGE_MS);
}

/* --- Main tick: stops timed moves when duration expires --- */
void motor_tick(void)
{
  if (!timed_move_active) return;

  if ((HAL_GetTick() - timed_move_start) >= timed_move_dur) {
    motor_stop();
    timed_move_active = 0;
  }
}

/* --- Autonomous tick: call after motor_tick() in main loop --- */
void motor_auto_tick(void)
{
  if (!auto_mode)        return;  /* autonomous mode off */
  if (timed_move_active) return;  /* wait for current move to finish */

  uint16_t speed      = percent_speed(39);
  uint16_t turn_speed = percent_speed(39);

  uint16_t dist_front = sonar_get_distance(SONAR_FRONT);
  uint16_t dist_left  = sonar_get_distance(SONAR_LEFT);
  uint16_t dist_right = sonar_get_distance(SONAR_RIGHT);

  /* --- Priority 1: front wall approaching --- */
  if (dist_front < AUTO_FRONT_WALL_CM) {

    uint8_t left_open  = (dist_left  > AUTO_BOTH_BLOCKED_CM);
    uint8_t right_open = (dist_right > AUTO_BOTH_BLOCKED_CM);

    if (left_open && right_open) {
      /* Both open: pick the side with more space */
      if (dist_left >= dist_right)
        motor_turn_left_90(turn_speed);
      else
        motor_turn_right_90(turn_speed);

    } else if (left_open) {
      motor_turn_left_90(turn_speed);

    } else if (right_open) {
      motor_turn_right_90(turn_speed);

    } else {
      /* All three sides blocked: back up */
      motor_turn_right_90(turn_speed);
      motor_turn_right_90(turn_speed);
    }

    return;
  }

  /* --- Priority 2: side wall correction --- */
//  if (dist_left < AUTO_SIDE_WALL_CM) {
//    /* Too close to left wall: nudge right */
//    motor_nudge_right(turn_speed);
//    return;
//  }
//
//  if (dist_right < AUTO_SIDE_WALL_CM) {
//    /* Too close to right wall: nudge left */
//    motor_nudge_left(turn_speed);
//    return;
//  }

  /* --- Priority 3: path is clear, move forward --- */
  motor_forward_1cell(speed);
}

uint8_t motor_is_busy(void)
{
  return timed_move_active;
}

uint8_t motor_auto_is_on(void)
{
  return auto_mode;
}

/* --- Continuous control --- */

void motor_forward(uint16_t speed)
{
  motor_led_set(MOTOR_LED_FORWARD);
  motor_set_m1(MOTOR_FORWARD, speed);
  motor_set_m2(MOTOR_FORWARD, speed);
}

void motor_backward(uint16_t speed)
{
  motor_led_set(MOTOR_LED_BACKWARD);
  motor_set_m1(MOTOR_BACKWARD, speed);
  motor_set_m2(MOTOR_BACKWARD, speed);
}

void motor_turn_left(uint16_t speed)
{
  motor_led_set(MOTOR_LED_LEFT);
  motor_set_m1(MOTOR_BACKWARD, speed);
  motor_set_m2(MOTOR_FORWARD, speed);
}

void motor_turn_right(uint16_t speed)
{
  motor_led_set(MOTOR_LED_RIGHT);
  motor_set_m1(MOTOR_FORWARD, speed);
  motor_set_m2(MOTOR_BACKWARD, speed);
}

void motor_stop(void)
{
  motor_set_m1(MOTOR_STOP, 0);
  motor_set_m2(MOTOR_STOP, 0);
  motor_led_set(MOTOR_LED_IDLE);
}

/* --- USART command handler --- */
void motor_usart_command(uint8_t cmd)
{
  uint16_t speed      = percent_speed(45);
  uint16_t turn_speed = percent_speed(40);

  switch(cmd) {
    case 'p': case 'P':
      auto_mode = !auto_mode;
      if (!auto_mode) { motor_stop(); timed_move_active = 0; }
      break;

    /* All manual movement commands below disable auto mode first */
    case 'w': case 'W':
      auto_mode = 0;
      motor_forward(speed);
      break;
    case 's': case 'S':
      auto_mode = 0;
      motor_backward(speed);
      break;
    case 'a': case 'A':
      auto_mode = 0;
      motor_turn_left(turn_speed);
      break;
    case 'd': case 'D':
      auto_mode = 0;
      motor_turn_right(turn_speed);
      break;
    case 'x': case 'X':
      auto_mode = 0;
      motor_stop();
      timed_move_active = 0;
      break;
    case 't': case 'T':
      auto_mode = 0;
      if (!timed_move_active) motor_forward_1cell(speed);
      break;
    case 'g': case 'G':
      auto_mode = 0;
      if (!timed_move_active) motor_backward_1cell(speed);
      break;
    case 'f': case 'F':
      auto_mode = 0;
      if (!timed_move_active) motor_turn_left_90(turn_speed);
      break;
    case 'h': case 'H':
      auto_mode = 0;
      if (!timed_move_active) motor_turn_right_90(turn_speed);
      break;
    default: break;
  }
}
