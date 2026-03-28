#include "sonar.h"

bool sonar_init_done = false;
SonarDriver_t sonar_driver = {0};

void sonar_init(GPIO_TypeDef* trigger_port, uint16_t trigger_pin, TIM_HandleTypeDef* timer)
{
  sonar_driver.trigger_port = trigger_port;
  sonar_driver.trigger_pin = trigger_pin;
  sonar_driver.timer = timer;

  // Initialize all sensor structures
  for (int i = 0; i < 4; i++) {
    sonar_driver.sensors[i].echo_start = 0;
    sonar_driver.sensors[i].echo_end = 0;
    sonar_driver.sensors[i].distance_cm = 0;
  }

  // Start the timer
  HAL_TIM_Base_Start(timer);

  sonar_init_done = true;
}

void sonar_trigger(void)
{
  if (!sonar_init_done) return;

  // Send 10µs pulse on trigger pin
  HAL_GPIO_WritePin(sonar_driver.trigger_port, sonar_driver.trigger_pin, GPIO_PIN_SET);
  
  // Wait 10µs (at system clock, this is a simple delay)
  uint32_t start = __HAL_TIM_GET_COUNTER(sonar_driver.timer);
  while (__HAL_TIM_GET_COUNTER(sonar_driver.timer) - start < 10);
  
  HAL_GPIO_WritePin(sonar_driver.trigger_port, sonar_driver.trigger_pin, GPIO_PIN_RESET);
}

void sonar_echo_callback(uint8_t sensor_index, uint8_t edge)
{
  if (!sonar_init_done) return;
  
  if (sensor_index >= 4) return;
  
  uint32_t current_count = __HAL_TIM_GET_COUNTER(sonar_driver.timer);
  
  // edge parameter: non-zero = high/rising, zero = low/falling
  if (edge != 0) {
    // Rising edge - echo starts
    sonar_driver.sensors[sensor_index].echo_start = current_count;
  } else {
    // Falling edge - echo ends
    sonar_driver.sensors[sensor_index].echo_end = current_count;
  }
}

uint16_t sonar_get_distance(uint8_t sensor_index)
{
  if (!sonar_init_done) return 0;
  
  if (sensor_index >= 4) return 0;
  
  uint32_t echo_start = sonar_driver.sensors[sensor_index].echo_start;
  uint32_t echo_end = sonar_driver.sensors[sensor_index].echo_end;
  
  // Calculate pulse width in microseconds (timer runs at ~1µs per tick with prescaler 83)
  uint32_t pulse_width;
  
  if (echo_end >= echo_start) {
    pulse_width = echo_end - echo_start;
  } else {
    // Handle timer overflow
    pulse_width = (0xFFFFFFFF - echo_start) + echo_end;
  }
  
  // Convert to distance: distance = (pulse_width / 2) / 29.1 cm/µs
  // Using: distance = pulse_width / 58.2
  uint16_t distance = pulse_width / 58;
  
  // Limit to reasonable values
  if (distance > 400) distance = 400;
  
  sonar_driver.sensors[sensor_index].distance_cm = distance;
  
  return distance;
}
