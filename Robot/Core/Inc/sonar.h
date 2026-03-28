#ifndef __SONAR_H__
#define __SONAR_H__

#include <stdbool.h>
#include "main.h"

typedef struct {
  uint32_t echo_start;
  uint32_t echo_end;
  uint16_t distance_cm;
} SonarSensor_t;

typedef struct {
  SonarSensor_t sensors[4];
  GPIO_TypeDef* trigger_port;
  uint16_t trigger_pin;
  TIM_HandleTypeDef* timer;
} SonarDriver_t;

extern bool sonar_init_done;
extern SonarDriver_t sonar_driver;

void sonar_init(GPIO_TypeDef* trigger_port, uint16_t trigger_pin, TIM_HandleTypeDef* timer);
void sonar_trigger(void);
void sonar_echo_callback(uint8_t sensor_index, uint8_t edge);
uint16_t sonar_get_distance(uint8_t sensor_index);

#endif /* __SONAR_H__ */
