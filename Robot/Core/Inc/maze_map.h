#ifndef __MAZE_MAP_H__
#define __MAZE_MAP_H__

#include <stdint.h>
#include "main.h"

#define MAZE_MAP_SIZE 10U

typedef enum {
  ACTION_IDLE = 0,
  ACTION_DELAY,
  ACTION_TURN,
  ACTION_FORWARD,
  ACTION_FINISH_FORWARD
} ActionState_t;

void maze_map_init(UART_HandleTypeDef* huart);
void maze_map_start(void);
void maze_map_stop(void);
void maze_map_tick(void);

uint8_t maze_map_is_running(void);
uint8_t maze_map_is_done(void);

const uint8_t (*maze_map_get_matrix(void))[MAZE_MAP_SIZE];
void maze_map_copy_int_matrix(int out[MAZE_MAP_SIZE][MAZE_MAP_SIZE]);
uint16_t maze_map_export_text(char* out, uint16_t out_size);

#endif /* __MAZE_MAP_H__ */