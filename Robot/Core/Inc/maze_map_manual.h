#ifndef __MAZE_MAP_MANUAL_H__
#define __MAZE_MAP_MANUAL_H__

#include <stdint.h>
#include "main.h"

#define MAZE_MANUAL_SIZE 10U

void maze_map_manual_init(UART_HandleTypeDef* huart);
void maze_map_manual_reset(void);
uint8_t maze_map_manual_scan_next(void);
uint8_t maze_map_manual_is_done(void);
uint16_t maze_map_manual_export_text(char* out, uint16_t out_size);

#endif /* __MAZE_MAP_MANUAL_H__ */