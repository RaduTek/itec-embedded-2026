#ifndef __MAZE_SOLVE_H__
#define __MAZE_SOLVE_H__

#include <stdint.h>

#define MAZE_SOLVE_SIZE 10U

void maze_solve_set_map_text(const char* text);
const char* maze_solve_get_map_text(void);

uint8_t maze_solve_parse_map_text(const char* text, int out[MAZE_SOLVE_SIZE][MAZE_SOLVE_SIZE]);
uint16_t maze_solve_serialize_map_text(const int maze[MAZE_SOLVE_SIZE][MAZE_SOLVE_SIZE], char* out, uint16_t out_size);

#endif /* __MAZE_SOLVE_H__ */