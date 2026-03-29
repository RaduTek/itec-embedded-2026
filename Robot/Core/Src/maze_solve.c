#include "maze_solve.h"

#include <stdio.h>
#include <string.h>

#define MAZE_TEXT_BUFFER_SIZE 512U

static char g_maze_text[MAZE_TEXT_BUFFER_SIZE];

void maze_solve_set_map_text(const char* text)
{
	if (text == NULL) {
		g_maze_text[0] = '\0';
		return;
	}

	strncpy(g_maze_text, text, MAZE_TEXT_BUFFER_SIZE - 1U);
	g_maze_text[MAZE_TEXT_BUFFER_SIZE - 1U] = '\0';
}

const char* maze_solve_get_map_text(void)
{
	return g_maze_text;
}

uint8_t maze_solve_parse_map_text(const char* text, int out[MAZE_SOLVE_SIZE][MAZE_SOLVE_SIZE])
{
	uint8_t row;
	uint8_t col;
	int consumed = 0;
	int value;

	if ((text == NULL) || (out == NULL)) {
		return 0U;
	}

	for (row = 0U; row < MAZE_SOLVE_SIZE; row++) {
		for (col = 0U; col < MAZE_SOLVE_SIZE; col++) {
			if (sscanf(text, " %d%n", &value, &consumed) != 1) {
				return 0U;
			}

			if (value < 0) value = 0;
			if (value > 15) value = 15;
			out[row][col] = value;

			text += consumed;
		}
	}

	return 1U;
}

uint16_t maze_solve_serialize_map_text(const int maze[MAZE_SOLVE_SIZE][MAZE_SOLVE_SIZE], char* out, uint16_t out_size)
{
	uint8_t row;
	uint8_t col;
	int written;
	uint16_t index = 0U;

	if ((maze == NULL) || (out == NULL) || (out_size == 0U)) {
		return 0U;
	}

	out[0] = '\0';

	for (row = 0U; row < MAZE_SOLVE_SIZE; row++) {
		for (col = 0U; col < MAZE_SOLVE_SIZE; col++) {
			written = snprintf(&out[index], (size_t)(out_size - index), "%d%s",
												 maze[row][col],
												 (col == (MAZE_SOLVE_SIZE - 1U)) ? "" : " ");
			if ((written < 0) || ((uint16_t)written >= (out_size - index))) {
				out[out_size - 1U] = '\0';
				return index;
			}

			index += (uint16_t)written;
		}

		written = snprintf(&out[index], (size_t)(out_size - index), "\n");
		if ((written < 0) || ((uint16_t)written >= (out_size - index))) {
			out[out_size - 1U] = '\0';
			return index;
		}

		index += (uint16_t)written;
	}

	return index;
}
