#include "maze_map_manual.h"

#include <stdio.h>
#include <string.h>

#include "sonar.h"

#define WALL_NORTH 0x1U
#define WALL_EAST  0x2U
#define WALL_SOUTH 0x4U
#define WALL_WEST  0x8U

#define WALL_THRESHOLD_CM 9U

/* Sensor indices per requested mapping */
#define SONAR_BACK   0U
#define SONAR_FRONT  1U
#define SONAR_LEFT   2U
#define SONAR_RIGHT  3U

static UART_HandleTypeDef* g_uart = NULL;
static uint8_t g_maze[MAZE_MANUAL_SIZE][MAZE_MANUAL_SIZE];
static uint8_t g_scan_index = 0U; /* 0..99 */

static void uart_print(const char* text)
{
	if ((g_uart == NULL) || (text == NULL)) return;
	HAL_UART_Transmit(g_uart, (uint8_t*)text, (uint16_t)strlen(text), HAL_MAX_DELAY);
}

static void set_cell_wall(uint8_t x, uint8_t y, uint8_t wall_bit)
{
	if ((x >= MAZE_MANUAL_SIZE) || (y >= MAZE_MANUAL_SIZE)) return;
	g_maze[y][x] |= wall_bit;
}

static void set_wall_with_neighbor(uint8_t x, uint8_t y, uint8_t wall_bit)
{
	set_cell_wall(x, y, wall_bit);

	if (wall_bit == WALL_NORTH) {
		if (y > 0U) set_cell_wall(x, (uint8_t)(y - 1U), WALL_SOUTH);
	} else if (wall_bit == WALL_EAST) {
		if (x < (MAZE_MANUAL_SIZE - 1U)) set_cell_wall((uint8_t)(x + 1U), y, WALL_WEST);
	} else if (wall_bit == WALL_SOUTH) {
		if (y < (MAZE_MANUAL_SIZE - 1U)) set_cell_wall(x, (uint8_t)(y + 1U), WALL_NORTH);
	} else if (wall_bit == WALL_WEST) {
		if (x > 0U) set_cell_wall((uint8_t)(x - 1U), y, WALL_EAST);
	}
}

static void map_current_cell(uint8_t x, uint8_t y)
{
	uint16_t d_front = sonar_get_distance(SONAR_FRONT);
	uint16_t d_back = sonar_get_distance(SONAR_BACK);
	uint16_t d_left = sonar_get_distance(SONAR_LEFT);
	uint16_t d_right = sonar_get_distance(SONAR_RIGHT);

	/* Robot front always points North in manual mode */
	if ((y == 0U) || (d_front <= WALL_THRESHOLD_CM)) set_wall_with_neighbor(x, y, WALL_NORTH);
	if ((x == (MAZE_MANUAL_SIZE - 1U)) || (d_right <= WALL_THRESHOLD_CM)) set_wall_with_neighbor(x, y, WALL_EAST);
	if ((y == (MAZE_MANUAL_SIZE - 1U)) || (d_back <= WALL_THRESHOLD_CM)) set_wall_with_neighbor(x, y, WALL_SOUTH);
	if ((x == 0U) || (d_left <= WALL_THRESHOLD_CM)) set_wall_with_neighbor(x, y, WALL_WEST);
}

void maze_map_manual_init(UART_HandleTypeDef* huart)
{
	g_uart = huart;
	maze_map_manual_reset();
}

void maze_map_manual_reset(void)
{
	memset(g_maze, 0, sizeof(g_maze));
	g_scan_index = 0U;
	uart_print("\r\nManual maze mapping reset. Next cell: (0,0)\r\n");
}

uint8_t maze_map_manual_is_done(void)
{
	return (g_scan_index >= (MAZE_MANUAL_SIZE * MAZE_MANUAL_SIZE)) ? 1U : 0U;
}

uint16_t maze_map_manual_export_text(char* out, uint16_t out_size)
{
	uint8_t row;
	uint8_t col;
	uint16_t index = 0U;
	int written;

	if ((out == NULL) || (out_size == 0U)) return 0U;
	out[0] = '\0';

	for (row = 0U; row < MAZE_MANUAL_SIZE; row++) {
		for (col = 0U; col < MAZE_MANUAL_SIZE; col++) {
			written = snprintf(&out[index], (size_t)(out_size - index), "%u%s",
												 g_maze[row][col],
												 (col == (MAZE_MANUAL_SIZE - 1U)) ? "" : " ");
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

uint8_t maze_map_manual_scan_next(void)
{
	char msg[96];
	char text[512];
	uint8_t x;
	uint8_t y;
	uint16_t len;

	if (maze_map_manual_is_done()) {
		uart_print("\r\nAll cells already mapped (0,0 to 9,9).\r\n");
		return 0U;
	}

	/* Requested sequence:
	 * 0:(0,0), 1:(0,1), ... 9:(0,9), 10:(1,0), ... 99:(9,9)
	 */
	x = (uint8_t)(g_scan_index / MAZE_MANUAL_SIZE);
	y = (uint8_t)(g_scan_index % MAZE_MANUAL_SIZE);

	map_current_cell(x, y);

	(void)snprintf(msg, sizeof(msg), "Mapped cell (%u,%u) = %u\r\n", x, y, g_maze[y][x]);
	uart_print(msg);

	g_scan_index++;

	if (maze_map_manual_is_done()) {
		uart_print("\r\nMANUAL_MAZE_MAP_BEGIN\r\n");
		len = maze_map_manual_export_text(text, (uint16_t)sizeof(text));
		(void)len;
		uart_print(text);
		uart_print("MANUAL_MAZE_MAP_END\r\n");
	} else {
		uint8_t nx = (uint8_t)(g_scan_index / MAZE_MANUAL_SIZE);
		uint8_t ny = (uint8_t)(g_scan_index % MAZE_MANUAL_SIZE);
		(void)snprintf(msg, sizeof(msg), "Next cell: (%u,%u)\r\n", nx, ny);
		uart_print(msg);
	}

	return 1U;
}
