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
static uint8_t g_scan_index = 0U; /* total scans performed */
static uint8_t g_current_x = 0U;
static uint8_t g_current_y = 0U;
static uint8_t g_prev_x = 0U;
static uint8_t g_prev_y = 0U;
static uint8_t g_heading = 0U; /* 0=N,1=E,2=S,3=W */

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

void maze_map_manual_move(uint8_t cmd)
{
	char msg[96];
	const char* heading_name;
	const char* movement = "UNKNOWN";

	switch (g_heading) {
		case 0: heading_name = "NORTH"; break;
		case 1: heading_name = "EAST"; break;
		case 2: heading_name = "SOUTH"; break;
		case 3: heading_name = "WEST"; break;
		default: heading_name = "UNKNOWN"; break;
	}

	if (cmd == 'w' || cmd == 'W') {
		movement = "FORWARD";
		if (g_heading == 0 && g_current_y > 0) g_current_y--;
		else if (g_heading == 1 && (g_current_x < MAZE_MANUAL_SIZE - 1U)) g_current_x++;
		else if (g_heading == 2 && (g_current_y < MAZE_MANUAL_SIZE - 1U)) g_current_y++;
		else if (g_heading == 3 && g_current_x > 0) g_current_x--;
	} else if (cmd == 's' || cmd == 'S') {
		movement = "BACKWARD";
		if (g_heading == 0 && (g_current_y < MAZE_MANUAL_SIZE - 1U)) g_current_y++;
		else if (g_heading == 1 && g_current_x > 0) g_current_x--;
		else if (g_heading == 2 && g_current_y > 0) g_current_y--;
		else if (g_heading == 3 && (g_current_x < MAZE_MANUAL_SIZE - 1U)) g_current_x++;
	} else if (cmd == 'a' || cmd == 'A') {
		movement = "TURN_LEFT";
		g_heading = (uint8_t)((g_heading + 3U) & 0x3U);
	} else if (cmd == 'd' || cmd == 'D') {
		movement = "TURN_RIGHT";
		g_heading = (uint8_t)((g_heading + 1U) & 0x3U);
	} else {
		return;
	}

	(void)snprintf(msg, sizeof(msg), "MOVE: %s, pos=(%u,%u), heading=%s\r\n", movement, g_current_x, g_current_y, heading_name);
	uart_print(msg);
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
	g_current_x = 0U;
	g_current_y = 0U;
	g_prev_x = 0U;
	g_prev_y = 0U;
	g_heading = 0U; /* North */
	uart_print("\r\nManual maze mapping reset. Position (0,0), heading NORTH\r\n");
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

	/* Position is kept in current_x/current_y and based on move direction. */
	x = g_current_x;
	y = g_current_y;

	if (g_scan_index == 0) {
		uart_print("From start: (0,0)\r\n");
	} else {
		(void)snprintf(msg, sizeof(msg), "From previous cell: (%u,%u)\r\n", g_prev_x, g_prev_y);
		uart_print(msg);
		const char* heading_name;
		switch (g_heading) {
			case 0: heading_name = "NORTH"; break;
			case 1: heading_name = "EAST"; break;
			case 2: heading_name = "SOUTH"; break;
			case 3: heading_name = "WEST"; break;
			default: heading_name = "UNKNOWN"; break;
		}
		(void)snprintf(msg, sizeof(msg), "Current position (%u,%u), heading %s\r\n", x, y, heading_name);
		uart_print(msg);
	}

	map_current_cell(x, y);
	/* mark visited so map export is non-zero when paths are explored */
	g_maze[y][x] |= 0x10;

	(void)snprintf(msg, sizeof(msg), "Mapped cell (%u,%u) = %u (walls=%u, visited=1)\r\n", x, y, g_maze[y][x], g_maze[y][x] & 0x0F);
	uart_print(msg);

	g_prev_x = x;
	g_prev_y = y;

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

void maze_map_manual_print_status(void)
{
	char msg[128];
	if (maze_map_manual_is_done()) {
		uart_print("Manual mapping done. All cells mapped (0,0..9,9).\r\n");
	} else {
		uint8_t x = (uint8_t)(g_scan_index / MAZE_MANUAL_SIZE);
		uint8_t y = (uint8_t)(g_scan_index % MAZE_MANUAL_SIZE);
		(void)snprintf(msg, sizeof(msg), "Manual mapping next: (%u,%u)  (mapped %u/100)\r\n", x, y, g_scan_index);
		uart_print(msg);
	}
}

void maze_map_manual_print_map(void)
{
	char text[512];
	uint16_t len;
	maze_map_manual_print_status();
	uart_print("\r\nMANUAL_MAZE_MAP_SNAPSHOT_BEGIN\r\n");
	len = maze_map_manual_export_text(text, (uint16_t)sizeof(text));
	(void)len;
	uart_print(text);
	uart_print("MANUAL_MAZE_MAP_SNAPSHOT_END\r\n");
}

