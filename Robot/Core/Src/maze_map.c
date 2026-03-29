#include "maze_map.h"

#include <stdio.h>
#include <string.h>

#include "motor.h"
#include "sonar.h"
#include "maze_solve.h"

#define MAP_SPEED_PWM          26214U
#define WALL_THRESHOLD_CM      9U

/* User-requested sensor indexing */
#define SONAR_BACK_SENSOR      0U
#define SONAR_FRONT_SENSOR     1U
#define SONAR_LEFT_SENSOR      2U
#define SONAR_RIGHT_SENSOR     3U

#define WALL_NORTH             0x1U
#define WALL_EAST              0x2U
#define WALL_SOUTH             0x4U
#define WALL_WEST              0x8U

typedef enum {
	HEADING_NORTH = 0,
	HEADING_EAST  = 1,
	HEADING_SOUTH = 2,
	HEADING_WEST  = 3
} Heading_t;

typedef enum {
	ACTION_IDLE = 0,
	ACTION_TURN,
	ACTION_FORWARD,
	ACTION_FINISH_FORWARD
} ActionState_t;

typedef struct {
	int8_t x;
	int8_t y;
} Cell_t;

static UART_HandleTypeDef* map_uart = NULL;

static uint8_t maze[MAZE_MAP_SIZE][MAZE_MAP_SIZE];
static uint8_t visited[MAZE_MAP_SIZE][MAZE_MAP_SIZE];

static Cell_t path_stack[MAZE_MAP_SIZE * MAZE_MAP_SIZE];
static int16_t stack_top = -1;

static int8_t current_x = 0;
static int8_t current_y = 0;
static Heading_t heading = HEADING_NORTH;

static int8_t target_x = 0;
static int8_t target_y = 0;

static uint8_t run_enabled = 0U;
static uint8_t run_done = 0U;

static ActionState_t action_state = ACTION_IDLE;
static uint8_t turn_steps_remaining = 0U;
static uint8_t turn_right = 1U;

static const int8_t dx[4] = { 0, 1, 0, -1 };
static const int8_t dy[4] = { -1, 0, 1, 0 };

static uint8_t wall_bit_for_heading(Heading_t dir)
{
	switch (dir) {
		case HEADING_NORTH: return WALL_NORTH;
		case HEADING_EAST:  return WALL_EAST;
		case HEADING_SOUTH: return WALL_SOUTH;
		case HEADING_WEST:  return WALL_WEST;
		default:            return 0U;
	}
}

static Heading_t opposite_heading(Heading_t dir)
{
	return (Heading_t)((dir + 2) & 0x3);
}

static uint8_t in_bounds(int8_t x, int8_t y)
{
	if (x < 0 || y < 0) return 0U;
	if (x >= (int8_t)MAZE_MAP_SIZE || y >= (int8_t)MAZE_MAP_SIZE) return 0U;
	return 1U;
}

static void stack_push(int8_t x, int8_t y)
{
	if (stack_top >= (int16_t)(MAZE_MAP_SIZE * MAZE_MAP_SIZE - 1)) return;
	stack_top++;
	path_stack[stack_top].x = x;
	path_stack[stack_top].y = y;
}

static uint8_t stack_pop(Cell_t* out)
{
	if ((stack_top < 0) || (out == NULL)) return 0U;
	*out = path_stack[stack_top--];
	return 1U;
}

static void set_wall_abs(int8_t x, int8_t y, Heading_t dir)
{
	int8_t nx;
	int8_t ny;
	uint8_t bit;

	if (!in_bounds(x, y)) return;

	bit = wall_bit_for_heading(dir);
	maze[y][x] |= bit;

	nx = x + dx[dir];
	ny = y + dy[dir];

	if (in_bounds(nx, ny)) {
		maze[ny][nx] |= wall_bit_for_heading(opposite_heading(dir));
	}
}

static uint8_t has_wall_abs(int8_t x, int8_t y, Heading_t dir)
{
	if (!in_bounds(x, y)) return 1U;
	return (maze[y][x] & wall_bit_for_heading(dir)) ? 1U : 0U;
}

static void set_outer_boundary_walls(int8_t x, int8_t y)
{
	if (y == 0) set_wall_abs(x, y, HEADING_NORTH);
	if (x == ((int8_t)MAZE_MAP_SIZE - 1)) set_wall_abs(x, y, HEADING_EAST);
	if (y == ((int8_t)MAZE_MAP_SIZE - 1)) set_wall_abs(x, y, HEADING_SOUTH);
	if (x == 0) set_wall_abs(x, y, HEADING_WEST);
}

static void mark_local_walls(void)
{
	uint16_t dist_front = sonar_get_distance(SONAR_FRONT_SENSOR);
	uint16_t dist_back = sonar_get_distance(SONAR_BACK_SENSOR);
	uint16_t dist_left = sonar_get_distance(SONAR_LEFT_SENSOR);
	uint16_t dist_right = sonar_get_distance(SONAR_RIGHT_SENSOR);

	Heading_t dir_front = heading;
	Heading_t dir_right = (Heading_t)((heading + 1) & 0x3);
	Heading_t dir_back = (Heading_t)((heading + 2) & 0x3);
	Heading_t dir_left = (Heading_t)((heading + 3) & 0x3);

	set_outer_boundary_walls(current_x, current_y);

	if (dist_front <= WALL_THRESHOLD_CM) set_wall_abs(current_x, current_y, dir_front);
	if (dist_right <= WALL_THRESHOLD_CM) set_wall_abs(current_x, current_y, dir_right);
	if (dist_back <= WALL_THRESHOLD_CM) set_wall_abs(current_x, current_y, dir_back);
	if (dist_left <= WALL_THRESHOLD_CM) set_wall_abs(current_x, current_y, dir_left);
}

static void transmit_text(const char* text)
{
	if ((map_uart == NULL) || (text == NULL)) return;
	HAL_UART_Transmit(map_uart, (uint8_t*)text, (uint16_t)strlen(text), HAL_MAX_DELAY);
}

static void schedule_move_to_heading(Heading_t desired_heading, int8_t next_x, int8_t next_y)
{
	uint8_t diff = (uint8_t)((desired_heading - heading + 4) & 0x3);

	target_x = next_x;
	target_y = next_y;

	if (diff == 0U) {
		action_state = ACTION_FORWARD;
		return;
	}

	action_state = ACTION_TURN;

	if (diff == 1U) {
		turn_right = 1U;
		turn_steps_remaining = 1U;
	} else if (diff == 3U) {
		turn_right = 0U;
		turn_steps_remaining = 1U;
	} else {
		turn_right = 1U;
		turn_steps_remaining = 2U;
	}
}

static void finish_mapping(void)
{
	char text[512];
	uint16_t len;

	run_enabled = 0U;
	run_done = 1U;
	action_state = ACTION_IDLE;

	len = maze_map_export_text(text, (uint16_t)sizeof(text));
	(void)len;

	maze_solve_set_map_text(text);

	transmit_text("\r\nMAZE_MAP_BEGIN\r\n");
	transmit_text(text);
	transmit_text("MAZE_MAP_END\r\n");
}

void maze_map_init(UART_HandleTypeDef* huart)
{
	map_uart = huart;
	maze_map_stop();
}

void maze_map_start(void)
{
	memset(maze, 0, sizeof(maze));
	memset(visited, 0, sizeof(visited));

	stack_top = -1;
	current_x = 0;
	current_y = 0;
	target_x = 0;
	target_y = 0;
	heading = HEADING_NORTH;

	run_enabled = 1U;
	run_done = 0U;

	action_state = ACTION_IDLE;
	turn_steps_remaining = 0U;
	turn_right = 1U;

	maze_solve_set_map_text("");
	transmit_text("\r\nMaze mapping started at (0,0), heading NORTH\r\n");
}

void maze_map_stop(void)
{
	run_enabled = 0U;
	action_state = ACTION_IDLE;
	turn_steps_remaining = 0U;
}

uint8_t maze_map_is_running(void)
{
	return run_enabled;
}

uint8_t maze_map_is_done(void)
{
	return run_done;
}

const uint8_t (*maze_map_get_matrix(void))[MAZE_MAP_SIZE]
{
	return maze;
}

void maze_map_copy_int_matrix(int out[MAZE_MAP_SIZE][MAZE_MAP_SIZE])
{
	uint8_t row;
	uint8_t col;

	if (out == NULL) return;

	for (row = 0U; row < MAZE_MAP_SIZE; row++) {
		for (col = 0U; col < MAZE_MAP_SIZE; col++) {
			out[row][col] = maze[row][col];
		}
	}
}

uint16_t maze_map_export_text(char* out, uint16_t out_size)
{
	uint8_t row;
	uint8_t col;
	int written;
	uint16_t index = 0U;

	if ((out == NULL) || (out_size == 0U)) return 0U;
	out[0] = '\0';

	for (row = 0U; row < MAZE_MAP_SIZE; row++) {
		for (col = 0U; col < MAZE_MAP_SIZE; col++) {
			written = snprintf(&out[index], (size_t)(out_size - index), "%u%s",
												 maze[row][col],
												 (col == (MAZE_MAP_SIZE - 1U)) ? "" : " ");
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

void maze_map_tick(void)
{
	uint8_t i;
	uint8_t found_next = 0U;
	Heading_t dir;
	int8_t nx = 0;
	int8_t ny = 0;
	Cell_t back_cell;

	if (!run_enabled) return;
	if (motor_is_busy()) return;

	if (action_state == ACTION_TURN) {
		if (turn_steps_remaining > 0U) {
			if (turn_right) {
				motor_turn_right_90(MAP_SPEED_PWM);
				heading = (Heading_t)((heading + 1) & 0x3);
			} else {
				motor_turn_left_90(MAP_SPEED_PWM);
				heading = (Heading_t)((heading + 3) & 0x3);
			}
			turn_steps_remaining--;
			return;
		}

		action_state = ACTION_FORWARD;
	}

	if (action_state == ACTION_FORWARD) {
		motor_forward_1cell(MAP_SPEED_PWM);
		action_state = ACTION_FINISH_FORWARD;
		return;
	}

	if (action_state == ACTION_FINISH_FORWARD) {
		current_x = target_x;
		current_y = target_y;
		action_state = ACTION_IDLE;
	}

	visited[current_y][current_x] = 1U;
	mark_local_walls();

	for (i = 0U; i < 4U; i++) {
		dir = (Heading_t)i;
		nx = current_x + dx[dir];
		ny = current_y + dy[dir];

		if (!in_bounds(nx, ny)) continue;
		if (has_wall_abs(current_x, current_y, dir)) continue;
		if (visited[ny][nx]) continue;

		found_next = 1U;
		break;
	}

	if (found_next) {
		stack_push(current_x, current_y);
		schedule_move_to_heading(dir, nx, ny);
		return;
	}

	if (stack_pop(&back_cell)) {
		int8_t back_dx = back_cell.x - current_x;
		int8_t back_dy = back_cell.y - current_y;
		Heading_t back_dir = heading;

		if (back_dx == 1 && back_dy == 0) back_dir = HEADING_EAST;
		else if (back_dx == -1 && back_dy == 0) back_dir = HEADING_WEST;
		else if (back_dx == 0 && back_dy == 1) back_dir = HEADING_SOUTH;
		else if (back_dx == 0 && back_dy == -1) back_dir = HEADING_NORTH;

		schedule_move_to_heading(back_dir, back_cell.x, back_cell.y);
		return;
	}

	finish_mapping();
}
