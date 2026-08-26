#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    LINE_FOLLOW_WAITING_LINE = 0,
    LINE_FOLLOW_TRACKING,
    LINE_FOLLOW_LOST_SEARCH,
    LINE_FOLLOW_STOPPED,
} line_follow_state_t;

typedef struct {
    line_follow_state_t state;
    int error;
    int forward;
    int turn;
    bool state_changed;
} line_follow_result_t;

typedef struct {
    line_follow_state_t state;
    int last_error;
    bool has_last_error;
    uint32_t lost_ms;
    uint32_t all_black_ms;
    uint32_t invalid_ms;
    int last_direction;
    bool last_cycle_tracking;
} line_follow_controller_t;

void line_follow_init(line_follow_controller_t *controller);
line_follow_result_t line_follow_update(line_follow_controller_t *controller,
                                        uint8_t black_mask,
                                        uint32_t elapsed_ms);
const char *line_follow_state_name(line_follow_state_t state);

#ifdef __cplusplus
}
#endif
