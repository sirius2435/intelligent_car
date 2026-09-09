#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    LINE_FOLLOW_WAITING_LINE = 0,
    LINE_FOLLOW_TRACKING,
    LINE_FOLLOW_CORNER_CANDIDATE,
    LINE_FOLLOW_CORNER_ROTATE,
    LINE_FOLLOW_CORNER_EXIT,
    LINE_FOLLOW_LOST_SEARCH,
    LINE_FOLLOW_STOPPED,
} line_follow_state_t;

typedef enum {
    LINE_SEARCH_IDLE = 0,
    LINE_SEARCH_ROTATE,
    LINE_SEARCH_SETTLE,
} line_search_phase_t;

typedef enum {
    LINE_STOP_REASON_NONE = 0,
    LINE_STOP_REASON_FINISH,          /* confirmed all-black finish bar */
    LINE_STOP_REASON_LOST_TIMEOUT,    /* line lost for LINE_LOST_STOP_MS */
    LINE_STOP_REASON_SEARCH_EXHAUSTED,/* all search legs used up */
    LINE_STOP_REASON_SEARCH_STALL,    /* stuck during a search leg */
} line_follow_stop_reason_t;

typedef struct {
    line_follow_state_t state;
    int error;
    int forward;
    int turn;
    bool state_changed;
} line_follow_result_t;

typedef struct {
    line_follow_state_t state;
    line_follow_stop_reason_t stop_reason; /* set when state becomes STOPPED */
    int last_error;
    bool has_last_error;
    uint32_t lost_ms;
    uint32_t all_black_ms;
    uint32_t invalid_ms;
    int last_direction;
    bool last_cycle_tracking;
    int corner_direction;
    uint32_t corner_arm_ms;
    uint32_t corner_candidate_ms;
    uint32_t corner_rotate_ms;
    uint32_t corner_centered_ms;
    uint32_t corner_exit_ms;
    bool corner_rearm_ready;
    line_search_phase_t search_phase;
    unsigned search_leg;
    int search_direction;
    int search_start_left_count;
    int search_start_right_count;
    int64_t search_progress_counts;
    int64_t search_target_counts;
    int64_t search_last_motion_counts;
    uint32_t search_settle_ms;
    uint32_t search_stall_ms;
} line_follow_controller_t;

void line_follow_init(line_follow_controller_t *controller);
line_follow_result_t line_follow_update(line_follow_controller_t *controller,
                                        uint8_t black_mask,
                                        uint32_t elapsed_ms,
                                        int left_count,
                                        int right_count);
const char *line_follow_state_name(line_follow_state_t state);
const char *line_follow_search_phase_name(line_search_phase_t phase);
const char *line_follow_stop_reason_name(line_follow_stop_reason_t reason);

#ifdef __cplusplus
}
#endif
