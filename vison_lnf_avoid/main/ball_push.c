/* Deterministic ball-push controller.
 *
 * Required physical sequence:
 *   START_FORWARD -> FIND_BALL/SCAN -> APPROACH (STRAFE, then FORWARD)
 *   -> ALIGN (ROTATE pocket, STRAFE ball to pocket x, re-check rotation)
 *   -> HARD PUSH -> REVERSE -> next ball.
 *
 * The important rule is that ALIGN never drives forward.  The ball is only
 * hit after the target edge and the ball have been geometrically aligned.
 */
#include "ball_push.h"
#include <limits.h>
#include <stdlib.h>
#include "board_config.h"

static ball_push_status_t s_status = { .state = BALL_PUSH_START_FORWARD };

static int abs_i(int x) { return x < 0 ? -x : x; }
static int clamp_i(int v, int lo, int hi) { return v < lo ? lo : (v > hi ? hi : v); }
static uint32_t sat_add(uint32_t a, uint32_t b) { return UINT32_MAX-a < b ? UINT32_MAX : a+b; }
static int cx_center(const ball_vision_result_t *v) { return v->width > 1 ? (int)v->width/2 : 0; }

static ball_color_t task_color(unsigned attempt) {
    int c = attempt == 0 ? BALL_TASK_FIRST_COLOR : BALL_TASK_SECOND_COLOR;
    return (c >= 0 && c < (int)BALL_COLOR_COUNT) ? (ball_color_t)c : BALL_COLOR_RED;
}
static unsigned task_side(unsigned attempt) {
    int s = attempt == 0 ? BALL_TASK_FIRST_POCKET : BALL_TASK_SECOND_POCKET;
    return s <= 0 ? 0U : 1U;
}
static const pocket_blob_t *task_pocket(const ball_vision_result_t *v, unsigned side) {
    if (!v || side >= 2 || !v->pockets[side].visible) return NULL;
    return &v->pockets[side];
}

static void publish(const ball_push_controller_t *c, bool ball, bool pocket) {
    if (!c) return;
    s_status.state=c->state; s_status.attempt=c->attempt; s_status.retry=c->retry;
    s_status.target_ball_visible=ball; s_status.target_pocket_visible=pocket;
    s_status.red_pocketed=c->red_pocketed; s_status.blue_pocketed=c->blue_pocketed;
}
void ball_push_get_status(ball_push_status_t *status) { if (status) *status=s_status; }

static void reset_stage(ball_push_controller_t *c, ball_push_state_t state,
                        int64_t target, int l, int r) {
    c->state=state; c->stage_ms=0; c->stage_started=true;
    c->stage_start_l=l; c->stage_start_r=r; c->stage_progress=0;
    c->stage_target=target; c->stage_last_motion=0; c->stage_stall_ms=0;
    c->align_ok_ms=0; c->approach_phase=0; c->approach_forward_ms=0; c->approach_strafe_ms=0;
    c->pocket_hit_frames=0; c->ball_lost_frames=0;
    c->last_ball_near_pocket=false; c->last_ball_approaching_pocket=false;
    c->push_turn_latched=false; c->push_turn_sign=0;
    if (state==BALL_PUSH_SCAN) { c->scan_leg=0; c->scan_direction=1; }
}

static bool tick_counts(ball_push_controller_t *c, uint32_t ms, int l, int r) {
    if (!c->stage_started) { c->stage_started=true; c->stage_start_l=l; c->stage_start_r=r; }
    c->stage_ms=sat_add(c->stage_ms,ms);
    c->stage_progress=llabs((long long)l-c->stage_start_l)+llabs((long long)r-c->stage_start_r);
    if (c->stage_progress-c->stage_last_motion >= PUSH_SCAN_STALL_COUNTS) {
        c->stage_last_motion=c->stage_progress; c->stage_stall_ms=0;
    } else c->stage_stall_ms=sat_add(c->stage_stall_ms,ms);
    return c->stage_stall_ms >= PUSH_SCAN_STALL_MS;
}

static ball_push_result_t zr(ball_push_controller_t *c, bool changed) {
    return (ball_push_result_t){.state=c->state,.forward=0,.lateral=0,.turn=0,
        .active=false,.state_changed=changed,.attempt=c->attempt,.retry=c->retry,
        .drive_mode=BALL_DRIVE_OPEN};
}

void ball_push_init(ball_push_controller_t *c) {
    if (!c) return;
    *c=(ball_push_controller_t){.state=BALL_PUSH_START_FORWARD,.attempt=0,
        .scan_direction=1,.red_pocketed=false,.blue_pocketed=false};
    publish(c,false,false);
}

ball_push_result_t ball_push_update(ball_push_controller_t *c,
    const ball_vision_result_t *v, bool fresh, uint32_t ms,
    int l, int r, int rear) {
    (void)rear;
    static const ball_vision_result_t empty={0};
    const ball_vision_result_t *vr=v ? v : &empty;
    const bool valid=fresh && v && vr->valid;
    const bool new_frame=valid && vr->frame_seq && vr->frame_seq!=c->last_vision_seq;
    if (!c) return (ball_push_result_t){.state=BALL_PUSH_FAULT_STOP,.state_changed=true};
    if (c->state==BALL_PUSH_DONE || c->state==BALL_PUSH_FAULT_STOP) return zr(c,false);

    c->task_ms=sat_add(c->task_ms,ms);
    if (c->task_ms>=BALL_TASK_TIMEOUT_MS) { c->state=BALL_PUSH_FAULT_STOP; publish(c,false,false); return zr(c,true); }

    const ball_color_t color=task_color(c->attempt);
    const unsigned side=task_side(c->attempt);
    const ball_blob_t *ball=valid ? ball_vision_find_ball(vr,color) : NULL;
    const pocket_blob_t *pocket=valid ? task_pocket(vr,side) : NULL;
    const int center=cx_center(vr);

    if (new_frame) {
        c->last_vision_seq=vr->frame_seq;
        if (ball) { c->last_ball_cx=ball->cx; c->last_ball_cy=ball->cy; c->last_ball_radius=ball->radius; c->ball_present_last=true; }
        else c->ball_present_last=false;
    }

    for (unsigned guard=0; guard<12; ++guard) {
        ball_push_result_t out=zr(c,false);
        switch(c->state) {
        case BALL_PUSH_START_FORWARD:
            /* Use the same ordinary straight drive API as the original car. */
            c->stage_ms=sat_add(c->stage_ms,ms);
            out.forward=PUSH_START_FORWARD; out.active=true; out.drive_mode=BALL_DRIVE_APPROACH;
            if (c->stage_ms>=PUSH_START_FORWARD_MS) { reset_stage(c,BALL_PUSH_FIND_BALL,-1,l,r); out.state_changed=true; }
            break;

        case BALL_PUSH_FIND_BALL:
            if (ball) { reset_stage(c,BALL_PUSH_APPROACH_BALL,-1,l,r); out.state_changed=true; break; }
            reset_stage(c,BALL_PUSH_SCAN,PUSH_SCAN_FIRST_COUNTS,l,r); out.state_changed=true;
            break;

        case BALL_PUSH_APPROACH_BALL:
            c->stage_ms=sat_add(c->stage_ms,ms);
            if (!ball) {
                if (new_frame && c->ball_lost_frames<UINT_MAX) ++c->ball_lost_frames;
                if (c->ball_lost_frames>=PUSH_APPROACH_LOST_MAX_FRAMES) { reset_stage(c,BALL_PUSH_SCAN,PUSH_SCAN_FIRST_COUNTS,l,r); out.state_changed=true; break; }
                break;
            }
            c->ball_lost_frames=0;
            out.active=true;
            if (c->approach_phase==0) {
                /* Phase 0: only strafe until the ball is centered. */
                int e=ball->cx-center;
                if (abs_i(e)>PUSH_APPROACH_DEADBAND_PX) {
                    out.lateral=e<0 ? PUSH_APPROACH_LATERAL_SPEED : -PUSH_APPROACH_LATERAL_SPEED;
                    out.drive_mode=BALL_DRIVE_LATERAL;

                    /* Accumulate strafe time for timeout protection */
                    c->approach_strafe_ms = sat_add(c->approach_strafe_ms, ms);

                    /* Timeout protection: if strafing for too long without centering,
                     * force transition to phase 1 to prevent deadlock caused by
                     * unstable ball detection or pocket flicker. */
                    if (c->approach_strafe_ms >= PUSH_APPROACH_STRAFE_TIMEOUT_MS) {
                        c->approach_phase=1;
                        c->approach_forward_ms=0;
                        c->approach_strafe_ms=0;
                    }
                    break;
                }
                /* Phase 1: while the ball is still FAR away, rotate toward the
                 * real target pocket. This is deliberately done before the car
                 * gets beside the ball. */
                c->approach_phase=1;
                c->approach_forward_ms=0;
                c->approach_strafe_ms=0;
            }

            if (c->approach_phase==1) {
                /* Re-center after each small heading correction. */
                int e=ball->cx-center;
                if (abs_i(e)>PUSH_APPROACH_DEADBAND_PX) {
                    out.lateral=e<0 ? PUSH_APPROACH_LATERAL_SPEED : -PUSH_APPROACH_LATERAL_SPEED;
                    out.drive_mode=BALL_DRIVE_LATERAL;
                    break;
                }
                const bool still_far = ball->radius <= 5 && ball->cy < 40;
                if (still_far && pocket) {
                    const int pe=pocket->cx-center;
                    if (abs_i(pe)>PUSH_APPROACH_HEADING_DEADBAND) {
                        out.turn=clamp_i(2*pe,-PUSH_APPROACH_HEADING_MAX,PUSH_APPROACH_HEADING_MAX);
                        if (out.turn>0 && out.turn<PUSH_APPROACH_HEADING_TURN) out.turn=PUSH_APPROACH_HEADING_TURN;
                        if (out.turn<0 && out.turn>-PUSH_APPROACH_HEADING_TURN) out.turn=-PUSH_APPROACH_HEADING_TURN;
                        out.active=true; out.drive_mode=BALL_DRIVE_OPEN;
                        break;
                    }
#if PUSH_FAR_SPRINT_ENABLE
                    /* Far-sprint trigger: pocket heading is already aligned
                     * (we just passed the deadband above) AND the ball is
                     * already on the ball->pocket line. Charging now skips
                     * the entire near-field phase where the ball is lost in
                     * the chassis shadow / below the Y cutoff. The distance
                     * is estimated from cy and locked in now, because the
                     * ball will vanish from view once we are on top of it. */
                    if (ball->radius <= PUSH_FAR_SPRINT_MAX_RADIUS_PX &&
                        ball->cy <= PUSH_FAR_SPRINT_MAX_CY_PX) {
                        const int be = ball->cx - pocket->cx;
                        if (abs_i(be) <= PUSH_FAR_SPRINT_BALL_POCKET_PX) {
                            int64_t counts = (int64_t)PUSH_FAR_SPRINT_BASE_COUNTS +
                                (int64_t)(PUSH_FAR_SPRINT_CY_REF_PX - ball->cy) *
                                PUSH_FAR_SPRINT_CY_SCALE;
                            if (counts < PUSH_FAR_SPRINT_MIN_COUNTS) counts = PUSH_FAR_SPRINT_MIN_COUNTS;
                            if (counts > PUSH_FAR_SPRINT_MAX_COUNTS) counts = PUSH_FAR_SPRINT_MAX_COUNTS;
                            c->far_sprint_ball_cx = ball->cx;
                            c->far_sprint_ball_cy = ball->cy;
                            reset_stage(c, BALL_PUSH_FAR_SPRINT, counts, l, r);
                            out.state_changed = true;
                            break;
                        }
                    }
#endif
                }
                /* Once the ball is no longer safely far away, never rotate near it. */
                c->approach_phase=2;
            }

            /* Phase 2: straight, slow approach. NO rotation and no lateral
             * correction in this phase unless we re-enter phase 0 after a loss. */
            c->approach_forward_ms=sat_add(c->approach_forward_ms,ms);
            out.forward=ball->cy>=38 ? PUSH_APPROACH_FORWARD_NEAR :
                         (ball->cy>=25 ? PUSH_APPROACH_FORWARD_MID : PUSH_APPROACH_FORWARD_FAR);
            out.drive_mode=BALL_DRIVE_APPROACH;
            if (c->approach_forward_ms>=PUSH_APPROACH_FORWARD_MIN_MS &&
                (ball->cy>=PUSH_APPROACH_NEAR_Y_PX || ball->radius>=PUSH_APPROACH_NEAR_RADIUS_PX)) {
                reset_stage(c,BALL_PUSH_ALIGN,-1,l,r); out.state_changed=true;
            }
            break;

        case BALL_PUSH_CREEP_OFF_FINISH:
            reset_stage(c,BALL_PUSH_APPROACH_BALL,-1,l,r); out.state_changed=true; break;

        case BALL_PUSH_SCAN: {
            if (ball) { reset_stage(c,BALL_PUSH_APPROACH_BALL,-1,l,r); out.state_changed=true; break; }
            bool stalled=tick_counts(c,ms,l,r);
            if (stalled) { c->state=BALL_PUSH_FAULT_STOP; return zr(c,true); }
            if (c->stage_progress>=c->stage_target) {
                if (++c->scan_leg>=PUSH_SCAN_MAX_LEGS) { c->state=BALL_PUSH_FAULT_STOP; return zr(c,true); }
                c->scan_direction=-c->scan_direction; c->stage_start_l=l; c->stage_start_r=r;
                c->stage_progress=0; c->stage_last_motion=0; c->stage_stall_ms=0;
                c->stage_target=(c->scan_leg&1)?PUSH_SCAN_SECOND_COUNTS:PUSH_SCAN_FIRST_COUNTS;
            }
            out.turn=c->scan_direction*(c->stage_progress+PUSH_SCAN_30_DEG_COUNTS/2<c->stage_target?PUSH_SCAN_TURN:PUSH_SCAN_FINE_TURN);
            out.active=true; out.drive_mode=BALL_DRIVE_OPEN; break; }

        case BALL_PUSH_ALIGN: {
            c->stage_ms=sat_add(c->stage_ms,ms);
            if (!ball) {
                if (new_frame && c->ball_lost_frames<UINT_MAX) ++c->ball_lost_frames;
                /* Never move forward without the ball in ALIGN. */
                if (c->ball_lost_frames>=PUSH_APPROACH_LOST_MAX_FRAMES) { reset_stage(c,BALL_PUSH_SCAN,PUSH_SCAN_FIRST_COUNTS,l,r); out.state_changed=true; }
                break;
            }
            c->ball_lost_frames=0;
            /* If the ball is at bumper range, back away instead of strafing it. */
            if (ball->radius>=PUSH_ALIGN_SAFE_RADIUS_PX) {
                reset_stage(c,BALL_PUSH_BACKOFF,PUSH_BACKOFF_COUNTS,l,r); out.state_changed=true; break;
            }
            /* IMPORTANT: once the ball is close, ALIGN must NEVER rotate.
             * Rotation beside the ball is exactly what was making the blue ball
             * roll away and disappear. Heading was already corrected during the
             * far-away APPROACH phase. */
            const int target_x = pocket ? pocket->cx :
                (int)((long long)vr->width * (side ? POCKET_FALLBACK_RIGHT_X_PERCENT
                                                    : POCKET_FALLBACK_LEFT_X_PERCENT) / 100LL);
            const int be=ball->cx-target_x;
            if (abs_i(be)>PUSH_ALIGN_DEADBAND_PX) {
                out.lateral=clamp_i(-PUSH_ALIGN_STRAFE_KP*be,-PUSH_ALIGN_STRAFE_MAX,PUSH_ALIGN_STRAFE_MAX);
                if (out.lateral>0 && out.lateral<PUSH_ALIGN_STRAFE_MIN_PWM) out.lateral=PUSH_ALIGN_STRAFE_MIN_PWM;
                if (out.lateral<0 && out.lateral>-PUSH_ALIGN_STRAFE_MIN_PWM) out.lateral=-PUSH_ALIGN_STRAFE_MIN_PWM;
                out.active=true; out.drive_mode=BALL_DRIVE_LATERAL; break;
            }
            /* Stage C: re-check the target after strafing. Only when BOTH are
             * centred for a stable window may the hard push begin. */
            c->align_ok_ms=sat_add(c->align_ok_ms,ms);
            if (c->align_ok_ms>=PUSH_ALIGN_CONFIRM_MS) { reset_stage(c,BALL_PUSH_PUSH,PUSH_HARD_PUSH_MS,l,r); out.state_changed=true; break; }
            break; }

        case BALL_PUSH_BACKOFF: {
            bool stalled=tick_counts(c,ms,l,r);
            if (stalled) { c->state=BALL_PUSH_FAULT_STOP; return zr(c,true); }
            out.forward=-PUSH_BACKOFF_SPEED; out.active=true; out.drive_mode=BALL_DRIVE_OPEN;
            if (c->stage_progress>=PUSH_BACKOFF_COUNTS) { reset_stage(c,BALL_PUSH_ALIGN,-1,l,r); out.state_changed=true; }
            break; }

        case BALL_PUSH_FAR_SPRINT: {
            /* Open-loop charge: fixed strong straight command, supervised only
             * by the encoder count target locked in at trigger time. No visual
             * steering here - the ball is expected to vanish from view as the
             * car closes in, and any last-minute correction would be based on
             * the near-field noise this state exists to avoid. */
            bool stalled=tick_counts(c,ms,l,r);
            if (stalled) {
                /* Wedged on the ball or the table edge: treat as a miss and
                 * back out for a retry, exactly like a stalled HARD_PUSH. */
                reset_stage(c,BALL_PUSH_BACKOUT,-1,l,r);
                out.state_changed=true;
                break;
            }
            out.forward=PUSH_FAR_SPRINT_FORWARD; out.active=true; out.drive_mode=BALL_DRIVE_APPROACH;
            if (c->stage_progress>=c->stage_target) {
                /* Sprint distance reached. The ball should now be at or just
                 * past the bumper. Hand over to the ordinary HARD_PUSH for the
                 * final impact: it is short, strong and already has the
                 * pocketed-ball bookkeeping wired in. */
                reset_stage(c,BALL_PUSH_PUSH,PUSH_HARD_PUSH_MS,l,r);
                out.state_changed=true;
            }
            break; }

        case BALL_PUSH_PUSH:
            /* HARD PUSH: fixed strong straight command. No visual steering can
             * kick the ball sideways once geometry has been confirmed. */
            c->stage_ms=sat_add(c->stage_ms,ms);
            out.forward=PUSH_HARD_FORWARD; out.active=true; out.drive_mode=BALL_DRIVE_OPEN;
            if (c->stage_ms>=PUSH_HARD_PUSH_MS) {
                if (c->attempt==0) c->red_pocketed=true; else c->blue_pocketed=true;
                if (c->attempt+1>=2) { c->state=BALL_PUSH_DONE; return zr(c,true); }
                ++c->attempt; c->retry=0; reset_stage(c,BALL_PUSH_EGRESS,PUSH_EGRESS_REVERSE_COUNTS,l,r); out.state_changed=true;
            }
            break;

        case BALL_PUSH_EGRESS: {
            bool stalled=tick_counts(c,ms,l,r);
            if (stalled) { c->state=BALL_PUSH_FAULT_STOP; return zr(c,true); }
            out.forward=-PUSH_EGRESS_SPEED; out.active=true; out.drive_mode=BALL_DRIVE_APPROACH;
            if (c->stage_progress>=c->stage_target) {
                /* After first ball (red) pocketed, turn right 50 deg then move forward
                 * to get better view for finding the second ball (blue). */
                const int64_t turn_counts = (int64_t)PUSH_SCAN_30_DEG_COUNTS * PUSH_POST_EGRESS_TURN_DEG / 30;
                reset_stage(c,BALL_PUSH_POST_EGRESS_TURN,turn_counts,l,r);
                out.state_changed=true;
            }
            break; }

        case BALL_PUSH_POST_EGRESS_TURN: {
            bool stalled=tick_counts(c,ms,l,r);
            if (stalled) { c->state=BALL_PUSH_FAULT_STOP; return zr(c,true); }
            out.turn=PUSH_POST_EGRESS_TURN_SPEED; out.active=true; out.drive_mode=BALL_DRIVE_OPEN;
            if (c->stage_progress>=c->stage_target) {
                reset_stage(c,BALL_PUSH_POST_EGRESS_FORWARD,PUSH_POST_EGRESS_FORWARD_COUNTS,l,r);
                out.state_changed=true;
            }
            break; }

        case BALL_PUSH_POST_EGRESS_FORWARD: {
            bool stalled=tick_counts(c,ms,l,r);
            if (stalled) { c->state=BALL_PUSH_FAULT_STOP; return zr(c,true); }
            out.forward=PUSH_POST_EGRESS_FORWARD_SPEED; out.active=true; out.drive_mode=BALL_DRIVE_APPROACH;
            if (c->stage_progress>=c->stage_target) { reset_stage(c,BALL_PUSH_FIND_BALL,-1,l,r); out.state_changed=true; }
            break; }

        case BALL_PUSH_BACKOUT:
            /* BACKOUT is reached either after a stalled FAR_SPRINT (ball not
             * pocketed, must retry) or as the legacy post-miss path. If the
             * current attempt's ball was NOT yet pocketed, go back and find
             * it again instead of advancing to the egress/next-ball flow. */
            if ((c->attempt==0 && !c->red_pocketed) ||
                (c->attempt==1 && !c->blue_pocketed)) {
                reset_stage(c,BALL_PUSH_FIND_BALL,-1,l,r);
            } else {
                reset_stage(c,BALL_PUSH_EGRESS,PUSH_EGRESS_REVERSE_COUNTS,l,r);
            }
            out.state_changed=true; break;
        case BALL_PUSH_DONE:
        case BALL_PUSH_FAULT_STOP:
            return zr(c,false);
        default:
            c->state=BALL_PUSH_FAULT_STOP; return zr(c,true);
        }
        out.state=c->state; out.attempt=c->attempt; out.retry=c->retry;
        if (c->state==BALL_PUSH_START_FORWARD || c->state==BALL_PUSH_APPROACH_BALL ||
            c->state==BALL_PUSH_ALIGN || c->state==BALL_PUSH_PUSH || c->state==BALL_PUSH_SCAN ||
            c->state==BALL_PUSH_FIND_BALL || c->state==BALL_PUSH_BACKOFF || c->state==BALL_PUSH_EGRESS ||
            c->state==BALL_PUSH_FAR_SPRINT ||
            c->state==BALL_PUSH_POST_EGRESS_TURN || c->state==BALL_PUSH_POST_EGRESS_FORWARD) {
            publish(c,ball!=NULL,pocket!=NULL);
        }
        if (out.state_changed) { /* run the new state in the same control tick */ continue; }
        return out;
    }
    c->state=BALL_PUSH_FAULT_STOP; return zr(c,true);
}

const char *ball_push_state_name(ball_push_state_t s) {
    switch(s) {
    case BALL_PUSH_START_FORWARD:return "START_FORWARD";
    case BALL_PUSH_FIND_BALL:return "FIND_BALL";
    case BALL_PUSH_APPROACH_BALL:return "APPROACH_BALL";
    case BALL_PUSH_CREEP_OFF_FINISH:return "CREEP";
    case BALL_PUSH_SCAN:return "SCAN";
    case BALL_PUSH_ALIGN:return "ALIGN";
    case BALL_PUSH_BACKOFF:return "BACKOFF";
    case BALL_PUSH_FAR_SPRINT:return "FAR_SPRINT";
    case BALL_PUSH_PUSH:return "HARD_PUSH";
    case BALL_PUSH_BACKOUT:return "BACKOUT";
    case BALL_PUSH_EGRESS:return "EGRESS";
    case BALL_PUSH_POST_EGRESS_TURN:return "POST_EGRESS_TURN";
    case BALL_PUSH_POST_EGRESS_FORWARD:return "POST_EGRESS_FWD";
    case BALL_PUSH_DONE:return "DONE";
    case BALL_PUSH_FAULT_STOP:return "FAULT_STOP";
    default:return "UNKNOWN";
    }
}
const char *ball_push_drive_mode_name(ball_push_drive_mode_t m) {
    switch(m){case BALL_DRIVE_OPEN:return "OPEN";case BALL_DRIVE_LATERAL:return "LATERAL";case BALL_DRIVE_APPROACH:return "APPROACH";default:return "UNKNOWN";}
}
