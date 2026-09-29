#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
typedef struct {
    bool active, consumed, moved;
    int x, y;
    int64_t started, last_seen;
} hold_gesture_t;

static inline bool hold_update(hold_gesture_t *g, bool down, int x, int y, int64_t now)
{
    if (!down) {
        if (g->active && now - g->last_seen >= 150) g->active = false;
        return false;
    }
    if (!g->active) {
        *g = (hold_gesture_t){.active = true, .x = x, .y = y,
                              .started = now, .last_seen = now};
    }
    g->last_seen = now;
    if (abs(x - g->x) > 20 || abs(y - g->y) > 20) g->moved = true;
    if (!g->consumed && !g->moved && now - g->started >= 1200) {
        g->consumed = true;
        return true;
    }
    return false;
}
