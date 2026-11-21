#pragma once

// Same LCG as seat.h. One state per pixel, so rows can run on separate threads.

struct Rng {
    unsigned state;

    float uniform() {
        state = state * 1664525u + 1013904223u;
        return (state >> 8) * (1.f / 16777216.f);
    }
};
