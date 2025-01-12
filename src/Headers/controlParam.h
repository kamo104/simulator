#pragma once

enum class ALT_MODE { AUTO, SET, CHANGE, SET_CHANGE };

struct controlParam {
    Velocity vel;
    double alt;
    double altChange;
    ALT_MODE altMode;
};