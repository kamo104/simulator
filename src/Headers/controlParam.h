#pragma once

enum class ALT_MODE { AUTO, SET, CHANGE, SET_CHANGE };

struct controlParam {
    Velocity vel;
    double alt;
    bool overwriteVel;
    bool overwriteAlt;
    double altChange;
};