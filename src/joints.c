#include "joints.h"

int clamp(int v) {
    if (v < 0) return 0;
    if (v > 180) return 180;
    return v;
}

void initJoints(RobotJoints *j) {
    j->base = j->shoulder = j->elbow = j->wrist = 90;
}

void updateJoint(RobotJoints *j, char key) {
    int step = 2;

    switch (key) {
        case 'q': j->base += step; break;
        case 'a': j->base -= step; break;

        case 'w': j->shoulder += step; break;
        case 's': j->shoulder -= step; break;

        case 'e': j->elbow += step; break;
        case 'd': j->elbow -= step; break;

        case 'r': j->wrist += step; break;
        case 'f': j->wrist -= step; break;
    }
}

void clampJoints(RobotJoints *j) {
    j->base = clamp(j->base);
    j->shoulder = clamp(j->shoulder);
    j->elbow = clamp(j->elbow);
    j->wrist = clamp(j->wrist);
}
