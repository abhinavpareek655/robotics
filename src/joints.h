#ifndef JOINTS_H
#define JOINTS_H

typedef struct {
    int base;
    int shoulder;
    int elbow;
    int wrist;
} RobotJoints;

void initJoints(RobotJoints *j);
void updateJoint(RobotJoints *j, char key);
void clampJoints(RobotJoints *j);

#endif
