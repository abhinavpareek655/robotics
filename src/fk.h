#ifndef FK_H
#define FK_H

#include "joints.h"

int forward_kinematics(const RobotJoints *j, double *x, double *y, double *z);

void fk_set_links(double base_height, double L1, double L2, double L3);

#endif
