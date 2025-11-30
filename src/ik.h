#ifndef IK_H
#define IK_H

#include "joints.h"

int inverse_kinematics(double x, double y, double z, RobotJoints *out);

void ik_set_links(double base_height, double L1, double L2, double L3);

#endif
