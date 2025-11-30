#ifndef SERIAL_H
#define SERIAL_H

#include "joints.h"

void initSerial(const char *port);
void sendJointAngles(RobotJoints j);

#endif
