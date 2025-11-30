#ifndef FK_LOG_H
#define FK_LOG_H

#include "joints.h"

void fk_log_init();
void fk_log_step(const RobotJoints *j);
void fk_log_close();

#endif
