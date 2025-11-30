#include <stdio.h>
#include "fk_ui.h"
#include "fk.h"

void print_fk(const RobotJoints *j) {
    double x, y, z;

    if (forward_kinematics(j, &x, &y, &z) == 0) {
        printf("\nFK → X=%.3f m   Y=%.3f m   Z=%.3f m", x, y, z);
    } else {
        printf("\nFK calculation error.");
    }
}
