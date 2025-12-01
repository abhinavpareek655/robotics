#include <stdio.h>
#include "fk_ui.h"
#include "fk.h"

void print_fk(const RobotJoints *math_joints) {
    if (!math_joints) return;
    
    double x, y, z;
    if (forward_kinematics(math_joints, &x, &y, &z) == 0) {
        printf("FK: X=%.3f m, Y=%.3f m, Z=%.3f m\n", x, y, z);
    } else {
        printf("FK: Calculation error\n");
    }
}