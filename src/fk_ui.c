#include <stdio.h>
#include "fk_ui.h"
#include "fk.h"
#include "angle_conv.h"
#define M_PI 3.14159265358979323846

void print_fk(const RobotJoints *j) {
    double x, y, z;
    double base_rad, sh_rad, el_rad, wr_rad;
    servo_to_math(j, &base_rad, &sh_rad, &el_rad, &wr_rad);
    RobotJoints mathJ;
    mathJ.base = (int)(base_rad * 180.0/M_PI);
    mathJ.shoulder = (int)(sh_rad * 180.0/M_PI);
    mathJ.elbow = (int)(el_rad * 180.0/M_PI);
    mathJ.wrist = (int)(wr_rad * 180.0/M_PI);

    if (forward_kinematics(&mathJ, &x, &y, &z) == 0) {
        printf("\nFK -> X=%.3f m   Y=%.3f m   Z=%.3f m", x, y, z);
    } else {
        printf("\nFK calculation error.");
    }
}
