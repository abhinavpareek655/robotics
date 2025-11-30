#include <stdio.h>
#include "ui.h"

void printStatus(RobotJoints j) {
    printf("\rBASE=%3d   SHOULDER=%3d   ELBOW=%3d   WRIST=%3d   ",
        j.base, j.shoulder, j.elbow, j.wrist);
    fflush(stdout);
}
