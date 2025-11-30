#include <stdio.h>
#include "fk_log.h"
#include "fk.h"

static FILE *fklog = NULL;
static int step_counter = 0;

void fk_log_init() {
    fklog = fopen("fk_log.csv", "w");
    if (fklog) {
        fprintf(fklog, "step,base,shoulder,elbow,wrist,x,y,z\n");
    }
}

void fk_log_step(const RobotJoints *j) {
    if (!fklog) return;

    double x, y, z;
    forward_kinematics(j, &x, &y, &z);

    fprintf(fklog, "%d,%d,%d,%d,%d,%.5f,%.5f,%.5f\n",
            step_counter++,
            j->base, j->shoulder, j->elbow, j->wrist,
            x, y, z
    );
}

void fk_log_close() {
    if (fklog) {
        fclose(fklog);
        fklog = NULL;
    }
}
