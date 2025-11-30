#include <stdio.h>
#include "teleop.h"
#include "input.h"
#include "joints.h"
#include "ui.h"
#include "teach.h"
#include "serial.h"
#include "fk_ui.h"
#include "fk_log.h"
#include "fk_ui.h"
#include "fk_log.h"
#include "ik.h"
#include <math.h>
#include "angle_conv.h"
#define M_PI 3.14159265358979323846

static double deg2rad(double deg) {
    return deg * M_PI / 180.0;
}

void runTeleoperation() {

    RobotJoints j;
    initJoints(&j);

    TeachData t;
    initTeach(&t);

    printf("\nTeleoperation Mode");
    printf("\nT = Start/Stop Recording");
    printf("\nP = Playback Recording");
    printf("\nC = Clear recording file");
    printf("\nX = Exit (auto-save)");
    printf("\n---------------------------------------\n");

    initSerial("COM3");    // change COM port if needed
    fk_log_init();

    setvbuf(stdin, NULL, _IONBF, 0);

    while (1) {

        char key = getKey();

        if (key == 'i' || key == 'I') {
            double x, y, z;

            printf("\nEnter target X Y Z (meters): ");
            if (scanf("%lf %lf %lf", &x, &y, &z) != 3) {
                printf("\nInvalid input.\n");
                fflush(stdin);
                continue;
            }

            RobotJoints ikMath;
            int result = inverse_kinematics(x, y, z, &ikMath);

            if (result == 0) {
                printf("\nIK solution found! Converting to servo angles...\n");

                RobotJoints ikServo;
                math_to_servo(
                    deg2rad(ikMath.base),
                    deg2rad(ikMath.shoulder),
                    deg2rad(ikMath.elbow),
                    deg2rad(ikMath.wrist),
                    &ikServo
                );

                sendJointAngles(ikServo);

                print_fk(&ikServo);

                fk_log_step(&ikServo);
            }
            else {
                printf("\nIK failed: target unreachable.\n");
            }

            continue;
        }

        if (key == 'x' || key == 'X') {
            printf("\nExiting and auto-saving...\n");
            autoSaveTeach(&t);
            break;
        }

        if (key == 'c' || key == 'C') {
            clearTeachFile();
            continue;
        }

        if (key == 't' || key == 'T') {
            if (!t.recording) startRecording(&t);
            else{
                stopRecording(&t);
                autoSaveTeach(&t);
            }
            continue;
        }

        if (key == 'p' || key == 'P') {
            if (t.count > 0) {
                playback(&t);
                continue;
            }
            if (fileExists(RECORD_FILE)) {
                loadTeachFile(&t);
                playback(&t);
                continue;
            }
            printf("\nNo recordings available.\n");
            continue;
        }

        updateJoint(&j, key);
        clampJoints(&j);

        printStatus(j);
        print_fk(&j); 

        sendJointAngles(j);
        fk_log_step(&j);

        if (t.recording) recordStep(&t, j);
    }

    fk_log_close();
    printf("\nGoodbye!\n");
}
