#include "teleop.h"
#include "input.h"
#include "joints.h"
#include "ui.h"
#include "serial.h"
#include <stdio.h>
#include "teach.h"

void runTeleoperation() {
    RobotJoints j;
    initJoints(&j);

    TeachData t;
    initTeach(&t);

    printf("Teleoperation Mode (X to exit, T=record, P=play, C=clear)\n");

    initSerial("COM3");    // change COM port if needed

    while (1) {
        char key = getKey();
        if (key == 'x') break;

        if (key == 't') {
            if (!t.recording) startRecording(&t);
            else stopRecording(&t);
            continue;
        }

        if (key == 'p') {
            playback(&t);
            continue;
        }

        if (key == 'c') {
            clearTeach(&t);
            continue;
        }

        updateJoint(&j, key);
        clampJoints(&j);

        printStatus(j);

        sendJointAngles(j);

        recordStep(&t, j);
    }
}
