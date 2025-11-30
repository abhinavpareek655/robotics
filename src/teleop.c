#include <stdio.h>
#include "teleop.h"
#include "input.h"
#include "joints.h"
#include "ui.h"
#include "teach.h"
#include "serial.h"

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

    while (1) {

        char key = getKey();

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
        sendJointAngles(j);

        if (t.recording) recordStep(&t, j);
    }

    printf("\nGoodbye!\n");
}
