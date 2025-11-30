#include <stdio.h>
#include <windows.h>
#include "teach.h"
#include "serial.h"

void initTeach(TeachData *t) {
    t->count = 0;
    t->recording = 0;
}

void startRecording(TeachData *t) {
    t->count = 0;
    t->recording = 1;
    printf("\nRecording started...\n");
}

void stopRecording(TeachData *t) {
    t->recording = 0;
    printf("\nRecording stopped. %d steps saved.\n", t->count);
}

void recordStep(TeachData *t, RobotJoints j) {
    if (!t->recording) return;

    if (t->count < MAX_STEPS) {
        t->steps[t->count++] = j;
        printf("\nRecorded step %d", t->count);
    } else {
        printf("\nTeach Mode storage full!");
        t->recording = 0;
    }
}

void playback(TeachData *t) {
    if (t->count == 0) {
        printf("\nNo steps recorded.\n");
        return;
    }

    printf("\nReplaying %d steps...\n", t->count);

    for (int i = 0; i < t->count; i++) {
        sendJointAngles(t->steps[i]);
        Sleep(200);
    }

    printf("\nPlayback finished.\n");
}

void clearTeach(TeachData *t) {
    t->count = 0;
    printf("\nTeach data cleared.\n");
}
