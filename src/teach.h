#ifndef TEACH_H
#define TEACH_H

#include "joints.h"

#define MAX_STEPS 200   // enough for student project

typedef struct {
    RobotJoints steps[MAX_STEPS];
    int count;
    int recording;
} TeachData;

void initTeach(TeachData *t);
void startRecording(TeachData *t);
void stopRecording(TeachData *t);
void recordStep(TeachData *t, RobotJoints j);
void playback(TeachData *t);
void clearTeach(TeachData *t);

#endif
