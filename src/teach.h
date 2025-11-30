#ifndef TEACH_H
#define TEACH_H

#include "joints.h"

#define MAX_STEPS 200
#define RECORD_FILE "recording.rec"

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
int fileExists(const char *filename);
void saveTeachOverwrite(TeachData *t);
void saveTeachAppend(TeachData *t);
void autoSaveTeach(TeachData *t);
void clearTeachFile();
void loadTeachFile(TeachData *t);

#endif
