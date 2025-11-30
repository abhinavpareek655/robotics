#include <stdio.h>
#include <stdlib.h>
#include <windows.h>
#include "teach.h"
#include "serial.h"
#include <conio.h>

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
        printf("\nStep %d recorded.", t->count);
    } else {
        printf("\nRecording buffer full!");
        t->recording = 0;
    }
}

void playback(TeachData *t) {
    if (t->count == 0) {
        printf("\nNo steps to playback.\n");
        return;
    }

    printf("\nPlaying back %d steps...\n", t->count);

    for (int i = 0; i < t->count; i++) {
        sendJointAngles(t->steps[i]);
        Sleep(200);
    }

    printf("\nPlayback finished.\n");
}

int fileExists(const char *filename) {
    FILE *f = fopen(filename, "r");
    if (f) {
        fclose(f);
        return 1;
    }
    return 0;
}

void saveTeachOverwrite(TeachData *t) {
    FILE *f = fopen(RECORD_FILE, "w");
    if (!f) {
        printf("\nError saving file.\n");
        return;
    }

    fprintf(f, "%d\n", t->count);
    for (int i = 0; i < t->count; i++) {
        fprintf(f, "%d %d %d %d\n",
            t->steps[i].base,
            t->steps[i].shoulder,
            t->steps[i].elbow,
            t->steps[i].wrist
        );
    }

    fclose(f);
    printf("\nRecording saved (overwrite).\n");
}

void saveTeachAppend(TeachData *t) {
    FILE *f = fopen(RECORD_FILE, "a");
    if (!f) {
        printf("\nError appending to file.\n");
        return;
    }

    for (int i = 0; i < t->count; i++) {
        fprintf(f, "%d %d %d %d\n",
            t->steps[i].base,
            t->steps[i].shoulder,
            t->steps[i].elbow,
            t->steps[i].wrist
        );
    }

    fclose(f);
    printf("\nRecording appended to file.\n");
}

void autoSaveTeach(TeachData *t) {
    if (t->count == 0) {
        printf("\nNo steps to save.\n");
        return;
    }

    if (!fileExists(RECORD_FILE)) {
        saveTeachOverwrite(t);
        return;
    }

    printf("\nA recording already exists.\n");
    printf("Overwrite (O), Append (A) or Cancel (C)?: ");

    char c = getch();
    printf("%c\n", c);

    if (c == 'o' || c == 'O') {
        saveTeachOverwrite(t);
    } 
    else if (c == 'a' || c == 'A') {
        saveTeachAppend(t);
    } 
    else {
        printf("\nAuto-save cancelled.\n");
    }
}

void clearTeachFile() {
    if (fileExists(RECORD_FILE)) {
        remove(RECORD_FILE);
        printf("\nRecording file deleted.\n");
    } else {
        printf("\nNo recording file exists.\n");
    }
}

void loadTeachFile(TeachData *t) {
    FILE *f = fopen(RECORD_FILE, "r");
    if (!f) {
        printf("\nCannot load: recording file missing.\n");
        return;
    }

    fscanf(f, "%d", &t->count);
    if (t->count > MAX_STEPS) t->count = MAX_STEPS;

    for (int i = 0; i < t->count; i++) {
        fscanf(f, "%d %d %d %d",
               &t->steps[i].base,
               &t->steps[i].shoulder,
               &t->steps[i].elbow,
               &t->steps[i].wrist
        );
    }

    fclose(f);
    printf("\nLoaded %d steps from %s\n", t->count, RECORD_FILE);
}
