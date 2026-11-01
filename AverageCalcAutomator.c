#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>

#define FILE_NAME "DynamicCaseTimingLog1.csv"
#define TEMP_FILE "temp.csv"
#define BACKUP_FILE "backup.csv"
#define COLS 4


int main(void) {
    FILE *in = fopen(FILE_NAME, "r");
    FILE *out;
    char line[256], original[256], *token;
    double sum[COLS] = {0};
    int rows = 0, i;

    if (!in) { perror("Could not open data.csv"); return 1; }
    out = fopen(TEMP_FILE, "w");
    if (!out) { perror("Could not create temp.csv"); fclose(in); return 1; }

    if (fgets(line, sizeof(line), in)) fputs(line, out);

    while (fgets(line, sizeof(line), in)) {
        double values[COLS];
        int valid = 1;
        strcpy(original, line);
        token = strtok(line, ",");

        if (!token || strncmp(token, "Average", 7) == 0) continue;

        for (i = 0; i < COLS; i++) {
            token = strtok(NULL, ",");
            if (!token) { valid = 0; break; }
            values[i] = atof(token);
        }

        fputs(original, out);
        if (valid) {
            for (i = 0; i < COLS; i++) sum[i] += values[i];
            rows++;
        }
    }

    if (rows > 0) {
        fprintf(out, "Average");
        for (i = 0; i < COLS; i++) fprintf(out, ", %.2f", sum[i] / rows);
        fprintf(out, "\n");
    }

    fclose(in);
    fclose(out);

    remove(BACKUP_FILE);
    if (rename(FILE_NAME, BACKUP_FILE) != 0) {
        perror("Could not back up data.csv"); return 1;
    }
    if (rename(TEMP_FILE, FILE_NAME) != 0) {
        perror("Could not update data.csv");
        rename(BACKUP_FILE, FILE_NAME);
        return 1;
    }
    remove(BACKUP_FILE);

    printf("Updated %s with averages of %d cycles.\n", FILE_NAME, rows);
    return 0;
}