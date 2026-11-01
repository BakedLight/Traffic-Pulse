#include <stdio.h>

int main() {
    FILE *fp = fopen("NormalCaseTimingLog2.csv", "a");
    if (fp == NULL) {
        printf("Error opening file!\n");
        return 1;
    }
    return 0;
}