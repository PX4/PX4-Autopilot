#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <math.h>

#define PI 3.14159265358979323846

int main() {
    FILE *gp;
    bool experiment;
    int input_type, output_type;
    double magnitude = 0.1; 
    double duration = 20;
    double Ts = 0.02;
    double step, doublet, sine_sweep;
    int num_samples = duration/Ts;
    double time[num_samples];
    double input[num_samples];
    double throttle[num_samples], elevator[num_samples];

    printf("Enter experiment value (0 for false, 1 for true): ");
    int i;
    scanf("%d", &i);
    if (i) {
    experiment = 1;
    } else {
    experiment = 0;
    }

    if (experiment) {
        while (true) {
            printf("Experiment started.\n");
            printf("Enter input type (1 for step, 2 for doublet, 3 for sine sweep): ");
            scanf("%d", &input_type);
            int n = 0;
            switch (input_type) {
                case 1:
                    printf("Generating step with magnitude %lf and duration %lf\n", magnitude, duration);
                    // Generate step input
                    for (double t = 0; t <= duration; t += Ts) {
                        time[n] = t;
                        if (t <= 2) {
                            step = 0;
                        } else if (2 <= t <= duration) {
                            step = magnitude;
                        } else {
                            step = 0;
                        }
                        input[n] = step;
                        n++;
                    }
                    break;
                case 2:
                    printf("Generating doublet with magnitude %lf and duration %lf\n", magnitude, duration);
                    // Generate doublet input
                    for (double t = 0; t <= duration; t += Ts) {
                        time[n] = t;
                        if (t <= 2) {
                            doublet = 0;
                        } else if (2 < t && t <= 2.5) {
                            doublet = magnitude;
                        } else if (2.5 < t && t <= 3) {
                            doublet = -magnitude;
                        } else {
                            doublet = 0;
                        }
                        input[n] = doublet;
                        n++;
                    }
                    break;
                case 3:
                    printf("Generating sine sweep with magnitude %lf and duration %lf\n", magnitude, duration);
                    // Generate sine sweep input
                    double freq_start = 0.1;
                    double freq_end = 1;
                    double k = (freq_end - freq_start) / (duration-2);
                    double f;
                    for (double t = 0; t <= duration; t += Ts) {
                        time[n] = t;
                        if (t <= 2) {
                            sine_sweep = 0;
                        } else if (t >= 2 && t <= duration) {
                            f = freq_start + k*(t-2);
                            sine_sweep = magnitude*sin(2*PI*f*t);
                        } else {
                            sine_sweep = 0;
                        }
                        input[n] = sine_sweep;
                        n++;
                    }
                    break;
                default:
                    printf("Invalid input type. Please try again.\n");
                    continue;
            }
            printf("Enter output type (4 for throttle, 5 for elevator): ");
            scanf("%d", &output_type);
            switch(output_type){
                case 4:
                printf("Generating throttle with magnitude %lf and duration %lf\n", magnitude, duration);
                for (int i = 0; i < num_samples; i++) {
                throttle[i] = input[i];
                }
                // Plot the signal using gnuplot
                gp = popen("gnuplot -persistent", "w");
                fprintf(gp, "set title 'Throttle vs Time'\n");
                fprintf(gp, "set xlabel 'Time (s)'\n");
                fprintf(gp, "set ylabel 'Throttle (V)'\n");
                fprintf(gp, "plot '-' with lines title 'Throttle'\n");
                for (int i = 0; i < num_samples; i++) {
                    fprintf(gp, "%f %f\n", time[i], throttle[i]);
                }
                fprintf(gp, "e\n");
                fflush(gp);
                break;
                case 5:
                printf("Generating elevator with magnitude %lf and duration %lf\n", magnitude, duration);
                for (int i = 0; i < num_samples; i++) {
                elevator[i] = input[i];
                }
                // Plot the signal using gnuplot
                gp = popen("gnuplot -persistent", "w");
                fprintf(gp, "set title 'Elevator vs Time'\n");
                fprintf(gp, "set xlabel 'Time (s)'\n");
                fprintf(gp, "set ylabel 'Elevator (V)'\n");
                fprintf(gp, "plot '-' with lines title 'Elevator'\n");
                for (int i = 0; i < num_samples; i++) {
                    fprintf(gp, "%f %f\n", time[i], elevator[i]);
                }
                fprintf(gp, "e\n");
                fflush(gp);
                break;
            }
            break;
        }
        printf("Experiment completed.\n");
    }else {
        printf("No Experiment, default control.\n");
    }
    return 0;
}

// cd "/home/deekshakota/Desktop/just_code/" && gcc code.c -o code -lm  && "/home/deekshakota/Desktop/just_code/"code 