// pid.h
#ifndef PID_H
#define PID_H

#include <stdbool.h>

typedef struct {
    // Gains
    double Kp, Ki, Kd;
    // Timing & filter
    double dt;        // seconds
    double tau;       // derivative LPF time constant (s)
    double beta;      // setpoint weight for P (0..1), usually 1
    // Saturation & anti-windup
    double out_min, out_max;
    double Kb;        // anti-windup back-calc gain (1/s)

    // State
    double integrator;
    double prev_meas;
    double d_filt;
    double out;
    bool   primed;    // false until the first measurement is seen
} PID;

void   PID_Init(PID *pid, double Kp, double Ki, double Kd,
                double dt, double tau, double out_min, double out_max,
                double beta, double Kb);

/* Clear integrator/filter state. The next update seeds prev_meas from its
 * measurement, so there is no derivative kick after a reset. */
void   PID_Reset(PID *pid);

void   PID_SetGains(PID *pid, double Kp, double Ki, double Kd);

double PID_Update(PID *pid, double setpoint, double measurement);

#endif
