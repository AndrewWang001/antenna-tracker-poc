// pid.c
#include "pid.h"

void PID_Init(PID *pid, double Kp, double Ki, double Kd,
              double dt, double tau, double out_min, double out_max,
              double beta, double Kb)
{
    pid->Kp = Kp; pid->Ki = Ki; pid->Kd = Kd;
    pid->dt = dt; pid->tau = tau;
    pid->out_min = out_min; pid->out_max = out_max;
    pid->beta = beta; pid->Kb = Kb;

    PID_Reset(pid);
}

void PID_Reset(PID *pid)
{
    pid->integrator = 0.0;
    pid->prev_meas  = 0.0;
    pid->d_filt     = 0.0;
    pid->out        = 0.0;
    pid->primed     = false;
}

void PID_SetGains(PID *pid, double Kp, double Ki, double Kd)
{
    pid->Kp = Kp; pid->Ki = Ki; pid->Kd = Kd;
}

double PID_Update(PID *pid, double setpoint, double measurement)
{
    // 0) First sample after init/reset: no history, so no derivative yet
    if (!pid->primed) {
        pid->prev_meas = measurement;
        pid->d_filt    = 0.0;
        pid->primed    = true;
    }

    // 1) P-term with setpoint weighting
    double P = pid->Kp * (pid->beta * setpoint - measurement);

    // 2) I-term (integrate error)
    double e = setpoint - measurement;
    pid->integrator += pid->Ki * pid->dt * e;

    // 3) D-term: derivative on measurement with low-pass filter
    double dy = (measurement - pid->prev_meas) / pid->dt;
    double alpha = pid->tau / (pid->tau + pid->dt);     // 0..1
    pid->d_filt = alpha * pid->d_filt + (1.0 - alpha) * dy;
    double D = - pid->Kd * pid->d_filt;                 // minus sign!

    // 4) Combine & saturate
    double u_unsat = P + pid->integrator + D;
    double u = u_unsat;
    if (u > pid->out_max) u = pid->out_max;
    if (u < pid->out_min) u = pid->out_min;

    // 5) Anti-windup back-calculation
    pid->integrator += pid->Kb * (u - u_unsat) * pid->dt;

    // 6) Save state
    pid->prev_meas = measurement;
    pid->out = u;
    return u;
}
