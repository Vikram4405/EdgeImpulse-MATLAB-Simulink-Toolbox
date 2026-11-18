/*
 * motor_bearing_accel_sfun.cpp
 * 
 * Level-2 C-MEX S-Function: Industrial Motor Bearing & 3-Axis MEMS Accelerometer Physics Engine
 * 
 * Models a 6205 Deep-Groove Ball Bearing supporting a rotating induction motor shaft,
 * solving 2nd-order structural housing resonance ODEs and outputting realistic 3-axis
 * MEMS accelerometer readings [accX, accY, accZ] in m/s^2.
 *
 * Ports:
 *   Input Port 0 (width 1): Fault Mode
 *       1 = HEALTHY (Normal motor operation)
 *       2 = OUTER RACE FAULT (BPFO periodic ball-crack shock impulses)
 *       3 = INNER RACE FAULT (BPFI shock impulses modulated by 1X shaft load zone)
 *       4 = ROTOR UNBALANCE (Eccentric mass centrifugal shaking at 1X & 2X RPM)
 *   Input Port 1 (width 1): Motor Shaft Speed in RPM (e.g., 1800 RPM)
 *   Input Port 2 (width 1): Fault Severity Scale (e.g., 1.0 = nominal)
 *
 *   Output Port 0 (width 3): 3-Axis Accelerometer [accX, accY, accZ] (m/s^2)
 *   Output Port 1 (width 1): Instantaneous Bearing Defect Impact Force (Newtons)
 *   Output Port 2 (width 2): Live Characteristic Fault Frequencies [BPFO_Hz, BPFI_Hz]
 */

#define S_FUNCTION_NAME  motor_bearing_accel_sfun
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"
#include <math.h>
#include <stdlib.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// DWork indices for continuous physical states across time steps
#define DWORK_STATE_VEC  0  // [theta_shaft, phase_bpfo, phase_bpfi, x_pos, x_vel, y_pos, y_vel, z_pos, z_vel, rng_seed]
#define STATE_LEN        10

// Simple deterministic Box-Muller Gaussian noise generator
static double randn_gaussian(unsigned int *seed) {
    *seed = (*seed * 1664525u + 1013904223u);
    double u1 = ((*seed & 0xFFFFFF) + 1.0) / 16777217.0;
    *seed = (*seed * 1664525u + 1013904223u);
    double u2 = ((*seed & 0xFFFFFF) + 1.0) / 16777217.0;
    return sqrt(-2.0 * log(u1)) * cos(2.0 * M_PI * u2);
}

static void mdlInitializeSizes(SimStruct *S) {
    // 1 Parameter: Sample Time (default 0.001 s = 1000 Hz)
    ssSetNumSFcnParams(S, 1);
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) {
        return;
    }

    // 3 Input Ports: Fault_Mode, Motor_RPM, Severity
    if (!ssSetNumInputPorts(S, 3)) return;
    for (int i = 0; i < 3; i++) {
        ssSetInputPortWidth(S, i, 1);
        ssSetInputPortDirectFeedThrough(S, i, 1);
        ssSetInputPortRequiredContiguous(S, i, 1);
    }

    // 3 Output Ports:
    // Port 0: [accX, accY, accZ] (3-axis accelerometer in m/s^2)
    // Port 1: Defect Impact Force (N)
    // Port 2: [BPFO_Hz, BPFI_Hz]
    if (!ssSetNumOutputPorts(S, 3)) return;
    ssSetOutputPortWidth(S, 0, 3);
    ssSetOutputPortWidth(S, 1, 1);
    ssSetOutputPortWidth(S, 2, 2);

    ssSetNumSampleTimes(S, 1);

    // DWork state vector
    ssSetNumDWork(S, 1);
    ssSetDWorkWidth(S, DWORK_STATE_VEC, STATE_LEN);
    ssSetDWorkDataType(S, DWORK_STATE_VEC, SS_DOUBLE);

    ssSetOptions(S, SS_OPTION_EXCEPTION_FREE_CODE);
}

static void mdlInitializeSampleTimes(SimStruct *S) {
    real_T ts = 0.001; // 1000 Hz default
    if (mxGetNumberOfElements(ssGetSFcnParam(S, 0)) > 0) {
        ts = mxGetScalar(ssGetSFcnParam(S, 0));
    }
    ssSetSampleTime(S, 0, ts);
    ssSetOffsetTime(S, 0, 0.0);
}

#define MDL_START
static void mdlStart(SimStruct *S) {
    real_T *state = (real_T*)ssGetDWork(S, DWORK_STATE_VEC);
    for (int i = 0; i < STATE_LEN; i++) {
        state[i] = 0.0;
    }
    state[9] = 13579.0; // Initial PRNG seed
}

static void mdlOutputs(SimStruct *S, int_T tid) {
    const real_T *u_fault = ssGetInputPortRealSignal(S, 0);
    const real_T *u_rpm   = ssGetInputPortRealSignal(S, 1);
    const real_T *u_sev   = ssGetInputPortRealSignal(S, 2);

    real_T *y_accel = ssGetOutputPortRealSignal(S, 0);
    real_T *y_force = ssGetOutputPortRealSignal(S, 1);
    real_T *y_freqs = ssGetOutputPortRealSignal(S, 2);

    real_T *state = (real_T*)ssGetDWork(S, DWORK_STATE_VEC);

    real_T dt = ssGetSampleTime(S, 0);
    if (dt <= 0.0) dt = 0.001;

    int fault_mode = (int)floor(u_fault[0] + 0.5);
    double rpm     = (u_rpm[0] >= 0.0) ? u_rpm[0] : 0.0;
    double sev     = (u_sev[0] >= 0.0) ? u_sev[0] : 1.0;

    // 1. Compute 6205 Bearing Characteristic Frequencies from RPM
    //    Nb = 9 balls, d = 7.94 mm, D = 39.04 mm, contact angle = 0 deg
    double fr = rpm / 60.0; // Shaft rotational frequency (Hz)
    double ratio = 7.94 / 39.04;
    double bpfo = (9.0 / 2.0) * fr * (1.0 - ratio); // ~3.585 * fr (107.55 Hz @ 1800 RPM)
    double bpfi = (9.0 / 2.0) * fr * (1.0 + ratio); // ~5.415 * fr (162.45 Hz @ 1800 RPM)

    y_freqs[0] = bpfo;
    y_freqs[1] = bpfi;

    // Unpack state variables
    double theta_shaft = state[0];
    double phase_bpfo  = state[1];
    double phase_bpfi  = state[2];
    double x_pos       = state[3];
    double x_vel       = state[4];
    double y_pos       = state[5];
    double y_vel       = state[6];
    double z_pos       = state[7];
    double z_vel       = state[8];
    unsigned int seed  = (unsigned int)state[9];

    // Advance angular phases
    theta_shaft += 2.0 * M_PI * fr * dt;
    if (theta_shaft > 2.0 * M_PI) theta_shaft -= 2.0 * M_PI;

    double prev_bpfo = phase_bpfo;
    phase_bpfo += bpfo * dt;
    bool hit_outer = false;
    if (phase_bpfo >= 1.0) {
        phase_bpfo -= floor(phase_bpfo);
        hit_outer = true;
    }

    phase_bpfi += bpfi * dt;
    bool hit_inner = false;
    if (phase_bpfi >= 1.0) {
        phase_bpfi -= floor(phase_bpfi);
        hit_inner = true;
    }

    // 2. Calculate Excitation Forces on Bearing Housing
    double F_impact = 0.0;
    double F_unbal_x = 0.0;
    double F_unbal_y = 0.0;
    double F_axial_z = 0.0;

    // Baseline subtle shaft rotation harmonic (present in all running motors)
    F_unbal_x += 0.35 * cos(theta_shaft);
    F_unbal_y += 0.35 * sin(theta_shaft);
    F_axial_z += 0.15 * sin(theta_shaft);

    if (fault_mode == 2) {
        // OUTER RACE FAULT (BPFO): Stationary crack in load zone -> consistent strong radial shock pulses
        if (hit_outer || phase_bpfo < 0.08) {
            double pulse_env = exp(-phase_bpfo * 35.0);
            F_impact = 45.0 * sev * pulse_env;
        }
    } else if (fault_mode == 3) {
        // INNER RACE FAULT (BPFI): Rotating crack moves in and out of radial load zone at shaft speed fr
        if (hit_inner || phase_bpfi < 0.08) {
            double load_zone_mod = 0.55 + 0.45 * cos(theta_shaft); // Modulated by 1X shaft angle
            double pulse_env = exp(-phase_bpfi * 40.0);
            F_impact = 52.0 * sev * load_zone_mod * pulse_env;
        }
    } else if (fault_mode == 4) {
        // ROTOR UNBALANCE / MISALIGNMENT: Heavy centrifugal force F = m * e * w^2 at 1X and 2X shaft speed
        double speed_scale = (fr / 30.0) * (fr / 30.0);
        F_unbal_x += sev * speed_scale * (7.5 * cos(theta_shaft) + 2.8 * cos(2.0 * theta_shaft));
        F_unbal_y += sev * speed_scale * (7.5 * sin(theta_shaft) + 2.8 * sin(2.0 * theta_shaft));
        F_axial_z += sev * speed_scale * (3.5 * cos(theta_shaft)); // Misalignment axial wobble
    }

    // 3. Solve 2nd-Order Structural Housing Resonance ODE
    //    Natural resonant frequency fn = 310 Hz (radial), 260 Hz (axial), damping zeta = 0.065
    double wn_rad = 2.0 * M_PI * 310.0;
    double zeta   = 0.065;

    // Spring-damper restoring acceleration + impact shock excitation
    // Scale impact force into housing acceleration excitation
    double exc_x = (F_impact * 0.85) + F_unbal_x;
    double exc_y = (F_impact * 1.15) + F_unbal_y;
    double exc_z = (F_impact * 0.45) + F_axial_z;

    // Sub-step integration (4 sub-steps per 1 ms sample for numerical stability of 310 Hz resonance)
    int substeps = 4;
    double h = dt / substeps;
    double acc_res_x = 0.0, acc_res_y = 0.0, acc_res_z = 0.0;

    for (int s = 0; s < substeps; s++) {
        acc_res_x = exc_x - 2.0 * zeta * wn_rad * x_vel - (wn_rad * wn_rad) * x_pos;
        x_vel += acc_res_x * h;
        x_pos += x_vel * h;

        acc_res_y = exc_y - 2.0 * zeta * wn_rad * y_vel - (wn_rad * wn_rad) * y_pos;
        y_vel += acc_res_y * h;
        y_pos += y_vel * h;

        double wn_ax = 2.0 * M_PI * 260.0;
        acc_res_z = exc_z - 2.0 * zeta * wn_ax * z_vel - (wn_ax * wn_ax) * z_pos;
        z_vel += acc_res_z * h;
        z_pos += z_vel * h;
    }

    // 4. Compute Final 3-Axis MEMS Accelerometer Readings (m/s^2)
    //    Includes resonant casing acceleration + direct centrifugal acceleration + MEMS sensor floor noise
    double noise_x = 0.18 * randn_gaussian(&seed);
    double noise_y = 0.18 * randn_gaussian(&seed);
    double noise_z = 0.18 * randn_gaussian(&seed);

    // Output Port 0: [accX (Horizontal Radial), accY (Vertical Radial + Gravity 9.81), accZ (Axial)]
    y_accel[0] = (acc_res_x * 0.35) + F_unbal_x + noise_x;
    y_accel[1] = 9.81 + (acc_res_y * 0.35) + F_unbal_y + noise_y;
    y_accel[2] = (acc_res_z * 0.35) + F_axial_z + noise_z;

    // Output Port 1: Bearing Defect Impact Force (N)
    y_force[0] = F_impact;

    // Save updated state
    state[0] = theta_shaft;
    state[1] = phase_bpfo;
    state[2] = phase_bpfi;
    state[3] = x_pos;
    state[4] = x_vel;
    state[5] = y_pos;
    state[6] = y_vel;
    state[7] = z_pos;
    state[8] = z_vel;
    state[9] = (double)seed;
}

static void mdlTerminate(SimStruct *S) {
}

#ifdef MATLAB_MEX_FILE
#include "simulink.c"
#else
#include "cg_sfun.h"
#endif
