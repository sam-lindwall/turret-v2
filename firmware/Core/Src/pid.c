/*
 * pid.c
 *
 *  Created on: Jul 17, 2026
 *      Author: samlindwall
 */
#include "pid.h"
#include <math.h>

typedef enum {
    DIR_FORWARD,
    DIR_REVERSE,
    DIR_BRAKE
} Direction;


// ----- Sets motor direction via the driver pins. Both HIGH = short brake -----
void set_direction(float output, GPIO GPIO_1, GPIO GPIO_2) {

    Direction direction;

    if (output == 0.0f) {
        direction = DIR_BRAKE;
    }
    else if (output < 0.0f) {
        direction = DIR_REVERSE;
    }
    else {
        direction = DIR_FORWARD;
    }

    if (direction == DIR_BRAKE) {
        HAL_GPIO_WritePin(GPIO_1.port, GPIO_1.pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIO_2.port, GPIO_2.pin, GPIO_PIN_SET);
    } else if (direction == DIR_FORWARD) {
        HAL_GPIO_WritePin(GPIO_1.port, GPIO_1.pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIO_2.port, GPIO_2.pin, GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(GPIO_1.port, GPIO_1.pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIO_2.port, GPIO_2.pin, GPIO_PIN_SET);
    }
}


// ----- PID Controller -----
float pid(float target_position, float current_position, GPIO GPIO_1, GPIO GPIO_2, PID_t* pid) {

    const float dt = 1.0f / 160.0f;   // TODO: derive from TIM3 config instead of hardcoding

    float error      = target_position - current_position;
    float v_meas     = (current_position - pid->prev_measurement) / dt;
    float derivative = -v_meas;   // on measurement, not error, so no setpoint kick

    // used by the ff fade, the slew gate, and the friction floor gate
    pid->speed_f += 0.2f * (fabsf(v_meas) - pid->speed_f);


    // ----- Target velocity -----
    // pi updates the setpoint way slower than this loop runs, so raw tvel is a bunch of spikes. needs heavy filtering before anything uses it.
    if (!pid->started) {
        pid->prev_target = target_position;   // no velocity spike on the first call
        pid->started     = 1;
    }
    float tvel = (target_position - pid->prev_target) / dt;
    pid->prev_target = target_position;
    pid->tvel_f += 0.05f * (tvel - pid->tvel_f);

    uint8_t tracking = (pid->track_vel > 0.0f) &&
                       (fabsf(pid->tvel_f) > pid->track_vel);


    // ----- Stall escape: powered but the encoder isn't moving -----
    // uses prev_measurement from last tick, so it has to run before that gets overwritten below. stall_ticks = 0 turns it off
    uint8_t force_release = 0;
    if (pid->stall_ticks != 0) {
        if (fabsf(current_position - pid->prev_measurement) < 0.03f &&
            fabsf(pid->last_output) > 60.0f) {
            if (++pid->stall_count > pid->stall_ticks) {
                force_release    = 1;
                pid->stall_count = 0;
            }
        } else {
            pid->stall_count = 0;
        }
    }


    // ----- Deadband with hysteresis -----
    // skipped while tracking. stopping mid-sweep is what turned smooth tracking into stepping.
    // force_release is its own OR term, not OR'd into in_band -- doing that latches the axis in the band and it never comes out.
    float aerr = fabsf(error);
    if (!tracking && (force_release || ( pid->in_band && aerr < pid->band_go) || (!pid->in_band && aerr < pid->band_stop))) {
        pid->in_band = 1;
        HAL_GPIO_WritePin(GPIO_1.port, GPIO_1.pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIO_2.port, GPIO_2.pin, GPIO_PIN_SET);
        pid->integral         = 0.0f;
        pid->last_output      = 0.0f;   // slew ramp starts from zero on the next move
        pid->prev_measurement = current_position;
        pid->stall_count      = 0;
        return 0.0f;
    }
    pid->in_band = 0;

    float pd = (pid->kp * error) + (pid->kd * derivative);


    // ----- Friction feedforward -----
    float ff_dir, ff_scale;
    if (tracking) {
        ff_dir   = (pid->tvel_f > 0.0f) ? 1.0f : -1.0f;
        ff_scale = 1.0f;
    } else {
        ff_dir   = (error > 0.0f) ? 1.0f : -1.0f;
        ff_scale = 1.0f - (pid->speed_f / pid->ff_vel_fade);
        if (ff_scale < 0.0f) ff_scale = 0.0f;
    }
    float ff_mag = (ff_dir > 0.0f) ? pid->ff_fwd : pid->ff_rev;
    float ff     = ff_mag * ff_scale * ff_dir;


    // ----- Velocity feedforward (the duty the target's own motion needs) -----
    float vff = pid->kv * pid->tvel_f;


    // ----- Position integral candidate -----
    float integral_candidate;
    if (pid->ki != 0.0f) {
        integral_candidate = pid->integral + (error * dt);
        float i_max = pid->integral_limit / pid->ki;
        if (integral_candidate >  i_max) integral_candidate =  i_max;
        if (integral_candidate < -i_max) integral_candidate = -i_max;
    } else {
        integral_candidate = 0.0f;   // don't accumulate while I is set to 0
    }

    // what the controller wants before any limiting
    float desired = pd + ff + vff + (pid->ki * integral_candidate);


    // ----- Coulomb friction floor -----
    // Stationary + powered + outside the release band means stiction.
    // Guarantee the floor and let I solve the steady-state error.

    // gated on band_stop not band_go -- applying it inside the hysteresis gap
    // bounces the axis across the deadband. gated on speed_f so it drops out
    // the moment the axis breaks free, otherwise this is just bang-bang.
    // skipped while tracking since ff is already full magnitude there.
    // fric_fwd and fric_rev set to 0 to disable pan-axis floor
    if (!tracking && (pid->fric_fwd > 0.0f || pid->fric_rev > 0.0f)) {
        float fric_vel = (pid->fric_vel > 0.0f) ? pid->fric_vel : 1.0f;
        if (aerr > pid->band_stop && pid->speed_f < fric_vel) {
            float fdir = (error > 0.0f) ? 1.0f : -1.0f;
            float fmag = (fdir > 0.0f) ? pid->fric_fwd : pid->fric_rev;
            // (desired * fdir) < fmag catches "too small to move" and "pointing
            // the wrong way", which happens when D is braking
            if (fmag > 0.0f && (desired * fdir) < fmag) {
                desired = fdir * fmag;
            }
        }
    }


    // ----- Slew limit -----
    // Ramp gently through breakaway from rest.
    float applied = desired;
    if (pid->slew > 0.0f) {
        float slew_now = (pid->speed_f > 20.0f) ? (pid->slew * 4.0f) : pid->slew;

        if (desired * pid->last_output < 0.0f) {
            applied = 0.0f;   // reversal: let go now, ramp back up next tick
        } else if (fabsf(desired) > fabsf(pid->last_output)) {
            float d = desired - pid->last_output;
            if (d >  slew_now) applied = pid->last_output + slew_now;
            if (d < -slew_now) applied = pid->last_output - slew_now;
        }
    }


    // ----- Output clamp -----
    if (applied >  pid->out_limit) applied =  pid->out_limit;
    if (applied < -pid->out_limit) applied = -pid->out_limit;


    // ----- Conditional integration, checked against what actually got applied -----
    // if the slew or the clamp is holding us back in the same direction the
    // error is pushing, freeze the integral rather than wind it up. this also
    // handles the friction floor -- while the slew ramp climbs toward
    // a raised desired, the integral stays put.
    uint8_t under_driving_pos = (applied < desired - 0.001f);
    uint8_t under_driving_neg = (applied > desired + 0.001f);

    uint8_t limited_into_error = (under_driving_pos && error > 0.0f) ||
                                 (under_driving_neg && error < 0.0f);

    if (!limited_into_error) {
        pid->integral = integral_candidate;
    }

    set_direction(applied, GPIO_1, GPIO_2);

    pid->last_output      = applied;   // signed, post-slew, post-clamp
    pid->prev_measurement = current_position;

    return (applied < 0.0f) ? -applied : applied;   // magnitude -> CCR
}
