/*
 * pid.h
 *
 *  Created on: Jul 17, 2026
 *      Author: samlindwall
 */

#ifndef INC_PID_H_
#define INC_PID_H_

#include "main.h"   /* HAL types, __HAL_TIM_SET_COMPARE, GPIO_TypeDef */

typedef struct {
    GPIO_TypeDef *port;
    uint16_t      pin;
} GPIO;

typedef struct {
    /* position loop gains */
    float kp, ki, kd;

    /* limits */
    float out_limit;        /* max |duty| in CCR counts                        */
    float integral_limit;   /* max |ki * integral| contribution, in counts     */
    float slew;             /* max increase in |output| per tick; <=0 disables */

    /* friction feedforward -- from the open-loop duty/velocity fit:
     * duty = kv * velocity + ff, so ff is the Coulomb intercept and kv the slope */
    float ff_fwd;           /* counts, forward direction                       */
    float ff_rev;           /* counts, reverse direction                       */
    float ff_vel_fade;      /* deg/s at which static ff fades out; only applied
                             * when NOT tracking (Coulomb friction does not
                             * actually fall off with speed -- the fade exists
                             * to stop the ff sign relay chattering when the
                             * setpoint is stationary and error dithers)       */
    float kv;               /* counts per deg/s of target velocity             */

    /* deadband, stationary target */
    float band_stop;        /* deg: enter band below this                      */
    float band_go;          /* deg: leave band above this (hysteresis)         */

    /* tracking mode: while |target velocity| exceeds track_vel the deadband is
     * bypassed entirely, so the axis never stops and never has to break
     * stiction again mid-sweep. track_vel <= 0 disables tracking mode.        */
    float track_vel;        /* deg/s of target motion that counts as tracking  */


    /* stall escape */
    uint16_t stall_ticks;   /* ticks stuck under power before release; 0 off   */
    uint16_t stall_count;

    /* state */
    float   prev_measurement;
    float   prev_target;
    float   tvel_f;         /* filtered target velocity, deg/s                 */
    float   speed_f;        /* filtered |measured velocity|, deg/s             */
    float   integral;
    float   v_integral;
    float   last_output;    /* signed, post-slew, post-clamp                   */
    uint8_t in_band;
    uint8_t started;
    uint8_t reseed;         /* set by i2c_recover, cleared in main loop        */

    /* I2C recovery (owned by main.c) */
    uint8_t fail_count;



    float fric_fwd;   /* min |output| to break stiction, +ve direction */
    float fric_rev;   /* min |output| to break stiction, -ve direction */
    float fric_vel;   /* deg/s; floor only applies below this speed_f */
} PID_t;

void  set_direction(float output, GPIO g1, GPIO g2);
float pid(float target_position, float current_position,
          GPIO g1, GPIO g2, PID_t *pid);

#endif /* INC_PID_H_ */
