/*
 * telemetry.h
 *
 *  Created on: Jul 17, 2026
 *      Author: samlindwall
 */

#ifndef INC_TELEMETRY_H_
#define INC_TELEMETRY_H_


#include "main.h"   /* for UART_HandleTypeDef, HAL types */
#include "pid.h"    /* for PID_t */

/*
 * Non-blocking CSV telemetry over USART2 (the ST-Link Virtual COM Port).
 * Plug the Nucleo's USB into your Mac and this streams to /dev/tty.usbmodem*.
 *
 * Line format (matches the host-sim CSV so plot.py / the tuner both work):
 *   tick,time_s,target,measured,error,output,kp,ki,kd\r\n
 *
 * Timing: uses DMA (fire-and-forget). If a previous send is still in flight,
 * the sample is DROPPED rather than blocking the control loop. Call it from
 * the main loop after the PID cycle, NOT from an ISR.
 */

/* Emit every Nth PID cycle. 160 Hz / 2 = 80 Hz telemetry. */
#define TELEM_DECIMATE 2

/* Call once per PID cycle. Handles decimation internally.
 *   tick     - monotonically increasing PID cycle counter
 *   target   - commanded angle (deg)
 *   measured - encoder angle (deg)
 *   error    - target - measured (deg)
 *   output   - signed PID output (CCR units)
 *   pid      - pointer to the axis's gains (kp/ki/kd are read for display)
 */
void telemetry_emit(uint32_t tick, float target, float measured,
                    float error, float output, const PID_t *pid);


#endif /* INC_TELEMETRY_H_ */
