/*
 * telemetry.c
 *
 *  Created on: Jul 17, 2026
 *      Author: samlindwall
 */


#include "telemetry.h"
#include <stdio.h>

extern UART_HandleTypeDef huart2;   /* ST-Link VCP */

#define LOOP_HZ 160.0f

void telemetry_emit(uint32_t tick, float target, float measured,
                    float error, float output, const PID_t *pid)
{
    /* Decimate: only emit every Nth cycle. */
    if (tick % TELEM_DECIMATE != 0) {
        return;
    }

    /* Static so it survives while the DMA transfer reads from it. */
    static char buf[96];

    /* If the previous DMA send hasn't finished, DROP this sample rather than
     * block. Telemetry is best-effort; the control loop is not. */
    if (huart2.gState != HAL_UART_STATE_READY) {
        return;
    }

    int len = snprintf(buf, sizeof(buf),
        "%lu,%.4f,%.2f,%.2f,%.2f,%.2f,%.4f,%.4f,%.4f\r\n",
        (unsigned long)tick,
        tick / LOOP_HZ,
        target, measured, error, output,
        pid->kp, pid->ki, pid->kd);

    if (len > 0) {
        HAL_UART_Transmit_DMA(&huart2, (uint8_t *)buf, (uint16_t)len);
    }
}
