/* The Clear BSD License
 *
 * Copyright (c) 2025 EdgeImpulse Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted (subject to the limitations in the disclaimer
 * below) provided that the following conditions are met:
 *
 *   * Redistributions of source code must retain the above copyright notice,
 *   this list of conditions and the following disclaimer.
 *
 *   * Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 *
 *   * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY
 * THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
 * IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @brief   Sample & upload data to Edge Impulse Studio.
 * @details Select 1 or multiple sensors by un-commenting the defines and select
 * a desired sample frequency. When this sketch runs, you can see raw sample
 * values outputted over the serial line. Now connect to the studio using the
 * `edge-impulse-data-forwarder` and start capturing data
 */
// #define SAMPLE_ACCELEROMETER

/**
 * Configure the sample frequency. This is the frequency used to send the data
 * to the studio regardless of the frequency used to sample the data from the
 * sensor. This differs per sensors, and can be modified in the API of the sensor
 */
#define FREQUENCY_HZ        10


/* Include ----------------------------------------------------------------- */
#include "Particle.h"
#include "ADXL362DMA.h"
#include "adxl345.h"

SYSTEM_THREAD(ENABLED);
SerialLogHandler logHandler(LOG_LEVEL_ERROR);

/* Constants --------------------------------------------------------------- */
#if (FREQUENCY_HZ <= 0)
#error "FREQUENCY_HZ should have a value greater dan 0"
#endif
#define INTERVAL_MS         (1000 / FREQUENCY_HZ)
#define CONVERT_G_TO_MS2    9.80665f

/* Forward declerations ---------------------------------------------------- */
void ei_printf(const char *format, ...);

/* Private variables ------------------------------------------------------- */
#ifdef SAMPLE_ACCELEROMETER
ADXL362DMA accel(SPI, A2);
#else
ADXL345 accel;
#endif

void setup()
{
    delay(2000);
    ei_printf("Edge Impulse sensor data ingestion\r\n");

    /* Init & start sensors */
#ifdef SAMPLE_ACCELEROMETER
    accel.softReset();
    while(accel.readStatus() == 0) {
        ei_printf("no status yet, waiting for accelerometer\r\n");
        delay(1000);
    }

    accel.writeFilterControl(accel.RANGE_2G, false, false, accel.ODR_200);
    accel.setMeasureMode(true);
#else
    accel.powerOn();
    accel.setRangeSetting(2);
#endif

}

void loop() {

    delay(INTERVAL_MS);

#ifdef SAMPLE_ACCELEROMETER
    int16_t acc[3];
    accel.readXYZ(acc[0], acc[1], acc[2]);
    ei_printf("%f, %f, %f,"
        ,(((float)(acc[0] * 2)) / 2048.f) * CONVERT_G_TO_MS2
        ,(((float)(acc[1] * 2)) / 2048.f) * CONVERT_G_TO_MS2
        ,(((float)(acc[2] * 2)) / 2048.f) * CONVERT_G_TO_MS2
    );
#else
    signed short xyz[3];
    accel.readAccel(xyz);
    ei_printf("%f, %f, %f,"
        ,(((float)xyz[0]) * 0.00389f) * CONVERT_G_TO_MS2
        ,(((float)xyz[1]) * 0.00389f) * CONVERT_G_TO_MS2
        ,(((float)xyz[2]) * 0.00389f) * CONVERT_G_TO_MS2
    );
#endif

    ei_printf("\r\n");
}

/**
* @brief      Printf function uses vsnprintf and output using Arduino Serial
*
* @param[in]  format     Variable argument list
*/
void ei_printf(const char *format, ...)
{
    static char print_buf[1024] = { 0 };

    va_list args;
    va_start(args, format);
    int r = vsnprintf(print_buf, sizeof(print_buf), format, args);
    va_end(args);

    if (r > 0) {
        Serial.write(print_buf);
    }
}
