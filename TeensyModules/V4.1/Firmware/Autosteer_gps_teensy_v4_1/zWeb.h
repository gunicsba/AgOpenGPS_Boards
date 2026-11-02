/* zWeb.h - declarations shared by the web page files (zWeb.ino, zOTA.ino, zBoard.ino, TM171.ino).
   Autosteer_gps_teensy_v4_1, 2026. Web page, firmware update and TM171 tools ported from AOG_CAN_Teensy4.1,
   which took them from renatorgr/Boards, Firmware/v4.5_AiO_WEB (written 2026 by Claude (Anthropic) for renatorgr).
   Licence: GNU General Public License v3 or later (file COPYING), WITHOUT ANY WARRANTY.
   SPDX-License-Identifier: GPL-3.0-or-later
*/
// In a header because the Arduino IDE puts the function prototypes it generates near the top of the
// sketch, before any type defined in an .ino file.
#ifndef ZWEB_H
#define ZWEB_H

#include <stdint.h>
#include "zWebJson.h"

#define FW_NAME    "Autosteer_gps_teensy_v4_1"
#define FW_VERSION "02.11.2026 web"

// Board setup from the web page (zBoard.ino), kept in the EEPROM (200 = magic, 201-207 = this).
// Serial ports: BOARD_PORT_AUTO = detected at start-up as before, BOARD_PORT_NONE = not used, 1-8 = SerialN.
#define BOARD_PORT_AUTO 0xFF
#define BOARD_PORT_NONE 0
struct BoardCfg
{
    uint8_t gnss;       // main receiver (GGA): auto (Serial2 / Serial7, swapped while there is no GGA) or SerialN
    uint8_t gnss2;      // dual heading receiver (RELPOSNED): auto when gnss is auto, else none or SerialN
    uint8_t imu;        // TM171: auto (Serial7, then Serial5), none or SerialN
    uint8_t radio;      // RTK radio: none or SerialN (Serial3 before the web page)
    uint8_t imuPref;    // 1 = BNO08x, 2 = TM171, else BNO08x first (when both are found)
    uint8_t fema;       // 1 = John Deere FEMA valve: the pressure sensor input reads its PWM signal (JOHNDEERE)
    uint8_t remote;     // 1 = the remote input is a pulse sensor: pulses / s sent as the pressure sensor value
};

// TM171 settings form from the web page (TM171.ino, zWeb.ino)
struct TmSetForm
{
    bool     gyro, acc, mag;           // sensors used by the fusion
    bool     gyroErr, selfAdapt;       // GyroErrFilter, magnetometer Self-Adapt filter
    bool     status;                   // status output (temperature, QoS)
    bool     save;                     // save to the TM171's flash (else this power-on only)
    uint8_t  boot;                     // 0 auto, 1 static, 2 dynamic
    uint16_t accGain, magGain;         // x 0.01
    uint16_t silent;                   // power-on silent time, s (0..65)
    uint16_t odr;                      // output data rate, Hz (0 = keep as read)
};

// A line for the Messages tab of the web page (and USB serial)
void webLog(const char* fmt, ...) __attribute__((format(printf, 1, 2)));

#endif
