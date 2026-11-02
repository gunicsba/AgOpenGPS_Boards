/* zCan.ino - CAN bus, read only: SASA steering wheel sensor (kickout) and the CNH engage button.
   Autosteer_gps_teensy_v4_1, 2026, from the branches hydliftv2_TM171_SASA_special (CAN_SASA.ino) and
   hydlift_tm171_CNH (CAN_Fendt_KBus.ino). Switched on on the web page (Setup tab, zBoard.ino).
   Licence: GNU General Public License v3 or later (file COPYING), WITHOUT ANY WARRANTY.
   SPDX-License-Identifier: GPL-3.0-or-later
*/
// CAN3 (Teensy pins 30 CRX3 / 31 CTX3, through the board's CAN transceiver), 250 kbit/s, LISTEN_ONLY: the
// Teensy never sends on the bus and does not acknowledge frames. Started only once one of the two is on.
//
// SASA steering wheel sensor: its rotation speed is the pressure sensor value (Autosteer.ino), so the kickout
// of the pressure sensor in AgOpenGPS works with it. Both frames are taken:
//   0x0CFF104D (extended, J1939): bytes 0-1 angle (12 bit LE), bytes 2-3 speed (16 bit LE, 0x5000 = 0, / 20)
//   0x301 (standard, ~100 Hz):    bytes 0-1 angle (12 bit LE), byte 2 counter; speed = angle change over
//                                 20 frames (~200 ms)
// CNH engage button, frame 0x14FF7706 (extended): toggles autosteer like a second steer button, so Steer
// button must be selected in AgOpenGPS.
//   armrest button:          byte 0 = 0x82 and byte 1 = 1 pressed / 0 released
//   hydraulic joystick up:   byte 3 = 0xF1 pressed / 0xF0 released

// <FlexCAN_T4.h> is included in Autosteer_gps_teensy_v4_1.ino, before the prototypes the IDE generates

#define CAN_SASA_ID_EXT   0x0CFF104D   // SASA, machine 1 (J1939)
#define CAN_SASA_ID_STD   0x301        // SASA, machine 2
#define CAN_BTN_ID        0x14FF7706   // CNH armrest / joystick buttons
#define CAN_SASA_WINDOW   20           // 0x301: frames per speed value

FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> canBus;
bool     canStarted = false;
uint32_t canFrames = 0;              // frames received (the filters let only the three IDs in)
uint32_t canFps = 0;                 // frames per second (webRates, zWeb.ino)
uint32_t canSasaMs = 0;              // millis of the last SASA frame, 0 = none yet
uint32_t canBtnMs = 0;               // millis of the last button frame, 0 = none yet
bool     canBtnDown = false;         // engage button held
uint32_t canLastId = 0;              // the last frame, for the web page
bool     canLastExt = false;
uint8_t  canLastLen = 0;
uint8_t  canLastBuf[8];
int16_t  canSasaWinAngle = -1;       // 0x301: angle at the start of the window, -1 = not seeded
uint8_t  canSasaWinCount = 0;

// From setup() and from the web page: start the bus once the SASA sensor or the engage button is on
void canSetup()
{
    if (canStarted) return;
    if (!canSasa && !canBtnMode) { Serial.println("CAN3: off (SASA sensor and engage button off on the web page)"); return; }
    canBus.begin();
    canBus.setBaudRate(250000, LISTEN_ONLY);
    canBus.enableFIFO();
    canBus.setFIFOFilter(REJECT_ALL);
    canBus.setFIFOFilter(0, CAN_SASA_ID_EXT, EXT);
    canBus.setFIFOFilter(1, CAN_SASA_ID_STD, STD);
    canBus.setFIFOFilter(2, CAN_BTN_ID, EXT);
    canStarted = true;
    Serial.println("CAN3 started: 250 kbit/s, listen only (SASA 0x0CFF104D / 0x301, engage button 0x14FF7706)");
}

// From the web page, when a setting changes: start again from nothing
void canReset()
{
    canSteeringWheelSpeed = 0;
    canSasaMs = 0;
    canSasaWinAngle = -1;
    canSasaWinCount = 0;
    canBtnDown = false;
    canBtnMs = 0;
    canEngagePress = false;
    sensorReading = 0;
}

static void canSasaFrame(const CAN_message_t& m)
{
    int16_t speed = 0;
    if (m.id == CAN_SASA_ID_EXT)
    {
        // bytes 2-3 = signed speed (16 bit LE, centred at 0x5000, / 20)
        const uint16_t raw16 = (uint16_t)m.buf[2] | ((uint16_t)m.buf[3] << 8);
        speed = (int16_t)(((int32_t)raw16 - 0x5000) / 20);
    }
    else
    {
        // speed = angle change over the window
        const int16_t angle = (int16_t)(((uint16_t)m.buf[0] | ((uint16_t)m.buf[1] << 8)) & 0x0FFF);
        if (canSasaWinAngle < 0)
        {
            canSasaWinAngle = angle;                    // first frame: seed the window
            canSasaWinCount = 0;
        }
        else if (++canSasaWinCount >= CAN_SASA_WINDOW)
        {
            int16_t delta = angle - canSasaWinAngle;
            if (delta > 2048)  delta -= 4096;           // 12 bit wrap-around
            if (delta < -2048) delta += 4096;
            speed = delta;
            canSasaWinAngle = angle;
            canSasaWinCount = 0;
        }
        else speed = canSteeringWheelSpeed;             // still collecting: hold the last value
    }
    if (abs(speed) < 2) speed = 0;                      // dead zone: standing still
    canSteeringWheelSpeed = speed;
    canSasaMs = millis() | 1;
}

static void canBtnFrame(const CAN_message_t& m)
{
    bool down = canBtnDown;
    if ((canBtnMode & 1) && m.buf[0] == 0x82)
    {
        if (m.buf[1] == 1) down = true;
        else if (m.buf[1] == 0) down = false;
    }
    if (canBtnMode & 2)
    {
        if (m.buf[3] == 0xF1) down = true;
        else if (m.buf[3] == 0xF0) down = false;
    }
    if (down && !canBtnDown) canEngagePress = true;     // taken by autosteerLoop, Autosteer.ino
    canBtnDown = down;
    canBtnMs = millis() | 1;
}

// From loop(): every frame waiting
void canLoop()
{
    if (!canStarted) return;
    CAN_message_t m;
    while (canBus.read(m))
    {
        canFrames++;
        canLastId = m.id;
        canLastExt = m.flags.extended;
        canLastLen = m.len;
        memcpy(canLastBuf, m.buf, sizeof(canLastBuf));
        if (m.len < 4) continue;
        if (canSasa && (m.id == CAN_SASA_ID_EXT || m.id == CAN_SASA_ID_STD)) canSasaFrame(m);
        else if (canBtnMode && m.id == CAN_BTN_ID) canBtnFrame(m);
    }
}

// The CAN part of /api/status
void canJson(Json& j, uint32_t now)
{
    j.obj("can");
    j.bo("on", canStarted);
    j.u("fps", canFps);
    j.bo("sasa", canSasa);
    j.i("speed", canSteeringWheelSpeed);
    j.i("sasaAgo", canSasaMs ? (long)(now - canSasaMs) : -1);
    j.u("btn", canBtnMode);
    j.bo("down", canBtnDown);
    j.i("btnAgo", canBtnMs ? (long)(now - canBtnMs) : -1);
    if (canFrames)
    {
        char t[48];
        int k = snprintf(t, sizeof(t), canLastExt ? "%08lX:" : "%03lX:", (unsigned long)canLastId);
        for (uint8_t i = 0; i < canLastLen && i < 8 && k < (int)sizeof(t) - 3; i++) k += snprintf(t + k, sizeof(t) - k, " %02X", canLastBuf[i]);
        j.str("last", t);
    }
    j.close();
}
