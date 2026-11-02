/* zMotor.ino - direct drive steering motors on the CAN bus: Keya, Allynav, FJD.
   Autosteer_gps_teensy_v4_1, 2026, from the branches Keya_TM171_Allynav (KeyaCANBUS.ino: Keya by lansalot,
   Allynav after Andreas' AllyNavKeyaCANBUS.ino) and fjd (KeyaCANBUS.ino: the FJD steering wheel).
   Chosen on the web page (Setup tab, zBoard.ino); the Teensy restarts to take it.
   Licence: GNU General Public License v3 or later (file COPYING), WITHOUT ANY WARRANTY.
   SPDX-License-Identifier: GPL-3.0-or-later
*/
// CAN3 (pins 30 / 31) then belongs to the motor, in normal mode (it sends; zCan.ino starts it). motorDrive
// (AutosteerPID.ino) gives the PID output here instead of to the PWM pins, every 25 ms; 0 = motor off.
//   Keya:    250, 500, 125 or 1000 kbit/s, tried in turn until its heartbeat comes. Commands to 0x06000001
//            (extended): 23 0D 20 01 enable, 23 0C 20 01 disable, 23 00 20 01 <speed -995 .. 998> steer.
//            Heartbeat 0x07000001: bytes 4-5 current (signed, high byte first, A), 6-7 error code.
//   Allynav: the same commands to 0x06C73001, speed -900 .. 900. Heartbeat 0x04854001: bytes 0-1 current
//            (0.1 A), 6-7 error bits. Once found, parameter 0x0132 is set to 2. It sends its heartbeat only after
//            a command, so every bit rate tried gets a disable.
//   FJD:     1000 kbit/s. Commands to 0x201 (standard): 02 00 00 19 00 10 00 00 switches it on / off, sent while
//            its state is not the wanted one; 00 <speed -5000 .. 5000> 19 00 00 00 00 steers. Status 0x200,
//            byte 0: 00 ready, 05 06 07 09 on, 08 shutting down, 13 error (cleared with a trigger). It sends no
//            current: the kickout is the current sensor input.
// The motor current goes to the current kickout (Autosteer.ino), which takes the higher of it and the current
// sensor input: amps * 20, 0 - 255 = 0 - 12.75 A (100 % in AgOpenGPS), averaged 95 / 5 per heartbeat (20 ms), as on
// the Keya branch since its int8 fix (before it the value stopped at 127, about 6.4 A). No data from the motor: on
// the web page, and on the AgOpenGPS screen (PGN 221) while autosteer is on there.

#define MOTOR_KEYA           1
#define MOTOR_ALLYNAV        2
#define MOTOR_FJD            3

#define KEYA_CMD_ID          0x06000001
#define KEYA_HEARTBEAT_ID    0x07000001
#define ALLYNAV_CMD_ID       0x06C73001
#define ALLYNAV_HEARTBEAT_ID 0x04854001
#define FJD_CMD_ID           0x201
#define FJD_STATUS_ID        0x200
#define MOTOR_SCAN_STEP_MS   400        // Keya / Allynav: time per bit rate until the heartbeat comes
#define MOTOR_LOST_MS        500        // no heartbeat / status for this long: no data from the motor
#define MOTOR_AMPS_FULL      12.75      // motor current that is 255 (100 %) for the current kickout: amps * 20

static const uint32_t motorBauds[] = { 250000, 500000, 125000, 1000000 };
uint8_t  motorBaudIdx = 0;
uint32_t motorBaud = 0;              // bit rate in use
bool     motorFound = false;         // its heartbeat / status came at least once
uint32_t motorDataMs = 0;            // millis of its last heartbeat / status, 0 = none yet
uint32_t motorScanMs = 0;
uint32_t motorMsgMs = 0;             // last "no data" message to AgOpenGPS
bool     motorLost = true;           // as logged
float    motorAmps = 0;              // the last heartbeat's current, A
float    motorAmpsPeak = 0;          // the highest in the last 5 s, A (web page)
uint32_t motorAmpsPeakMs = 0;
uint16_t keyaCode = 0;               // Keya heartbeat bytes 6-7
uint8_t  allynavErr1 = 0, allynavErr2 = 0;   // Allynav heartbeat bytes 7, 6
uint8_t  fjdState = 0xAA;            // FJD status byte 0, 0xAA = none yet
bool     fjdOn = false;
bool     fjdStatusSinceTrigger = true;
uint32_t fjdTriggerMs = 0;

const char* motorName(uint8_t m)
{
    return m == MOTOR_KEYA ? "Keya" : m == MOTOR_ALLYNAV ? "Allynav" : m == MOTOR_FJD ? "FJD" : "none";
}

static void motorSend(uint32_t id, bool ext, const uint8_t* d)
{
    CAN_message_t m;
    m.id = id;
    m.flags.extended = ext;
    m.len = 8;
    memcpy(m.buf, d, 8);
    canBus.write(m);
}

// Keya / Allynav: 23 <cmd> 20 01 <speed: high, low byte, then the sign>
static void keyaCmd(uint8_t cmd, int16_t speed)
{
    const uint8_t sign = speed < 0 ? 0xFF : 0x00;
    const uint8_t d[8] = { 0x23, cmd, 0x20, 0x01, (uint8_t)(speed >> 8), (uint8_t)speed, sign, sign };
    motorSend(steerMotor == MOTOR_ALLYNAV ? ALLYNAV_CMD_ID : KEYA_CMD_ID, true, d);
}

static void allynavInit()
{
    keyaCmd(0x0D, 0);                                                 // enable
    delay(10);
    static const uint8_t d[8] = { 0x24, 0x01, 0x32, 0x02, 0x00, 0x02, 0x00, 0x00 };   // parameter 0x0132 = 2
    motorSend(ALLYNAV_CMD_ID, true, d);
    delay(10);
    keyaCmd(0x0C, 0);                                                 // disable
}

// FJD: the trigger switches the wheel on / off. Sent while its state is not the wanted one, but once per status
// frame from it (or every 200 ms without status), so it is not switched twice before it answers.
static void fjdTrigger(bool on)
{
    if (fjdOn == on) return;
    if (!fjdStatusSinceTrigger && millis() - fjdTriggerMs < 200) return;
    static const uint8_t d[8] = { 0x02, 0x00, 0x00, 0x19, 0x00, 16, 0x00, 0x00 };
    motorSend(FJD_CMD_ID, false, d);
    fjdStatusSinceTrigger = false;
    fjdTriggerMs = millis();
}

static const char* fjdStateName(uint8_t s)
{
    static char t[24];
    switch (s)
    {
        case 0x00: return "ready";
        case 0x05: case 0x06: case 0x07: case 0x09: snprintf(t, sizeof(t), "on (0x%02X)", s); return t;
        case 0x08: return "shutting down";
        case 0x13: return "error";
        default:   snprintf(t, sizeof(t), "0x%02X", s); return t;
    }
}

static void allynavErrText(char* t, size_t n)
{
    static const char* const e1[8] = { "MotorNotEnabled", "OverVoltage", "HWOverCurrent", "EEPROM", "UnderVoltage",
                                       "PosDeviation", "SWOverCurrent", "ControlModeErr" };
    static const char* const e2[8] = { "WorkModeFail", "SpeedDeviation", "OverTemp", "EncoderFail", "OtherError",
                                       nullptr, "CANCommLost", "Stall" };
    size_t k = 0;
    t[0] = '\0';
    for (uint8_t i = 0; i < 8; i++)
    {
        if (bitRead(allynavErr1, i) && k < n) k += snprintf(t + k, n - k, "%s ", e1[i]);
        if (bitRead(allynavErr2, i) && e2[i] && k < n) k += snprintf(t + k, n - k, "%s ", e2[i]);
    }
    if (!t[0]) snprintf(t, n, "OK");
}

// A message on the AgOpenGPS screen (PGN 221, hardware message), shown for "seconds"
static void aogHardwareMessage(const char* text, uint8_t seconds)
{
    if (!Ethernet_running) return;
    uint8_t p[80] = { 0x80, 0x81, 0x7E, 221 };
    size_t n = strlen(text);
    if (n > sizeof(p) - 8) n = sizeof(p) - 8;
    p[4] = n + 2;
    p[5] = seconds;
    p[6] = 0;                                                         // colour
    memcpy(p + 7, text, n);
    uint8_t ck = 0;
    for (size_t i = 2; i < 7 + n; i++) ck += p[i];
    p[7 + n] = ck;
    SendUdp(p, 8 + n, Eth_ipDestination, portDestination);
}

static void motorScanBaud()
{
    motorBaud = motorBauds[motorBaudIdx];
    canBus.setBaudRate(motorBaud);
    motorScanMs = millis();
    keyaCmd(0x0C, 0);                                                 // a disable: the Allynav answers after a command
}

// From canSetup (zCan.ino), after canBus.begin()
void motorSetup()
{
    if (steerMotor == MOTOR_FJD)
    {
        motorBaud = 1000000;
        canBus.setBaudRate(motorBaud);
        Serial.println("Steering: FJD motor on CAN3, 1000 kbit/s");
    }
    else
    {
        motorBaudIdx = 0;
        motorScanBaud();
        Serial.printf("Steering: %s motor on CAN3, bit rate: trying 250, 500, 125, 1000 kbit/s\r\n", motorName(steerMotor));
    }
}

// A heartbeat's current: into the kickout value (0 - 255 = 0 - MOTOR_AMPS_FULL) and the web page's peak
static void motorAmpsIn(float amps)
{
    motorAmps = amps;
    motorCurrent = (0.95 * motorCurrent) + (0.05 * amps * 255.0 / MOTOR_AMPS_FULL);
    motorCurrent = min(motorCurrent, 255);
    const uint32_t now = millis();
    if (amps >= motorAmpsPeak || now - motorAmpsPeakMs > 5000) { motorAmpsPeak = amps; motorAmpsPeakMs = now; }
}

static void motorFoundNow()
{
    motorDataMs = millis() | 1;
    if (motorFound) return;
    motorFound = true;
    webLog("[motor] %s found, %lu kbit/s", motorName(steerMotor), (unsigned long)(motorBaud / 1000));
    if (steerMotor == MOTOR_ALLYNAV) allynavInit();
}

// Every frame from the CAN bus with a motor (zCan.ino)
void motorFrame(const CAN_message_t& m)
{
    if (steerMotor == MOTOR_KEYA && m.id == KEYA_HEARTBEAT_ID)
    {
        motorFoundNow();
        const int16_t a = (int16_t)((m.buf[4] << 8) | m.buf[5]);      // A
        motorAmpsIn(abs(a));
        keyaCode = (uint16_t)((m.buf[6] << 8) | m.buf[7]);
    }
    else if (steerMotor == MOTOR_ALLYNAV && m.id == ALLYNAV_HEARTBEAT_ID)
    {
        motorFoundNow();
        const int16_t a = (int16_t)((m.buf[0] << 8) | m.buf[1]);       // 0.1 A
        motorAmpsIn(abs(a) * 0.1);
        if (m.buf[7] != allynavErr1 || m.buf[6] != allynavErr2)
        {
            allynavErr1 = m.buf[7];
            allynavErr2 = m.buf[6];
            char t[160];
            allynavErrText(t, sizeof(t));
            webLog("[motor] Allynav: %s (0x%02X 0x%02X)", t, allynavErr1, allynavErr2);
        }
    }
    else if (steerMotor == MOTOR_FJD && m.id == FJD_STATUS_ID)
    {
        motorFoundNow();
        fjdStatusSinceTrigger = true;
        const uint8_t s = m.buf[0];
        if (s == 0x13) fjdTrigger(false);                             // error: clear it
        fjdOn = s == 0x05 || s == 0x06 || s == 0x07 || s == 0x09;
        if (s != fjdState) webLog("[motor] FJD: %s", fjdStateName(s));
        fjdState = s;
    }
}

// From canLoop (zCan.ino), every loop
void motorLoop()
{
    const uint32_t now = millis();
    // Keya / Allynav: nothing heard at this bit rate - the next one
    if (steerMotor != MOTOR_FJD && !motorFound && now - motorScanMs > MOTOR_SCAN_STEP_MS)
    {
        motorBaudIdx = (motorBaudIdx + 1) % (sizeof(motorBauds) / sizeof(motorBauds[0]));
        motorScanBaud();
    }
    const bool lost = !motorDataMs || now - motorDataMs > MOTOR_LOST_MS;
    if (motorFound && lost != motorLost) webLog("[motor] %s: %s", motorName(steerMotor), lost ? "no data for 0.5 s" : "data again");
    motorLost = lost;
    if (lost && bitRead(guidanceStatus, 0) && now - motorMsgMs > 3000)
    {
        motorMsgMs = now;
        char t[64];
        snprintf(t, sizeof(t), "%s motor: no data on the CAN bus", motorName(steerMotor));
        aogHardwareMessage(t, 2);
    }
}

// From motorDrive (AutosteerPID.ino), every 25 ms: the PID output -255 .. 255, 0 = off
void motorSteer(int16_t pwm)
{
    if (steerMotor == MOTOR_FJD)
    {
        if (pwm == 0) { fjdTrigger(false); return; }
        fjdTrigger(true);
        const int16_t speed = map(pwm, -255, 255, -5000, 5000);
        const uint8_t d[8] = { 0x00, (uint8_t)(speed >> 8), (uint8_t)speed, 0x19, 0x00, 0x00, 0x00, 0x00 };
        motorSend(FJD_CMD_ID, false, d);
        return;
    }
    if (!motorFound) return;                                          // bit rate not found yet (the scan disables)
    if (pwm == 0) { keyaCmd(0x0C, 0); return; }                       // disable
    keyaCmd(0x00, steerMotor == MOTOR_ALLYNAV ? map(pwm, -255, 255, -900, 900) : map(pwm, -255, 255, -995, 998));
    keyaCmd(0x0D, 0);                                                 // enable
}

// The motor part of /api/status
void motorJson(Json& j, uint32_t now)
{
    j.obj("motor");
    j.u("type", steerMotor);
    j.str("name", motorName(steerMotor));
    if (steerMotor)
    {
        j.bo("found", motorFound);
        j.u("kbps", motorBaud / 1000);
        j.i("ago", motorDataMs ? (long)(now - motorDataMs) : -1);
        if (steerMotor != MOTOR_FJD)
        {
            j.num("amps", motorAmps, 1);                              // last heartbeat
            j.num("peak", motorAmpsPeak, 1);                          // highest in the last 5 s
            j.num("kick", motorCurrent, 0);                           // averaged, amps * 20 (0 - 255): the kickout value
        }
        char t[160];
        if (steerMotor == MOTOR_KEYA && motorFound) { snprintf(t, sizeof(t), keyaCode ? "error code 0x%04X" : "OK", keyaCode); j.str("state", t); }
        if (steerMotor == MOTOR_ALLYNAV && motorFound) { allynavErrText(t, sizeof(t)); j.str("state", t); }
        if (steerMotor == MOTOR_FJD && fjdState != 0xAA) j.str("state", fjdStateName(fjdState));
    }
    j.close();
}
