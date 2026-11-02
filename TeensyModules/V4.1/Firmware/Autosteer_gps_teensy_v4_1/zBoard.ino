/* zBoard.ino - board setup from the web page: the serial ports of the GNSS receivers, the TM171 and the RTK radio,
   which IMU feeds AgOpenGPS when two are found, the John Deere FEMA valve input, the remote input as a pulse sensor
   and the CAN bus inputs (SASA sensor, CNH engage button).
   Autosteer_gps_teensy_v4_1, 2026, after zBoard.ino of AOG_CAN_Teensy4.1. Chosen on the web page (Setup and IMU
   tabs), saved in the EEPROM (200 = magic, 201-209 = BoardCfg, zWeb.h), taken at start-up (boardCfgLoad, from setup()).
   Licence: GNU General Public License v3 or later (file COPYING), WITHOUT ANY WARRANTY.
   SPDX-License-Identifier: GPL-3.0-or-later
*/
// Every serial port is "Auto" until it is set on the web page, and Auto works as before the web page:
//   GNSS + dual:  GGA looked for on Serial2, then Serial7. Found on Serial7: dual receiver on Serial2. Else
//                 GNSS Serial2, dual Serial5. Swapped every 10 s while there is no GGA.
//   TM171:        looked for on Serial7, then Serial5 (not the GNSS port).
//   RTK radio:    Serial3.
// A port set on the web page is used as it is: no detection, no swapping.
// Offered: Serial1 (pins 0/1), Serial2 (7/8), Serial3 (14/15), Serial5 (20/21), Serial7 (28/29). Not offered, their
// pins are used by this firmware: Serial4 (16/17 = Wire1, the ADS1115), Serial6 (24 = A10, the pressure sensor),
// Serial8 (34 = the work switch).

#define BOARD_EE_ADDR  200
#define BOARD_EE_MAGIC 0x5C

BoardCfg boardCfg = { BOARD_PORT_AUTO, BOARD_PORT_AUTO, BOARD_PORT_AUTO, 3, 0, 0, 0, 0, 0 };

HardwareSerial* boardSerial(uint8_t n)
{
    switch (n)
    {
        case 1: return &Serial1;
        case 2: return &Serial2;
        case 3: return &Serial3;
        case 4: return &Serial4;
        case 5: return &Serial5;
        case 6: return &Serial6;
        case 7: return &Serial7;
        case 8: return &Serial8;
        default: return nullptr;
    }
}

// SerialN -> N, nullptr -> 0
uint8_t boardPortNum(HardwareSerial* p)
{
    for (uint8_t n = 1; n <= 8; n++) if (p && p == boardSerial(n)) return n;
    return 0;
}

// Teensy 4.1 TX pin of SerialN (for the messages: the TM171's RX must be wired to it)
uint8_t boardTxPin(uint8_t n)
{
    static const uint8_t tx[9] = { 0, 1, 8, 14, 17, 20, 24, 29, 35 };
    return n <= 8 ? tx[n] : 0;
}

static bool boardPortFree(uint8_t n) { return n == 1 || n == 2 || n == 3 || n == 5 || n == 7; }

// nullptr when the settings can be used together, else why not
const char* boardCfgCheck(const BoardCfg& c)
{
    const bool gnssAuto = c.gnss == BOARD_PORT_AUTO;
    if (!gnssAuto && !boardPortFree(c.gnss))                        return "GNSS port: Auto or Serial1, 2, 3, 5 or 7";
    if (gnssAuto && c.gnss2 != BOARD_PORT_AUTO)                     return "dual receiver port: Auto when the GNSS port is Auto";
    if (!gnssAuto && c.gnss2 && !boardPortFree(c.gnss2))            return "dual receiver port: none or Serial1, 2, 3, 5 or 7";
    if (c.imu != BOARD_PORT_AUTO && c.imu && !boardPortFree(c.imu)) return "IMU port: Auto, none or Serial1, 2, 3, 5 or 7";
    if (c.radio && !boardPortFree(c.radio))                         return "radio port: none or Serial1, 2, 3, 5 or 7";
    // the ports set by hand must all differ
    const uint8_t p[4] = { c.gnss, c.gnss2, c.imu, c.radio };
    for (uint8_t i = 0; i < 4; i++)
        for (uint8_t k = i + 1; k < 4; k++)
            if (p[i] == p[k] && p[i] != BOARD_PORT_AUTO && p[i] != BOARD_PORT_NONE) return "each device needs its own serial port";
    // Auto GNSS detection uses Serial2 and Serial7
    if (gnssAuto && (c.imu == 2 || c.imu == 7 || c.radio == 2 || c.radio == 7))
        return "with the GNSS port on Auto, Serial2 and Serial7 are kept for the receivers";
    return nullptr;
}

// From setup(), first: the settings saved from the web page, or the defaults
void boardCfgLoad()
{
    BoardCfg c;
    EEPROM.get(BOARD_EE_ADDR + 1, c);
    if (EEPROM.read(BOARD_EE_ADDR) == BOARD_EE_MAGIC && !boardCfgCheck(c)) boardCfg = c;
    if (boardCfg.imuPref != 1 && boardCfg.imuPref != 2) boardCfg.imuPref = 0;
    jdFema = boardCfg.fema == 1;
    remoteSensor = boardCfg.remote == 1;
    if (boardCfg.canBtn > 3) boardCfg.canBtn = 0;      // not saved yet (0xFF)
    canSasa = boardCfg.sasa == 1 && !remoteSensor;     // one source for the pressure sensor value
    canBtnMode = boardCfg.canBtn;
}

void boardCfgSave()
{
    EEPROM.put(BOARD_EE_ADDR + 1, boardCfg);
    EEPROM.update(BOARD_EE_ADDR, BOARD_EE_MAGIC);
}

static void boardPrintPort(const char* what, uint8_t n)
{
    Serial.print(what);
    if (n == BOARD_PORT_AUTO) Serial.print("Auto");
    else if (n) Serial.printf("Serial%u", n);
    else Serial.print("none");
}

// From setup(), before the GNSS ports are started: SerialGPS, SerialGPS2, SerialRTK
void boardGnssSetup()
{
    boardPrintPort("Serial ports (web page, Setup): GNSS ", boardCfg.gnss);
    boardPrintPort(", dual ", boardCfg.gnss2);
    boardPrintPort(", TM171 ", boardCfg.imu);
    boardPrintPort(", RTK radio ", boardCfg.radio);
    Serial.println();

    if (boardCfg.gnss == BOARD_PORT_AUTO)
    {
        Serial.println("Detecting GNSS port (Serial2 / Serial7)...");
        bool foundGnssOnSerial2 = detectGGAOnPort(&Serial2, 300);
        bool foundGnssOnSerial7 = false;

        if (!foundGnssOnSerial2)
        {
            foundGnssOnSerial7 = detectGGAOnPort(&Serial7, 300);
        }

        if (foundGnssOnSerial7)
        {
            Serial.println("GNSS detected on Serial7");
            SerialGPS = &Serial7;
            SerialGPS2 = &Serial2;
        }
        else
        {
            if (foundGnssOnSerial2)
            {
                Serial.println("GNSS detected on Serial2");
            }
            else
            {
                Serial.println("GNSS not detected on Serial2 or Serial7, using defaults");
            }
            SerialGPS = &Serial2;
            SerialGPS2 = &Serial5;
        }

        // Serial5 set by hand for the radio or the TM171: no dual receiver
        if (SerialGPS2 == boardSerial(boardCfg.radio) || SerialGPS2 == boardSerial(boardCfg.imu)) SerialGPS2 = nullptr;
        gpsAutoSwap = true;
    }
    else
    {
        SerialGPS = boardSerial(boardCfg.gnss);
        SerialGPS2 = boardSerial(boardCfg.gnss2);
        gpsAutoSwap = false;
    }
    SerialRTK = boardSerial(boardCfg.radio);
}

// The ports for the u-blox set-up through USB (!AOGR1 / !AOGR2): Serial7 / Serial2 on Auto (the AiO board, also when
// the receivers did not send GGA at start-up yet), else the ports set on the web page
void boardGnssConfigPorts()
{
    if (gpsAutoSwap)
    {
        SerialGPS = &Serial7;
        SerialGPS2 = &Serial2;
    }
    else
    {
        SerialGPS = boardSerial(boardCfg.gnss);
        SerialGPS2 = boardSerial(boardCfg.gnss2);
    }
}

// From setup(), after the BNO08x search: is a TM171 there? (tm171Found, SerialImu)
void boardImuSetup()
{
    bool found = false;
    if (boardCfg.imu == BOARD_PORT_NONE)
    {
        Serial.println("\r\nTM171: no IMU port (web page, Setup)");
    }
    else if (boardCfg.imu != BOARD_PORT_AUTO)
    {
        Serial.printf("\r\nChecking for TM171 on Serial%u (web page, Setup)\r\n", boardCfg.imu);
        found = TM171detectOnPort(boardSerial(boardCfg.imu), 1500);
    }
    else
    {
        Serial.println("\r\nChecking for TM171 on Serial7 / Serial5");
        const uint8_t candidates[2] = { 7, 5 };
        for (uint8_t i = 0; i < 2 && !found; i++)
        {
            HardwareSerial* port = boardSerial(candidates[i]);
            if (port == SerialGPS || port == SerialRTK || (!gpsAutoSwap && port == SerialGPS2)) continue;
            found = TM171detectOnPort(port, 1500);
            if (!found && port == SerialGPS2)
            {
                // not a TM171 there: back to the dual receiver's baud rate and buffers
                SerialGPS2->begin(baudGPS);
                SerialGPS2->addMemoryForRead(GPS2rxbuffer, serial_buffer_size);
                SerialGPS2->addMemoryForWrite(GPS2txbuffer, serial_buffer_size);
            }
        }
    }

    if (found)
    {
        tm171Found = true;
        Serial.printf("Received data from TM171 on Serial%u\r\n", boardPortNum(SerialImu));
        if (SerialGPS2 == SerialImu)
        {
            SerialGPS2 = nullptr;
            Serial.println("No dual receiver: its port has the TM171");
        }
    }
    else
    {
        SerialImu = nullptr;
        if (boardCfg.imu != BOARD_PORT_NONE) Serial.println("TM171 not Connected or Found");
    }
}

// ---------------------------------------------------------------- which IMU feeds AgOpenGPS
// Both a BNO08x (I2C) and a TM171 (serial) can be fitted. Both keep running when found (the TM171's settings and
// calibration stay usable); the one chosen on the web page (IMU tab) gives roll / pitch / heading to AgOpenGPS.
// Without a choice it is the BNO08x first. If the chosen one is not there, the other is used.
void imuSelect()
{
    const bool wantTm = boardCfg.imuPref == 2 || (boardCfg.imuPref != 1 && !bnoFound);
    useTM171  = tm171Found && (wantTm || !bnoFound);
    useBNO08x = bnoFound && !useTM171;
    imuTrigger = false;
    Serial.printf("IMU in use: %s%s\r\n", useCMPS ? "CMPS14" : useBNO08x ? "BNO08x" : useTM171 ? "TM171" : "none",
                  bnoFound && tm171Found ? " (both found, chosen on the web page IMU tab)" : "");
}

// From the web page (autosteer off): saved, and taken at once
void imuPrefSave(uint8_t pref)
{
    boardCfg.imuPref = pref;
    boardCfgSave();
    imuSelect();
    if (useTM171) imuHandler();
}

// From the web page (autosteer off): saved, and taken at once
void femaSave(bool on)
{
    boardCfg.fema = on ? 1 : 0;
    boardCfgSave();
    jdFema = on;
    pressureInputSetup();
}

// From the web page (autosteer off): saved, and taken at once
void remoteSave(bool on)
{
    boardCfg.remote = on ? 1 : 0;
    boardCfgSave();
    remoteSensor = on;
    remoteInputSetup();
}

// From the web page (autosteer off): saved, and taken at once (the bus is started if needed)
void canSave(bool sasa, uint8_t btn)
{
    boardCfg.sasa = sasa ? 1 : 0;
    boardCfg.canBtn = btn;
    boardCfgSave();
    canSasa = sasa;
    canBtnMode = btn;
    canReset();
    canSetup();
}
