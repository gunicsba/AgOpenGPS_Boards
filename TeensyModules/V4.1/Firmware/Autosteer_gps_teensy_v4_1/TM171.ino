//Based on the work of Paulius a.k.a. babtai RTK!

// 2026 web page additions (settings and calibration from the web page, zWeb.ino):
// commands TO the TM171 (tm171Send), the Setting object (21) read and written, and the calibrations of the
// Calibration object (23). Ported from AOG_CAN_Teensy4.1 / renatorgr/Boards Firmware/v4.5_AiO_WEB/TM171.ino
// (written 2026 by Claude (Anthropic) for renatorgr, GPL-3.0-or-later), extended after the SYD Dynamics
// "TransducerM TM3xx User Guide" V1.35: enable gyro / accelerometer, power-on silent time, gyro auto-finish
// and revoke, magnetometer and heading calibrations.
// Needs the Teensy TX pin of the IMU port (TX5 = pin 20 on Serial5, TX7 = pin 29 on Serial7) wired to the TM171
// RXD. Without it roll / pitch / yaw still work, but the TM171 never answers: the web page says so.

HardwareSerial* SerialImu = nullptr; // the port the TM171 was found on, set by TM171detectOnPort; nullptr = none

uint8_t SerialImurxbuffer[serial_buffer_size];    //Extra serial tx buffer
uint8_t SerialImutxbuffer[serial_buffer_size];    //Extra serial tx buffer

//State machine from the almighty ChatGPT
enum TM171ParseState {
  WAIT_HEADER_1,
  WAIT_HEADER_2,
  WAIT_LENGTH,
  WAIT_PAYLOAD
};

TM171ParseState parseState = WAIT_HEADER_1;
uint8_t packetLength = 0;
uint8_t payloadIndex = 0;
bool gotPacket = false;

uint8_t ImuData[64];
uint8_t ImuDC = 0 ;

union Onion
{
    uint8_t     fBytes[sizeof( float )];
    float       fValue;
};

Onion YawV;
Onion RollV;
Onion PitchV;

Onion TemperatureV;
uint8_t qos;

bool TM171DataSeen = false;

//#define TM171DEBUG

uint32_t TM171crcErrors = 0;          // packets dropped for a bad CRC or a bad length (web page)
uint32_t TM171rpyCount = 0;           // roll-pitch-yaw packets this second
uint32_t TM171rpyRate = 0;            // roll-pitch-yaw packets per second (web page)
uint32_t TM171statusMs = 0;           // last status packet (temperature, QoS; 0 = none)
uint16_t tm171Id = 0;                 // the TM171's node ID (FROM_ID of its frames; 0 = none received yet)
static uint8_t tmSeenType[16];        // frame types the firmware does not handle, logged once each
static uint8_t tmTxLast[3 + 4 + 64 + 2];   // the last frame sent, for the log
static uint8_t tmTxLen = 0;

#define TM171_SET_LEN 20              // Setting object content, bytes
static void tm171SettingGot(const uint8_t* p);
static void tm171CalReport(uint8_t type, uint8_t ctrl, uint16_t p1);
static void tm171LogFrame(const char* what, const uint8_t* frame, uint8_t n);
uint32_t tmSetAckMs = 0;              // the TM171 acknowledged our settings write

void TM171setup() {
  SerialImu->begin(115200);
  SerialImu->addMemoryForWrite(SerialImutxbuffer, serial_buffer_size);
  SerialImu->addMemoryForRead(SerialImurxbuffer, serial_buffer_size);
}

void TM171process() {
  if (!SerialImu) return;
  while (SerialImu->available())
  {
    uint8_t temp = SerialImu->read();
    switch (parseState)
    {
      case WAIT_HEADER_1:
        if (temp == 0xAA)
        {
          ImuData[0] = temp;
          parseState = WAIT_HEADER_2;
        }
        break;

      case WAIT_HEADER_2:
        if (temp == 0x55)
        {
          ImuData[1] = temp;
          parseState = WAIT_LENGTH;
        }
        else
        {
          parseState = WAIT_HEADER_1; //Reset if second byte isn't 0x55
        }
        break;

      case WAIT_LENGTH:
        packetLength = temp;
        ImuData[2] = temp;
        payloadIndex = 3; // We've stored 3 bytes so far
        parseState = WAIT_PAYLOAD;

        //check for unreasonable lengths
        if ((size_t)packetLength + 5 > sizeof(ImuData))
        {
          parseState = WAIT_HEADER_1;
          TM171crcErrors++;
#ifdef TM171DEBUG
          Serial.println("Too big data from TM171!");
#endif
        }
        break;

      case WAIT_PAYLOAD:
        ImuData[payloadIndex++] = temp;

        if (payloadIndex >= packetLength + 5)
        {
          gotPacket = true;
          parseState = WAIT_HEADER_1;
        }
        break;
    }
    if (gotPacket)
    {
      gotPacket = false;
//      if (GoodCRC(ImuData,packetLength+5))
      if (GoodCRC(ImuData,ImuData[2]+5))
      {
        TM171lastData = 0;
        TM171DataSeen = true;
        uint8_t functionCode = ImuData[3] & 0x7F; // Function ID (documented at 4th byte), CMD is 7 bits
        // the sender's ID (FROM_ID, bits 10-20 of the payload information word), to address commands to it
        tm171Id = (uint16_t)((((uint32_t)ImuData[4] << 8) | ((uint32_t)ImuData[5] << 16)) >> 10) & 0x7FF;
        switch (functionCode)
        {
        case 35: //RPY Output (X and Y swapped, so the axes are the same as the BNO's)
          TM171rpyCount++;
          PitchV.fBytes[0] = ImuData[11];
          PitchV.fBytes[1] = ImuData[12];
          PitchV.fBytes[2] = ImuData[13];
          PitchV.fBytes[3] = ImuData[14];

          RollV.fBytes[0] = ImuData[15];
          RollV.fBytes[1] = ImuData[16];
          RollV.fBytes[2] = ImuData[17];
          RollV.fBytes[3] = ImuData[18];

          YawV.fBytes[0] = ImuData[19];
          YawV.fBytes[1] = ImuData[20];
          YawV.fBytes[2] = ImuData[21];
          YawV.fBytes[3] = ImuData[22];

#ifdef TM171DEBUG
          Serial.print("yaw=");
          Serial.print(YawV.fValue);
          Serial.print(",pitch=");
          Serial.print(PitchV.fValue);
          Serial.print(",Roll=");
          Serial.println(RollV.fValue);
#endif
        break;

        case 22: // Status Output
          TemperatureV.fBytes[0] = ImuData[11];
          TemperatureV.fBytes[1] = ImuData[12];
          TemperatureV.fBytes[2] = ImuData[13];
          TemperatureV.fBytes[3] = ImuData[14];
          qos = ImuData[17] & 0x07;
          TM171statusMs = millis() | 1;
#ifdef TM171DEBUG
          Serial.print("Temp status:"); Serial.print(TemperatureV.fValue);
          Serial.print(",QoS:"); Serial.println(qos);
#endif
        break;

        case 23: // Calibration status report: type, control value, parameter 1 (progress)
          // the whole frame to the log (the codes are taken from the user guide; the bytes show what really came),
          // except the magnetometer progress reports, which can come often
          if (!(packetLength >= 4 + 2 && ImuData[7] == 0x13 && ImuData[8] == 0x00)) tm171LogFrame("calibration report received", ImuData, ImuData[2] + 5);
          if (packetLength >= 4 + 4) tm171CalReport(ImuData[7], ImuData[8], (uint16_t)(ImuData[9] | (ImuData[10] << 8)));
        break;

        case 21: // Setting object, the answer to our request
          if (packetLength == 4 + TM171_SET_LEN) tm171SettingGot(ImuData + 7);
        break;

        case 13: // Acknowledge: byte 0 = the object acknowledged
          if (ImuData[7] == 21) tmSetAckMs = millis() | 1;
          tm171LogFrame("acknowledge", ImuData, ImuData[2] + 5);
        break;

        default:
          // the first frame of each type the firmware does not handle goes to the log
          if (!(tmSeenType[functionCode >> 3] & (1 << (functionCode & 7))))
          {
              tmSeenType[functionCode >> 3] |= (1 << (functionCode & 7));
              tm171LogFrame("frame of a type not handled", ImuData, ImuData[2] + 5);
          }
#ifdef TM171DEBUG
          Serial.print("Unhandled packet type: ");
          Serial.println(functionCode, HEX);
#endif
        break;
        }
      }
       else {
        // An occasional bad CRC is normal (a corrupted byte on the wire): the packet is dropped and the next
        // one replaces it. Counted (web page) instead of printed.
        TM171crcErrors++;
#ifdef TM171DEBUG
        Serial.println("CRC was bad :(");
        Serial.print("Length: "); Serial.println(ImuData[2]);
        Serial.print("Full len: "); Serial.println(packetLength + 5);
#endif
      }
    } //gotPacket
  } //while
}

bool TM171detectOnPort(HardwareSerial* port, uint32_t detectionMs)
{
    SerialImu = port;
    TM171setup();

    TM171DataSeen = false;
    TM171lastData = 0;
    parseState = WAIT_HEADER_1;
    packetLength = 0;
    payloadIndex = 0;
    gotPacket = false;

    while (SerialImu->available())
    {
        SerialImu->read();
    }

    delay(200);

    uint32_t start = millis();
    while (millis() - start < detectionMs)
    {
        TM171process();
        if (TM171DataSeen)
        {
            return true;
        }
        delay(1);
    }
    return false;
}

// ---------------------------------------------------------------- commands to the TM171
// EasyProtocol frame (user guide, "Communication Protocol"): AA 55, length (4 + content), payload information
// (32 bits little-endian: CMD 7 bits, 3 reserved, FROM_ID 11 bits, TO_ID 11 bits), content, Modbus CRC16 of
// the length and payload (little-endian). The host is ID 2. TO_ID is the TM171's own ID, read from the frames
// it sends: a real TM171 answered requests sent to everyone (TO_ID 0) but ignored settings written that way
// (renatorgr, 24 Oct 2026). Until a frame has been received the commands go to everyone.
static uint16_t MODBUS_CRC16_v3(const unsigned char* buf, unsigned int len);   // below
static void tm171Send(uint8_t cmd, const uint8_t* content, uint8_t n)
{
    if (n > 64 || !SerialImu) return;
    uint8_t* f = tmTxLast;
    f[0] = 0xAA; f[1] = 0x55; f[2] = (uint8_t)(4 + n);
    const uint32_t info = (uint32_t)(cmd & 0x7F) | (2UL << 10) | ((uint32_t)tm171Id << 21);   // from the host (2) to the TM171
    f[3] = (uint8_t)info; f[4] = (uint8_t)(info >> 8); f[5] = (uint8_t)(info >> 16); f[6] = (uint8_t)(info >> 24);
    memcpy(f + 7, content, n);
    const uint16_t crc = MODBUS_CRC16_v3(f, 3 + 4 + n);              // (it skips the 2 head bytes itself)
    f[7 + n] = (uint8_t)crc; f[8 + n] = (uint8_t)(crc >> 8);
    tmTxLen = 9 + n;
    SerialImu->write(f, tmTxLen);
}

// bytes as "AA 55 18 ..." (for the log and the page); out holds 3 * n bytes
static void tm171Hex(const uint8_t* p, uint8_t n, char* out)
{
    for (uint8_t i = 0; i < n; i++) snprintf(out + 3 * i, 4, "%02X ", p[i]);
    out[n ? 3 * n - 1 : 0] = '\0';
}

// a whole frame to the log; frames longer than 40 bytes are cut
static void tm171LogFrame(const char* what, const uint8_t* frame, uint8_t n)
{
    char hex[3 * 40 + 1];
    if (n > 40) n = 40;
    tm171Hex(frame, n, hex);
    webLog("[tm171] %s: %s", what, hex);
}

// ---------------------------------------------------------------- settings (Setting object 21)
// 20 bytes, user guide pages 39-42:
//   0-3   function enable / disable switches (bit fields below)
//   4-5   reserved, "should always be 1152"
//   6-7   UART 1 baud rate, x 100 bps (1152 = 115200)
//   8-9   CAN baud rate, x 100 bps
//   10-11 accelerometer gain, x 0.01 (the knob by GyroErrFilter in ImuAssistant; >= 2.00 with GyroErrFilter on)
//   12-13 magnetometer gain, x 0.01 (the knob by Self-Adapt)
//   14-15 inhibit time, ms (shortest gap between two packets)
//   16-19 power-on silent time, s (0..65), a 32-bit word in the guide.
// A real TM171 read 3B A8 18 00 .. .. 80 04 .. .. 5E 01 64 00 0A 00 00 00 19 00 (renatorgr, 24 Oct 2026):
// bytes 18-19 = 25 is its output data rate in Hz (the TM171 is one of the models with an ODR setting), and
// 16-17 = 0 the silent time, which never needs the upper half (65 s at most). So the firmware reads and
// writes the silent time as the 16-bit word at 16, and the output data rate as the word at 18. On a model
// without ODR the word at 18 reads 0 and is written back unchanged.
// Switches, bit 0 first: 0 gyroscope, 1 accelerometer, 2 magnetometer, 3 GyroErrFilter, 4 magnetometer
// Self-Adapt filter, 5 status output, 6 raw data output, 7 reserved 0, 8 quaternion, 9 reserved 0, 10 Euler,
// 11 roll-pitch-yaw, 12 gravity, 13 reserved 1, 14-16 reserved 2, 17 request acknowledge, 18 save to flash,
// 19 UART 1 output, 20 reserved, 21 CAN output, 22-23 reserved, 24-25 boot mode (0 auto, 1 static, 2 dynamic),
// 26-31 unused.
// Before anything is written the firmware checks what it read: bit 13 = 1, bits 14-16 = 2 and the baud word
// = 1152 (the Teensy talks to it at 115200). If that does not hold the page shows the raw bytes and nothing is
// written. The roll-pitch-yaw and UART outputs always stay on (the firmware needs them), the baud rate and the
// words the page does not show (4, 8, 14) stay as read.
#define TM171_SET_BIT(w, n)  (((w) >> (n)) & 1UL)
uint8_t  tmSet[TM171_SET_LEN];   // as read from the TM171
bool     tmSetValid  = false;    // read, and the layout check passed
bool     tmSetOdd    = false;    // read, but the layout check failed
uint32_t tmSetMs     = 0;        // when read (0 = never)
uint32_t tmSetReqMs  = 0;        // request sent, no answer yet (0 = none pending)
uint32_t tmSetWriteMs = 0;       // our write, to read the settings back after it
uint32_t tmSetNoAnswer = 0;      // requests without an answer (the TX5 wire?)

static uint32_t tmSetU32(const uint8_t* p) { return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24); }
static uint16_t tmSetU16(const uint8_t* p) { return (uint16_t)(p[0] | (p[1] << 8)); }
static void     tmSetPut16(uint8_t* p, uint16_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }
static void     tmSetPut32(uint8_t* p, uint32_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24); }

// The AgOpenGPS set-up (ImuAssistant settings from the AgOpenGPS forum): magnetometer off, GyroErrFilter on
// with accelerometer gain 3.50, Self-Adapt on with magnetometer gain 1.00, roll-pitch-yaw and status output,
// UART 115200, 25 Hz, Auto boot mode. The page's "AgOpenGPS defaults" button fills the form with these.
#define TM171_AOG_ACC 350
#define TM171_AOG_MAG 100
static bool tm171OdrOk(uint16_t hz) { return hz == 25 || hz == 50 || hz == 100; }

// "" when the TM171 is set up as above, else what differs (web page)
const char* tm171SetupWhy()
{
    static char why[120];
    why[0] = '\0';
    if (!tmSetValid) return why;
    auto add = [&](const char* s) { if (why[0]) strlcat(why, ", ", sizeof(why)); strlcat(why, s, sizeof(why)); };
    const uint32_t w = tmSetU32(tmSet);
    if (!TM171_SET_BIT(w, 0))  add("gyroscope off");
    if (!TM171_SET_BIT(w, 1))  add("accelerometer off");
    if (TM171_SET_BIT(w, 2))   add("magnetometer on");
    if (!TM171_SET_BIT(w, 3))  add("GyroErrFilter off");
    if (!TM171_SET_BIT(w, 4))  add("Self-Adapt off");
    if (((w >> 24) & 3) != 0)  add("boot mode not Auto");
    if (tmSetU16(tmSet + 10) != TM171_AOG_ACC) add("accelerometer gain");
    if (tmSetU16(tmSet + 12) != TM171_AOG_MAG) add("magnetometer gain");
    if (tmSetU16(tmSet + 18) && !tm171OdrOk(tmSetU16(tmSet + 18))) add("output rate");
    return why;
}

// No reply ever to a settings request: the TM171's RX pin is not wired to Teensy TX5 (the web page explains)
bool tm171NoWire() { return tmSetNoAnswer > 1 && tmSetMs == 0; }

// A timer set with millis() | 1 can be one millisecond ahead of the caller's "now": signed, so it is not
// taken for expired at once
static bool tmElapsed(uint32_t now, uint32_t since, uint32_t ms) { return (int32_t)(now - since) > (int32_t)ms; }

// Ask the TM171 for its Setting object (Request object 12)
void tm171SettingRead()
{
    const uint8_t c[4] = { 21, 0, 0, 0 };
    tm171Send(12, c, sizeof(c));
    tmSetReqMs = millis() | 1;
}

static void tm171SettingGot(const uint8_t* p)
{
    memcpy(tmSet, p, TM171_SET_LEN);
    const uint32_t w = tmSetU32(tmSet);
    const bool layoutOk = TM171_SET_BIT(w, 13) == 1 && ((w >> 14) & 7UL) == 2 && tmSetU16(tmSet + 6) == 1152;
    tmSetMs = millis() | 1;
    tmSetReqMs = 0;
    tmSetValid = layoutOk;
    tmSetOdd   = !layoutOk;
    {   // every read goes to the log (at start-up, from the page, after a write): the raw bytes show what changed
        char raw[3 * TM171_SET_LEN + 1];
        tm171Hex(tmSet, TM171_SET_LEN, raw);
        webLog("[tm171] settings read from ID %u: switches 0x%08lX, silent %u s, %u Hz (bytes %s)%s", (unsigned)tm171Id, (unsigned long)w,
               (unsigned)tmSetU16(tmSet + 16), (unsigned)tmSetU16(tmSet + 18), raw,
               layoutOk ? "" : " - LAYOUT NOT RECOGNISED, writing from the page is disabled");
    }
}

// The web page's form. Returns false with the reason.
bool tm171SettingWrite(const TmSetForm& f, const char** why)
{
    if (!tm171Found)          { *why = "no TM171 found at start-up"; return false; }
    if (!tmSetValid)          { *why = tmSetOdd ? "settings layout not recognised; writing is disabled" : "read the settings from the TM171 first"; return false; }
    if (f.accGain < 1 || f.accGain > 2000 || f.magGain < 1 || f.magGain > 2000) { *why = "gain out of range (0.01 - 20.00)"; return false; }
    if (f.silent > 65)        { *why = "power-on silent time: 0 - 65 s"; return false; }
    if (f.odr && !tm171OdrOk(f.odr)) { *why = "output data rate: 25, 50 or 100 Hz"; return false; }
    if (f.boot > 2)           { *why = "boot mode out of range"; return false; }
    uint8_t s[TM171_SET_LEN];
    memcpy(s, tmSet, TM171_SET_LEN);
    uint32_t w = tmSetU32(s);
    auto setBit = [&](uint8_t n, bool v) { if (v) w |= (1UL << n); else w &= ~(1UL << n); };
    setBit(0, f.gyro);
    setBit(1, f.acc);
    setBit(2, f.mag);
    setBit(3, f.gyroErr);
    setBit(4, f.selfAdapt);
    setBit(5, f.status);
    setBit(11, true);                            // roll-pitch-yaw: the firmware reads it
    setBit(17, true);                            // acknowledge our write
    setBit(18, f.save);
    setBit(19, true);                            // UART output: never off (the Teensy would lose the TM171)
    w = (w & ~(3UL << 24)) | ((uint32_t)f.boot << 24);
    tmSetPut32(s, w);
    tmSetPut16(s + 10, f.accGain);
    tmSetPut16(s + 12, f.magGain);
    tmSetPut16(s + 16, f.silent);
    if (f.odr && tmSetU16(tmSet + 18)) tmSetPut16(s + 18, f.odr);   // only on a model that has the ODR word
    tm171Send(21, s, TM171_SET_LEN);
    tmSetAckMs = 0;
    tmSetWriteMs = millis() | 1;
    webLog("[tm171] settings written to ID %u: switches 0x%08lX, gains %u/%u, silent %u s, %u Hz%s", (unsigned)tm171Id, (unsigned long)w,
           (unsigned)f.accGain, (unsigned)f.magGain, (unsigned)f.silent, (unsigned)tmSetU16(s + 18),
           f.save ? ", saved to flash" : ", this power-on only");
    tm171LogFrame("frame sent", tmTxLast, tmTxLen);
    return true;
}

// For the web page: the settings as read, decoded
void tm171SettingJson(Json& j)
{
    const uint32_t now = millis();
    j.obj("set");
    j.bo("valid", tmSetValid);
    j.bo("odd", tmSetOdd);
    j.i("age", tmSetMs ? (long)((now - tmSetMs) / 1000) : -1);
    j.bo("pending", tmSetReqMs != 0 || tmSetWriteMs != 0);
    j.i("ackAge", tmSetAckMs ? (long)((now - tmSetAckMs) / 1000) : -1);
    j.u("noAnswer", tmSetNoAnswer);
    j.bo("noWire", tm171NoWire());
    j.str("setupWhy", tm171SetupWhy());
    if (tmSetMs)
    {
        const uint32_t w = tmSetU32(tmSet);
        j.u("sw", w);
        j.bo("gyro", TM171_SET_BIT(w, 0)); j.bo("acc", TM171_SET_BIT(w, 1)); j.bo("mag", TM171_SET_BIT(w, 2));
        j.bo("gyroErr", TM171_SET_BIT(w, 3)); j.bo("selfAdapt", TM171_SET_BIT(w, 4));
        j.bo("status", TM171_SET_BIT(w, 5)); j.bo("raw", TM171_SET_BIT(w, 6)); j.bo("quat", TM171_SET_BIT(w, 8));
        j.bo("euler", TM171_SET_BIT(w, 10)); j.bo("rpy", TM171_SET_BIT(w, 11)); j.bo("gravity", TM171_SET_BIT(w, 12));
        j.bo("uart", TM171_SET_BIT(w, 19)); j.bo("can", TM171_SET_BIT(w, 21));
        j.u("boot", (w >> 24) & 3);
        j.u("baud", (unsigned long)tmSetU16(tmSet + 6) * 100UL);
        j.u("accGain", tmSetU16(tmSet + 10)); j.u("magGain", tmSetU16(tmSet + 12));
        j.u("inhibit", tmSetU16(tmSet + 14)); j.u("silent", tmSetU16(tmSet + 16)); j.u("odr", tmSetU16(tmSet + 18));
        char raw[3 * TM171_SET_LEN + 1];
        tm171Hex(tmSet, TM171_SET_LEN, raw);
        j.str("hex", raw);
    }
    j.close();
}

// ---------------------------------------------------------------- calibration (Calibration object 23)
// User guide pages 36-38. Content: type u8, control u8, parameter 1 u16 (the key), parameter 2 u32,
// parameter 3 u32. The TM171 reports with the same object: type 0x14 new install, 0x15 forced gyro,
// 0x13 magnetometer / heading.
enum { TMG_INSTALL, TMG_GYRO, TMG_MAG };
enum { TMC_IDLE, TMC_STARTING, TMC_RUNNING, TMC_FINISHING, TMC_OK, TMC_FAILED, TMC_NO_ANSWER, TMC_ABORTED, TMC_REVOKED, TMC_SENT };

struct TmCalDef
{
    const char* code;      // the web page's name
    uint8_t     type, ctrl;
    uint16_t    key;       // parameter 1
    uint8_t     group;
    uint32_t    waitMs;    // no report in this time = no answer (0: a one-shot command, no report expected)
    bool        still;     // the tractor must stand still and autosteer be off
    const char* what;
};

#define TM171_CALIB_S       20     // seconds of data for the timed gyro calibration (12 s or more is a full one, SYD AN002)
#define TM171_ANSWER_MS     3000

static const TmCalDef tmCalDefs[] = {
    // New install (boot) calibration, the CalibB button: ~20 s still, saved to flash, then a power cycle
    { "install",     0x04, 0x02, 0xAA35, TMG_INSTALL, 60000, true,  "boot calibration (new install, CalibB, saved to flash)" },
    // Run-time forced gyro calibration, continuous: start, the firmware finishes it after TM171_CALIB_S (TM firmware V5.1+)
    { "gyro",        0x05, 0x03, 0xAA65, TMG_GYRO,    TM171_ANSWER_MS, true, "gyro bias calibration (timed)" },
    // Forced static gyro calibration that finishes by itself
    { "gyroAuto",    0x05, 0x02, 0xAA65, TMG_GYRO,    60000, true,  "gyro bias calibration (finishes by itself)" },
    { "gyroStop",    0x05, 0x04, 0xAA65, TMG_GYRO,    TM171_ANSWER_MS, false, "finish the gyro calibration now" },
    { "gyroRevoke",  0x05, 0x01, 0xAA65, TMG_GYRO,    TM171_ANSWER_MS, false, "revoke the forced gyro calibration" },
    // Magnetometer hard-iron calibration: rotate the module (drive circles); progress reports 0..100 %
    { "mag",         0x03, 0x01, 0xAA32, TMG_MAG,     300000, false, "magnetometer hard-iron calibration (saved to flash)" },
    { "magTmp",      0x03, 0x02, 0xAA32, TMG_MAG,     300000, false, "magnetometer hard-iron calibration (this power-on only)" },
    { "magClear",    0x03, 0x08, 0xAA32, TMG_MAG,     TM171_ANSWER_MS, false, "clear the magnetometer calibration (flash)" },
    { "magClearTmp", 0x03, 0x09, 0xAA32, TMG_MAG,     TM171_ANSWER_MS, false, "clear the magnetometer calibration (this power-on only)" },
    // Heading to local magnetic north: collect data (p2 = time ms, p3 = confidence), then stop and apply
    { "headStart",   0x03, 0x0A, 0xAA32, TMG_MAG,     95000, false, "heading to magnetic north: collecting data" },
    { "headStop",    0x03, 0x0B, 0xAA32, TMG_MAG,     TM171_ANSWER_MS, false, "heading to magnetic north: stop and apply" },
    // Manual heading fix: p2 = heading in 0.001 deg, p3 = confidence (this power-on only)
    { "headSet",     0x03, 0x0C, 0xAA32, TMG_MAG,     TM171_ANSWER_MS, false, "set the heading" },
};
#define TM_CAL_N (sizeof(tmCalDefs) / sizeof(tmCalDefs[0]))
#define TM_CAL_GYRO_STOP 3                     // tmCalDefs[3], "gyroStop": ends the timed gyro calibration

static int8_t   tmCalCur   = -1;       // index of the last command (-1 none)
static uint8_t  tmCalState = TMC_IDLE;
static uint32_t tmCalMs    = 0;        // when the state was entered
static uint32_t tmCalDoneMs = 0;       // last result (0 = none yet)
static int16_t  tmCalProgress = -1;    // magnetometer progress, %
static uint32_t tmCalWaitMs = 0;       // the wait of the running command
static uint8_t  tmCalLastType = 0, tmCalLastCtrl = 0;
static uint32_t tmCalLastMs = 0;       // last report (0 = none)

static bool tmCalBusy() { return tmCalState == TMC_STARTING || tmCalState == TMC_RUNNING || tmCalState == TMC_FINISHING; }

static const char* tmCalStateName(uint8_t s)
{
    switch (s)
    {
        case TMC_STARTING:  return "starting";
        case TMC_RUNNING:   return "running";
        case TMC_FINISHING: return "finishing";
        case TMC_OK:        return "ok";
        case TMC_FAILED:    return "failed";
        case TMC_NO_ANSWER: return "noanswer";
        case TMC_ABORTED:   return "aborted";
        case TMC_REVOKED:   return "revoked";
        case TMC_SENT:      return "sent";
        default:            return "";
    }
}

// k = index in tmCalDefs (an index, not the struct: the Arduino IDE's prototypes come before the struct)
static void tm171CalCmd(uint8_t k, uint32_t p2, uint32_t p3)
{
    const TmCalDef& d = tmCalDefs[k];
    const uint8_t c[12] = { d.type, d.ctrl, (uint8_t)d.key, (uint8_t)(d.key >> 8),
                            (uint8_t)p2, (uint8_t)(p2 >> 8), (uint8_t)(p2 >> 16), (uint8_t)(p2 >> 24),
                            (uint8_t)p3, (uint8_t)(p3 >> 8), (uint8_t)(p3 >> 16), (uint8_t)(p3 >> 24) };
    tm171Send(23, c, sizeof(c));
    tm171LogFrame("calibration frame sent", tmTxLast, tmTxLen);   // compare with ImuAssistant's TX
}

static void tmCalEnd(uint8_t state)
{
    tmCalState = state;
    tmCalMs = millis();
    tmCalDoneMs = tmCalMs | 1;
}

// Web page buttons. p2 / p3 from the page (the firmware checks them). False with the reason.
// Nothing new starts while autosteer is engaged (the stop / clear commands always go); the gyro and new
// install calibrations also need the tractor stopped.
bool tm171CalStart(const char* code, uint32_t p2, uint32_t p3, bool engaged, bool moving, const char** why)
{
    int8_t k = -1;
    for (uint8_t i = 0; i < TM_CAL_N; i++) if (!strcmp(tmCalDefs[i].code, code)) k = i;
    if (k < 0)       { *why = "unknown calibration"; return false; }
    const TmCalDef& d = tmCalDefs[k];
    if (!tm171Found) { *why = "no TM171 found at start-up"; return false; }
    const bool stopper = !strcmp(code, "gyroStop") || !strcmp(code, "magClear") || !strcmp(code, "magClearTmp") || !strcmp(code, "headStop");
    if (engaged && !stopper)  { *why = "autosteer is engaged - disengage it first"; return false; }
    if (d.still && moving)    { *why = "the tractor is moving - stop first"; return false; }
    if (tmCalBusy() && !stopper) { *why = "a calibration is already running"; return false; }
    // the parameters, user guide page 36-38
    if (!strcmp(code, "gyro") || !strcmp(code, "gyroAuto"))
    {
        if (p2 < 5 || p2 > 500)  { *why = "gyro error limit: 5 - 500 (x 0.01 deg/s)"; return false; }
        if (p3 > 10000)          { *why = "confidence: 0 - 10000 (x 0.01 %)"; return false; }
        if (!strcmp(code, "gyroAuto")) p3 = 0;
    }
    else if (!strcmp(code, "headStart"))
    {
        if (p2 > 90000)          { *why = "collection time: 0 - 90 s"; return false; }
        if (p3 > 10000)          { *why = "confidence: 0 - 10000 (x 0.01 %)"; return false; }
    }
    else if (!strcmp(code, "headSet"))
    {
        if (p2 > 360000)         { *why = "heading: 0 - 360 deg"; return false; }
        if (p3 > 10000)          { *why = "confidence: 0 - 10000 (x 0.01 %)"; return false; }
    }
    else p2 = p3 = 0;
    // a stop command ends the running calibration of its group only
    if (stopper && tmCalBusy() && tmCalCur >= 0 && tmCalDefs[tmCalCur].group != d.group) { *why = "another calibration is running"; return false; }

    tm171CalCmd(k, p2, p3);
    tmCalCur = k;
    tmCalMs = millis();
    tmCalProgress = -1;
    tmCalWaitMs = d.waitMs;
    if (!strcmp(code, "headStart")) tmCalWaitMs = p2 ? p2 + 5000 : 0;   // 0 s: runs until "stop and apply"
    if (!strcmp(code, "gyro"))      tmCalState = TMC_STARTING;    // waits for "started" (0x15 0x00)
    else if (!strcmp(code, "gyroStop")) tmCalState = TMC_FINISHING;
    else                            tmCalState = TMC_RUNNING;
    webLog("[tm171] %s: sent (type 0x%02X control 0x%02X, %lu, %lu)", d.what, d.type, d.ctrl, (unsigned long)p2, (unsigned long)p3);
    return true;
}

// The TM171's report
static void tm171CalReport(uint8_t type, uint8_t ctrl, uint16_t p1)
{
    tmCalLastType = type; tmCalLastCtrl = ctrl; tmCalLastMs = millis() | 1;
    const int8_t g = type == 0x14 ? TMG_INSTALL : type == 0x15 ? TMG_GYRO : type == 0x13 ? TMG_MAG : -1;
    const char* gName = g == TMG_INSTALL ? "boot calibration (new install)" : g == TMG_GYRO ? "gyro calibration" : g == TMG_MAG ? "magnetometer / heading" : "calibration";
    const bool mine = tmCalCur >= 0 && tmCalDefs[tmCalCur].group == g;
    if (g == TMG_MAG && ctrl == 0x00)                  // progress, 0..100 %
    {
        if (mine) { tmCalProgress = p1; if (tmCalBusy()) tmCalMs = millis(); }
        static int16_t lastLogged = -100;
        if (p1 >= lastLogged + 20 || p1 < lastLogged || p1 >= 100) { lastLogged = p1; webLog("[tm171] %s: %u %%", gName, (unsigned)p1); }
        return;
    }
    if (g == TMG_GYRO && ctrl == 0x00)                 // continuous forced calibration started
    {
        if (mine && tmCalState == TMC_STARTING) { tmCalState = TMC_RUNNING; tmCalMs = millis(); }
        webLog("[tm171] gyro calibration started, collecting data");
        return;
    }
    const char* res;
    uint8_t st;
    if (ctrl == 0x01)                     { st = TMC_OK;      res = g == TMG_INSTALL ? "done, saved to flash - switch the module off and on" : "done"; }
    else if (g == TMG_GYRO && ctrl == 0x05) { st = TMC_REVOKED; res = "results revoked"; }
    else                                  { st = TMC_FAILED;  res = g == TMG_INSTALL ? "FAILED (did the tractor move?) - switch the module off and on" : "FAILED"; }
    webLog("[tm171] %s: %s (report type 0x%02X control 0x%02X)", gName, res, type, ctrl);
    if (mine && (tmCalBusy() || tmCalState == TMC_SENT)) tmCalEnd(st);
}

// From loop() with a TM171: read the settings once at the start, read them back after a write, give up on a
// request without an answer, count the output rate, run the timed gyro calibration
void tm171Loop(bool tractorStill)
{
    static uint32_t firstMs = 0, rateMs = 0;
    static bool asked = false;
    const uint32_t now = millis();
    if (!firstMs) { firstMs = now | 1; rateMs = now; }
    if (!asked && tmElapsed(now, firstMs, 2000)) { asked = true; tm171SettingRead(); }
    if (tmSetWriteMs && tmElapsed(now, tmSetWriteMs, 500)) { tmSetWriteMs = 0; tm171SettingRead(); }
    if (tmSetReqMs && tmElapsed(now, tmSetReqMs, TM171_ANSWER_MS))
    {
        tmSetReqMs = 0;
        if (tmSetNoAnswer++ == 0 && !tmSetMs) tm171SettingRead();   // once more before "not wired" is concluded
        else if (tmSetNoAnswer == 2 && !tmSetMs)
            webLog("[tm171] no reply - its RX pin is not wired to Teensy TX%u (pin %u). Roll, pitch and yaw work; settings and calibration from the page do not",
                   boardPortNum(SerialImu), boardTxPin(boardPortNum(SerialImu)));
    }
    if (now - rateMs >= 1000) { rateMs = now; TM171rpyRate = TM171rpyCount; TM171rpyCount = 0; }

    // calibration
    if (!tmCalBusy() || tmCalCur < 0) return;
    const TmCalDef& d = tmCalDefs[tmCalCur];
    const bool gyroCollecting = !strcmp(d.code, "gyro") && (tmCalState == TMC_STARTING || tmCalState == TMC_RUNNING);
    if (gyroCollecting && !tractorStill)
    {
        // autosteer on or the tractor moving: finish now, the TM171 decides what it can use
        tm171CalCmd(TM_CAL_GYRO_STOP, 0, 0);
        webLog("[tm171] gyro calibration finished early: autosteer on or the tractor moving");
        tmCalState = TMC_FINISHING; tmCalMs = now; tmCalWaitMs = TM171_ANSWER_MS;
        return;
    }
    if (!strcmp(d.code, "gyro") && tmCalState == TMC_RUNNING)
    {
        if (now - tmCalMs >= (uint32_t)TM171_CALIB_S * 1000UL)
        {
            tm171CalCmd(TM_CAL_GYRO_STOP, 0, 0);
            tmCalState = TMC_FINISHING; tmCalMs = now; tmCalWaitMs = TM171_ANSWER_MS;
        }
        return;
    }
    if (tmCalWaitMs && now - tmCalMs > tmCalWaitMs)
    {
        // a one-shot command (stop, clear, set) may get no report at all: "sent"; a calibration: "no answer"
        const bool oneShot = d.waitMs == TM171_ANSWER_MS && strcmp(d.code, "gyro") != 0;
        if (oneShot)                           { tmCalEnd(TMC_SENT); webLog("[tm171] %s: no report from the TM171", d.what); }
        else if (tmCalState == TMC_FINISHING)  { tmCalEnd(TMC_NO_ANSWER); webLog("[tm171] %s: no result after finishing", d.what); }
        else                                   { tmCalEnd(TMC_NO_ANSWER); webLog("[tm171] %s: no answer - is Teensy TX%u (pin %u) wired to the TM171 RX?", d.what, boardPortNum(SerialImu), boardTxPin(boardPortNum(SerialImu))); }
    }
}

// Seconds of data collection left for the timed gyro calibration, -1 when not running
static int tm171CalLeft()
{
    if (tmCalState != TMC_RUNNING || tmCalCur < 0 || strcmp(tmCalDefs[tmCalCur].code, "gyro")) return -1;
    const uint32_t el = (millis() - tmCalMs) / 1000UL;
    return el >= TM171_CALIB_S ? 0 : (int)(TM171_CALIB_S - el);
}

void tm171CalJson(Json& j)
{
    const uint32_t now = millis();
    j.obj("cal");
    j.str("code", tmCalCur >= 0 ? tmCalDefs[tmCalCur].code : "");
    j.str("what", tmCalCur >= 0 ? tmCalDefs[tmCalCur].what : "");
    j.str("state", tmCalStateName(tmCalState));
    j.bo("busy", tmCalBusy());
    j.i("left", tm171CalLeft());
    j.i("progress", tmCalProgress);
    j.i("ago", tmCalDoneMs ? (long)((now - tmCalDoneMs) / 1000) : -1);
    j.i("secs", TM171_CALIB_S);
    if (tmCalLastMs)
    {
        char t[24];
        snprintf(t, sizeof(t), "0x%02X 0x%02X", tmCalLastType, tmCalLastCtrl);
        j.str("last", t);
        j.i("lastAgo", (long)((now - tmCalLastMs) / 1000));
    }
    j.close();
}

bool GoodCRC(byte Data[], byte Length)
{
	uint16_t ck = MODBUS_CRC16_v3(Data,Length-2);
	bool Result = (ck == Data[Length - 2] + (Data[Length - 1]<<8));
	return Result;
}

static uint16_t MODBUS_CRC16_v3( const unsigned char *buf, unsigned int len )
{
	static const uint16_t table[256] = {
	0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
	0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
	0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
	0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
	0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
	0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
	0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
	0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
	0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
	0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
	0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
	0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
	0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
	0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
	0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
	0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
	0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
	0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
	0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
	0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
	0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
	0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
	0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
	0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
	0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
	0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
	0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
	0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
	0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
	0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
	0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
	0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040 };

	uint8_t tor = 0;
	uint16_t crc = 0xFFFF;

  //mod for not counting first to bytes
  buf++;
  buf++;
  len = len -2;


	while( len-- )
	{
		tor = (*buf++) ^ crc;
		crc >>= 8;
		crc ^= table[tor];
	}

	return crc;
}
