// =============================================================
// TM171 (SYD Dynamics TransducerM) PARAMETER READ/WRITE - talks the IMU's native EasyProtocol
// UART protocol directly, so the web UI can show and adjust its settings (UART1 baud rate,
// output rate, sensor-fusion gains) without SYD's own "ImuAssistant" tool or a Windows virtual
// COM port. A raw TCP<->UART bridge for that tool was tried first and deliberately scrapped -
// see CLAUDE.md - in favor of this: driving the documented config protocol ourselves.
//
// Protocol reference: SYD Dynamics TransducerM TM3xx User Guide v1.35-R1 ("For newest
// TM171/TM151/TM210, please refer to this document" - syd-dynamics.com/download-center/).
// Packet framing (0xAA 0x55 [Package Length] [4-byte header: cmd:7,res:3,fromId:11,toId:11,
// packed little-endian, first-declared-field=LSB] [payload] [CRC16 lo,hi]) is identical to
// what TM171.ino already parses on receive - MODBUS_CRC16_v3() from that file is reused here
// unmodified, with the exact same buffer/count convention GoodCRC() already uses (skip the
// 0xAA/0x55 sync bytes, count everything from the Package Length byte through the end of the
// payload). Cross-checked against the manual's own worked example
// ("aa55080c08000016000000e0ed" = broadcast Request for the Status object, id 0x16/22).
//
// Two object types used here, both un-timestamped (unlike the RPY/Status/Euler/Raw/Gravity
// telemetry objects TM171.ino parses, which all have a leading 4-byte timestamp before their
// named fields - Setting and Request do not):
//   Request (id 12): 4-byte payload, byte 0 = id of the object being requested.
//   Setting (id 21): 20-byte payload - switches(u32) / reserved(u16, must be 1152) /
//     uart1Baud(u16, unit 100bps) / canBaud(u16) / gainAcc(u16, unit 0.01) / gainMag(u16,
//     unit 0.01) / inhibitTime(u16, ms) / silentTime(u32, seconds).
//
// The `switches` 32-bit bitfield's exact layout is reconstructed from the manual's C struct
// listing (which the PDF text extraction reflowed, shifting several comments off their real
// field) - reconstruction cross-checked by confirming every field's declared width sums to
// exactly 32 bits once reassembled, which only happens for one specific ordering. Only the
// bits we have full confidence in (named, unambiguous, matching widths) are ever synthesized
// by this code - see tm171BuildSwitches(). Everything else about `switches` is always
// preserved verbatim from a real device response (read-modify-write, exactly as the manual
// itself recommends: "firstly request Setting Object, make the modifications and then send
// back. This ensures only setting of interest gets changed.") - tm171WriteSettings() refuses
// to run until a real settings response has been received at least once.
// =============================================================

#ifdef ARDUINO_TEENSY41

#define TM171_SW_ENABLE_GYRO      (1UL << 0)
#define TM171_SW_ENABLE_ACC       (1UL << 1)
#define TM171_SW_ENABLE_MAG       (1UL << 2)
#define TM171_SW_ENABLE_GYROERR   (1UL << 3)
#define TM171_SW_ENABLE_MAGADAPT  (1UL << 4)
#define TM171_SW_OUTPUT_STATUS    (1UL << 5)
#define TM171_SW_OUTPUT_RAW       (1UL << 6)
#define TM171_SW_OUTPUT_QUAT      (1UL << 8)
#define TM171_SW_OUTPUT_EULER     (1UL << 10)
#define TM171_SW_OUTPUT_RPY       (1UL << 11)
#define TM171_SW_OUTPUT_GRAVITY   (1UL << 12)
#define TM171_SW_REQUEST_ACK      (1UL << 17)
#define TM171_SW_SAVE_PERMANENT   (1UL << 18)
#define TM171_SW_ENABLE_UART1     (1UL << 19)
#define TM171_SW_ENABLE_CAN1      (1UL << 21)

struct Tm171Settings {
  bool     valid = false;          // becomes true once a real Setting object has been received
  uint32_t lastUpdateMs = 0;
  uint32_t switchesRaw = 0;        // preserved verbatim from the device - never hand-built
  uint16_t uart1Baud100bps = 1152; // 115200 bps, shown before the first real read
  uint16_t canBaud100bps = 0;
  uint16_t gainAcc = 0;            // x0.01
  uint16_t gainMag = 0;            // x0.01
  uint16_t inhibitTimeMs = 0;
  uint32_t silentTimeS = 0;
};
Tm171Settings tm171Settings;

static uint16_t tm171ReadU16(const uint8_t *p) { return (uint16_t)p[0] | ((uint16_t)p[1] << 8); }
static uint32_t tm171ReadU32(const uint8_t *p)
{
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

// Builds and sends one EasyProtocol packet, broadcast (fromId=toId=0) - see MODBUS_CRC16_v3()
// in TM171.ino for the CRC algorithm this reuses unmodified.
static void tm171SendObject(uint8_t cmd, const uint8_t *payload, uint8_t payloadLen)
{
  uint8_t pkt[32];
  uint8_t n = 0;
  pkt[n++] = 0xAA;
  pkt[n++] = 0x55;
  pkt[n++] = 4 + payloadLen; // Package Length: 4-byte header + payload content
  uint32_t header = (uint32_t)cmd & 0x7F; // res=0, fromId=0, toId=0 (broadcast)
  pkt[n++] = (uint8_t)(header);
  pkt[n++] = (uint8_t)(header >> 8);
  pkt[n++] = (uint8_t)(header >> 16);
  pkt[n++] = (uint8_t)(header >> 24);
  for (uint8_t i = 0; i < payloadLen; i++) pkt[n++] = payload[i];

  // Same convention as TM171.ino's GoodCRC(): count = n (bytes written so far), the function
  // itself skips the first 2 (0xAA/0x55) and covers everything through the payload's last byte.
  uint16_t crc = MODBUS_CRC16_v3(pkt, n);
  pkt[n++] = (uint8_t)(crc & 0xFF);
  pkt[n++] = (uint8_t)(crc >> 8);

  SerialImu->write(pkt, n);
}

void tm171RequestSettings()
{
  uint8_t payload[4] = { 21, 0, 0, 0 }; // ask for the Setting object (id 21)
  tm171SendObject(12 /* Request */, payload, 4);
}

// Called from TM171.ino's parser (case 21) with the Setting object's 20-byte content.
void tm171HandleSettingObject(uint8_t *content, uint8_t len)
{
  if (len < 20) return;

  tm171Settings.switchesRaw    = tm171ReadU32(&content[0]);
  tm171Settings.uart1Baud100bps = tm171ReadU16(&content[6]);
  tm171Settings.canBaud100bps  = tm171ReadU16(&content[8]);
  tm171Settings.gainAcc        = tm171ReadU16(&content[10]);
  tm171Settings.gainMag        = tm171ReadU16(&content[12]);
  tm171Settings.inhibitTimeMs  = tm171ReadU16(&content[14]);
  tm171Settings.silentTimeS    = tm171ReadU32(&content[16]);
  tm171Settings.valid          = true;
  tm171Settings.lastUpdateMs   = millis();

  Serial.println("[IMU] Setting object received from TM171.");
}

// Writes new UART1 baud / gains / inhibit time / silent time to the TM171. Requires a prior
// successful tm171RequestSettings() response - refuses otherwise, since `switches` must come
// from a real device read (see file header). Only the request-acknowledge and
// save-permanently bits of `switches` are ever changed here; everything else is passed
// through exactly as last read from the device.
bool tm171WriteSettings(uint16_t uart1Baud100bps, uint16_t canBaud100bps,
                         uint16_t gainAcc, uint16_t gainMag,
                         uint16_t inhibitTimeMs, uint32_t silentTimeS,
                         bool savePermanently)
{
  if (!tm171Settings.valid) return false;

  uint32_t switches = tm171Settings.switchesRaw;
  switches &= ~TM171_SW_SAVE_PERMANENT;
  if (savePermanently) switches |= TM171_SW_SAVE_PERMANENT;
  switches &= ~TM171_SW_REQUEST_ACK; // don't need an ack for this UI

  uint8_t payload[20];
  payload[0] = (uint8_t)(switches);
  payload[1] = (uint8_t)(switches >> 8);
  payload[2] = (uint8_t)(switches >> 16);
  payload[3] = (uint8_t)(switches >> 24);
  payload[4] = 0x80; payload[5] = 0x04; // reserved, must be 1152 (0x0480) - see the doc note
  payload[6] = (uint8_t)(uart1Baud100bps); payload[7] = (uint8_t)(uart1Baud100bps >> 8);
  payload[8] = (uint8_t)(canBaud100bps);   payload[9] = (uint8_t)(canBaud100bps >> 8);
  payload[10] = (uint8_t)(gainAcc);        payload[11] = (uint8_t)(gainAcc >> 8);
  payload[12] = (uint8_t)(gainMag);        payload[13] = (uint8_t)(gainMag >> 8);
  payload[14] = (uint8_t)(inhibitTimeMs);  payload[15] = (uint8_t)(inhibitTimeMs >> 8);
  payload[16] = (uint8_t)(silentTimeS);       payload[17] = (uint8_t)(silentTimeS >> 8);
  payload[18] = (uint8_t)(silentTimeS >> 16); payload[19] = (uint8_t)(silentTimeS >> 24);

  tm171SendObject(21 /* Setting */, payload, 20);

  Serial.print("[IMU] Setting written: uart1Baud="); Serial.print(uart1Baud100bps * 100UL);
  Serial.print(" gainAcc="); Serial.print(gainAcc);
  Serial.print(" gainMag="); Serial.print(gainMag);
  Serial.print(" inhibitMs="); Serial.print(inhibitTimeMs);
  Serial.print(" savePermanently="); Serial.println(savePermanently ? 1 : 0);

  // If the UART1 baud rate actually changed, our own port needs to follow it or we'll never
  // hear from the IMU again - TM171setup() re-begin()s SerialImu at the module's fixed
  // 115200 bps default, so mirror the module's new rate here instead.
  if (uart1Baud100bps != tm171Settings.uart1Baud100bps)
  {
    delay(50); // let the module apply the change before we retune our own UART
    SerialImu->begin((uint32_t)uart1Baud100bps * 100UL);
  }

  return true;
}

// -----------------------------------------------------------------
// Web UI - /imu (page), POST /imu/read (send a Request, redirect back), POST /savesettings
// (write new values, redirect back). Never auto-refreshes (it's a form, same reasoning as
// Board/Wasless) - the Request/response round-trip is asynchronous over UART anyway, so a
// "Read" button plus a manual reload to see the result is the honest representation of what's
// actually happening, rather than faking a synchronous read.
// -----------------------------------------------------------------

static const uint16_t TM171_BAUD_VALUES[] = { 10000, 9216, 4608, 2304, 1152, 576, 384, 96, 24, 12 };

static void sendImuPage(EthernetClient &c)
{
  sendOK(c, "text/html");
  sendHead(c, "AgOpenGPS IMU Settings", false);
  sendNav(c, 5);

  c.println("<div class='desc' style='color:#888;margin-bottom:10px'>");
  c.println("Reads and writes the TM171's own configuration directly over its UART connection - "
             "the same parameters SYD Dynamics' ImuAssistant tool would set, without needing that "
             "tool or a Windows virtual COM port.");
  c.println("</div>");

  if (!useTM171)
  {
    c.println("<div class='desc' style='color:#f0a030'>TM171 is not currently detected/active - "
               "these controls will have no effect until it is.</div>");
  }

  c.println("<form method='POST' action='/imu/read'><button style='margin-top:6px'>&#128260; Request Current Settings from IMU</button></form>");

  c.println("<div class='ioblock' style='margin-top:14px'>");
  c.println("<div class='iotitle'>&#128202; Last Known Settings</div>");
  if (!tm171Settings.valid)
  {
    c.println("<div class='desc' style='color:#7a8ab0'>Not read yet - click the button above, then reload this page.</div>");
  }
  else
  {
    c.println("<div class='iogrid'>");
    c.print("<div class='iocard'><span class='iolbl'>UART1 baud</span><span class='ioval'>");
    c.print((uint32_t)tm171Settings.uart1Baud100bps * 100UL); c.println("</span></div>");
    c.print("<div class='iocard'><span class='iolbl'>Inhibit time</span><span class='ioval'>");
    c.print(tm171Settings.inhibitTimeMs); c.println(" ms</span></div>");
    c.print("<div class='iocard'><span class='iolbl'>Accel gain</span><span class='ioval'>");
    c.print(tm171Settings.gainAcc / 100.0f, 2); c.println("</span></div>");
    c.print("<div class='iocard'><span class='iolbl'>Mag gain</span><span class='ioval'>");
    c.print(tm171Settings.gainMag / 100.0f, 2); c.println("</span></div>");
    c.println("</div>");
    c.print("<div class='desc' style='color:#7a8ab0;font-size:.72em;margin-top:8px'>Read ");
    c.print((millis() - tm171Settings.lastUpdateMs) / 1000UL);
    c.println("s ago.</div>");
  }
  c.println("</div>");

  c.println("<form method='POST' action='/savesettings'>");

  rowSelectStart(c, "UART1 baud rate", "uartBaud");
  for (uint8_t i = 0; i < 10; i++)
  {
    char buf[16];
    uint32_t bps = (uint32_t)TM171_BAUD_VALUES[i] * 100UL;
    snprintf(buf, sizeof(buf), "%lu", (unsigned long)bps);
    char val[8];
    snprintf(val, sizeof(val), "%u", TM171_BAUD_VALUES[i]);
    rowSelectOption(c, val, buf, tm171Settings.valid && TM171_BAUD_VALUES[i] == tm171Settings.uart1Baud100bps);
  }
  rowSelectEnd(c, "Also re-tunes this board's own UART to match, so it doesn't lose contact with the IMU.");

  rowNum(c, "Inhibit time", "inhibitMs", tm171Settings.inhibitTimeMs, 0, "ms",
         "Minimum gap between spontaneous data packages - smaller means a higher output rate. 0 is allowed.");

  rowNum(c, "Accelerometer gain", "gainAcc", tm171Settings.gainAcc / 100.0f, 2, "",
         "SYD Dynamics recommends 2.10 for ground vehicles (sensor fusion tuning).");

  rowNum(c, "Magnetometer gain", "gainMag", tm171Settings.gainMag / 100.0f, 2, "",
         "Default is 1.00.");

  rowToggle(c, "Save permanently to IMU flash", "savePerm", 0,
            "Off: applies for this power-on session only (safer, lost on power cycle). On: "
            "writes to the TM171's internal flash - make sure power is stable while saving.");

  c.println("<button>&#128190; Write Settings to IMU</button>");
  c.println("</form>");

  c.println("</body></html>");
}

static bool handleImuSave(const String &body)
{
  if (!tm171Settings.valid) return false; // must Read first - see file header

  uint16_t baud100 = (uint16_t)extractFloat(body, "uartBaud", tm171Settings.uart1Baud100bps);
  uint16_t inhibitMs = (uint16_t)extractFloat(body, "inhibitMs", tm171Settings.inhibitTimeMs);
  uint16_t gainAcc = (uint16_t)(extractFloat(body, "gainAcc", tm171Settings.gainAcc / 100.0f) * 100.0f);
  uint16_t gainMag = (uint16_t)(extractFloat(body, "gainMag", tm171Settings.gainMag / 100.0f) * 100.0f);
  bool savePerm = body.indexOf("savePerm=1") >= 0;

  return tm171WriteSettings(baud100, tm171Settings.canBaud100bps, gainAcc, gainMag,
                             inhibitMs, tm171Settings.silentTimeS, savePerm);
}

#endif // ARDUINO_TEENSY41
