// =============================================================
// CONFIG WEB SERVER - port 80
// Reachable from a browser at http://<board IP>
// webConfigSetup() is called from EthernetStart() after Ethernet.begin() (see zEthernet.ino)
// webConfigLoop()  is called from the main loop() (NOT from autosteerLoop) - see the main .ino
//
// Ported from AIO_Keya_WasKeyaFiltre. Covers the settings that don't have a home in
// AgOpenGPS's own Steer Config screen: wasless auto-zero tuning, Keya encoder calibration,
// and BNO anti-jitter filters. SteerDriverType/PressureSensorType selection is planned for
// a later pass (EEPROM consolidation), not yet exposed here.
// =============================================================

#ifdef ARDUINO_TEENSY41

#include <NativeEthernet.h>
#include <NativeEthernetUdp.h>
#include <EEPROM.h>

EthernetServer webServer(80);

#define EEPROM_ADDR_EMA_YAW   170
#define EEPROM_ADDR_EMA_ROLL  174
#define EEPROM_ADDR_EMA_PITCH 178
#define EEPROM_ADDR_EMA_STOP  182

void emaParamsLoad() {
    float v;

    EEPROM.get(EEPROM_ADDR_EMA_YAW, v);
    if (!isnan(v) && v >= 0.0f && v <= 1.0f) emaYawAlpha = v;

    EEPROM.get(EEPROM_ADDR_EMA_ROLL, v);
    if (!isnan(v) && v >= 0.0f && v <= 1.0f) emaRollAlpha = v;

    EEPROM.get(EEPROM_ADDR_EMA_PITCH, v);
    if (!isnan(v) && v >= 0.0f && v <= 1.0f) emaPitchAlpha = v;

    EEPROM.get(EEPROM_ADDR_EMA_STOP, v);
    if (!isnan(v) && v >= 0.0f && v <= 20.0f) emaStopKmh = v;
}

// -----------------------------------------------------------------
// Setup - call after Ethernet.begin()
// -----------------------------------------------------------------
void webConfigSetup()
{
  webServer.begin();
  Serial.print("[WEB] Config server started: http://");
  Serial.print(Eth_myip[0]); Serial.print(".");
  Serial.print(Eth_myip[1]); Serial.print(".");
  Serial.print(Eth_myip[2]); Serial.print(".");
  Serial.println(Eth_myip[3]);
}

// -----------------------------------------------------------------
// HTTP helpers
// -----------------------------------------------------------------
static void sendOK(EthernetClient& c, const char* contentType)
{
  c.println("HTTP/1.1 200 OK");
  c.print("Content-Type: "); c.println(contentType);
  c.println("Connection: close");
  c.println();
}

static void sendRedirect(EthernetClient& c)
{
  c.println("HTTP/1.1 303 See Other");
  c.println("Location: /setup");
  c.println("Connection: close");
  c.println();
}

static float extractFloat(const String& body, const char* key, float defVal)
{
  String k = String(key) + "=";
  int idx = body.indexOf(k);
  if (idx < 0) return defVal;
  idx += k.length();
  int end = body.indexOf('&', idx);
  String val = (end < 0) ? body.substring(idx) : body.substring(idx, end);
  val.trim();
  return val.toFloat();
}

static uint32_t extractUint(const String& body, const char* key, uint32_t defVal)
{
  return (uint32_t)extractFloat(body, key, (float)defVal);
}

// -----------------------------------------------------------------
// Handle a POST /save request. Returns true if a board-setup field (driver type, kickout
// sensor type, or serial port assignment) changed, meaning the board needs to reboot for
// the new value to actually take effect (they're only read once, in setup()).
static bool handlePost(const String& body)
{
  bool needsReboot = false;

  // ---- Board setup: steering driver / kickout sensor type ----
  uint8_t newDriverType = (uint8_t)extractFloat(body, "steerDriverType", steerConfig.SteerDriverType);
  if (newDriverType <= 1 && newDriverType != steerConfig.SteerDriverType)
  {
    steerConfig.SteerDriverType = newDriverType;
    needsReboot = true;
  }

  uint8_t newPressureType = (uint8_t)extractFloat(body, "pressureSensorType", steerConfig.PressureSensorType);
  if (newPressureType <= 2 && newPressureType != steerConfig.PressureSensorType)
  {
    steerConfig.PressureSensorType = newPressureType;
    needsReboot = true;
  }

  EEPROM.put(40, steerConfig);

  float newMaxHz = extractFloat(body, "pressureSensorMaxHz", pressureSensorMaxHz);
  if (newMaxHz > 1.0f && newMaxHz < 5000.0f)
  {
    pressureSensorMaxHz = newMaxHz;
    EEPROM.put(EEPROM_ADDR_PRESSURE_MAX_HZ, pressureSensorMaxHz);
  }

  // ---- Board setup: serial port assignment ----
  uint8_t newGpsCount = (uint8_t)extractFloat(body, "gpsCount",  portConfig.gpsCount);
  uint8_t newGps1     = (uint8_t)extractFloat(body, "gps1Port",  portConfig.gps1Port);
  uint8_t newGps2     = (uint8_t)extractFloat(body, "gps2Port",  portConfig.gps2Port);
  uint8_t newTm171    = (uint8_t)extractFloat(body, "tm171Port", portConfig.tm171Port);
  newGpsCount = (newGpsCount == 2) ? 2 : 1;
  if (newGps1  > SERIAL_PORT_7) newGps1  = SERIAL_PORT_AUTO;
  if (newGps2  > SERIAL_PORT_7) newGps2  = SERIAL_PORT_AUTO;
  if (newTm171 > SERIAL_PORT_7) newTm171 = SERIAL_PORT_AUTO;

  if (newGpsCount != portConfig.gpsCount || newGps1 != portConfig.gps1Port ||
      newGps2 != portConfig.gps2Port || newTm171 != portConfig.tm171Port)
  {
    portConfig.gpsCount  = newGpsCount;
    portConfig.gps1Port  = newGps1;
    portConfig.gps2Port  = newGps2;
    portConfig.tm171Port = newTm171;
    portConfigSave();
    needsReboot = true;
  }

  // Checkboxes are only present in the POST body when checked
  azParams.useBno = body.indexOf("useBno=1") >= 0 ? 1 : 0;
  azParams.useGps = body.indexOf("useGps=1") >= 0 ? 1 : 0;

  float b = extractFloat(body, "beta", azParams.beta);
  if (b >= 0.001f && b <= 1.0f) azParams.beta = b;

  azParams.speedMin   = extractFloat(body, "speedMin",   azParams.speedMin);
  azParams.yawRateMax = extractFloat(body, "yawRateMax", azParams.yawRateMax);
  azParams.gpsHdgMax  = extractFloat(body, "gpsHdgMax",  azParams.gpsHdgMax);
  azParams.timeSlowMs = extractUint (body, "timeSlowMs", azParams.timeSlowMs);
  azParams.timeFastMs = extractUint (body, "timeFastMs", azParams.timeFastMs);
  azParams.speedSlow  = extractFloat(body, "speedSlow",  azParams.speedSlow);
  azParams.speedFast  = extractFloat(body, "speedFast",  azParams.speedFast);
  EEPROM.put(EEPROM_ADDR_AZ_PARAMS, azParams);

  float ticks = extractFloat(body, "keyaTicks", keyaTicksPerDeg);
  if (ticks > 1.0f && ticks < 500.0f) {
    keyaTicksPerDeg = ticks;
    EEPROM.put(EEPROM_ADDR_KEYA_TICKS, keyaTicksPerDeg);
  }

  // BNO EMA filters
  {
    float v;
    v = extractFloat(body, "emaYaw", emaYawAlpha);
    if (v >= 0.0f && v <= 1.0f) {
        emaYawAlpha = v;
        EEPROM.put(EEPROM_ADDR_EMA_YAW, emaYawAlpha);
    }
    v = extractFloat(body, "emaRoll", emaRollAlpha);
    if (v >= 0.0f && v <= 1.0f) {
        emaRollAlpha = v;
        EEPROM.put(EEPROM_ADDR_EMA_ROLL, emaRollAlpha);
    }
    v = extractFloat(body, "emaPitch", emaPitchAlpha);
    if (v >= 0.0f && v <= 1.0f) {
        emaPitchAlpha = v;
        EEPROM.put(EEPROM_ADDR_EMA_PITCH, emaPitchAlpha);
    }
    v = extractFloat(body, "emaStop", emaStopKmh);
    if (v >= 0.0f && v <= 20.0f) {
        emaStopKmh = v;
        EEPROM.put(EEPROM_ADDR_EMA_STOP, emaStopKmh);
    }
  }

  Serial.print("[WEB] Saved. useBno="); Serial.print(azParams.useBno);
  Serial.print(" useGps=");             Serial.print(azParams.useGps);
  Serial.print(" beta=");               Serial.println(azParams.beta, 3);

  if (needsReboot)
  {
    Serial.println("[WEB] Board setup changed - rebooting to apply.");
  }

  return needsReboot;
}

// -----------------------------------------------------------------
// Row helpers
// -----------------------------------------------------------------
static void rowNum(EthernetClient& c,
                   const char* label, const char* name,
                   float val, int dec,
                   const char* unit, const char* desc)
{
  c.print("<div class='row'><label>"); c.print(label); c.println("</label>");
  c.print("<input type='number' name='"); c.print(name);
  c.print("' value='"); c.print(val, dec);
  c.println("' step='any'>");
  c.print("<span class='unit'>"); c.print(unit); c.println("</span></div>");
  if (desc && desc[0]) {
    c.print("<div class='desc'>"); c.print(desc); c.println("</div>");
  }
}

static void rowSelectStart(EthernetClient& c, const char* label, const char* name)
{
  c.print("<div class='row'><label>"); c.print(label); c.println("</label>");
  c.print("<select name='"); c.print(name); c.println("'>");
}

static void rowSelectOption(EthernetClient& c, const char* value, const char* label, bool selected)
{
  c.print("<option value='"); c.print(value); c.print("'");
  if (selected) c.print(" selected");
  c.print(">"); c.print(label); c.println("</option>");
}

static void rowSelectEnd(EthernetClient& c, const char* desc)
{
  c.println("</select></div>");
  if (desc && desc[0]) {
    c.print("<div class='desc'>"); c.print(desc); c.println("</div>");
  }
}

static void rowToggle(EthernetClient& c,
                      const char* label, const char* name,
                      uint8_t val, const char* desc)
{
  c.print("<div class='trow'><span class='tlabel'>"); c.print(label); c.println("</span>");
  c.print("<label class='sw'><input type='checkbox' name='"); c.print(name);
  c.print("' value='1'");
  if (val) c.print(" checked");
  c.println("><span class='sl'></span></label></div>");
  if (desc && desc[0]) {
    c.print("<div class='desc'>"); c.print(desc); c.println("</div>");
  }
}

// -----------------------------------------------------------------
// Shared head/CSS + nav. Split into two pages on purpose: Status auto-refreshes (nothing on
// it is ever mid-edit, so reloading it is always safe) and Setup never reloads itself (so a
// dropdown or slider you're in the middle of choosing can never get wiped out from under you).
// -----------------------------------------------------------------
static void sendHead(EthernetClient& c, const char* title, bool autoRefresh)
{
  c.println("<!DOCTYPE html><html><head>");
  c.println("<meta charset='utf-8'>");
  c.println("<meta name='viewport' content='width=device-width,initial-scale=1'>");
  if (autoRefresh) c.println("<meta http-equiv='refresh' content='4'>");
  c.print("<title>"); c.print(title); c.println("</title>");
  c.println("<style>");
  c.println("*{box-sizing:border-box}");
  c.println("body{font-family:sans-serif;max-width:560px;margin:20px auto;padding:0 14px;background:#1a1a2e;color:#eee}");
  c.println("h1{color:#e94560;font-size:1.3em;margin-bottom:6px}");
  c.println("h2{color:#fff;background:#0f3460;padding:7px 11px;border-radius:5px;font-size:.91em;margin:20px 0 8px}");
  c.println(".status{background:#16213e;border-radius:6px;padding:10px 14px;margin-bottom:14px;font-size:.84em;line-height:2}");
  c.println(".status b{color:#e94560;font-size:1.25em}");
  c.println(".ok{color:#00c853;font-weight:bold}.nok{color:#e94560;font-weight:bold}");
  c.println(".row{display:flex;align-items:center;margin:5px 0}");
  c.println(".row label{font-size:.84em;color:#bbb;flex:1;padding-right:6px}");
  c.println(".row input{width:88px;padding:4px 7px;background:#16213e;color:#eee;border:1px solid #0f3460;border-radius:4px;font-size:.93em}");
  c.println(".row select{width:220px;padding:4px 7px;background:#16213e;color:#eee;border:1px solid #0f3460;border-radius:4px;font-size:.93em}");
  c.println(".unit{font-size:.73em;color:#556;margin-left:5px;width:52px}");
  c.println(".trow{display:flex;align-items:center;margin:7px 0}");
  c.println(".tlabel{font-size:.84em;color:#bbb;flex:1;padding-right:6px}");
  c.println(".sw{position:relative;display:inline-block;width:48px;height:26px;flex-shrink:0}");
  c.println(".sw input{opacity:0;width:0;height:0}");
  c.println(".sl{position:absolute;cursor:pointer;inset:0;background:#444;border-radius:26px;transition:.25s}");
  c.println(".sl:before{content:'';position:absolute;width:20px;height:20px;left:3px;top:3px;background:#fff;border-radius:50%;transition:.25s}");
  c.println("input:checked+.sl{background:#e94560}");
  c.println("input:checked+.sl:before{transform:translateX(22px)}");
  c.println(".desc{font-size:.71em;color:#5a8a6a;margin:-1px 0 7px 0;padding-left:2px;line-height:1.5}");
  c.println(".desc b{color:#7bc}");
  c.println(".sep{border:none;border-top:1px solid #0f3460;margin:12px 0 8px}");
  c.println("button{width:100%;padding:11px;background:#e94560;color:#fff;border:none;border-radius:6px;font-size:1em;cursor:pointer;margin-top:18px}");
  c.println("button:hover{background:#c73652}");
  c.println(".foot{text-align:center;font-size:.78em;color:#444;margin-top:8px}");
  c.println(".azblock{background:#0d2b1e;border:1px solid #1a5c38;border-radius:8px;padding:12px 14px;margin-bottom:14px}");
  c.println(".aztitle{font-size:.82em;color:#4ecb8d;font-weight:bold;margin-bottom:10px;letter-spacing:.04em}");
  c.println(".azgrid{display:grid;grid-template-columns:1fr 1fr;gap:8px}");
  c.println(".azcard{background:#112b1e;border-radius:6px;padding:8px 10px}");
  c.println(".azlbl{display:block;font-size:.72em;color:#6a9a80;margin-bottom:3px;white-space:nowrap}");
  c.println(".azval{display:block;font-size:1.55em;font-weight:bold;color:#eee;line-height:1.1;letter-spacing:.01em}");
  c.println(".azval.big{font-size:2em;color:#4ecb8d}");
  c.println(".azwarn{color:#f0a030!important}");
  c.println(".azbar{background:#1a3a28;border-radius:3px;height:9px;margin-top:10px;overflow:hidden}");
  c.println(".azfill{background:#4ecb8d;height:100%;border-radius:3px;transition:width .4s}");
  c.println(".emablock{background:#1a1a3a;border:1px solid #2a2a7a;border-radius:8px;padding:12px 14px;margin-bottom:14px}");
  c.println(".emarow{display:flex;align-items:center;margin:6px 0;gap:8px}");
  c.println(".emarow label{font-size:.84em;color:#bbb;flex:1}");
  c.println(".emarow input[type=range]{flex:2;accent-color:#7b7be0}");
  c.println(".emaval{font-size:.9em;font-weight:bold;color:#a0a0f0;min-width:36px;text-align:right}");
  c.println(".emabadge{font-size:.7em;padding:2px 7px;border-radius:10px;margin-left:4px}");
  c.println(".off{background:#333;color:#777}.on{background:#2a2a6a;color:#a0a0f0}");
  c.println(".nav{display:flex;gap:8px;margin-bottom:14px}");
  c.println(".nav a{flex:1;text-align:center;padding:9px;border-radius:6px;text-decoration:none;font-size:.88em;color:#bbb;background:#16213e}");
  c.println(".nav a.active{background:#e94560;color:#fff;font-weight:bold}");
  c.println("</style></head><body>");

  c.println("<h1>&#9881; AgOpenGPS Board Config</h1>");
}

static void sendNav(EthernetClient& c, bool statusActive)
{
  c.print("<div class='nav'>");
  c.print("<a href='/'");      if (statusActive)  c.print(" class='active'"); c.print(">&#128202; Status</a>");
  c.print("<a href='/setup'"); if (!statusActive) c.print(" class='active'"); c.print(">&#9881; Setup</a>");
  c.println("</div>");
}

// -----------------------------------------------------------------
// Status page - auto-refreshes every 4s, no inputs, nothing to lose on a reload.
// -----------------------------------------------------------------
static void sendStatusPage(EthernetClient& c)
{
  sendOK(c, "text/html");
  sendHead(c, "AgOpenGPS Board Status", true);
  sendNav(c, true);

  // ---- LIVE STATUS ----
  c.print("<div class='status'>");
  c.print("&#127973; WAS angle: <b>"); c.print(steerAngleActual, 2); c.print(" deg</b> &nbsp; ");
  c.print("&#128663; Speed: <b>"); c.print(gpsSpeed, 1); c.print(" km/h</b><br>");
  c.print("&#127748; Filtered GPS heading: <b>"); c.print(emaGpsHdg / 10.0f, 1); c.print(" deg</b> &nbsp; ");
  c.print("Zero established: ");
  if (wasZeroDone) c.print("<span class='ok'>&#10003; YES</span>");
  else             c.print("<span class='nok'>&#10007; NO</span>");
  c.print("<br>&#129504; IMU detected: <b>");
  if (useTM171)       c.print("TM171");
  else if (useBNO08x) c.print("BNO08x");
  else if (useCMPS)   c.print("CMPS14");
  else                c.print("<span class='nok'>none</span>");
  c.println("</b></div>");

  // ---- AUTO-ZERO TRACKING ----
  {
    float zeroDeg = (keyaTicksPerDeg > 0.0f) ? ((float)keyaZeroTicks / keyaTicksPerDeg) : 0.0f;

    c.print("<div class='azblock'>");
    c.println("<div class='aztitle'>&#127919; Keya Auto-Zero Tracking</div>");
    c.println("<div class='azgrid'>");

    c.print("<div class='azcard'>");
    c.print("<span class='azlbl'>Current WAS angle</span>");
    c.print("<span class='azval big'>"); c.print(steerAngleActual, 2); c.print(" deg</span>");
    c.println("</div>");

    c.print("<div class='azcard'>");
    c.print("<span class='azlbl'>WAS offset (zero)</span>");
    c.print("<span class='azval'>"); c.print(zeroDeg, 2); c.print(" deg</span>");
    c.println("</div>");

    c.print("<div class='azcard'>");
    c.print("<span class='azlbl'>Encoder zero (ticks)</span>");
    c.print("<span class='azval'>"); c.print(keyaZeroTicks); c.println("</span></div>");

    c.print("<div class='azcard'>");
    c.print("<span class='azlbl'>Accumulated correction</span>");
    c.print("<span class='azval");
    if (fabsf(azCorrAccum) > 0.3f) c.print(" azwarn");
    c.print("'>"); c.print(azCorrAccum, 3); c.println(" tk</span></div>");

    c.println("</div>");

    {
      float pct = constrain((azCorrAccum + 1.0f) / 2.0f, 0.0f, 1.0f) * 100.0f;
      c.print("<div class='azbar'><div class='azfill' style='width:");
      c.print(pct, 0); c.println("%'></div></div>");
    }

    c.println("</div>");
  }

  c.println("</body></html>");
}

// -----------------------------------------------------------------
// Setup page - the only place with a form. Never auto-refreshes, so there's no reload to
// race against while picking a dropdown or dragging a slider - just save when you're ready.
// -----------------------------------------------------------------
static void sendSetupPage(EthernetClient& c)
{
  sendOK(c, "text/html");
  sendHead(c, "AgOpenGPS Board Setup", false);
  sendNav(c, false);

  c.println("<form method='POST' action='/save'>");

  // ================================================================
  // SECTION 0: BOARD SETUP
  // ================================================================
  c.println("<h2>&#128736; Board setup</h2>");

  c.print("<div class='desc' style='color:#f0a030;margin-bottom:9px'>");
  c.print("Everything in this section only takes effect after a reboot - saving a change here restarts the board automatically.");
  c.println("</div>");

  rowSelectStart(c, "Steering driver", "steerDriverType");
  rowSelectOption(c, "0", "Hydraulic / DC valve (Cytron, IBT2, Danfoss)", steerConfig.SteerDriverType == 0);
  rowSelectOption(c, "1", "Keya CAN motor", steerConfig.SteerDriverType == 1);
  rowSelectEnd(c, "");

  rowSelectStart(c, "Kickout sensor type", "pressureSensorType");
  rowSelectOption(c, "0", "Generic analog / pulse-count", steerConfig.PressureSensorType == 0);
  rowSelectOption(c, "1", "John Deere (PWM duty cycle)", steerConfig.PressureSensorType == 1);
  rowSelectOption(c, "2", "Danfoss (pulse frequency)", steerConfig.PressureSensorType == 2);
  rowSelectEnd(c, "Only matters if AgOpenGPS's steer settings has the pressure-sensor kickout enabled.");

  rowNum(c,
    "Danfoss max frequency",
    "pressureSensorMaxHz", pressureSensorMaxHz, 0, "Hz",
    "Pulse frequency that reads as 100% pressure. Only used when the kickout sensor type above is Danfoss.");

  c.println("<hr class='sep'>");

  rowSelectStart(c, "GPS receivers", "gpsCount");
  rowSelectOption(c, "1", "1 (single antenna)", portConfig.gpsCount == 1);
  rowSelectOption(c, "2", "2 (dual antenna)", portConfig.gpsCount == 2);
  rowSelectEnd(c, "");

  rowSelectStart(c, "GPS 1 port", "gps1Port");
  rowSelectOption(c, "0", "Auto-detect", portConfig.gps1Port == 0);
  rowSelectOption(c, "1", "Serial2", portConfig.gps1Port == 1);
  rowSelectOption(c, "2", "Serial5", portConfig.gps1Port == 2);
  rowSelectOption(c, "3", "Serial7", portConfig.gps1Port == 3);
  rowSelectEnd(c, "");

  rowSelectStart(c, "GPS 2 port", "gps2Port");
  rowSelectOption(c, "0", "Auto-detect", portConfig.gps2Port == 0);
  rowSelectOption(c, "1", "Serial2", portConfig.gps2Port == 1);
  rowSelectOption(c, "2", "Serial5", portConfig.gps2Port == 2);
  rowSelectOption(c, "3", "Serial7", portConfig.gps2Port == 3);
  rowSelectEnd(c, "Only used when GPS receivers above is 2.");

  rowSelectStart(c, "TM171 IMU port", "tm171Port");
  rowSelectOption(c, "0", "Auto-detect", portConfig.tm171Port == 0);
  rowSelectOption(c, "1", "Serial2", portConfig.tm171Port == 1);
  rowSelectOption(c, "2", "Serial5", portConfig.tm171Port == 2);
  rowSelectOption(c, "3", "Serial7", portConfig.tm171Port == 3);
  rowSelectEnd(c, "Leave on Auto-detect unless it isn't finding your TM171 - then pick the port it's actually wired to. A manual pick is also tried even if a CMPS/BNO was already found (auto-detect skips the TM171 probe in that case).");

  c.println("<hr class='sep'>");

  c.print("<div class='desc' style='color:#888;margin:-6px 0 12px'>");
  c.print("Everything below only applies when the driver type above is Keya CAN and the AOG \"Danfoss\" checkbox is on (wasless mode).");
  c.println("</div>");

  // ================================================================
  // SECTION 1: HEADING SOURCES
  // ================================================================
  c.println("<h2>&#128268; Heading sources for auto-zero</h2>");

  c.print("<div class='desc' style='color:#888;margin-bottom:9px'>");
  c.print("The WAS zero is corrected automatically once the firmware detects the tractor is driving <b>straight</b>. ");
  c.print("Every source enabled here must be stable <b>at the same time</b> for a correction to apply.");
  c.println("</div>");

  rowToggle(c,
    "Gyro source (BNO08x or TM171) - yaw rate",
    "useBno", azParams.useBno,
    "Uses whichever gyro is active (BNO08x or TM171) to measure rotation rate around heading [deg/s], "
    "which should be near zero when driving straight. "
    "<b>Threshold: max yaw rate (below).</b> "
    "Disable only if neither IMU is present or working.");

  rowToggle(c,
    "GPS VTG source - heading variation",
    "useGps", azParams.useGps,
    "Compares GPS heading between cycles - if it isn't changing, the tractor is going straight. "
    "<b>Threshold: max GPS heading delta (below).</b> "
    "Works well in fast, straight passes. Noisy at low speed or with a poor single-antenna GPS fix. "
    "If both sources are disabled, zeroing relies on speed and duration alone.");

  c.println("<hr class='sep'>");

  rowNum(c,
    "Beta - correction speed (guidance active)",
    "beta", azParams.beta, 3, "",
    "Only applies while <b>guidance is active</b> (precise mode). "
    "Fraction of the angular error applied as a correction each stable cycle. "
    "<b>0.01</b> = very slow (several seconds of straight driving to correct 1 deg). "
    "<b>0.05</b> = a reasonable balance. "
    "<b>0.15</b> = responsive (can oscillate if the road isn't perfectly straight). "
    "Without active guidance the zero jumps directly instead.");

  // ================================================================
  // SECTION 2: STABILITY CONDITIONS
  // ================================================================
  c.println("<h2>&#9202; Stability conditions</h2>");

  c.print("<div class='desc' style='color:#888;margin-bottom:9px'>");
  c.print("All of these must hold at once for the stability timer to start. ");
  c.print("Losing any one of them resets the timer.");
  c.println("</div>");

  rowNum(c,
    "Minimum speed",
    "speedMin", azParams.speedMin, 1, "km/h",
    "Below this, auto-zero is <b>completely blocked</b> - avoids corrections while stopped, "
    "maneuvering, or turning around. Recommended: <b>1.0 km/h</b>.");

  rowNum(c,
    "Max gyro yaw rate",
    "yawRateMax", azParams.yawRateMax, 2, "deg/s",
    "Above this, the gyro (BNO08x or TM171) is considered to be turning. "
    "<b>Lower</b> = stricter, zero only on very straight sections. "
    "<b>Higher</b> = more permissive, risks correcting through a gentle curve. "
    "Inactive if the gyro source is disabled. Recommended: <b>0.5 - 1.0</b> deg/s.");

  rowNum(c,
    "Max GPS heading delta",
    "gpsHdgMax", azParams.gpsHdgMax, 2, "deg",
    "Tolerated GPS heading change between cycles (25ms). "
    "<b>Lower</b> = stricter. <b>Higher</b> = more permissive. "
    "Inactive if the GPS source is disabled. Recommended: <b>0.5 - 1.0</b> deg.");

  // ================================================================
  // SECTION 3: STABILITY DURATIONS
  // ================================================================
  c.println("<h2>&#9200; Required stability durations</h2>");

  c.print("<div class='desc' style='color:#888;margin-bottom:9px'>");
  c.print("Duration is <b>interpolated</b> between the two speed thresholds. "
          "E.g. at 6 km/h (between 3 and 12), the duration is halfway between slow and fast.");
  c.println("</div>");

  rowNum(c,
    "Duration required at low speed",
    "timeSlowMs", (float)azParams.timeSlowMs, 0, "ms",
    "Stability time required when speed &lt;= the low threshold. "
    "Longer = safer, but corrections happen less often. Recommended: <b>400 - 800 ms</b>.");

  rowNum(c,
    "Duration required at high speed",
    "timeFastMs", (float)azParams.timeFastMs, 0, "ms",
    "Stability time required when speed &gt;= the high threshold. "
    "At higher speed the tractor is naturally straighter, so a shorter duration is fine. "
    "Recommended: <b>150 - 300 ms</b>.");

  rowNum(c,
    "Low-speed threshold",
    "speedSlow", azParams.speedSlow, 1, "km/h",
    "Below this, the long duration applies. Recommended: <b>3 km/h</b>.");

  rowNum(c,
    "High-speed threshold",
    "speedFast", azParams.speedFast, 1, "km/h",
    "Above this, the short duration applies. Recommended: <b>10 - 15 km/h</b>.");

  // ================================================================
  // SECTION 4: KEYA ENCODER CALIBRATION
  // ================================================================
  c.println("<h2>&#128295; Keya encoder calibration</h2>");

  rowNum(c,
    "Encoder ticks per degree of wheel steer",
    "keyaTicks", keyaTicksPerDeg, 1, "t/deg",
    "Mechanical ratio: encoder ticks per degree of wheel steer. "
    "<b>Default 24.0</b> = 4 motor turns lock-to-lock over 60 deg. "
    "If AOG's reported angle is too large, increase this; too small, decrease it. "
    "Formula: (motor_turns x 65535) / total_steer_angle_deg.");

  // ================================================================
  // SECTION 5: BNO ANTI-JITTER EMA FILTERS
  // ================================================================
  c.println("<h2>&#127919; BNO08x anti-jitter EMA filters</h2>");

  c.print("<div class='desc' style='color:#888;margin-bottom:9px'>");
  c.print("Exponential smoothing applied to yaw, roll and pitch before they go into the PANDA sentence. "
          "<b>0.0</b> = disabled (raw value). "
          "<b>0.05</b> = very smooth. "
          "<b>0.10</b> = balanced. "
          "<b>0.30</b> = nearly raw.");
  c.println("</div>");

  c.print("<div class='emarow'>");
  c.print("<label>Yaw (heading)</label>");
  c.print("<input type='range' name='emaYaw' min='0' max='0.5' step='0.01' value='");
  c.print(emaYawAlpha, 2);
  c.print("' oninput=\"document.getElementById('vy').textContent=parseFloat(this.value).toFixed(2);\">");
  c.print("<span class='emaval' id='vy'>"); c.print(emaYawAlpha, 2); c.print("</span>");
  if (emaYawAlpha == 0.0f) c.print("<span class='emabadge off'>OFF</span>");
  else                     c.print("<span class='emabadge on'>ON</span>");
  c.println("</div>");

  c.print("<div class='emarow'>");
  c.print("<label>Roll</label>");
  c.print("<input type='range' name='emaRoll' min='0' max='0.5' step='0.01' value='");
  c.print(emaRollAlpha, 2);
  c.print("' oninput=\"document.getElementById('vr').textContent=parseFloat(this.value).toFixed(2);\">");
  c.print("<span class='emaval' id='vr'>"); c.print(emaRollAlpha, 2); c.print("</span>");
  if (emaRollAlpha == 0.0f) c.print("<span class='emabadge off'>OFF</span>");
  else                      c.print("<span class='emabadge on'>ON</span>");
  c.println("</div>");

  c.print("<div class='emarow'>");
  c.print("<label>Pitch</label>");
  c.print("<input type='range' name='emaPitch' min='0' max='0.5' step='0.01' value='");
  c.print(emaPitchAlpha, 2);
  c.print("' oninput=\"document.getElementById('vp').textContent=parseFloat(this.value).toFixed(2);\">");
  c.print("<span class='emaval' id='vp'>"); c.print(emaPitchAlpha, 2); c.print("</span>");
  if (emaPitchAlpha == 0.0f) c.print("<span class='emabadge off'>OFF</span>");
  else                       c.print("<span class='emabadge on'>ON</span>");
  c.println("</div>");

  c.println("<hr class='sep'>");

  c.print("<div class='emarow'>");
  c.print("<label>Stop-reset threshold</label>");
  c.print("<input type='range' name='emaStop' min='0' max='10' step='0.5' value='");
  c.print(emaStopKmh, 1);
  c.print("' oninput=\"document.getElementById('vs').textContent=parseFloat(this.value).toFixed(1);\">");
  c.print("<span class='emaval' id='vs'>"); c.print(emaStopKmh, 1); c.print("</span>");
  c.print("<span style='font-size:.73em;color:#556;margin-left:4px'>km/h</span>");
  c.println("</div>");
  c.print("<div class='desc'>");
  c.print("Below this speed the EMA resets to the raw value. "
          "<b>0.0</b> = filter continuously, even while stopped.");
  c.println("</div>");

  // ================================================================
  // SAVE BUTTON
  // ================================================================
  c.println("<button type='submit'>&#128190; Save to EEPROM</button>");
  c.println("<p class='foot'>This page never reloads on its own - check the Status tab for live values.</p>");
  c.println("</form>");
  c.println("</body></html>");
}

// -----------------------------------------------------------------
// Loop - call from the main loop(), outside autosteerLoop(). Non-blocking.
// -----------------------------------------------------------------
void webConfigLoop()
{
  EthernetClient client = webServer.available();
  if (!client) return;

  uint32_t t = millis();
  String   requestLine = "";
  String   body        = "";
  bool     isPost      = false;
  int      contentLen  = 0;
  bool     headersDone = false;
  String   line        = "";

  while (client.connected() && (millis() - t < 200))
  {
    if (!client.available()) continue;
    char ch = client.read();

    if (ch == '\n')
    {
      if (!headersDone)
      {
        if (requestLine.length() == 0) requestLine = line;
        if (line.startsWith("POST"))            isPost     = true;
        if (line.startsWith("Content-Length:")) contentLen = line.substring(16).toInt();
        if (line.length() <= 1) {
          headersDone = true;
          if (!isPost) break;
        }
        line = "";
      }
    }
    else if (ch != '\r')
    {
      line += ch;
    }

    if (headersDone && isPost && contentLen > 0)
    {
      while (client.available() && (int)body.length() < contentLen)
        body += (char)client.read();
      break;
    }
  }

  bool needsReboot = false;

  if (isPost && body.length() > 0) {
    needsReboot = handlePost(body);
    sendRedirect(client);
  } else if (requestLine.indexOf("/setup") >= 0) {
    sendSetupPage(client);
  } else {
    sendStatusPage(client);
  }

  delay(1);
  client.stop();

  if (needsReboot)
  {
    delay(200); // let the redirect response actually leave the wire before resetting
    SCB_AIRCR = 0x05FA0004; // Teensy reset - same mechanism used for a subnet change
  }
}

#endif // ARDUINO_TEENSY41
