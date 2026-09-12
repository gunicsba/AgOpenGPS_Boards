// =============================================================
// CONFIG WEB SERVER - port 80
// Reachable from a browser at http://<board IP>
// webConfigSetup() is called from EthernetStart() after Ethernet.begin() (see zEthernet.ino)
// webConfigLoop()  is called from the main loop() (NOT from autosteerLoop) - see the main .ino
//
// Three pages, deliberately separate:
//   GET  /         Status  - live values, auto-refreshes every 4s (nothing on it to lose)
//   GET  /board    Board   - driver/kickout sensor type, serial port assignment, IMU EMA
//                            filters. POST /saveboard. May reboot the board (see below).
//   GET  /wasless  Wasless - the auto-zero engine's own tuning + Keya encoder calibration.
//                            POST /savewasless. Shows a banner for whether wasless mode is
//                            actually active right now, since these settings do nothing
//                            otherwise. Never reboots - everything here is read live.
// Ported from AIO_Keya_WasKeyaFiltre, covering the settings that don't have a home in
// AgOpenGPS's own Steer Config screen.
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

static void sendRedirect(EthernetClient& c, const char* location)
{
  c.println("HTTP/1.1 303 See Other");
  c.print("Location: "); c.println(location);
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
// Handle a POST /saveboard request (driver type, kickout sensor type, serial port
// assignment, BNO/TM171 anti-jitter EMA filters). Returns true if driver type, kickout
// sensor type, or port assignment changed - those are only read once in setup(), so the
// board needs to reboot for a change to actually take effect.
static bool handleBoardPost(const String& body)
{
  bool needsReboot = false;

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

  // ---- Serial port assignment (per-port role, see main .ino) ----
  uint8_t newSerial2Role = (uint8_t)extractFloat(body, "serial2Role", portConfig.serial2Role);
  uint8_t newSerial5Role = (uint8_t)extractFloat(body, "serial5Role", portConfig.serial5Role);
  uint8_t newSerial7Role = (uint8_t)extractFloat(body, "serial7Role", portConfig.serial7Role);
  if (newSerial2Role > PORT_ROLE_UNUSED) newSerial2Role = PORT_ROLE_AUTO;
  if (newSerial5Role > PORT_ROLE_UNUSED) newSerial5Role = PORT_ROLE_AUTO;
  if (newSerial7Role > PORT_ROLE_UNUSED) newSerial7Role = PORT_ROLE_AUTO;

  if (newSerial2Role != portConfig.serial2Role || newSerial5Role != portConfig.serial5Role ||
      newSerial7Role != portConfig.serial7Role)
  {
    portConfig.serial2Role = newSerial2Role;
    portConfig.serial5Role = newSerial5Role;
    portConfig.serial7Role = newSerial7Role;
    portConfigSave();
    needsReboot = true;
  }

  // ---- BNO/TM171 anti-jitter EMA filters (live, no reboot needed) ----
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

  Serial.print("[WEB] Board setup saved. driverType="); Serial.print(steerConfig.SteerDriverType);
  Serial.print(" pressureSensorType="); Serial.println(steerConfig.PressureSensorType);

  if (needsReboot)
  {
    Serial.println("[WEB] Board setup changed - rebooting to apply.");
  }

  return needsReboot;
}

// -----------------------------------------------------------------
// Handle a POST /savewasless request (auto-zero tuning + Keya encoder calibration). Every
// field here is read live every cycle - no reboot needed for any of it to take effect.
static void handleWaslessPost(const String& body)
{
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

  Serial.print("[WEB] Wasless setup saved. useBno="); Serial.print(azParams.useBno);
  Serial.print(" useGps=");                            Serial.print(azParams.useGps);
  Serial.print(" beta=");                              Serial.println(azParams.beta, 3);
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
  c.println(".ioblock{background:#16213e;border:1px solid #223262;border-radius:8px;padding:12px 14px;margin-bottom:14px}");
  c.println(".iotitle{font-size:.82em;color:#7d9dff;font-weight:bold;margin-bottom:10px;letter-spacing:.04em}");
  c.println(".iogrid{display:grid;grid-template-columns:1fr 1fr;gap:8px}");
  c.println(".iocard{background:#0f1730;border-radius:6px;padding:8px 10px}");
  c.println(".iolbl{display:block;font-size:.72em;color:#7a8ab0;margin-bottom:3px;white-space:nowrap}");
  c.println(".ioval{display:block;font-size:1.3em;font-weight:bold;color:#eee;line-height:1.2}");
  c.println(".off{background:#333;color:#777}.on{background:#2a2a6a;color:#a0a0f0}");
  c.println(".nav{display:flex;gap:8px;margin-bottom:14px}");
  c.println(".nav a{flex:1;text-align:center;padding:9px;border-radius:6px;text-decoration:none;font-size:.88em;color:#bbb;background:#16213e}");
  c.println(".nav a.active{background:#e94560;color:#fff;font-weight:bold}");
  c.println(".waslessBanner{padding:12px 14px;border-radius:8px;margin-bottom:14px;font-size:.9em;font-weight:bold;line-height:1.5}");
  c.println(".wOn{background:#0d2b1e;border:1px solid #1a5c38;color:#4ecb8d}");
  c.println(".wOff{background:#2b1f0d;border:1px solid #5c4a1a;color:#f0a030}");
  c.println("</style></head><body>");

  c.println("<h1>&#9881; AgOpenGPS Board Config</h1>");
}

// 0 = Status, 1 = Board, 2 = Wasless, 3 = OTA, 4 = Terminal, 5 = IMU
static void sendNav(EthernetClient& c, uint8_t activeTab)
{
  c.print("<div class='nav'>");
  c.print("<a href='/'");         if (activeTab == 0) c.print(" class='active'"); c.print(">&#128202; Status</a>");
  c.print("<a href='/board'");    if (activeTab == 1) c.print(" class='active'"); c.print(">&#128736; Board</a>");
  c.print("<a href='/wasless'");  if (activeTab == 2) c.print(" class='active'"); c.print(">&#127919; Wasless</a>");
  c.print("<a href='/ota'");      if (activeTab == 3) c.print(" class='active'"); c.print(">&#128228; OTA</a>");
  c.print("<a href='/terminal'"); if (activeTab == 4) c.print(" class='active'"); c.print(">&#128187; Term</a>");
  c.print("<a href='/imu'");      if (activeTab == 5) c.print(" class='active'"); c.print(">&#129504; IMU</a>");
  c.println("</div>");
}

// True when the board is actually running in wasless (Keya-encoder-as-WAS) mode right now.
static bool waslessActive()
{
  return (steerConfig.SteerDriverType == STEER_DRIVER_KEYA) && steerConfig.IsDanfoss;
}

// -----------------------------------------------------------------
// Status page - display-only, nothing to lose on a refresh, so it polls its own small JSON
// endpoint (/status/data) every 400ms and patches values in place - far more reactive than a
// full-page <meta refresh> (was every 4s; a quick button press could sit unreflected on
// screen for most of that window even though the firmware itself reacted immediately) and
// without the flicker/scroll-reset a full reload causes.
// -----------------------------------------------------------------
static void sendStatusData(EthernetClient& c)
{
  bool wasless = waslessActive();
  float zeroDeg = (keyaTicksPerDeg > 0.0f) ? ((float)keyaZeroTicks / keyaTicksPerDeg) : 0.0f;
  float azPct = constrain((azCorrAccum + 1.0f) / 2.0f, 0.0f, 1.0f) * 100.0f;

  const char* imu = "none";
  if      (useTM171)   imu = "TM171";
  else if (useBNO08x)  imu = "BNO08x";
  else if (useCMPS)    imu = "CMPS14";

  sendOK(c, "application/json");
  c.print("{\"was\":");     c.print(steerAngleActual, 2);
  c.print(",\"spd\":");     c.print(gpsSpeed, 1);
  c.print(",\"hdg\":");     c.print(emaGpsHdg / 10.0f, 1);
  c.print(",\"zero\":");    c.print(wasZeroDone ? 1 : 0);
  c.print(",\"imu\":\"");   c.print(imu); c.print("\"");
  c.print(",\"wasless\":"); c.print(wasless ? 1 : 0);
  c.print(",\"wasraw\":");  c.print(wasless ? 0 : wasRawCounts);
  c.print(",\"steer\":");   c.print(digitalRead(STEERSW_PIN) ? 1 : 0);
  c.print(",\"eng\":");     c.print(steerSwitch == 0 ? 1 : 0);
  c.print(",\"work\":");    c.print(digitalRead(WORKSW_PIN) ? 1 : 0);
  c.print(",\"rem\":");     c.print(digitalRead(REMOTE_PIN) ? 1 : 0);
  c.print(",\"sens\":");    c.print(sensorReading, 1);
  c.print(",\"sensraw\":"); c.print(sensorSample, 0);
  c.print(",\"azzero\":");  c.print(zeroDeg, 2);
  c.print(",\"azticks\":"); c.print(keyaZeroTicks);
  c.print(",\"azcorr\":");  c.print(azCorrAccum, 3);
  c.print(",\"azwarn\":");  c.print(fabsf(azCorrAccum) > 0.3f ? 1 : 0);
  c.print(",\"azpct\":");   c.print(azPct, 0);

  // ---- Motor output (see the Motor Output card / pin-role note on the status page) ----
  const char* drv = (steerConfig.SteerDriverType == STEER_DRIVER_KEYA) ? "Keya"
                     : (steerConfig.CytronDriver ? "Cytron" : "IBT2");
  c.print(",\"drv\":\"");   c.print(drv); c.print("\"");
  c.print(",\"pwm\":");     c.print(pwmDisplay);
  c.print(",\"dir\":");     c.print(motorDir > 0 ? 1 : 0);
  c.print(",\"lock\":");    c.print(digitalRead(PWM2_RPWM) ? 1 : 0);
  c.print(",\"dirpin\":");  c.print(digitalRead(DIR1_RL_ENABLE) ? 1 : 0);
  c.println("}");
}

static void sendStatusPage(EthernetClient& c)
{
  sendOK(c, "text/html");
  sendHead(c, "AgOpenGPS Board Status", false);
  sendNav(c, 0);

  // ---- LIVE STATUS ----
  c.print("<div class='status'>");
  c.print("&#127973; WAS angle: <b id='s_was'>"); c.print(steerAngleActual, 2); c.print(" deg</b> &nbsp; ");
  c.print("&#128663; Speed: <b id='s_spd'>"); c.print(gpsSpeed, 1); c.print(" km/h</b><br>");
  c.print("&#127748; Filtered GPS heading: <b id='s_hdg'>"); c.print(emaGpsHdg / 10.0f, 1); c.print(" deg</b> &nbsp; ");
  c.print("Zero established: <span id='s_zero'>");
  if (wasZeroDone) c.print("<span class='ok'>&#10003; YES</span>");
  else             c.print("<span class='nok'>&#10007; NO</span>");
  c.print("</span><br>&#129504; IMU detected: <b id='s_imu'>");
  if (useTM171)       c.print("TM171");
  else if (useBNO08x) c.print("BNO08x");
  else if (useCMPS)   c.print("CMPS14");
  else                c.print("<span class='nok'>none</span>");
  c.print("</b> &nbsp; &#128295; Wasless: <span id='s_wasless'>");
  if (waslessActive()) c.print("<span class='ok'>&#10003; ACTIVE</span>");
  else                 c.print("<span class='nok'>inactive</span>");
  c.println("</span></div>");

  // ---- RAW INPUTS ----
  {
    bool wasless = waslessActive();

    c.print("<div class='ioblock'>");
    c.println("<div class='iotitle'>&#128268; Raw Inputs</div>");
    c.println("<div class='iogrid'>");

    c.print("<div class='iocard'><span class='iolbl'>WAS pot (ADS1115)</span><span class='ioval' id='s_wasraw'>");
    if (wasless) c.print("n/a (wasless)");
    else         { c.print(wasRawCounts); c.print(" counts"); }
    c.println("</span></div>");

    c.print("<div class='iocard'><span class='iolbl'>Steer button (pin "); c.print(STEERSW_PIN);
    c.print(")</span><span class='ioval' id='s_steer'>"); c.print(digitalRead(STEERSW_PIN) ? "HIGH" : "LOW");
    c.print(" &nbsp; <span style='font-size:.6em;color:#7a8ab0' id='s_eng'>engaged: "); c.print(steerSwitch == 0 ? "YES" : "no");
    c.println("</span></span></div>");

    c.print("<div class='iocard'><span class='iolbl'>Work switch (pin "); c.print(WORKSW_PIN);
    c.print(")</span><span class='ioval' id='s_work'>"); c.print(digitalRead(WORKSW_PIN) ? "HIGH" : "LOW");
    c.println("</span></div>");

    c.print("<div class='iocard'><span class='iolbl'>Remote/auto-steer switch (pin "); c.print(REMOTE_PIN);
    c.print(")</span><span class='ioval' id='s_rem'>"); c.print(digitalRead(REMOTE_PIN) ? "HIGH" : "LOW");
    c.println("</span></div>");

    c.print("<div class='iocard'><span class='iolbl'>Kickout/current sensor");
    if (!steerConfig.PressureSensor && !steerConfig.CurrentSensor) c.print(" (disabled)");
    c.print("</span><span class='ioval' id='s_sens'>"); c.print(sensorReading, 1); c.print(" / raw "); c.print(sensorSample, 0);
    c.println("</span></div>");

    c.println("</div>");
    c.print("<div class='desc' style='color:#7a8ab0;font-size:.72em;margin-top:8px'>");
    c.print("WAS raw counts: ~0 to 6800 in single-input mode (0-5V), ~-6800 to +6800 in differential mode. ");
    c.print("Kickout/current sensor share the same reading - whichever of PressureSensor/CurrentSensor is enabled in AgOpenGPS's steer settings is what's shown.");
    c.println("</div>");
    c.println("</div>");
  }

  // ---- MOTOR OUTPUT ----
  {
    const char* drv = (steerConfig.SteerDriverType == STEER_DRIVER_KEYA) ? "Keya"
                       : (steerConfig.CytronDriver ? "Cytron" : "IBT2");

    c.print("<div class='ioblock'>");
    c.print("<div class='iotitle'>&#9889; Motor Output ("); c.print(drv); c.println(")</div>");
    c.println("<div class='iogrid'>");

    c.print("<div class='iocard'><span class='iolbl'>PWM value</span><span class='ioval' id='s_pwm'>");
    c.print(pwmDisplay); c.println(" / 255</span></div>");

    c.print("<div class='iocard'><span class='iolbl'>Direction</span><span class='ioval' id='s_dir'>");
    c.print(motorDir > 0 ? "Forward" : "Reverse"); c.println("</span></div>");

    c.print("<div class='iocard'><span class='iolbl'>Lock/enable, PWM2_RPWM (pin "); c.print(PWM2_RPWM);
    c.print(")</span><span class='ioval' id='s_lock'>"); c.print(digitalRead(PWM2_RPWM) ? "HIGH" : "LOW");
    c.println("</span></div>");

    c.print("<div class='iocard'><span class='iolbl'>DIR1_RL_ENABLE (pin "); c.print(DIR1_RL_ENABLE);
    c.print(")</span><span class='ioval' id='s_dirpin'>"); c.print(digitalRead(DIR1_RL_ENABLE) ? "HIGH" : "LOW");
    c.println("</span></div>");

    c.println("</div>");
    c.print("<div class='desc' style='color:#7a8ab0;font-size:.72em;margin-top:8px'>");
    c.print("Pin roles depend on driver type. Cytron: PWM1_LPWM carries PWM+direction together, "
            "DIR1_RL_ENABLE is the direction pin, PWM2_RPWM is the enable/lock line (also drives the "
            "steer-button LED backlight - see CLAUDE.md). IBT2: PWM1_LPWM/PWM2_RPWM are the forward/"
            "reverse PWM channels, DIR1_RL_ENABLE enables both H-bridge halves at once. Keya: none of "
            "these pins are used - the PWM value and direction shown are the command sent over CAN.");
    c.println("</div>");
    c.println("</div>");
  }

  // ---- AUTO-ZERO TRACKING ----
  {
    float zeroDeg = (keyaTicksPerDeg > 0.0f) ? ((float)keyaZeroTicks / keyaTicksPerDeg) : 0.0f;

    c.print("<div class='azblock'>");
    c.println("<div class='aztitle'>&#127919; Keya Auto-Zero Tracking</div>");
    c.println("<div class='azgrid'>");

    c.print("<div class='azcard'>");
    c.print("<span class='azlbl'>Current WAS angle</span>");
    c.print("<span class='azval big' id='s_azangle'>"); c.print(steerAngleActual, 2); c.print(" deg</span>");
    c.println("</div>");

    c.print("<div class='azcard'>");
    c.print("<span class='azlbl'>WAS offset (zero)</span>");
    c.print("<span class='azval' id='s_azzero'>"); c.print(zeroDeg, 2); c.print(" deg</span>");
    c.println("</div>");

    c.print("<div class='azcard'>");
    c.print("<span class='azlbl'>Encoder zero (ticks)</span>");
    c.print("<span class='azval' id='s_azticks'>"); c.print(keyaZeroTicks); c.println("</span></div>");

    c.print("<div class='azcard'>");
    c.print("<span class='azlbl'>Accumulated correction</span>");
    c.print("<span class='azval' id='s_azcorr'");
    if (fabsf(azCorrAccum) > 0.3f) c.print(" style='color:#f0a030'");
    c.print(">"); c.print(azCorrAccum, 3); c.println(" tk</span></div>");

    c.println("</div>");

    {
      float pct = constrain((azCorrAccum + 1.0f) / 2.0f, 0.0f, 1.0f) * 100.0f;
      c.print("<div class='azbar'><div class='azfill' id='s_azbar' style='width:");
      c.print(pct, 0); c.println("%'></div></div>");
    }

    c.println("</div>");
  }

  c.println("<script>");
  c.println("async function poll() {");
  c.println("  try {");
  c.println("    const j = await (await fetch('/status/data')).json();");
  c.println("    document.getElementById('s_was').textContent = j.was.toFixed(2) + ' deg';");
  c.println("    document.getElementById('s_spd').textContent = j.spd.toFixed(1) + ' km/h';");
  c.println("    document.getElementById('s_hdg').textContent = j.hdg.toFixed(1) + ' deg';");
  c.println("    document.getElementById('s_zero').innerHTML = j.zero ? \"<span class='ok'>&#10003; YES</span>\" : \"<span class='nok'>&#10007; NO</span>\";");
  c.println("    document.getElementById('s_imu').innerHTML = (j.imu === 'none') ? \"<span class='nok'>none</span>\" : j.imu;");
  c.println("    document.getElementById('s_wasless').innerHTML = j.wasless ? \"<span class='ok'>&#10003; ACTIVE</span>\" : \"<span class='nok'>inactive</span>\";");
  c.println("    document.getElementById('s_wasraw').textContent = j.wasless ? 'n/a (wasless)' : (j.wasraw + ' counts');");
  c.println("    document.getElementById('s_steer').firstChild.textContent = j.steer ? 'HIGH' : 'LOW';");
  c.println("    document.getElementById('s_eng').textContent = 'engaged: ' + (j.eng ? 'YES' : 'no');");
  c.println("    document.getElementById('s_work').textContent = j.work ? 'HIGH' : 'LOW';");
  c.println("    document.getElementById('s_rem').textContent = j.rem ? 'HIGH' : 'LOW';");
  c.println("    document.getElementById('s_sens').textContent = j.sens.toFixed(1) + ' / raw ' + j.sensraw.toFixed(0);");
  c.println("    document.getElementById('s_pwm').textContent = j.pwm + ' / 255';");
  c.println("    document.getElementById('s_dir').textContent = j.dir ? 'Forward' : 'Reverse';");
  c.println("    document.getElementById('s_lock').textContent = j.lock ? 'HIGH' : 'LOW';");
  c.println("    document.getElementById('s_dirpin').textContent = j.dirpin ? 'HIGH' : 'LOW';");
  c.println("    document.getElementById('s_azangle').textContent = j.was.toFixed(2) + ' deg';");
  c.println("    document.getElementById('s_azzero').textContent = j.azzero.toFixed(2) + ' deg';");
  c.println("    document.getElementById('s_azticks').textContent = j.azticks;");
  c.println("    const azc = document.getElementById('s_azcorr');");
  c.println("    azc.textContent = j.azcorr.toFixed(3) + ' tk'; azc.style.color = j.azwarn ? '#f0a030' : '';");
  c.println("    document.getElementById('s_azbar').style.width = j.azpct + '%';");
  c.println("  } catch (e) {}");
  c.println("  setTimeout(poll, 400);");
  c.println("}");
  c.println("poll();");
  c.println("</script>");

  // Compile timestamp, not a hand-maintained version number - differs automatically between
  // any two builds, which is exactly what's needed to confirm an OTA update actually replaced
  // the running code (vs. just rebooted), without relying on capturing a USB boot log.
  c.print("<div class='foot'>Build: "); c.print(__DATE__); c.print(" "); c.print(__TIME__); c.println("</div>");

  c.println("</body></html>");
}

// -----------------------------------------------------------------
// Board page - driver/sensor type, serial port assignment, IMU anti-jitter filters. Never
// auto-refreshes, so there's no reload to race against while picking a dropdown or dragging
// a slider - just save when you're ready.
// -----------------------------------------------------------------
static void sendBoardPage(EthernetClient& c)
{
  sendOK(c, "text/html");
  sendHead(c, "AgOpenGPS Board Setup", false);
  sendNav(c, 1);

  c.println("<form method='POST' action='/saveboard'>");

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

  c.print("<div class='desc' style='color:#888;margin-bottom:9px'>");
  c.print("Pick what's expected on each port rather than picking a port for each module - ");
  c.print("leave a port on Auto-detect unless it's guessing wrong for your wiring.");
  c.println("</div>");

  rowSelectStart(c, "Serial2", "serial2Role");
  rowSelectOption(c, "0", "Auto-detect", portConfig.serial2Role == 0);
  rowSelectOption(c, "1", "GPS 1", portConfig.serial2Role == 1);
  rowSelectOption(c, "2", "GPS 2", portConfig.serial2Role == 2);
  rowSelectOption(c, "3", "TM171 IMU", portConfig.serial2Role == 3);
  rowSelectOption(c, "4", "Unused", portConfig.serial2Role == 4);
  rowSelectEnd(c, "");

  rowSelectStart(c, "Serial5", "serial5Role");
  rowSelectOption(c, "0", "Auto-detect", portConfig.serial5Role == 0);
  rowSelectOption(c, "1", "GPS 1", portConfig.serial5Role == 1);
  rowSelectOption(c, "2", "GPS 2", portConfig.serial5Role == 2);
  rowSelectOption(c, "3", "TM171 IMU", portConfig.serial5Role == 3);
  rowSelectOption(c, "4", "Unused", portConfig.serial5Role == 4);
  rowSelectEnd(c, "");

  rowSelectStart(c, "Serial7", "serial7Role");
  rowSelectOption(c, "0", "Auto-detect", portConfig.serial7Role == 0);
  rowSelectOption(c, "1", "GPS 1", portConfig.serial7Role == 1);
  rowSelectOption(c, "2", "GPS 2", portConfig.serial7Role == 2);
  rowSelectOption(c, "3", "TM171 IMU", portConfig.serial7Role == 3);
  rowSelectOption(c, "4", "Unused", portConfig.serial7Role == 4);
  rowSelectEnd(c, "TM171's factory default wiring is Serial7 - if auto-detect isn't finding it, try pinning it here explicitly. A manual pick is tried even if a CMPS/BNO was already found (auto-detect skips the TM171 probe in that case). Don't assign the same role to two ports.");

  c.println("<hr class='sep'>");

  // ================================================================
  // SECTION 1: BNO/TM171 ANTI-JITTER EMA FILTERS
  // ================================================================
  c.println("<h2>&#127919; IMU anti-jitter EMA filters</h2>");

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
// Wasless page - the auto-zero engine's own tuning (heading sources, stability conditions
// and durations, Keya encoder calibration). Only actually does anything when the board is
// in wasless mode - the banner below makes that unmissable rather than a easy-to-miss note.
// -----------------------------------------------------------------
static void sendWaslessPage(EthernetClient& c)
{
  sendOK(c, "text/html");
  sendHead(c, "AgOpenGPS Wasless Setup", false);
  sendNav(c, 2);

  if (waslessActive())
  {
    c.print("<div class='waslessBanner wOn'>&#9989; WASLESS MODE ACTIVE — ");
    c.print("these settings are in effect right now.</div>");
  }
  else
  {
    c.print("<div class='waslessBanner wOff'>&#9888; Wasless mode is OFF — these settings currently have no effect. ");
    c.print("Turn it on from the <a href='/board' style='color:#f0a030'>Board</a> page (Steering driver = Keya CAN) ");
    c.print("and AgOpenGPS's own \"Danfoss\" checkbox in Steer Config.</div>");
  }

  c.println("<form method='POST' action='/savewasless'>");

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
  long     contentLen  = 0;
  bool     headersDone = false;
  bool     isOtaStage  = false; // POST /ota/stage - body is read raw off the stream, never
                                 // buffered into `body`, since it can be several hundred KB
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
          isOtaStage = isPost && requestLine.indexOf("/ota/stage") >= 0;
          if (!isPost || isOtaStage) break; // leave body bytes on the stream for OTA staging
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
      while (client.available() && (long)body.length() < contentLen)
        body += (char)client.read();
      break;
    }
  }

  bool needsReboot = false;

  if (isOtaStage) {
    OtaStageResult otaResult = otaStageFromClient(client, (uint32_t)contentLen);
    sendOtaResultPage(client, otaResult);
    delay(1);
    client.stop();
    return; // OTA has its own response path, skip the generic dispatch below
  } else if (isPost && requestLine.indexOf("/ota/confirm") >= 0) {
    handleOtaConfirm(client);
    return; // may not return at all if it reboots into new firmware
  } else if (isPost && requestLine.indexOf("/ota/cancel") >= 0) {
    handleOtaCancel(client);
    delay(1);
    client.stop();
    return;
  } else if (isPost && requestLine.indexOf("/saveboard") >= 0 && body.length() > 0) {
    needsReboot = handleBoardPost(body);
    sendRedirect(client, "/board");
  } else if (isPost && requestLine.indexOf("/savewasless") >= 0 && body.length() > 0) {
    handleWaslessPost(body);
    sendRedirect(client, "/wasless");
  } else if (isPost && requestLine.indexOf("/imu/read") >= 0) {
    tm171RequestSettings();
    sendRedirect(client, "/imu");
  } else if (isPost && requestLine.indexOf("/savesettings") >= 0 && body.length() > 0) {
    handleImuSave(body);
    sendRedirect(client, "/imu");
  } else if (isPost && requestLine.indexOf("/terminal/send") >= 0 && body.length() > 0) {
    handleTerminalSend(body);
    sendOK(client, "text/plain"); client.print("ok");
  } else if (isPost && requestLine.indexOf("/terminal/baud") >= 0 && body.length() > 0) {
    handleTerminalBaud(body);
    sendOK(client, "text/plain"); client.print("ok");
  } else if (requestLine.indexOf("/terminal/data") >= 0) {
    String qs = termQueryString(requestLine);
    sendTerminalData(client, (uint8_t)extractFloat(qs, "port", 0), (uint32_t)extractFloat(qs, "since", 0));
  } else if (requestLine.indexOf("/status/data") >= 0) {
    sendStatusData(client);
  } else if (requestLine.indexOf("/terminal") >= 0) {
    sendTerminalPage(client);
  } else if (requestLine.indexOf("/imu") >= 0) {
    sendImuPage(client);
  } else if (requestLine.indexOf("/ota") >= 0) {
    sendOtaPage(client);
  } else if (requestLine.indexOf("/wasless") >= 0) {
    sendWaslessPage(client);
  } else if (requestLine.indexOf("/board") >= 0) {
    sendBoardPage(client);
  } else {
    sendStatusPage(client);
  }

  // Do NOT call client.flush() here - NativeEthernetClient::flush() (this vendored library,
  // NativeEthernetClient.cpp:281) is a `while (sockindex < Ethernet.socket_num) { ... }` loop
  // that never mutates sockindex or advances the socket - the ONLY way out is one of its two
  // internal `return`s. If the socket is ESTABLISHED and its send buffer isn't fully drained
  // (exactly the case right after writing a multi-KB response, like /terminal/data), this
  // spins forever: draining that buffer requires processing an incoming ACK, which requires
  // the very main loop() this call is blocking to keep running. This is a real deadlock in
  // the library, not a PHY/hardware issue - confirmed to be the cause of the "board needs a
  // power cycle to recover" pattern seen throughout this session once this call was added.
  delay(1);
  client.stop();

  if (needsReboot)
  {
    delay(200); // let the redirect response actually leave the wire before resetting
    SCB_AIRCR = 0x05FA0004; // Teensy reset - same mechanism used for a subnet change
  }
}

#endif // ARDUINO_TEENSY41
