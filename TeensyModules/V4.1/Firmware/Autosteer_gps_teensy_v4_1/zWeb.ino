/* zWeb.ino - the module's web page: http://<subnet>.126/ (the subnet set by AgIO, e.g. http://192.168.5.126/).
   Ported from AOG_CAN_Teensy4.1, which took it from renatorgr/Boards, Firmware/v4.5_AiO_WEB/zWeb.ino and
   zEthernet.ino (written 2026 by Claude (Anthropic) for renatorgr).
   Autosteer_gps_teensy_v4_1, 2026: the pages of this firmware (status of steering, GPS / dual, IMU and hydraulics,
   serial ports, John Deere FEMA input and remote pulse sensor, commands to the GNSS receiver, TM171 settings and
   calibration, firmware update, restart, messages).
   Answers are sent a piece at a time from loop(), only what the socket takes, so steering and the GPS keep
   running while a browser loads the page. The network stack is polled from loop() (zNet.h).
   Licence: GNU General Public License v3 or later (file COPYING), WITHOUT ANY WARRANTY.
   SPDX-License-Identifier: GPL-3.0-or-later
*/
// One request at a time, answered with "Connection: close". There is no password: anyone on the
// module's network can open the page. What could disturb steering (serial ports and restart, FEMA input,
// remote input, GNSS commands, IMU choice, TM171 write and calibration, firmware update) is refused while
// autosteer is engaged; the firmware update and the gyro calibrations also while the tractor moves.

#include <stdarg.h>
#include "zWebPage.h"            // the page itself (HTML, CSS, JavaScript), kept in flash

#define WEB_PORT          80
#define WEB_HEAD_MAX      2048    // request line + headers + small body
#define WEB_BODY_MAX      512
#define WEB_TIMEOUT_MS    3000
#define WEB_FIRST_BYTE_MS 500     // a connection with no request yet (browser "preconnect") is closed after this
#define WEB_MOVING_KMH    1.0f    // no firmware update or gyro calibration above this speed
#define WEB_LOG_LINES     40
#define WEB_LOG_LEN       200

EthernetServer webServer(WEB_PORT);
static bool webRunning = false;

enum { WEB_IDLE, WEB_READ, WEB_SEND };
enum { WEB_AFTER_NONE, WEB_AFTER_RESTART };

static struct
{
    EthernetClient client;
    uint8_t  state;
    uint32_t startMs, lastMs;
    char     head[WEB_HEAD_MAX + 1];
    uint16_t len;
    int16_t  bodyAt;              // index of the body in head[] (-1: headers not complete yet)
    char     method[8];
    char     path[48];
    char     query[96];
    uint32_t contentLength;
    // the answer, sent a piece at a time by webSendSome()
    char        hdr[200];
    uint16_t    hdrLen, hdrSent;
    const char* body;
    uint32_t    bodyLen, bodySent;
    uint8_t     after;            // what to do once the answer is out (restart)
} rq;

static char webBuf[8192];        // JSON answers (stays valid until the answer is sent: one request at a time)
static uint32_t webReqN = 0, webErrN = 0, webDropN = 0;

// ---------------------------------------------------------------- messages (Messages tab, also USB serial)
struct WebLogLine { uint32_t seq, ms; char text[WEB_LOG_LEN]; };
static WebLogLine webLogLines[WEB_LOG_LINES];
static uint32_t webLogNext = 1;

void webLog(const char* fmt, ...)
{
    WebLogLine& l = webLogLines[webLogNext % WEB_LOG_LINES];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(l.text, sizeof(l.text), fmt, ap);
    va_end(ap);
    l.ms = millis();
    l.seq = webLogNext++;
    Serial.println(l.text);
}

// ---------------------------------------------------------------- the network stack, from loop() (zNet.h)
static bool     netPollOn = false;
static uint32_t netPollUs = 0;

void netPollFromLoop()
{
    netPollTimer(NetPollTimerTag())->end();       // no more fnet_poll() in the timer interrupt
    netPollOn = fnet_netif_get_default() != 0;    // FNET started (Ethernet.begin() worked)
    netPollUs = micros();
    if (netPollOn) fnet_poll();
}

void netPoll()
{
    if (!netPollOn) return;
    const uint32_t now = micros();
    if (now - netPollUs < 1000) return;           // once a millisecond, as the library's timer did
    netPollUs = now;
    fnet_poll();
}

void netDelay(uint32_t ms)
{
    const uint32_t t0 = millis();
    while (millis() - t0 < ms)
    {
        netPoll();
        delayMicroseconds(100);
    }
}

// ---------------------------------------------------------------- steering state
static bool webEngaged() { return watchdogTimer < WATCHDOG_THRESHOLD; }

// The speed from AgOpenGPS (PGN 254) while it is connected, or the receiver's own (VTG)
static float webKmh()
{
    const uint32_t now = millis();
    float kmh = 0;
    if (aogLastMs && now - aogLastMs < 3000) kmh = gpsSpeed;
    if (ggaLastMs && now - ggaLastMs < 3000) { const float v = atof(speedKnots) * 1.852f; if (v > kmh) kmh = v; }
    return kmh;
}
static bool webMoving() { return webKmh() > WEB_MOVING_KMH; }

// Steering off, as when AgOpenGPS stops steering, and the hydraulic outputs released as after a restart:
// before a restart or a firmware update (the loop does not run while the update is received)
static void webSteerOff()
{
    watchdogTimer = WATCHDOG_FORCE_VALUE;
    if (steerConfig.CytronDriver)
    {
        if (steerConfig.IsRelayActiveHigh) digitalWrite(PWM2_RPWM, 1);
        else digitalWrite(PWM2_RPWM, 0);
    }
    else digitalWrite(DIR1_RL_ENABLE, 0);         // IBT2
    pwmDrive = 0;
    motorDrive();
    digitalWrite(AUTOSTEER_STANDBY_LED, 1);
    digitalWrite(AUTOSTEER_ACTIVE_LED, 0);
    noTone(velocityPWM_Pin);
    if (HYDRAULIC_ENABLED)
    {
        digitalWrite(HYDRAULIC_LIFT_OR_UP, hydConfig.isRelayActiveHigh);
        digitalWrite(HYDRAULIC_LOWER_OR_DOWN, hydConfig.isRelayActiveHigh);
        digitalWrite(HYDRAULIC_TRAMLINE, hydConfig.isRelayActiveHigh);
        raiseTimer = lowerTimer = 0;
        hydLiftPrev = 3;                          // the next PGN 239 sets them again, as after start-up
    }
}

// Bytes per second, updated once a second
static uint32_t gpsBps = 0, ntripBps = 0, radioBps = 0;
static void webRates()
{
    static uint32_t ms = 0, g = 0, n = 0, r = 0;
    const uint32_t now = millis();
    if (now - ms < 1000) return;
    const uint32_t dt = now - ms;
    ms = now;
    gpsBps   = (uint32_t)((uint64_t)(gpsRxBytes - g) * 1000 / dt);   g = gpsRxBytes;
    ntripBps = (uint32_t)((uint64_t)(ntripRxBytes - n) * 1000 / dt); n = ntripRxBytes;
    radioBps = (uint32_t)((uint64_t)(radioRxBytes - r) * 1000 / dt); r = radioRxBytes;
}

// ---------------------------------------------------------------- answers
static void webFinish()
{
    rq.client.stop();                             // graceful: the queued data still goes out, then FIN
    rq.state = WEB_IDLE;
    rq.after = WEB_AFTER_NONE;
}

static void webReply(int code, const char* type, const char* body, uint32_t len)
{
    const char* text = code == 200 ? "OK" : code == 400 ? "Bad Request" : code == 404 ? "Not Found" :
                       code == 409 ? "Conflict" : code == 413 ? "Payload Too Large" : "Error";
    rq.hdrLen = snprintf(rq.hdr, sizeof(rq.hdr),
                         "HTTP/1.1 %d %s\r\nContent-Type: %s\r\nContent-Length: %lu\r\n"
                         "Cache-Control: no-store\r\nConnection: close\r\n\r\n",
                         code, text, type, (unsigned long)len);
    rq.hdrSent = 0;
    rq.body = body;
    rq.bodyLen = len;
    rq.bodySent = 0;
    rq.state = WEB_SEND;
    rq.lastMs = millis();
}

static void webReplyJson(int code, const char* json) { webReply(code, "application/json", json, strlen(json)); }

static void webReplyError(int code, const char* msg)
{
    Json j(webBuf, sizeof(webBuf));
    j.raw("{");
    j.bo("ok", false);
    j.str("error", msg);
    j.raw("}");
    webErrN++;
    webReplyJson(code, webBuf);
}

// Send what the socket takes now, never more: NativeEthernet's write waits in a loop for room otherwise,
// and with the stack polled from loop() that room would never come.
static void webSendSome()
{
    EthernetClient& c = rq.client;
    for (uint8_t k = 0; k < 8; k++)
    {
        const bool inHdr = rq.hdrSent < rq.hdrLen;
        const char* p = inHdr ? rq.hdr + rq.hdrSent : rq.body + rq.bodySent;
        uint32_t n    = inHdr ? (uint32_t)(rq.hdrLen - rq.hdrSent) : rq.bodyLen - rq.bodySent;
        if (n == 0)
        {
            const uint8_t after = rq.after;
            webFinish();
            if (after == WEB_AFTER_RESTART)
            {
                netDelay(300);                    // let the answer leave
                SCB_AIRCR = 0x05FA0004;           // Teensy reset
                for (;;) {}
            }
            return;
        }
        const int room = c.availableForWrite();
        if (room <= 0) break;
        if (n > (uint32_t)room) n = room;
        if (n > 1460) n = 1460;
        const size_t w = c.write((const uint8_t*)p, n);
        if (w == 0) break;
        if (inHdr) rq.hdrSent += w; else rq.bodySent += w;
        rq.lastMs = millis();
    }
    if (millis() - rq.lastMs > WEB_TIMEOUT_MS || !c.connected()) { webDropN++; webFinish(); }
}

// ---------------------------------------------------------------- request helpers
// Value of "name" in a query string / urlencoded body, decoded. False if absent.
static bool formValue(const char* src, const char* name, char* out, size_t outSize)
{
    const size_t nl = strlen(name);
    const char* p = src;
    while (p && *p)
    {
        if (!strncmp(p, name, nl) && p[nl] == '=')
        {
            p += nl + 1;
            size_t k = 0;
            while (*p && *p != '&' && k < outSize - 1)
            {
                if (*p == '+') { out[k++] = ' '; p++; }
                else if (*p == '%' && hexNib(p[1]) >= 0 && hexNib(p[2]) >= 0) { out[k++] = (char)((hexNib(p[1]) << 4) | hexNib(p[2])); p += 3; }
                else out[k++] = *p++;
            }
            out[k] = '\0';
            return true;
        }
        p = strchr(p, '&');
        if (p) p++;
    }
    return false;
}

// Strict integer: the whole text must be a number
static bool parseIntStrict(const char* s, long& v)
{
    if (!*s) return false;
    char* end;
    v = strtol(s, &end, 10);
    return *end == '\0';
}

// A number field of a form: absent = dflt; present = must be a whole number in lo..hi
static bool formNum(const char* body, const char* name, long dflt, long lo, long hi, long& v)
{
    char t[16];
    if (!formValue(body, name, t, sizeof(t))) { v = dflt; return true; }
    return parseIntStrict(t, v) && v >= lo && v <= hi;
}

// ---------------------------------------------------------------- GET /api/status
static void apiStatus()
{
    const uint32_t now = millis();
    Json j(webBuf, sizeof(webBuf));
    j.raw("{");
    j.str("fw", FW_NAME);
    j.str("ver", FW_VERSION);
    j.str("built", __DATE__ " " __TIME__);
    j.str("build", fwBuildInfo);
    j.str("marker", otaMarker);
    j.str("target", otaFlashId);
    j.u("up", now / 1000);
    {
        char t[20];
        IPAddress a = Ethernet.localIP();
        snprintf(t, sizeof(t), "%u.%u.%u.%u", (unsigned)a[0], (unsigned)a[1], (unsigned)a[2], (unsigned)a[3]);
        j.str("ip", t);
    }
    j.bo("link", Ethernet.linkStatus() == LinkON);
    j.num("cpuC", tempmonGetTemp(), 1);
    j.bo("engaged", webEngaged());
    j.bo("moving", webMoving());
    j.num("kmh", webKmh(), 1);

    j.obj("aog");
    j.bo("seen", aogLastMs != 0 && now - aogLastMs < 3000);
    j.i("ago", aogLastMs ? (long)((now - aogLastMs) / 1000) : -1);
    j.num("kmh", gpsSpeed, 1);
    j.i("guidance", guidanceStatus);
    j.num("setAngle", steerAngleSetPoint, 2);
    j.close();

    j.obj("steer");
    j.bo("ads", Autosteer_running);              // ADS1115 found: steering module, else GPS only
    j.num("angle", steerAngleActual, 2);
    j.i("wasRaw", helloSteerPosition);
    j.i("pwm", pwmDisplay);
    j.str("driver", steerConfig.IsDanfoss ? "Danfoss" : steerConfig.CytronDriver ? "Cytron" : "IBT2");
    j.bo("steerSw", steerSwitch == 0);
    j.bo("work", workSwitch == 0);
    j.bo("remote", remoteSwitch == 0);
    j.str("kick", steerConfig.PressureSensor ? "pressure" : steerConfig.CurrentSensor ? "current" : steerConfig.ShaftEncoder ? "encoder" : "");
    j.num("sensor", sensorReading, 0);
    j.u("pulses", pulseCount);
    j.u("kickMax", steerConfig.PulseCountMax);
    j.bo("fema", jdFema);
    j.num("femaUs", dutyTimeCurrent, 0);
    j.bo("remoteSensor", remoteSensor);         // remote input as a pulse sensor
    j.u("remoteHz", remotePulseRate);
    j.close();

    j.obj("hyd");
    j.bo("on", HYDRAULIC_ENABLED && hydConfig.enableToolLift);
    j.u("lift", hydLift);
    j.u("tram", tramline);
    j.u("mode", hydConfig.user4);
    j.u("raise", hydConfig.raiseTime);
    j.u("lower", hydConfig.lowerTime);
    j.bo("activeHigh", hydConfig.isRelayActiveHigh);
    j.close();

    j.obj("ports");                              // in use now (0 = none) and as set on the web page (zBoard.ino)
    j.u("gnss", boardPortNum(SerialGPS));
    j.u("gnss2", boardPortNum(SerialGPS2));
    j.u("imu", boardPortNum(SerialImu));
    j.u("radio", boardPortNum(SerialRTK));
    j.u("imuTx", boardTxPin(boardPortNum(SerialImu)));
    j.bo("auto", gpsAutoSwap);
    j.close();

    j.obj("gps");
    j.u("bps", gpsBps);
    j.u("ntripBps", ntripBps);
    j.u("radioBps", radioBps);
    j.bo("pass", passThroughGPS || passThroughGPS2);
    j.bo("ksxt", ksxtReceived && now - ksxtLastReceived < 1000);
    const bool gga = ggaLastMs != 0 && now - ggaLastMs < 3000;
    j.bo("gga", gga);
    if (gga)
    {
        j.str("fix", fixQuality);
        j.str("sats", numSats);
        j.str("hdop", HDOP);
        j.str("age", ageDGPS);
        j.num("kmh", atof(speedKnots) * 1.852, 1);
    }
    j.bo("dual", useDual);
    if (useDual)
    {
        j.num("heading", heading, 2);
        j.num("roll", rollDual, 2);
        j.num("baseline", baseline, 1);
    }
    j.close();

    j.obj("imu");
    // the one feeding AgOpenGPS, its values as sent (x / y axis as set in AgOpenGPS)
    j.str("type", useCMPS ? "CMPS14" : useBNO08x ? "BNO08x" : useTM171 ? "TM171" : "");
    j.bo("bnoFound", bnoFound);
    j.bo("tmFound", tm171Found);
    j.i("pref", boardCfg.imuPref);
    j.bo("yAxis", steerConfig.IsUseY_Axis);
    if (useTM171)
    {
        j.num("roll", steerConfig.IsUseY_Axis ? RollV.fValue : PitchV.fValue, 2);
        j.num("pitch", steerConfig.IsUseY_Axis ? PitchV.fValue : RollV.fValue, 2);
        j.num("yaw", YawV.fValue, 2);
    }
    else if (useBNO08x) { j.num("roll", roll / 10.0, 2); j.num("pitch", pitch / 10.0, 2); j.num("yaw", yaw / 10.0, 2); }
    else if (useCMPS)   { j.num("roll", roll / 10.0, 2); j.num("yaw", correctionHeading * RAD_TO_DEG, 1); }
    if (tm171Found)                              // the TM171's own values, also while the BNO08x is in use
    {
        j.num("tmRoll", RollV.fValue, 2);
        j.num("tmPitch", PitchV.fValue, 2);
        j.num("tmYaw", YawV.fValue, 2);
        j.bo("fresh", TM171lastData < 500);
        j.u("rate", TM171rpyRate);
        j.u("crcErr", TM171crcErrors);
        j.u("id", tm171Id);
        if (TM171statusMs && now - TM171statusMs < 5000) { j.num("tempC", TemperatureV.fValue, 1); j.i("qos", qos); }
        tm171SettingJson(j);
        tm171CalJson(j);
    }
    j.close();

    j.obj("web");
    j.u("req", webReqN);
    j.u("err", webErrN);
    j.u("drop", webDropN);
    j.close();
    j.raw("}");
    webReplyJson(200, webBuf);
}

// ---------------------------------------------------------------- GET /api/log?after=N
static void apiLog()
{
    char v[16];
    long n = 0;
    const uint32_t after = (formValue(rq.query, "after", v, sizeof(v)) && parseIntStrict(v, n) && n > 0) ? (uint32_t)n : 0;
    Json j(webBuf, sizeof(webBuf));
    j.raw("{");
    j.u("next", webLogNext);
    j.u("now", millis());
    j.arr("lines");
    const uint32_t first = webLogNext > WEB_LOG_LINES ? webLogNext - WEB_LOG_LINES : 1;
    for (uint32_t s = (after + 1 > first ? after + 1 : first); s < webLogNext; s++)
    {
        const WebLogLine& l = webLogLines[s % WEB_LOG_LINES];
        if (l.seq != s) continue;
        if (j.n + WEB_LOG_LEN + 64 > j.cap) break;          // the page asks again for the rest
        j.open(); j.u("s", l.seq); j.u("t", l.ms); j.str("x", l.text); j.close();
    }
    j.closeArr();
    j.raw("}");
    webReplyJson(200, webBuf);
}

// ---------------------------------------------------------------- /api/config: board setup (zBoard.ino)
static void apiConfigGet()
{
    Json j(webBuf, sizeof(webBuf));
    j.raw("{");
    j.bo("ok", true);
    j.u("gnssPort", boardCfg.gnss);              // 255 = Auto, 0 = none
    j.u("gnss2Port", boardCfg.gnss2);
    j.u("imuPort", boardCfg.imu);
    j.u("radioPort", boardCfg.radio);
    j.bo("fema", jdFema);
    j.bo("remote", remoteSensor);
    j.raw("}");
    webReplyJson(200, webBuf);
}

// gnssPort, gnss2Port, imuPort, radioPort (255 = Auto, 0 = none, 1-8 = SerialN): saved in the EEPROM and the
// Teensy restarts. fema (0 / 1, the John Deere FEMA valve on the pressure sensor input) and remote (0 / 1, the
// remote input as a pulse sensor, pulses / s as the pressure sensor value): saved, taken at once.
static void apiConfigPost(const char* body)
{
    long pg, p2, pi, pr, fema, remote;
    if (!formNum(body, "gnssPort", boardCfg.gnss, 0, 255, pg) || !formNum(body, "gnss2Port", boardCfg.gnss2, 0, 255, p2) ||
        !formNum(body, "imuPort", boardCfg.imu, 0, 255, pi) || !formNum(body, "radioPort", boardCfg.radio, 0, 255, pr))
                                                            { webReplyError(400, "invalid serial port"); return; }
    if (!formNum(body, "fema", jdFema ? 1 : 0, 0, 1, fema)) { webReplyError(400, "fema: 0 or 1"); return; }
    if (!formNum(body, "remote", remoteSensor ? 1 : 0, 0, 1, remote)) { webReplyError(400, "remote: 0 or 1"); return; }
    BoardCfg c = boardCfg;
    c.gnss = (uint8_t)pg; c.gnss2 = (uint8_t)p2; c.imu = (uint8_t)pi; c.radio = (uint8_t)pr;
    const char* portWhy = boardCfgCheck(c);
    if (portWhy) { webReplyError(400, portWhy); return; }
    const bool restart = c.gnss != boardCfg.gnss || c.gnss2 != boardCfg.gnss2 || c.imu != boardCfg.imu || c.radio != boardCfg.radio;
    const bool femaChanged = (fema == 1) != jdFema;
    const bool remoteChanged = (remote == 1) != remoteSensor;
    if ((restart || femaChanged || remoteChanged) && webEngaged()) { webReplyError(409, "autosteer is engaged - disengage it first"); return; }
    if (femaChanged)
    {
        femaSave(fema == 1);
        webLog("[web] pressure sensor input: %s (saved)", jdFema ? "John Deere FEMA valve (PWM)" : "analog sensor");
    }
    if (remoteChanged)
    {
        remoteSave(remote == 1);
        webLog("[web] remote input: %s (saved)", remoteSensor ? "pulse sensor, pulses / s as the pressure sensor value" : "switch");
    }
    if (restart)
    {
        boardCfg.gnss = c.gnss; boardCfg.gnss2 = c.gnss2; boardCfg.imu = c.imu; boardCfg.radio = c.radio;
        boardCfgSave();
        webLog("[web] serial ports GNSS %ld, dual %ld, TM171 %ld, radio %ld (255 = Auto, 0 = none) saved - restarting", pg, p2, pi, pr);
        webSteerOff();
        rq.after = WEB_AFTER_RESTART;
    }
    Json j(webBuf, sizeof(webBuf));
    j.raw("{");
    j.bo("ok", true);
    j.bo("restart", restart);
    j.raw("}");
    webReplyJson(200, webBuf);
}

// ---------------------------------------------------------------- /api/gnss: commands to the GNSS receiver (zGnss.ino)
// GET ?after=N: the command and answer lines kept after line N
static void apiGnssGet()
{
    char v[16];
    long n = 0;
    const uint32_t after = (formValue(rq.query, "after", v, sizeof(v)) && parseIntStrict(v, n) && n > 0) ? (uint32_t)n : 0;
    Json j(webBuf, sizeof(webBuf));
    j.raw("{");
    j.bo("ok", true);
    gnssJson(j, after);
    j.raw("}");
    webReplyJson(200, webBuf);
}

// POST cmd (printable ASCII, sent as CR LF <cmd> CR LF), all (1 = keep also the periodic lines of the answer)
static void apiGnssPost(const char* body)
{
    char cmd[GNSS_CMD_MAX + 2];
    if (!formValue(body, "cmd", cmd, sizeof(cmd))) { webReplyError(400, "no command"); return; }
    char* c = cmd;
    while (*c == ' ') c++;                                  // without the spaces around it
    size_t len = strlen(c);
    while (len && c[len - 1] == ' ') c[--len] = '\0';
    if (!len)                { webReplyError(400, "no command"); return; }
    if (len > GNSS_CMD_MAX)  { webReplyError(400, "command too long"); return; }
    for (size_t i = 0; i < len; i++)
        if ((uint8_t)c[i] < 0x20 || (uint8_t)c[i] > 0x7E) { webReplyError(400, "printable ASCII characters only (CR LF are added)"); return; }
    long all;
    if (!formNum(body, "all", 0, 0, 1, all)) { webReplyError(400, "all: 0 or 1"); return; }
    if (webEngaged()) { webReplyError(409, "autosteer is engaged - disengage it first"); return; }
    const char* why = gnssSend(c, all == 1);
    if (why) { webReplyError(409, why); return; }
    webLog("[web] GNSS command to Serial%u: %s", boardPortNum(SerialGPS), c);
    snprintf(webBuf, sizeof(webBuf), "{\"ok\":true,\"port\":%u}", boardPortNum(SerialGPS));
    webReplyJson(200, webBuf);
}

// ---------------------------------------------------------------- POST /api/imu (zBoard.ino, TM171.ino)
// act=select (imu = bno | tm171) | setread | setwrite (the settings form) | cal (c = calibration code, p2 / p3 its parameters)
static void apiImuPost(const char* body)
{
    char act[16], v[16];
    if (!formValue(body, "act", act, sizeof(act))) { webReplyError(400, "no act"); return; }
    if (!strcmp(act, "select"))                  // which IMU feeds AgOpenGPS (zBoard.ino)
    {
        if (!formValue(body, "imu", v, sizeof(v)) || (strcmp(v, "bno") && strcmp(v, "tm171"))) { webReplyError(400, "imu: bno or tm171"); return; }
        if (webEngaged()) { webReplyError(409, "autosteer is engaged - disengage it first"); return; }
        if (!bnoFound || !tm171Found) { webReplyError(409, "only one IMU was found at start-up"); return; }
        imuPrefSave(!strcmp(v, "bno") ? 1 : 2);
        webLog("[web] IMU in use: %s (saved)", useBNO08x ? "BNO08x" : "TM171");
        webReplyJson(200, "{\"ok\":true}");
        return;
    }
    if (!tm171Found) { webReplyError(409, "no TM171 found at start-up"); return; }
    const char* why = "";
    if (!strcmp(act, "setread")) tm171SettingRead();
    else if (!strcmp(act, "setwrite"))
    {
        if (webEngaged()) { webReplyError(409, "autosteer is engaged - disengage it first"); return; }
        auto flag = [&](const char* n) { return formValue(body, n, v, sizeof(v)) && !strcmp(v, "1"); };
        TmSetForm f;
        f.gyro = flag("gyro"); f.acc = flag("acc"); f.mag = flag("mag");
        f.gyroErr = flag("gyroErr"); f.selfAdapt = flag("selfAdapt"); f.status = flag("status"); f.save = flag("save");
        long boot, acc, mg, silent, odr;
        if (!formNum(body, "boot", 0, 0, 2, boot) || !formNum(body, "accGain", TM171_AOG_ACC, 1, 2000, acc) ||
            !formNum(body, "magGain", TM171_AOG_MAG, 1, 2000, mg) || !formNum(body, "silent", 0, 0, 65, silent) ||
            !formNum(body, "odr", 0, 0, 1000, odr))
        { webReplyError(400, "value out of range"); return; }
        f.boot = (uint8_t)boot; f.accGain = (uint16_t)acc; f.magGain = (uint16_t)mg;
        f.silent = (uint16_t)silent; f.odr = (uint16_t)odr;
        if (!tm171SettingWrite(f, &why)) { webReplyError(409, why); return; }
    }
    else if (!strcmp(act, "cal"))
    {
        char code[16];
        long p2, p3;
        if (!formValue(body, "c", code, sizeof(code))) { webReplyError(400, "no calibration"); return; }
        if (!formNum(body, "p2", 0, 0, 360000, p2) || !formNum(body, "p3", 0, 0, 10000, p3)) { webReplyError(400, "value out of range"); return; }
        if (!tm171CalStart(code, (uint32_t)p2, (uint32_t)p3, webEngaged(), webMoving(), &why)) { webReplyError(409, why); return; }
    }
    else { webReplyError(400, "unknown act"); return; }
    webReplyJson(200, "{\"ok\":true}");
}

// ---------------------------------------------------------------- POST /api/ota (zOTA.ino)
// Takes over the loop until the whole file is in (or fails); refused while autosteer is engaged or the
// tractor moves. Steering is off and the hydraulic outputs released meanwhile (webSteerOff).
// A refused update is answered only once the browser has sent the whole file (read and dropped): an answer
// while it is still sending cuts the connection, and the page could only say "the connection failed".
static void otaDrain(EthernetClient& c, uint32_t got)
{
    static uint8_t buf[512];
    uint32_t last = millis();
    while (got < rq.contentLength && c.connected() && millis() - last < 5000)
    {
        netPoll();
        const int n = c.available();
        if (n > 0)
        {
            const int r = c.read(buf, n < (int)sizeof(buf) ? n : (int)sizeof(buf));
            if (r > 0) { got += r; last = millis(); }
        }
        else netDelay(1);
    }
}

static void otaRefuse(int code, const char* why, uint32_t got)
{
    webLog("[web] firmware update REFUSED: %s (old firmware kept)", why);
    if (rq.contentLength <= 8000000) otaDrain(rq.client, got);
    webReplyError(code, why);
}

static void apiOta(const uint8_t* pre, size_t preLen)
{
    EthernetClient& c = rq.client;
    char v[8];
    const bool force = formValue(rq.query, "force", v, sizeof(v)) && !strcmp(v, "1");
    if (webEngaged()) { otaRefuse(409, "autosteer is engaged - disengage it first", preLen); return; }
    if (webMoving())  { otaRefuse(409, "the tractor is moving - stop first", preLen); return; }
    if (rq.contentLength < 1000 || rq.contentLength > 8000000) { otaRefuse(400, "no file, or file size not plausible", preLen); return; }

    webSteerOff();
    webLog("[web] firmware update: receiving %lu bytes, steering off", (unsigned long)rq.contentLength);
    const char* err = otaBegin();
    if (!err) err = otaFeed(pre, preLen);
    uint32_t got = preLen, last = millis();
    static uint8_t buf[2048];
    while (!err && got < rq.contentLength)
    {
        netPoll();                                         // the network stack: ACKs, window
        const int n = c.available();
        if (n > 0)
        {
            const int r = c.read(buf, n < (int)sizeof(buf) ? n : (int)sizeof(buf));
            if (r > 0)
            {
                err = otaFeed(buf, r);
                got += r;
                last = millis();
            }
        }
        else if (!c.connected())              err = "connection lost during the upload";
        else if (millis() - last > 15000)     err = "upload stopped (timeout)";
        else                                  netDelay(1);
    }
    uint32_t size = 0;
    if (!err) err = otaFinish(force, &size);
    if (err) { otaRefuse(400, err, got); return; }
    webLog("[web] firmware update: %lu bytes checked, installing and restarting", (unsigned long)size);
    snprintf(webBuf, sizeof(webBuf), "{\"ok\":true,\"size\":%lu}", (unsigned long)size);
    webReplyJson(200, webBuf);
    const uint32_t t0 = millis();
    while (rq.state == WEB_SEND && millis() - t0 < WEB_TIMEOUT_MS) { netPoll(); webSendSome(); }
    if (rq.state != WEB_IDLE) webFinish();
    netDelay(400);                                         // let the answer leave
    otaInstall(size);                                      // does not return
}

// ---------------------------------------------------------------- request handling
static void webDispatch()
{
    webReqN++;
    const bool get = !strcmp(rq.method, "GET"), post = !strcmp(rq.method, "POST");
    const char* body = rq.bodyAt >= 0 ? rq.head + rq.bodyAt : "";
    if      (get  && (!strcmp(rq.path, "/") || !strcmp(rq.path, "/index.html")))
        webReply(200, "text/html; charset=utf-8", WEB_PAGE, sizeof(WEB_PAGE) - 1);
    else if (get  && !strcmp(rq.path, "/api/status")) apiStatus();
    else if (get  && !strcmp(rq.path, "/api/log"))    apiLog();
    else if (get  && !strcmp(rq.path, "/api/config")) apiConfigGet();
    else if (post && !strcmp(rq.path, "/api/config")) apiConfigPost(body);
    else if (post && !strcmp(rq.path, "/api/imu"))    apiImuPost(body);
    else if (get  && !strcmp(rq.path, "/api/gnss"))   apiGnssGet();
    else if (post && !strcmp(rq.path, "/api/gnss"))   apiGnssPost(body);
    else if (post && !strcmp(rq.path, "/api/reboot"))
    {
        if (webEngaged()) { webReplyError(409, "autosteer is engaged - disengage it first"); return; }
        webLog("[web] restart requested");
        webSteerOff();
        rq.after = WEB_AFTER_RESTART;
        webReplyJson(200, "{\"ok\":true}");
    }
    else webReply(404, "text/plain", "Not found", 9);
}

// Request line and headers are complete in rq.head[0 .. headEnd)
static bool webParseHead(uint16_t headEnd)
{
    rq.head[headEnd] = '\0';
    char target[160] = {};
    if (sscanf(rq.head, "%7s %159s", rq.method, target) != 2) return false;
    char* q = strchr(target, '?');
    if (q) { *q = '\0'; const size_t k = strnlen(q + 1, sizeof(rq.query) - 1); memcpy(rq.query, q + 1, k); rq.query[k] = '\0'; }
    { const size_t k = strnlen(target, sizeof(rq.path) - 1); memcpy(rq.path, target, k); rq.path[k] = '\0'; }
    rq.contentLength = 0;
    for (char* l = strstr(rq.head, "\r\n"); l; l = strstr(l + 2, "\r\n"))
        if (!strncasecmp(l + 2, "Content-Length:", 15)) rq.contentLength = strtoul(l + 17, nullptr, 10);
    return true;
}

static void webRead()
{
    EthernetClient& c = rq.client;
    if (millis() - rq.startMs > WEB_TIMEOUT_MS ||
        (rq.len == 0 && millis() - rq.startMs > WEB_FIRST_BYTE_MS)) { if (rq.len) webDropN++; webFinish(); return; }

    const int n = c.available();
    if (n <= 0)
    {
        if (!c.connected()) { if (rq.len) webDropN++; webFinish(); }
        return;
    }
    const int room = WEB_HEAD_MAX - rq.len;
    if (room <= 0) { webReply(413, "text/plain", "Too large", 9); return; }
    const int r = c.read((uint8_t*)rq.head + rq.len, n < room ? n : room);
    if (r <= 0) return;
    rq.len += r;
    rq.head[rq.len] = '\0';

    if (rq.bodyAt < 0)
    {
        char* end = strstr(rq.head, "\r\n\r\n");
        if (!end) return;                                   // headers not complete yet
        const uint16_t headEnd = (uint16_t)(end - rq.head);
        rq.bodyAt = headEnd + 4;
        const uint16_t have = rq.len - rq.bodyAt;
        char saved[WEB_HEAD_MAX];                           // webParseHead puts a '\0' at headEnd
        memcpy(saved, rq.head + rq.bodyAt, have);
        if (!webParseHead(headEnd)) { webReply(400, "text/plain", "Bad request", 11); return; }
        memcpy(rq.head + rq.bodyAt, saved, have);
        rq.head[rq.len] = '\0';
        if (!strcmp(rq.method, "POST") && !strcmp(rq.path, "/api/ota"))
        {
            apiOta((const uint8_t*)rq.head + rq.bodyAt, have);
            return;
        }
        if (rq.contentLength > WEB_BODY_MAX) { webReply(413, "text/plain", "Too large", 9); return; }
    }
    if ((uint32_t)(rq.len - rq.bodyAt) >= rq.contentLength)
    {
        rq.head[rq.bodyAt + rq.contentLength] = '\0';
        webDispatch();
    }
}

// From EthernetStart(), after Ethernet.begin()
void webSetup()
{
    webServer.begin();
    webRunning = true;
    IPAddress a = Ethernet.localIP();
    webLog("[web] web page: http://%u.%u.%u.%u/", (unsigned)a[0], (unsigned)a[1], (unsigned)a[2], (unsigned)a[3]);
}

// From loop()
void webLoop()
{
    webRates();
    if (tm171Found) tm171Loop(!webEngaged() && !webMoving());
    if (!webRunning) return;
    if (rq.state == WEB_IDLE)
    {
        EthernetClient c = webServer.accept();
        if (!c) return;
        rq.client = c;
        rq.state = WEB_READ;
        rq.startMs = millis();
        rq.len = 0;
        rq.bodyAt = -1;
        rq.method[0] = rq.path[0] = rq.query[0] = '\0';
        rq.contentLength = 0;
        rq.after = WEB_AFTER_NONE;
    }
    if (rq.state == WEB_READ) webRead();
    if (rq.state == WEB_SEND) webSendSome();
}
