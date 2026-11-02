/* zWebPage.h - the module's web page (served by zWeb.ino).
   Autosteer_gps_teensy_v4_1, 2026. Look, firmware update and TM171 settings form after the web page of
   AOG_CAN_Teensy4.1 and renatorgr/Boards, Firmware/v4.5_AiO_WEB/zWebPage.h (written 2026 by Claude (Anthropic)
   for renatorgr).
   Licence: GNU General Public License v3 or later (file COPYING), WITHOUT ANY WARRANTY.
   SPDX-License-Identifier: GPL-3.0-or-later
*/
// One self-contained HTML file, no internet needed, kept in flash (PROGMEM). Dark, or light when the
// device asks for it; the header button switches and this browser remembers it.
// Tabs: Status (steering, AgOpenGPS, GPS / dual, IMU, hydraulics, module), Setup (serial ports, steering: hydraulic
// or a Keya / Allynav / FJD motor, John Deere FEMA input, remote pulse sensor, CAN bus SASA sensor and CNH engage
// button), GNSS (commands to the receiver and its
// answers), IMU (IMU choice, TM171 settings and calibrations), Firmware (update from a .hex file, restart),
// Messages (the webLog lines).
#ifndef ZWEBPAGE_H
#define ZWEBPAGE_H

const char WEB_PAGE[] PROGMEM = R"WEBPAGE(<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<meta name="color-scheme" content="dark light">
<title>AOG Teensy Autosteer</title>
<style>
:root{color-scheme:dark;--bg:#121511;--card:#1b1f1a;--ink:#e7ebe3;--mute:#9aa395;--line:#2e352b;--ok:#66bb6a;--warn:#ffb74d;--bad:#ef5350;--acc:#8cc068;--accInk:#10140f;--soft:#232a21;--mono:ui-monospace,SFMono-Regular,Menlo,Consolas,monospace}
:root[data-theme=light]{color-scheme:light;--bg:#f2f4ef;--card:#ffffff;--ink:#1c2318;--mute:#5f6b5a;--line:#d4dbcf;--ok:#2e7d32;--warn:#b45309;--bad:#c62828;--acc:#4c7d2a;--accInk:#ffffff;--soft:#e9eee5}
@media (prefers-color-scheme:light){:root:not([data-theme]){color-scheme:light;--bg:#f2f4ef;--card:#ffffff;--ink:#1c2318;--mute:#5f6b5a;--line:#d4dbcf;--ok:#2e7d32;--warn:#b45309;--bad:#c62828;--acc:#4c7d2a;--accInk:#ffffff;--soft:#e9eee5}}
*{box-sizing:border-box}
body{margin:0;background:var(--bg);color:var(--ink);accent-color:var(--acc);font:15px/1.45 system-ui,-apple-system,"Segoe UI",Roboto,sans-serif}
#top{position:sticky;top:0;z-index:3}
header{display:flex;align-items:center;gap:10px;padding:10px 16px;background:var(--card);border-bottom:1px solid var(--line)}
header h1{font-size:16px;margin:0;font-weight:650;white-space:nowrap}
header .sub{color:var(--mute);font-size:13px;flex:1;white-space:nowrap;overflow:hidden;text-overflow:ellipsis}
.dot{width:11px;height:11px;border-radius:50%;background:var(--bad);flex:none}
.dot.on{background:var(--ok)}
nav{display:flex;gap:6px;padding:8px 12px;overflow-x:auto;background:var(--card);border-bottom:1px solid var(--line)}
nav button{border:1px solid var(--line);background:var(--card);color:var(--ink);border-radius:999px;padding:7px 14px;font:inherit;font-size:14px;cursor:pointer;white-space:nowrap}
nav button.on{background:var(--acc);border-color:var(--acc);color:var(--accInk)}
main{padding:14px 16px 40px;max-width:1100px;margin:0 auto}
.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(280px,1fr));gap:12px;margin-bottom:12px}
.card{background:var(--card);border:1px solid var(--line);border-radius:12px;padding:12px 14px;min-width:0}
.card h2{font-size:12.5px;text-transform:uppercase;letter-spacing:.05em;color:var(--mute);margin:0 0 8px;font-weight:650}
.card h3{font-size:14px;margin:14px 0 2px;font-weight:650}
dl{display:grid;grid-template-columns:auto 1fr;gap:3px 12px;margin:0}
dt{color:var(--mute)}
dd{margin:0;text-align:right;font-variant-numeric:tabular-nums;overflow-wrap:anywhere}
.r{display:contents}
.pill{display:inline-block;padding:1px 9px;border-radius:999px;font-size:13px;font-weight:600;background:var(--soft)}
.ok{color:var(--ok)}.warn{color:var(--warn)}.bad{color:var(--bad)}
.pill.ok{background:color-mix(in srgb,var(--ok) 16%,transparent)}
.pill.warn{background:color-mix(in srgb,var(--warn) 18%,transparent)}
.pill.bad{background:color-mix(in srgb,var(--bad) 16%,transparent)}
.mono{font-family:var(--mono);font-size:12.5px;white-space:pre-wrap;overflow-wrap:anywhere}
.note{color:var(--mute);font-size:13.5px;margin:6px 0 0}
.note.bad{color:var(--bad)}.note.warn{color:var(--warn)}.note.ok{color:var(--ok)}
.row{display:flex;gap:8px;flex-wrap:wrap;align-items:center;margin:8px 0}
button.act,label.act{border:0;background:var(--acc);color:var(--accInk);border-radius:8px;padding:9px 16px;font:inherit;font-weight:600;cursor:pointer}
button.sec{border:1px solid var(--line);background:var(--card);color:var(--ink);border-radius:8px;padding:8px 14px;font:inherit;cursor:pointer}
button.danger{border:1px solid var(--bad);background:transparent;color:var(--bad);border-radius:8px;padding:8px 14px;font:inherit;cursor:pointer}
button:disabled{opacity:.45;cursor:default}
input:disabled,select:disabled,label.opt:has(input:disabled){opacity:.5;cursor:default}
.opt{display:flex;gap:10px;align-items:flex-start;justify-content:space-between;padding:5px 0}
.opt>input[type=checkbox]{margin-top:4px}
.opt>span:first-child,.opt>input+span{flex:1}
.opt small{display:block;color:var(--mute)}
select{font:inherit;padding:6px 8px;border-radius:8px;border:1px solid var(--line);background:var(--card);color:var(--ink);max-width:60%}
input[type=number]{font:inherit;padding:6px 8px;border:1px solid var(--line);border-radius:8px;background:var(--card);color:var(--ink);width:6.5em}
.pair{display:flex;gap:6px;align-items:center;white-space:nowrap}
fieldset{border:1px solid var(--line);border-radius:10px;padding:6px 12px 8px;margin:0;min-width:0}
legend{color:var(--mute);font-size:12.5px;text-transform:uppercase;letter-spacing:.05em;padding:0 4px}
.cols{display:grid;grid-template-columns:repeat(auto-fit,minmax(290px,1fr));gap:12px;margin:8px 0}
details summary{cursor:pointer;color:var(--mute);font-size:13.5px;margin-top:12px}
progress{width:100%;height:14px}
.hidden{display:none!important}
.alert{background:color-mix(in srgb,var(--warn) 16%,transparent);border:1px solid var(--warn);border-radius:12px;padding:9px 12px;margin:0 0 12px;font-weight:600}
#log,#gnLog{height:66vh;overflow:auto;background:var(--card);border:1px solid var(--line);border-radius:12px;padding:8px 10px}
#gnLog{height:50vh;margin-top:8px}
.ll.tx .mono{color:var(--acc);font-weight:650}
input[type=text]{font:inherit;font-family:var(--mono);padding:7px 9px;border:1px solid var(--line);border-radius:8px;background:var(--card);color:var(--ink);flex:1;min-width:12em}
.ll{display:flex;gap:10px;padding:2px 0;border-bottom:1px dashed var(--line)}
.ll .t{color:var(--mute);flex:none;font-variant-numeric:tabular-nums;width:86px}
.swrow{display:flex;align-items:center;gap:12px;margin:6px 0}
.swrow span.on{font-weight:700;color:var(--acc)}
.switch{position:relative;display:inline-block;width:54px;height:28px;flex:none}
.switch input{opacity:0;width:0;height:0}
.switch .knob{position:absolute;inset:0;cursor:pointer;background:var(--soft);border:1px solid var(--line);border-radius:999px;transition:.2s}
.switch .knob:before{content:"";position:absolute;height:20px;width:20px;left:3px;top:3px;background:var(--acc);border-radius:50%;transition:.2s}
.switch input:checked+.knob:before{transform:translateX(26px)}
.switch input:disabled+.knob{opacity:.45;cursor:default}
.tbtn{margin-left:auto;flex:none;border:1px solid var(--line);background:var(--soft);color:var(--ink);border-radius:999px;padding:3px 11px;font:inherit;font-size:13px;font-weight:650;cursor:pointer}
@media (max-width:520px){header .sub{display:none}}
</style>
</head>
<body>
<div id="top">
<header><span class="dot" id="dot" title="link to the module"></span><h1>AOG Teensy 4.1 Autosteer</h1><span class="sub" id="hdr">connecting…</span><button class="tbtn" id="theme" type="button" title="Light / dark">☀</button></header>
<nav id="nav"><button data-t="status" class="on">Status</button><button data-t="config">Setup</button><button data-t="gnss">GNSS</button><button data-t="imu">IMU</button><button data-t="firmware">Firmware</button><button data-t="log">Messages</button></nav>
</div>
<main>
<div id="alerts"></div>

<section id="t-status">
<div class="grid">
 <div class="card"><h2>Steering</h2><dl>
  <dt>Module</dt><dd id="sMode">–</dd>
  <dt>Autosteer</dt><dd id="sEng">–</dd>
  <dt>Wheel angle</dt><dd id="sAngle">–</dd>
  <dt>Set point</dt><dd id="sSet">–</dd>
  <dt>WAS raw</dt><dd id="sWas">–</dd>
  <dt>PWM / driver</dt><dd id="sPwm">–</dd>
  <dt>Steer switch / work / remote</dt><dd id="sSw">–</dd>
  <dt>Kickout</dt><dd id="sKick">–</dd>
 </dl></div>
 <div class="card"><h2>AgOpenGPS</h2><dl>
  <dt>Connection</dt><dd id="aSeen">–</dd>
  <dt>Speed</dt><dd id="aKmh">–</dd>
  <dt>Guidance</dt><dd id="aGuid">–</dd>
 </dl></div>
 <div class="card"><h2>GPS</h2><dl>
  <dt>Serial ports</dt><dd id="gPorts">–</dd>
  <dt>Receiver data</dt><dd id="gBps">–</dd>
  <dt>NTRIP from AgIO</dt><dd id="gNtrip">–</dd>
  <dt>RTK radio</dt><dd id="gRadio">–</dd>
  <dt>Fix</dt><dd id="gFix">–</dd>
  <div class="r" id="gSatsR"><dt>Satellites / HDOP</dt><dd id="gSats">–</dd></div>
  <div class="r" id="gAgeR"><dt>Correction age</dt><dd id="gAge">–</dd></div>
  <div class="r hidden" id="gDualR"><dt>Dual heading / roll</dt><dd id="gDual">–</dd></div>
  <div class="r hidden" id="gBaseR"><dt>Dual baseline</dt><dd id="gBase">–</dd></div>
  <div class="r hidden" id="gKsxtR"><dt>KSXT (UM982)</dt><dd>forwarded to AgIO</dd></div>
 </dl></div>
 <div class="card"><h2>IMU</h2><dl>
  <dt>Type</dt><dd id="iType">–</dd>
  <div class="r" id="iRPR"><dt>Roll / pitch</dt><dd id="iRP">–</dd></div>
  <div class="r" id="iYawR"><dt>Heading</dt><dd id="iYaw">–</dd></div>
  <div class="r" id="iRateR"><dt>Output rate</dt><dd id="iRate">–</dd></div>
 </dl></div>
 <div class="card"><h2>Hydraulic lift</h2><dl>
  <dt>Tool lift</dt><dd id="hOn">–</dd>
  <div class="r" id="hStR"><dt>State / tramline</dt><dd id="hSt">–</dd></div>
  <div class="r" id="hModeR"><dt>Mode (User 4)</dt><dd id="hMode">–</dd></div>
 </dl>
 <p class="note">Set in AgOpenGPS (machine settings: hydraulic lift, User 4).</p></div>
 <div class="card"><h2>Module</h2><dl>
  <dt>Firmware</dt><dd id="mFw">–</dd>
  <dt>Built</dt><dd id="mBuilt">–</dd>
  <dt>Running for</dt><dd id="mUp">–</dd>
  <dt>IP / link</dt><dd id="mIp">–</dd>
  <dt>CPU temperature</dt><dd id="mCpu">–</dd>
 </dl></div>
</div>
</section>

<section id="t-config" class="hidden">
<div class="card" style="margin-bottom:12px"><h2>Serial ports</h2>
 <div class="cols">
  <label class="opt"><span>GNSS receiver (GGA)<small>Auto: looked for on Serial2, then Serial7 at start-up, swapped every 10 s while there is no GGA.</small></span><select id="pGnss" data-ports="1"><option value="255">Auto (Serial2 / Serial7)</option></select></label>
  <label class="opt"><span>Dual heading receiver<small>RELPOSNED. Auto with the GNSS on Auto: Serial2 when the GNSS is on Serial7, else Serial5.</small></span><select id="pGnss2" data-ports="1"><option value="255">Auto</option><option value="0">none</option></select></label>
  <label class="opt"><span>IMU (TM171)<small>Auto: Serial7, then Serial5 (not the GNSS port). A BNO08x or CMPS14 is on I2C: no port.</small></span><select id="pImu" data-ports="1"><option value="255">Auto (Serial7 / Serial5)</option><option value="0">none</option></select></label>
  <label class="opt"><span>RTK radio<small>Its corrections go to the GNSS receiver (9600 baud).</small></span><select id="pRadio" data-ports="1"><option value="0">none</option></select></label>
 </div>
 <div class="row"><button class="sec" type="button" id="pAuto">Auto (as before the web page)</button></div>
 <p class="note">A port set here is used as it is: no detection and no swapping. Serial4 (Wire1, the ADS1115), Serial6 (pin 24, the pressure sensor) and Serial8 (pin 34, the work switch) are used by this firmware and not offered.</p>
 <div class="row"><button class="act" id="cSave" data-lock="e">Save and restart</button><button class="sec" id="cUndo">Undo changes</button><span id="cMsg" class="note"></span></div>
 <p class="note">Kept in the EEPROM; the Teensy restarts to take them.</p>
</div>
<div class="grid">
 <div class="card"><h2>Steering</h2>
  <div class="swrow"><span id="swHyd">Hydraulic valve / PWM driver</span><label class="switch"><input type="checkbox" id="motorSw" data-lock="e"><span class="knob"></span></label><span id="swDd">Direct drive motor</span></div>
  <label class="opt hidden" id="motorTypeR"><span>Motor<small>On the CAN bus (CAN3). Keya and Allynav: the bit rate is found by itself (250, 500, 125, 1000 kbit/s). FJD: 1000 kbit/s.</small></span><select id="motorType" data-lock="e"><option value="1">Keya</option><option value="2">Allynav</option><option value="3">FJD</option></select></label>
  <p class="note">Hydraulic: the PWM outputs (Cytron, IBT2 or Danfoss, as set in AgOpenGPS) and the read only CAN inputs. Direct drive: the steering goes to the motor over the CAN bus, which is then the motor's: no SASA sensor, no CNH engage button. Current kickout: the higher of the motor's own current (Keya, Allynav; amps × 20: 100 % = 12.75 A) and the current sensor input. A new board starts with the Keya motor. Kept in the EEPROM; the Teensy restarts to take it, only with autosteer off.</p>
  <div class="row"><button class="act" type="button" id="mSave" data-lock="e" data-off="1" disabled>Save and restart</button><span id="mMsg" class="note"></span></div>
  <dl id="mStat" class="hidden"><dt>Motor</dt><dd id="mFound">–</dd><div class="r hidden" id="mAmpR"><dt>Motor current</dt><dd id="mAmp">–</dd></div><div class="r hidden" id="mStateR"><dt>Motor state</dt><dd id="mState">–</dd></div></dl>
 </div>
 <div class="card"><h2>Pressure sensor input</h2>
  <div class="swrow"><span id="swAna">Analog pressure sensor</span><label class="switch"><input type="checkbox" id="femaSw" data-lock="e"><span class="knob"></span></label><span id="swFema">John Deere FEMA valve</span></div>
  <p class="note">The pressure sensor input (A10). FEMA: the valve's PWM signal is read and autosteer kicks out when the wheel is turned by hand. Either way the pressure sensor must be on in AgOpenGPS; its kickout value is the threshold. Saved and taken at once, only with autosteer off.</p>
  <dl><dt>AgOpenGPS kickout</dt><dd id="kSet">–</dd><dt>Value from</dt><dd id="kSrc">–</dd><dt>Reading / threshold</dt><dd id="kVal">–</dd><div class="r hidden" id="kUsR"><dt>FEMA signal</dt><dd id="kUs">–</dd></div></dl>
  <p id="femaMsg" class="note"></p>
 </div>
 <div class="card"><h2>Remote sensor input</h2>
  <div class="swrow"><span id="swRsw">Remote switch</span><label class="switch"><input type="checkbox" id="remoteSw" data-lock="e"><span class="knob"></span></label><span id="swRpl">Pulse sensor</span></div>
  <p class="note">The remote input (pin 37). Pulse sensor: the pulses of the last second (0–255) are sent to AgOpenGPS as the pressure sensor value, in place of the pressure sensor input, and autosteer kicks out at the pressure sensor's kickout value. The pressure sensor must be on in AgOpenGPS. Saved and taken at once, only with autosteer off.</p>
  <dl><div class="r hidden" id="rHzR"><dt>Pulses / s</dt><dd id="rHz">–</dd></div></dl>
  <p id="remoteMsg" class="note"></p>
 </div>
 <div class="card"><h2>CAN bus (read only)</h2>
  <p class="note">CAN3 (Teensy pins 30 / 31, the board's CAN connector), 250 kbit/s, listen only: the Teensy sends nothing on the bus. Started once one of these is on. Saved and taken at once, only with autosteer off.</p>
  <p id="canDdNote" class="note warn hidden"></p>
  <div class="swrow"><span id="swSasaOff">Off</span><label class="switch"><input type="checkbox" id="sasaSw" data-lock="e"><span class="knob"></span></label><span id="swSasa">SASA steering wheel sensor</span></div>
  <p class="note">Frames 0x0CFF104D or 0x301. The steering wheel speed is sent to AgOpenGPS as the pressure sensor value, in place of the pressure sensor input, and autosteer kicks out at the pressure sensor's kickout value. The pressure sensor must be on in AgOpenGPS. Not together with the remote pulse sensor.</p>
  <label class="opt"><span>CNH engage button<small>Frame 0x14FF7706. Turns autosteer on and off like a second steer button: Steer button must be selected in AgOpenGPS.</small></span><select id="canBtn" data-lock="e"><option value="0">off</option><option value="1">armrest button</option><option value="2">hydraulic joystick up</option><option value="3">both</option></select></label>
  <dl>
   <dt>CAN frames</dt><dd id="cFps">–</dd>
   <div class="r hidden" id="cSasaR"><dt>SASA wheel speed</dt><dd id="cSasa">–</dd></div>
   <div class="r hidden" id="cBtnR"><dt>Engage button</dt><dd id="cBtn">–</dd></div>
   <div class="r hidden" id="cLastR"><dt>Last frame</dt><dd id="cLast" class="mono">–</dd></div>
  </dl>
  <p id="canMsg" class="note"></p>
 </div>
</div>
</section>

<section id="t-gnss" class="hidden">
<div class="card"><h2>Command to the GNSS receiver</h2>
 <p class="note">For receivers that take text commands on their NMEA port, like the Unicore UM982 / UM980. Sent to the GNSS port the way the NTRIP corrections go: CR LF, the command, CR LF. What the receiver sends in the 5 s after it is shown below.</p>
 <dl><dt>GNSS port</dt><dd id="gnPort">–</dd><dt>Receiver data</dt><dd id="gnBps">–</dd></dl>
 <div class="row"><input type="text" id="gnCmd" list="gnHist" maxlength="160" placeholder="e.g. VERSIONA" autocomplete="off" spellcheck="false" autocapitalize="off"><button class="act" type="button" id="gnSend" data-lock="e">Send</button></div>
 <datalist id="gnHist"></datalist>
 <label class="opt"><input type="checkbox" id="gnAll"><span>All lines<small>Also the lines the receiver sends all the time (GGA, VTG, RMC, GSA, GSV, HPR, KSXT ...): many lines in 5 s.</small></span></label>
 <p id="gnMsg" class="note"></p>
 <p class="note">Only with autosteer off. A setting stays only until the receiver is powered off unless it is saved in the receiver (UM982: SAVECONFIG). A command that changes the receiver's baud rate or stops its GGA cuts the GPS for AgOpenGPS; with the GNSS port on Auto the Teensy then also swaps the port every 10 s.</p>
 <div class="row" style="margin-top:14px"><h2 style="margin:0;flex:1">Receiver answers</h2><button class="sec" type="button" id="gnClear">Clear</button></div>
 <div id="gnLog"></div>
</div>
</section>

<section id="t-imu" class="hidden">
<div class="card" id="imuSelCard" style="margin-bottom:12px"><h2>IMU in use</h2>
 <div class="swrow"><span id="swBno">BNO085</span><label class="switch"><input type="checkbox" id="imuSw" data-lock="e"><span class="knob"></span></label><span id="swTm">TM171</span></div>
 <p class="note" id="imuSelTxt"></p>
</div>
<div class="card hidden" id="imuNone"><h2>IMU</h2><p class="note" id="imuNoneTxt"></p></div>
<div id="imuTm" class="hidden">
<div class="grid">
 <div class="card"><h2>TM171</h2><dl>
  <dt>Roll / pitch</dt><dd id="tRP">–</dd>
  <dt>Heading (yaw)</dt><dd id="tYaw">–</dd>
  <dt>Output rate</dt><dd id="tRate">–</dd>
  <dt>Node ID</dt><dd id="tId">–</dd>
  <dt>Bad packets</dt><dd id="tCrc">–</dd>
  <div class="r hidden" id="tTempR"><dt>Temperature / QoS</dt><dd id="tTemp">–</dd></div>
  <dt>AgOpenGPS set-up</dt><dd id="tSetup">–</dd>
 </dl>
 <p class="note">As the TM171 sends them, X and Y swapped by the firmware so the axes are the BNO's. Which one AgOpenGPS gets as roll follows "Use Y axis" in its IMU settings (now: <span id="tAxis">–</span>).</p></div>
 <div class="card"><h2>Calibration</h2>
  <p id="tcMsg" class="note"></p>
  <h3>Gyroscope</h3>
  <div class="row"><button class="sec" type="button" data-cal="gyro" data-lock="m">Gyro bias calibration (20 s)</button><button class="danger hidden" type="button" id="tcStop">Stop now</button></div>
  <p class="note">The TM171 measures its gyro bias while the tractor stands still (the engine can run); the firmware finishes it after 20 s. Kept until the TM171 is powered off. TM171 firmware V5.1 or later.</p>
  <div class="row"><button class="sec" type="button" data-cal="gyroAuto" data-lock="m">Forced static calibration</button><button class="sec" type="button" data-cal="gyroRevoke" data-lock="e">Revoke</button></div>
  <p class="note">Forced static: the TM171 decides when it has enough data. Revoke: back to the values before the forced calibration.</p>
  <label class="opt"><span>Accepted gyro error<small>Larger errors are not compensated: the calibration fails.</small></span><span class="pair"><input type="number" id="tcLimit" min="0.05" max="5" step="0.05" value="0.50"> °/s</span></label>
  <h3>Boot calibration (new install, CalibB)</h3>
  <div class="row"><button class="sec" type="button" data-cal="install" data-lock="m">Boot calibration</button></div>
  <p class="note">Saved in the TM171's flash. Once after installing it, or after a large temperature change. About 20 s with the tractor parked; it cannot be stopped. Do not cut the power while it runs (SYD: this can damage the TM171). Then switch the whole module off and on.</p>
  <details><summary>Magnetometer and heading</summary>
   <p class="note">Only useful with Enable Mag on (AgOpenGPS normally runs with it off).</p>
   <h3>Hard-iron calibration</h3>
   <div class="row"><button class="sec" type="button" data-cal="mag" data-lock="e">Start (save to flash)</button><button class="sec" type="button" data-cal="magTmp" data-lock="e">Start (this power-on)</button></div>
   <p class="note">Then turn the module through every direction (drive slow circles) until it reports done.</p>
   <div class="row"><button class="sec" type="button" data-cal="magClear">Clear (flash)</button><button class="sec" type="button" data-cal="magClearTmp">Clear (this power-on)</button></div>
   <h3>Heading to magnetic north</h3>
   <div class="row"><span class="pair">Collect for <input type="number" id="tcHeadT" min="0" max="90" step="1" value="8"> s</span><button class="sec" type="button" data-cal="headStart" data-lock="e">Start</button><button class="sec" type="button" data-cal="headStop">Stop and apply</button></div>
   <p class="note">0 s: until Stop. Keep roll and pitch near zero and turn around the vertical axis while it collects (3 – 8 s). This power-on only.</p>
   <h3>Set heading</h3>
   <div class="row"><span class="pair"><input type="number" id="tcHead" min="0" max="360" step="0.1" value="0"> °</span><button class="sec" type="button" data-cal="headSet" data-lock="e">Set heading</button></div>
  </details>
 </div>
</div>
<div class="card"><h2>TM171 settings</h2>
 <p id="tsMsg" class="note"></p>
 <div class="cols" id="tsForm">
  <fieldset><legend>Sensors</legend>
   <label class="opt"><input type="checkbox" id="tsGyro"><span>Enable Gyro</span></label>
   <label class="opt"><input type="checkbox" id="tsAcc"><span>Enable Accel</span></label>
   <label class="opt"><input type="checkbox" id="tsMag"><span>Enable Mag<small>Off for AgOpenGPS: the tractor's steel and electronics disturb it.</small></span></label>
  </fieldset>
  <fieldset><legend>Sensor fusion</legend>
   <label class="opt"><input type="checkbox" id="tsGyroErr"><span>GyroErrFilter<small>Compensates the gyro error while running. Keep the accelerometer gain at 2.00 or more with it (2.50 or more with strong vibration).</small></span></label>
   <label class="opt"><span>Accelerometer gain</span><input type="number" id="tsAccG" min="0.01" max="20" step="0.01"></label>
   <label class="opt"><input type="checkbox" id="tsSelf"><span>Self-Adapt filter<small>Compensates magnetic disturbances for the magnetometer heading.</small></span></label>
   <label class="opt"><span>Magnetometer gain</span><input type="number" id="tsMagG" min="0.01" max="20" step="0.01"></label>
  </fieldset>
  <fieldset><legend>Communication</legend>
   <label class="opt"><span>Power-on silent time<small>Seconds after power-on without data (requests are still answered). Keep 0: the Teensy looks for the TM171 a few seconds after power-on and runs without it if it is silent then.</small></span><span class="pair"><input type="number" id="tsSilent" min="0" max="65" step="1"> s</span></label>
   <label class="opt" id="tsOdrR"><span>Output data rate</span><select id="tsOdr"><option value="25">25 Hz</option><option value="50">50 Hz</option><option value="100">100 Hz</option></select></label>
   <label class="opt"><input type="checkbox" id="tsStatus"><span>Status output<small>Temperature and QoS (the GPS LEDs and the "IMU not ready" message use the QoS).</small></span></label>
   <label class="opt"><span>Boot mode<small>Takes effect after a power cycle, and only when saved to flash.</small></span><select id="tsBoot"><option value="0">Auto</option><option value="1">Static</option><option value="2">Dynamic</option></select></label>
   <dl><dt>UART</dt><dd id="tsBaud">–</dd><dt>Inhibit time</dt><dd id="tsInhibit">–</dd><dt>Roll-pitch-yaw output</dt><dd>always on</dd></dl>
  </fieldset>
 </div>
 <label class="opt"><input type="checkbox" id="tsSave" checked><span>Save to the TM171's flash<small>Off: only until the TM171 is powered off (safer to try settings). Keep the power steady while saving.</small></span></label>
 <div class="row"><button class="sec" type="button" id="tsRead">Read from TM171</button><button class="sec" type="button" id="tsAog">AgOpenGPS defaults</button><button class="act" type="button" id="tsWrite" data-lock="e">Write to TM171</button></div>
 <p class="note mono" id="tsRaw"></p>
</div>
</div>
</section>

<section id="t-firmware" class="hidden">
<div class="grid">
 <div class="card"><h2>Firmware update</h2>
  <p class="note">The .hex file from the Arduino IDE (Sketch → Export Compiled Binary). The Teensy checks the whole file before it replaces the running firmware; the settings in the EEPROM stay. Autosteer must be off and the tractor stopped. Steering is off and the hydraulic outputs are released while it runs.</p>
  <div class="row"><label class="act">Choose .hex<input type="file" id="hexFile" accept=".hex" class="hidden"></label><span id="hexName" class="note">no file</span></div>
  <p id="hexInfo" class="note hidden"></p>
  <label class="opt"><input type="checkbox" id="force"><span>Other firmware<small>Install it even without this web page inside (the next update will then have to be by USB).</small></span></label>
  <div class="row"><button class="act" id="install" data-lock="m" data-off="1" disabled>Install</button></div>
  <progress id="prog" max="100" value="0" class="hidden"></progress>
  <p id="fwMsg" class="note"></p>
 </div>
 <div class="card"><h2>Restart</h2>
  <p class="note">Restarts the Teensy (a few seconds). Steering is switched off first.</p>
  <div class="row"><button class="danger" id="reboot" data-lock="e">Restart Teensy</button></div>
  <dl><dt>Running</dt><dd id="fRun">–</dd><dt>Web requests / errors / dropped</dt><dd id="fWeb">–</dd></dl>
 </div>
</div>
</section>

<section id="t-log" class="hidden"><div id="log"></div></section>
</main>
<script>
"use strict";
const $=id=>document.getElementById(id);
function set(id,t,c){const e=$(id);if(!e)return;e.textContent=t;if(c!==undefined)e.className=c;}
function pill(id,t,c){const e=$(id);if(!e)return;e.textContent="";const s=document.createElement("span");s.className="pill "+(c||"");s.textContent=t;e.appendChild(s);}
function vis(id,on){const e=$(id);if(e)e.classList.toggle("hidden",!on);}
async function getJ(u,o){const r=await fetch(u,Object.assign({cache:"no-store"},o||{}));return r.json();}
function post(u,b){return getJ(u,{method:"POST",headers:{"Content-Type":"application/x-www-form-urlencoded"},body:b});}
function fx(v,d){return (v===undefined||v===null)?"–":Number(v).toFixed(d);}
function dur(s){const d=Math.floor(s/86400),h=Math.floor(s%86400/3600),m=Math.floor(s%3600/60),x=s%60;return (d?d+"d ":"")+(d||h?h+"h ":"")+(d||h||m?m+"m ":"")+x+"s";}
function ago(s){return s<0?"never":s<90?s+" s ago":s<5400?Math.round(s/60)+" min ago":Math.round(s/3600)+" h ago";}
function pn(v){return v?"Serial"+v:"none";}

// ---- theme
(function(){let t=null;try{t=localStorage.getItem("theme");}catch(e){}if(t)document.documentElement.dataset.theme=t;
 $("theme").addEventListener("click",()=>{const cur=document.documentElement.dataset.theme||(matchMedia("(prefers-color-scheme: light)").matches?"light":"dark");
  const n=cur==="light"?"dark":"light";document.documentElement.dataset.theme=n;try{localStorage.setItem("theme",n);}catch(e){}});})();

// ---- tabs
let tab="status";
function show(t){tab=t;for(const b of $("nav").querySelectorAll("button"))b.classList.toggle("on",b.dataset.t===t);
 for(const s of document.querySelectorAll("main>section"))s.classList.toggle("hidden",s.id!=="t-"+t);
 try{sessionStorage.setItem("tab",t);}catch(e){}if(t==="log")$("log").scrollTop=$("log").scrollHeight;
 if(t==="gnss")setTimeout(pollGnss,0);}
$("nav").addEventListener("click",e=>{const b=e.target.closest("button");if(b)show(b.dataset.t);});
try{const h=location.hash.slice(1),t=$("t-"+h)?h:sessionStorage.getItem("tab");if(t&&$("t-"+t))show(t);}catch(e){}   /* #imu etc. opens that tab */

// ---- status
const FIX={"0":"no fix","1":"GPS","2":"DGPS","3":"PPS","4":"RTK fixed","5":"RTK float","6":"dead reckoning","7":"manual","8":"simulation"};
const LIFT={0:"–",1:"lowered",2:"raised"},HMODE={0:"0 – off",1:"1 – relay held",2:"2 – timed"};
let lastS=null,waitingBoot=false,busy=false;
function setOnline(on){$("dot").classList.toggle("on",on);if(!on)set("hdr","no connection to the module");}
function renderAlerts(list){const box=$("alerts");box.textContent="";for(const t of list){const d=document.createElement("div");d.className="alert";d.textContent=t;box.appendChild(d);}}
// buttons the module refuses now: data-lock="e" while autosteer is engaged, "m" also while the tractor moves
function locks(s){$("install").dataset.off=($("hexFile").files[0]&&!busy)?"":"1";
 for(const b of document.querySelectorAll("[data-lock]")){const l=b.dataset.lock==="e"?s.engaged:(s.engaged||s.moving);b.dataset.locked=l?"1":"";b.disabled=l||b.dataset.off==="1";}}
function kickTxt(st){
 if(!st.kick)return "none set in AgOpenGPS";
 if(st.kick==="encoder")return "encoder: "+st.pulses+" / "+st.kickMax+" pulses";
 return (st.kick==="pressure"?(st.remoteSensor?"remote pulses / s":st.sasa?"SASA":st.fema?"FEMA":"pressure"):"current")+": "+st.sensor+" / "+st.kickMax;}
function render(s){
 lastS=s;
 const i=s.imu,g=s.gps,st=s.steer,p=s.ports;
 set("hdr",s.ver+" · GNSS "+pn(p.gnss)+" · IMU "+(i.type||"none")+" · "+s.ip);
 const al=[];
 if(s.engaged)al.push("Autosteer is engaged: serial ports, steering, FEMA, remote and CAN inputs, GNSS commands, IMU choice, TM171 writes and calibration, restart and firmware update are locked.");
 else if(s.moving)al.push("The tractor is moving ("+fx(s.kmh,1)+" km/h): the firmware update and the gyro / new install calibrations are locked.");
 if(!st.ads)al.push("No ADS1115 found at start-up: the module runs as GPS only, without steering.");
 const mo=s.motor||{};if(mo.type&&(!mo.found||mo.ago>500))al.push(mo.name+" motor: no data on the CAN bus (CAN3)"+(mo.found?".":", trying "+mo.kbps+" kbit/s."));
 if(g.pass)al.push("u-blox set-up through USB is running (!AOGR): the GPS data go to USB, not to AgIO.");
 if(i.set&&i.set.noWire)al.push("TM171 does not answer: wire Teensy TX"+p.imu+" (pin "+p.imuTx+") to the TM171 RXD to use its settings and calibration. Roll and heading still work.");
 renderAlerts(al);
 pill("sMode",st.ads?"steering":"GPS only (no ADS1115)",st.ads?"ok":"warn");
 pill("sEng",s.engaged?"steering":"off",s.engaged?"ok":"");
 set("sAngle",fx(st.angle,2)+"°");set("sSet",fx(s.aog.setAngle,2)+"°");set("sWas",String(st.wasRaw));
 set("sPwm",st.pwm+" / "+st.driver);
 set("sSw",(st.steerSw?"on":"off")+" / "+(st.work?"on":"off")+" / "+(st.remoteSensor?"pulse sensor":st.remote?"on":"off"));
 set("sKick",kickTxt(st));
 pill("aSeen",s.aog.seen?"connected":(s.aog.ago<0?"not heard yet":"silent, last "+ago(s.aog.ago)),s.aog.seen?"ok":"bad");
 set("aKmh",fx(s.aog.kmh,1)+" km/h");set("aGuid",(s.aog.guidance&1)?"on":"off");
 set("gPorts","GNSS "+pn(p.gnss)+" · dual "+pn(p.gnss2)+(p.auto?" (Auto)":""));
 pill("gBps",g.bps?g.bps+" B/s":"no data",g.bps?"ok":"bad");
 set("gnPort",pn(p.gnss)+(p.auto?" (Auto)":""));pill("gnBps",g.bps?g.bps+" B/s":"no data",g.bps?"ok":"bad");
 set("gNtrip",g.ntripBps?g.ntripBps+" B/s":"–");set("gRadio",p.radio?pn(p.radio)+(g.radioBps?", "+g.radioBps+" B/s":", no data"):"none");
 if(g.gga){pill("gFix",FIX[g.fix]||g.fix,g.fix==="4"?"ok":g.fix==="0"?"bad":"warn");set("gSats",g.sats+" / "+g.hdop);set("gAge",g.age?g.age+" s":"–");}
 else pill("gFix","no GGA from the receiver","bad");
 vis("gSatsR",g.gga);vis("gAgeR",g.gga);
 vis("gDualR",g.dual);vis("gBaseR",g.dual);vis("gKsxtR",g.ksxt);
 if(g.dual){set("gDual",fx(g.heading,1)+"° / "+fx(g.roll,2)+"°");set("gBase",fx(g.baseline,1)+" cm");}
 set("iType",i.type?i.type+(i.bnoFound&&i.tmFound?" (both found)":"")+(g.dual?" – dual used for heading and roll":""):(g.dual?"none – dual heading and roll":"none found at start-up"));
 vis("iRPR",!!i.type);vis("iYawR",!!i.type);vis("iRateR",!!i.tmFound);
 if(i.type){set("iRP",fx(i.roll,2)+"° / "+(i.pitch===undefined?"–":fx(i.pitch,2)+"°"));set("iYaw",fx(i.yaw,1)+"°");}
 if(i.tmFound)pill("iRate",i.rate+" Hz (TM171)",i.fresh&&i.rate?"ok":"bad");
 const h=s.hyd;pill("hOn",h.on?"on":"off",h.on?"ok":"");vis("hStR",h.on);vis("hModeR",h.on);
 if(h.on){set("hSt",(LIFT[h.lift]||h.lift)+" / "+(h.tram?"on ("+h.tram+")":"off"));set("hMode",(HMODE[h.mode]||h.mode)+(h.mode===2?", raise "+h.raise+" s, lower "+h.lower+" s":""));}
 set("mFw",s.ver);set("mBuilt",s.built);set("mUp",dur(s.up));
 set("mIp",s.ip+" / "+(s.link?"link up":"no link"));set("mCpu",fx(s.cpuC,1)+" °C");
 set("fRun",s.ver+", built "+s.built);set("fWeb",s.web.req+" / "+s.web.err+" / "+s.web.drop);
 renderKick(st);
 renderMotor(s);
 renderCan(s);
 renderImu(s);
 cfgUi();
 locks(s);
}
function reloadAfterRestart(msg){try{sessionStorage.setItem("tab",tab);if(msg)sessionStorage.setItem("fwMsg",msg);}catch(e){}location.reload();}
async function pollStatus(){
 try{const s=await getJ("/api/status");
  if(lastS&&(s.up+3<lastS.up||s.built!==lastS.built)){reloadAfterRestart(waitingBoot?"Updated. Running: "+s.ver+", built "+s.built+".":"");return;}
  setOnline(true);render(s);
 }catch(e){setOnline(false);}
}

// ---- Setup: serial ports (zBoard.ino)
const PORTS=[[1,"0/1"],[2,"7/8"],[3,"14/15"],[5,"20/21"],[7,"28/29"]];
for(const sel of document.querySelectorAll("select[data-ports]"))for(const [n,pins] of PORTS){const o=document.createElement("option");o.value=n;o.textContent="Serial"+n+" (pins "+pins+")";sel.appendChild(o);}
let cfg=null,cfgDirty=false;
const PORT_IDS=["pGnss","pGnss2","pImu","pRadio"],PORT_KEYS=["gnssPort","gnss2Port","imuPort","radioPort"];
function portsOf(){return PORT_IDS.map(id=>+$(id).value);}
function portsChanged(){return !!cfg&&portsOf().some((v,i)=>v!==cfg[PORT_KEYS[i]]);}
function portsSet(g,g2,i,r){$("pGnss").value=String(g);$("pGnss2").value=String(g2);$("pImu").value=String(i);$("pRadio").value=String(r);cfgDirty=true;gnssUi();cfgUi();set("cMsg","");}
function fillCfg(j){PORT_IDS.forEach((id,k)=>{$(id).value=String(j[PORT_KEYS[k]]);});cfgDirty=false;gnssUi();cfgUi();}
async function loadCfg(){try{const j=await getJ("/api/config");if(j.ok){cfg=j;if(!cfgDirty)fillCfg(j);}}catch(e){}}
// dual receiver: Auto only together with the GNSS on Auto
function gnssUi(){const auto=$("pGnss").value==="255",g2=$("pGnss2");
 g2.options[0].disabled=!auto;if(auto)g2.value="255";else if(g2.value==="255")g2.value="0";g2.disabled=auto;}
function portsWhy(){const [g,g2,i,r]=portsOf(),fixed=[g,g2,i,r].filter(v=>v!==255&&v!==0);
 if(new Set(fixed).size!==fixed.length)return "Each device needs its own serial port.";
 if(g===255&&[i,r].some(v=>v===2||v===7))return "With the GNSS port on Auto, Serial2 and Serial7 are kept for the receivers.";
 return "";}
function cfgUi(){const ch=portsChanged();$("cUndo").disabled=!ch;
 $("cSave").dataset.off=ch?"":"1";$("cSave").disabled=!ch||$("cSave").dataset.locked==="1";}
for(const id of PORT_IDS)$(id).addEventListener("change",()=>{cfgDirty=true;if(id==="pGnss")gnssUi();cfgUi();set("cMsg","");});
$("pAuto").addEventListener("click",()=>portsSet(255,255,255,3));
$("cUndo").addEventListener("click",()=>{if(cfg)fillCfg(cfg);set("cMsg","");});
$("cSave").addEventListener("click",async()=>{
 const why=portsWhy();if(why){set("cMsg",why,"note bad");return;}
 const [g,g2,i,r]=portsOf(),nm=v=>v===255?"Auto":pn(v);
 if(!confirm("Save the serial ports?\n\nGNSS "+nm(g)+", dual "+nm(g2)+", TM171 "+nm(i)+", RTK radio "+nm(r)+"\n\nThe Teensy restarts to take them (a few seconds)."))return;
 try{const j=await post("/api/config","gnssPort="+g+"&gnss2Port="+g2+"&imuPort="+i+"&radioPort="+r);
  if(!j.ok){set("cMsg","Not saved: "+j.error,"note bad");return;}
  set("cMsg",j.restart?"Saved. The Teensy is restarting…":"Nothing changed.","note ok");if(j.restart)waitingBoot=true;
 }catch(e){set("cMsg","Cannot reach the module.","note bad");}});

// ---- Setup: pressure sensor input, John Deere FEMA (Autosteer.ino)
function renderKick(st){
 const sw=$("femaSw");if(!sw.dataset.busy)sw.checked=!!st.fema;
 $("swAna").className=st.fema?"":"on";$("swFema").className=st.fema?"on":"";
 set("kSet",st.kick==="pressure"?"pressure sensor, kickout "+st.kickMax:st.kick?st.kick+" (not the pressure sensor)":"off");
 set("kVal",st.kick==="pressure"?st.sensor+" / "+st.kickMax:"–");
 set("kSrc",st.remoteSensor?"remote input, pulses / s":st.sasa?"CAN bus, SASA wheel speed":st.fema?"pressure input, FEMA":"pressure input, analog");
 vis("kUsR",st.fema&&!st.remoteSensor&&!st.sasa);if(st.fema)set("kUs",st.femaUs>0?st.femaUs+" µs":"no signal");
 const rs=$("remoteSw");if(!rs.dataset.busy)rs.checked=!!st.remoteSensor;
 $("swRsw").className=st.remoteSensor?"":"on";$("swRpl").className=st.remoteSensor?"on":"";
 vis("rHzR",st.remoteSensor);if(st.remoteSensor)set("rHz",String(st.remoteHz));}
$("femaSw").addEventListener("change",async()=>{
 const sw=$("femaSw"),on=sw.checked;
 if(!confirm(on?"Read the pressure sensor input as a John Deere FEMA valve (PWM signal)?":"Read the pressure sensor input as an analog pressure sensor?")){sw.checked=!on;return;}
 sw.dataset.busy="1";
 try{const j=await post("/api/config","fema="+(on?1:0));
  if(!j.ok){set("femaMsg","Not changed: "+j.error,"note bad");sw.checked=!on;}else set("femaMsg","Saved: "+(on?"John Deere FEMA valve.":"analog pressure sensor."),"note ok");
 }catch(e){set("femaMsg","Cannot reach the module.","note bad");sw.checked=!on;}
 delete sw.dataset.busy;pollStatus();});
$("remoteSw").addEventListener("change",async()=>{
 const sw=$("remoteSw"),on=sw.checked;
 if(!confirm(on?"Use the remote input as a pulse sensor (pulses / s sent as the pressure sensor value)?":"Use the remote input as the remote switch?")){sw.checked=!on;return;}
 sw.dataset.busy="1";
 try{const j=await post("/api/config","remote="+(on?1:0));
  if(!j.ok){set("remoteMsg","Not changed: "+j.error,"note bad");sw.checked=!on;}else set("remoteMsg","Saved: "+(on?"pulse sensor.":"remote switch."),"note ok");
 }catch(e){set("remoteMsg","Cannot reach the module.","note bad");sw.checked=!on;}
 delete sw.dataset.busy;pollStatus();});

// ---- Setup: steering, hydraulic or a direct drive motor (zMotor.ino)
const MOTORS={1:"Keya",2:"Allynav",3:"FJD"};
let mDirty=false;
function motorSel(){return $("motorSw").checked?+$("motorType").value:0;}
function motorUi(){const dd=$("motorSw").checked;vis("motorTypeR",dd);
 $("swHyd").className=dd?"":"on";$("swDd").className=dd?"on":"";
 const ch=!!lastS&&!!lastS.motor&&motorSel()!==lastS.motor.type,b=$("mSave");
 b.dataset.off=ch?"":"1";b.disabled=!ch||b.dataset.locked==="1";}
function renderMotor(s){const m=s.motor||{type:0};
 if(!mDirty){$("motorSw").checked=m.type>0;if(m.type>0)$("motorType").value=String(m.type);}
 motorUi();vis("mStat",m.type>0);if(!m.type)return;
 const ok=m.found&&m.ago>=0&&m.ago<=500;
 pill("mFound",ok?m.name+", "+m.kbps+" kbit/s":m.found?"no data for "+ago(Math.round(m.ago/1000)):"no data yet"+(m.type===3?"":", trying "+m.kbps+" kbit/s"),ok?"ok":"bad");
 vis("mAmpR",m.amps!==undefined);
 if(m.amps!==undefined)set("mAmp",fx(m.amps,1)+" A, peak "+fx(m.peak,1)+" A (5 s) – kickout value "+m.kick+" ("+Math.round(m.kick/2.55)+" %)");
 vis("mStateR",!!m.state);if(m.state)set("mState",m.state);}
$("motorSw").addEventListener("change",()=>{mDirty=true;motorUi();set("mMsg","");});
$("motorType").addEventListener("change",()=>{mDirty=true;motorUi();set("mMsg","");});
$("mSave").addEventListener("click",async()=>{
 const v=motorSel(),nm=v?MOTORS[v]+" direct drive motor":"hydraulic valve / PWM driver";
 if(!confirm("Steering: "+nm+"?\n\nThe Teensy restarts to take it (a few seconds)."))return;
 try{const j=await post("/api/config","motor="+v);
  if(!j.ok){set("mMsg","Not saved: "+j.error,"note bad");return;}
  set("mMsg",j.restart?"Saved. The Teensy is restarting…":"Nothing changed.","note ok");if(j.restart)waitingBoot=true;
 }catch(e){set("mMsg","Cannot reach the module.","note bad");}});

// ---- Setup: CAN bus, read only: SASA sensor, CNH engage button (zCan.ino)
const CANBTN={0:"off",1:"armrest button",2:"hydraulic joystick up",3:"both"};
function renderCan(s){
 const c=s.can||{},sw=$("sasaSw"),dd=!!(s.motor&&s.motor.type);if(!sw.dataset.busy)sw.checked=!!c.sasa;
 sw.dataset.off=dd?"1":"";$("canBtn").dataset.off=dd?"1":"";        // the CAN bus is the motor's
 vis("canDdNote",dd);if(dd)set("canDdNote","CAN3 is used by the "+s.motor.name+" motor (Steering): no SASA sensor or engage button.");
 $("swSasaOff").className=c.sasa?"":"on";$("swSasa").className=c.sasa?"on":"";
 const sel=$("canBtn");if(!sel.dataset.busy)sel.value=String(c.btn||0);
 if(c.on)pill("cFps",c.fps?c.fps+" /s":"none",c.fps?"ok":"bad");else set("cFps","bus off");
 vis("cSasaR",!!c.sasa);
 if(c.sasa)set("cSasa",c.sasaAgo<0?"no SASA frame yet":c.speed+(c.sasaAgo>1000?" (last frame "+ago(Math.round(c.sasaAgo/1000))+")":""));
 vis("cBtnR",!!c.btn);
 if(c.btn)set("cBtn",(c.btnAgo<0?"no button frame yet":c.down?"pressed":"released")+(s.steer.steerBtn?"":" – Steer button is not selected in AgOpenGPS"));
 vis("cLastR",!!c.last);if(c.last)set("cLast",c.last);}
$("sasaSw").addEventListener("change",async()=>{
 const sw=$("sasaSw"),on=sw.checked;
 if(!confirm(on?"Use the SASA steering wheel sensor on the CAN bus as the pressure sensor value?":"Stop using the SASA steering wheel sensor?")){sw.checked=!on;return;}
 sw.dataset.busy="1";
 try{const j=await post("/api/config","sasa="+(on?1:0));
  if(!j.ok){set("canMsg","Not changed: "+j.error,"note bad");sw.checked=!on;}else set("canMsg","Saved: SASA sensor "+(on?"on.":"off."),"note ok");
 }catch(e){set("canMsg","Cannot reach the module.","note bad");sw.checked=!on;}
 delete sw.dataset.busy;pollStatus();});
$("canBtn").addEventListener("change",async()=>{
 const sel=$("canBtn"),v=sel.value,old=String(lastS&&lastS.can?lastS.can.btn:0);
 if(!confirm("CNH engage button on the CAN bus: "+CANBTN[v]+"?")){sel.value=old;return;}
 sel.dataset.busy="1";
 try{const j=await post("/api/config","canBtn="+v);
  if(!j.ok){set("canMsg","Not changed: "+j.error,"note bad");sel.value=old;}else set("canMsg","Saved: engage button "+CANBTN[v]+".","note ok");
 }catch(e){set("canMsg","Cannot reach the module.","note bad");sel.value=old;}
 delete sel.dataset.busy;pollStatus();});

// ---- GNSS: commands to the receiver and its answers (zGnss.ino)
let gnAfter=0,gnBusy=false;
const GN_PRESET=["VERSIONA","CONFIG","MODE"];
function gnHist(){let h=[];try{h=JSON.parse(localStorage.getItem("gnssHist")||"[]");}catch(e){}return Array.isArray(h)?h:[];}
function gnFillHist(){const dl=$("gnHist");dl.textContent="";for(const c of new Set([...gnHist(),...GN_PRESET])){const o=document.createElement("option");o.value=c;dl.appendChild(o);}}
gnFillHist();
async function gnSend(){
 const c=$("gnCmd").value.trim();if(!c||gnBusy)return;
 if(![...c].every(ch=>ch.charCodeAt(0)>=32&&ch.charCodeAt(0)<=126)){set("gnMsg","Printable ASCII characters only (CR LF are added).","note bad");return;}
 gnBusy=true;
 try{const j=await post("/api/gnss","cmd="+encodeURIComponent(c)+"&all="+($("gnAll").checked?1:0));
  if(!j.ok)set("gnMsg","Not sent: "+j.error,"note bad");
  else{set("gnMsg","Sent to "+pn(j.port)+".","note ok");
   try{localStorage.setItem("gnssHist",JSON.stringify([c,...gnHist().filter(x=>x!==c)].slice(0,15)));}catch(e){}
   gnFillHist();$("gnCmd").select();setTimeout(pollGnss,300);}
 }catch(e){set("gnMsg","Cannot reach the module.","note bad");}
 gnBusy=false;}
$("gnSend").addEventListener("click",gnSend);
$("gnCmd").addEventListener("keydown",e=>{if(e.key==="Enter"){e.preventDefault();if(!$("gnSend").disabled)gnSend();}});
$("gnClear").addEventListener("click",()=>{$("gnLog").textContent="";});
async function pollGnss(){
 if(tab!=="gnss")return;
 try{const j=await getJ("/api/gnss?after="+gnAfter);const box=$("gnLog"),atEnd=box.scrollTop+box.clientHeight>=box.scrollHeight-20;
  for(const l of j.lines||[]){const d=document.createElement("div");d.className="ll"+(l.tx?" tx":"");
   const t=document.createElement("span");t.className="t";t.textContent=dur(Math.floor(l.t/1000));
   const x=document.createElement("span");x.className="mono";x.textContent=l.tx?"→ "+pn(l.p)+": "+l.x:l.x;
   d.appendChild(t);d.appendChild(x);box.appendChild(d);gnAfter=l.s;}
  while(box.childNodes.length>400)box.removeChild(box.firstChild);
  if(atEnd)box.scrollTop=box.scrollHeight;
  if(j.lines&&j.lines.length&&gnAfter<j.next-1)setTimeout(pollGnss,50);
 }catch(e){}}
setInterval(pollGnss,1000);

// ---- IMU: choice and TM171 settings and calibration (zBoard.ino, TM171.ino)
let tsSeen=null,tsDirty=false,tsErrUntil=0,calErrUntil=0;
function tsSay(t,c){set("tsMsg",t,"note "+(c||""));tsErrUntil=Date.now()+6000;}
function calSay(t,c){set("tcMsg",t,"note "+(c||""));calErrUntil=Date.now()+6000;}
function odrSet(v){vis("tsOdrR",!!v);if(!v)return;const sel=$("tsOdr");
 if(![...sel.options].some(o=>+o.value===v)){const o=document.createElement("option");o.value=v;o.textContent=v+" Hz (as read)";sel.appendChild(o);}sel.value=String(v);}
function tsFill(s){$("tsGyro").checked=!!s.gyro;$("tsAcc").checked=!!s.acc;$("tsMag").checked=!!s.mag;$("tsGyroErr").checked=!!s.gyroErr;$("tsSelf").checked=!!s.selfAdapt;
 $("tsStatus").checked=!!s.status;$("tsBoot").value=String(s.boot||0);$("tsAccG").value=(s.accGain/100).toFixed(2);$("tsMagG").value=(s.magGain/100).toFixed(2);
 $("tsSilent").value=s.silent||0;if(s.odr!==undefined)odrSet(s.odr);tsDirty=false;}
$("tsForm").addEventListener("input",()=>{tsDirty=true;});
function renderImuSel(i){
 const both=i.bnoFound&&i.tmFound,sw=$("imuSw");
 if(!sw.dataset.busy)sw.checked=i.type==="TM171";
 sw.dataset.off=both?"":"1";sw.disabled=!both||sw.dataset.locked==="1";
 $("swBno").className=i.type==="BNO08x"?"on":"";$("swTm").className=i.type==="TM171"?"on":"";
 set("imuSelTxt",both?"Both found. Roll, pitch and heading to AgOpenGPS come from the "+(i.type==="TM171"?"TM171":"BNO085")+"; the other keeps running (the TM171's settings and calibration stay usable). The choice is saved and taken at once; it can only be changed with autosteer off."
  :i.type?"Only the "+i.type+" was found at start-up: nothing to choose.":"No IMU was found at start-up.");}
$("imuSw").addEventListener("change",async()=>{
 const sw=$("imuSw"),to=sw.checked?"tm171":"bno";
 if(!confirm("Use the "+(sw.checked?"TM171":"BNO085")+" for roll, pitch and heading?\n\nAgOpenGPS gets the other IMU's values at once: check roll and heading afterwards.")){sw.checked=!sw.checked;return;}
 sw.dataset.busy="1";
 try{const j=await post("/api/imu","act=select&imu="+to);if(!j.ok){alert("Not changed: "+j.error);sw.checked=!sw.checked;}}catch(e){sw.checked=!sw.checked;}
 delete sw.dataset.busy;pollStatus();});
function renderImu(s){
 const i=s.imu,tm=!!i.tmFound;vis("imuTm",tm);vis("imuNone",!tm);renderImuSel(i);
 if(!tm){set("imuNoneTxt",i.type?i.type+" on I2C: nothing to set up here. The settings and calibrations on this tab are for the TM171."
  :"No IMU was found at start-up. The TM171 (Serial7 / Serial5 on Auto, or the port set in Setup; 115200), the BNO08x (I2C 0x4A / 0x4B) or the CMPS14 (I2C 0x60) is looked for once, when the Teensy starts.");return;}
 set("tRP",fx(i.tmRoll,2)+"° / "+fx(i.tmPitch,2)+"°");set("tYaw",fx(i.tmYaw,2)+"°");
 pill("tRate",i.rate+" Hz",i.fresh&&i.rate?"ok":"bad");set("tId",String(i.id));set("tCrc",String(i.crcErr));
 vis("tTempR",i.tempC!==undefined);if(i.tempC!==undefined)set("tTemp",fx(i.tempC,1)+" °C / "+i.qos);
 set("tAxis",i.yAxis?"on":"off");
 const t=i.set;
 if(t.age<0)set("tSetup","not read");else if(t.setupWhy)pill("tSetup","differs: "+t.setupWhy,"warn");else pill("tSetup","yes","ok");
 const key=t.age>=0?[t.sw,t.accGain,t.magGain,t.silent,t.odr].join(":"):null;
 if(key!==null&&key!==tsSeen&&(!tsDirty||t.age<2)){tsSeen=key;tsFill(t);}
 if(Date.now()>tsErrUntil){let m="",c="";
  if(t.pending)m="Waiting for the TM171…";
  else if(t.odd){m="Settings received, but the layout is not recognised. Writing is disabled; use ImuAssistant, and send the bytes below to the firmware's author.";c="bad";}
  else if(t.age<0){m=t.noWire?"No reply from the TM171: wire Teensy TX"+s.ports.imu+" (pin "+s.ports.imuTx+") to the TM171 RXD.":t.noAnswer?"No reply yet, asking again…":"Not read yet.";c=t.noWire?"bad":"";}
  else{m="Read "+ago(t.age)+(t.ackAge>=0?". Last write acknowledged by the TM171 "+ago(t.ackAge):"")+".";c=t.ackAge>=0&&t.ackAge<30?"ok":"";}
  set("tsMsg",m,"note "+c);}
 set("tsBaud",t.baud?t.baud+" bps":"–");set("tsInhibit",t.inhibit!==undefined?t.inhibit+" ms":"–");
 set("tsRaw",t.hex?"Setting object as read: "+t.hex+"  (switches 0x"+(t.sw>>>0).toString(16).padStart(8,"0")+")":"");
 for(const e of $("tsForm").querySelectorAll("input,select"))e.disabled=!t.valid;
 $("tsWrite").dataset.off=(t.valid&&!t.pending)?"":"1";$("tsWrite").disabled=$("tsWrite").dataset.off==="1"||s.engaged;
 $("tsAog").disabled=!t.valid;$("tsRead").disabled=!!t.pending;
 renderCal(i.cal);
}
function renderCal(c){
 $("tcStop").classList.toggle("hidden",!(c.busy&&c.code==="gyro"));
 if(Date.now()<calErrUntil)return;
 let t,cl="";
 if(c.busy){cl="warn";
  if(c.code==="gyro"&&c.state==="running")t="Gyro calibration: collecting data, "+c.left+" s left. Keep the tractor still.";
  else if(c.state==="starting")t=c.what+": waiting for the TM171…";
  else if(c.state==="finishing")t=c.what+": finishing…";
  else t=c.what+": running"+(c.progress>=0?", "+c.progress+" %":"")+"…";}
 else if(c.state){const a=c.ago>=0?" ("+ago(c.ago)+")":"";
  const M={ok:["Done: ","ok"],failed:["FAILED: ","bad"],noanswer:["No answer from the TM171: ","bad"],aborted:["Stopped: ","warn"],revoked:["Revoked: ","ok"],sent:["Sent, no report: ",""]}[c.state]||["",""];
  t=M[0]+c.what+a+(c.state==="ok"&&c.code==="install"?". Switch the whole module off and on.":"");cl=M[1];}
 else t="No calibration since start-up.";
 set("tcMsg",t,"note "+cl);
}
async function imuPost(b,say){try{const j=await post("/api/imu",b);if(!j.ok)say("Error: "+j.error,"bad");return j.ok;}catch(e){say("Cannot reach the module.","bad");return false;}}
$("tsRead").addEventListener("click",()=>{tsDirty=false;tsSeen=null;imuPost("act=setread",tsSay);});
$("tsAog").addEventListener("click",()=>{const cur=lastS&&lastS.imu.set||{};
 tsFill({gyro:1,acc:1,mag:0,gyroErr:1,selfAdapt:1,status:1,boot:0,accGain:350,magGain:100,silent:0,odr:cur.odr?25:0});tsDirty=true;
 tsSay("AgOpenGPS set-up loaded into the form: gyro and accelerometer on, magnetometer off, GyroErrFilter with gain 3.50, Self-Adapt with gain 1.00, status output, silent time 0, 25 Hz, Auto boot. Click Write to TM171 to apply.","warn");});
$("tsWrite").addEventListener("click",async()=>{
 const n=id=>Math.round(parseFloat($(id).value)*100),acc=n("tsAccG"),mag=n("tsMagG"),sil=Math.round(+$("tsSilent").value);
 if(!(acc>=1&&acc<=2000&&mag>=1&&mag<=2000)){tsSay("Gains: 0.01 – 20.00","bad");return;}
 if(!(sil>=0&&sil<=65)){tsSay("Power-on silent time: 0 – 65 s","bad");return;}
 let q="Write these settings to the TM171"+($("tsSave").checked?" and save them to its flash":" (until it is powered off)")+"?";
 if(!$("tsGyro").checked||!$("tsAcc").checked)q+="\n\nWARNING: with the gyro or the accelerometer off the TM171 cannot give a usable roll.";
 if($("tsGyroErr").checked&&acc<200)q+="\n\nSYD recommends an accelerometer gain of 2.00 or more with GyroErrFilter on.";
 if(sil>0)q+="\n\nWith a silent time above 0 the Teensy may not find the TM171 when both start together, and then runs without it.";
 if(!confirm(q))return;
 const ck=id=>$(id).checked?1:0,odr=$("tsOdrR").classList.contains("hidden")?0:+$("tsOdr").value;
 const b="act=setwrite&gyro="+ck("tsGyro")+"&acc="+ck("tsAcc")+"&mag="+ck("tsMag")+"&gyroErr="+ck("tsGyroErr")+"&selfAdapt="+ck("tsSelf")+"&status="+ck("tsStatus")
  +"&save="+ck("tsSave")+"&boot="+$("tsBoot").value+"&accGain="+acc+"&magGain="+mag+"&silent="+sil+"&odr="+odr;
 tsSeen=null;tsDirty=false;if(await imuPost(b,tsSay))tsSay("Sent. Reading back…","");});
const CAL_ASK={gyro:"Calibrate the gyro bias now?\n\nKeep the tractor parked for 20 s (the engine can run). Autosteer must be off.",
 gyroAuto:"Run the forced static gyro calibration?\n\nKeep the tractor parked until it reports done.",
 gyroRevoke:"Revoke the forced gyro calibration results?",
 install:"Run the TM171 boot calibration (new install, CalibB)?\n\nKeep the tractor parked for about 20 s and do NOT cut the power while it runs (SYD warns this can damage the TM171). It cannot be stopped once started. The result is saved to flash; switch the whole module off and on afterwards.",
 mag:"Start the magnetometer hard-iron calibration (saved to flash)?\n\nTurn the module through every direction until it reports done.",
 magTmp:"Start the magnetometer hard-iron calibration for this power-on?\n\nTurn the module through every direction until it reports done.",
 magClear:"Clear the magnetometer calibration in the TM171's flash?",magClearTmp:"Clear the magnetometer calibration for this power-on?",
 headStart:"Start collecting data for the heading to magnetic north?",headSet:"Set the TM171 heading to {h}° (this power-on only)?"};
async function calGo(c){
 let p2=0,p3=0;
 if(c==="gyro"||c==="gyroAuto"){p2=Math.round(parseFloat($("tcLimit").value)*100);if(!(p2>=5&&p2<=500)){calSay("Accepted gyro error: 0.05 – 5.00 °/s","bad");return;}}
 if(c==="headStart"){p2=Math.round(parseFloat($("tcHeadT").value)*1000);if(!(p2>=0&&p2<=90000)){calSay("Collection time: 0 – 90 s","bad");return;}}
 if(c==="headSet"){const h=parseFloat($("tcHead").value);if(!(h>=0&&h<=360)){calSay("Heading: 0 – 360°","bad");return;}p2=Math.round(h*1000);}
 const q=CAL_ASK[c];if(q&&!confirm(q.replace("{h}",(p2/1000).toFixed(1))))return;
 calErrUntil=0;await imuPost("act=cal&c="+c+"&p2="+p2+"&p3="+p3,calSay);}
for(const b of document.querySelectorAll("[data-cal]"))b.addEventListener("click",()=>calGo(b.dataset.cal));
$("tcStop").addEventListener("click",()=>calGo("gyroStop"));

// ---- firmware update (zOTA.ino)
function fwMsg(t,c){set("fwMsg",t,"note "+(c||""));}
// What is inside a chosen .hex: the data records put together, then the "[fw] Autosteer_gps_teensy_v4_1, built ..." line
// gives the build date; "fw_teensy41" says it is a Teensy 4.1 firmware, the marker that this web page is inside.
let hexBuild=null;
function hexRead(text){
 let s="";
 for(const ln of text.split("\n")){if(ln.charCodeAt(0)!==58||ln.substr(7,2)!=="00")continue;const n=parseInt(ln.substr(1,2),16);let o="";for(let i=0;i<n;i++)o+=String.fromCharCode(parseInt(ln.substr(9+2*i,2),16));s+=o;}
 const m=/\[fw\] (Autosteer_gps_teensy_v4_1), built ([A-Z][a-z]{2} [ \d]\d \d{4} \d\d:\d\d:\d\d)/.exec(s);
 return {fw:m?m[1]:"",built:m?m[2]:"",t41:s.includes("fw_teensy41"),marker:s.includes("AOG-T41-AUTOSTEER-GPS-WEB-OTA-1")};
}
$("hexFile").addEventListener("change",()=>{const f=$("hexFile").files[0];hexBuild=null;
 set("hexName",f?f.name+" ("+Math.round(f.size/1024)+" kB)":"no file");if(lastS)locks(lastS);fwMsg("");
 vis("hexInfo",!!f);if(!f)return;set("hexInfo","Reading the file…","note");
 const r=new FileReader();
 r.onload=()=>{const b=hexBuild=hexRead(r.result);let t,c="";
  if(!b.t41){t="Not a Teensy 4.1 firmware (no fw_teensy41 inside): the Teensy refuses it.";c="bad";}
  else if(!b.marker){t="Another firmware (no web update page of this firmware inside). It needs Other firmware, and the next update will be by USB.";c="warn";}
  else{const same=!!lastS&&lastS.built===b.built;t=b.fw+", built "+b.built.replace("  "," ")+(same?" (the firmware running now)":"");c=same?"":"ok";}
  set("hexInfo",t,"note "+c);};
 r.onerror=()=>set("hexInfo","The file could not be read.","note bad");
 r.readAsText(f);});
$("install").addEventListener("click",()=>{
 const f=$("hexFile").files[0];if(!f||busy)return;
 if(!f.name.toLowerCase().endsWith(".hex")){fwMsg("Choose a .hex file.","bad");return;}
 const b=hexBuild&&hexBuild.fw?" ("+hexBuild.fw+", built "+hexBuild.built.replace("  "," ")+")":"";
 if(!confirm("Install "+f.name+b+"?\n\nAutosteer must be off and the tractor stopped. Steering is off during the update; do not switch the power off until the module is back."))return;
 busy=true;$("install").disabled=true;const p=$("prog");p.classList.remove("hidden");p.value=0;fwMsg("Sending the file…");
 const x=new XMLHttpRequest();
 x.open("POST","/api/ota"+($("force").checked?"?force=1":""));
 x.setRequestHeader("Content-Type","text/plain");
 x.upload.onprogress=e=>{if(e.lengthComputable)p.value=Math.round(e.loaded/e.total*95);};
 x.upload.onload=()=>fwMsg("File sent; the Teensy is checking and writing it…");
 const fail=t=>{busy=false;if(lastS)locks(lastS);p.classList.add("hidden");fwMsg(t,"bad");};
 x.onload=()=>{let j={};try{j=JSON.parse(x.responseText);}catch(e){}
  if(x.status===200&&j.ok){p.value=100;waitingBoot=true;setOnline(false);fwMsg("Checked ("+Math.round(j.size/1024)+" kB). Installing and restarting – do not switch the power off…","warn");}
  else fail("Not installed: "+(j.error||("error "+x.status))+". The current firmware keeps running.");};
 x.onerror=()=>fail("The connection failed. The current firmware keeps running.");
 x.send(f);});
$("reboot").addEventListener("click",async()=>{
 if(!confirm("Restart the Teensy now?"))return;
 try{const j=await post("/api/reboot","");if(j.ok){waitingBoot=true;fwMsg("Restarting…","warn");}else fwMsg("Not restarted: "+j.error,"bad");}catch(e){fwMsg("Cannot reach the module.","bad");}});
try{const m=sessionStorage.getItem("fwMsg");if(m){sessionStorage.removeItem("fwMsg");fwMsg(m,"ok");}}catch(e){}

// ---- messages
let logAfter=0;
async function pollLog(){
 try{const j=await getJ("/api/log?after="+logAfter);const box=$("log"),atEnd=box.scrollTop+box.clientHeight>=box.scrollHeight-20;
  for(const l of j.lines||[]){const d=document.createElement("div");d.className="ll";const t=document.createElement("span");t.className="t";t.textContent=dur(Math.floor(l.t/1000));
   const x=document.createElement("span");x.className="mono";x.textContent=l.x;d.appendChild(t);d.appendChild(x);box.appendChild(d);logAfter=l.s;}
  while(box.childNodes.length>400)box.removeChild(box.firstChild);
  if(atEnd)box.scrollTop=box.scrollHeight;
  if(j.lines&&j.lines.length&&logAfter<j.next-1)setTimeout(pollLog,50);
 }catch(e){}
}

loadCfg();pollStatus();pollLog();
setInterval(pollStatus,1000);setInterval(pollLog,2000);
</script>
</body>
</html>
)WEBPAGE";

#endif
