// =============================================================
// REMOTE SERIAL TERMINAL - view/send raw bytes on Serial2/3/5/7 over the web UI
//
// Lets you check what's actually coming out of a sensor's wiring (e.g. is anything toggling
// on TM171's TX pin at all) without physical USB access, and send text to a port directly -
// useful for probing a device or replaying the existing serial-menu commands (zAutoZeroMenu.ino's
// 'z' menu, zHandlers.ino's EY/ER/EP/ES) remotely instead of only from a USB serial monitor.
//
// Named to sort after zWebConfig.ino (compiled later in the concatenated sketch) so the
// static helpers it reuses (sendOK/sendHead/sendNav/sendRedirect/extractFloat) are already
// defined - avoids relying on Arduino's cross-file auto-prototyping for those calls.
//
// Tapping: rather than owning these ports outright, this taps the bytes at the points the
// firmware already reads them (GPS/GPS2/RTK ingestion in the main .ino, TM171 in TM171.ino -
// each just gained one extra termTapXxx() call) into a small per-port ring buffer here. The
// real parsing logic is completely untouched. Reads and writes to these buffers only ever
// happen from the single-threaded main loop, so no locking is needed.
//
//   GET  /terminal        the page
//   GET  /terminal/data   ?port=N&since=N - new bytes as hex, polled every ~500ms
//   POST /terminal/send   port=N&text=...&newline=0|1 - writes text (URL-decoded) to a port
//   POST /terminal/baud   port=N&baud=N - re-begin()s a port at a different baud rate
//                         (disrupts normal use of that port until changed back or rebooted)
// =============================================================

#ifdef ARDUINO_TEENSY41

#define TERM_BUF_SIZE 2048

struct TermRingBuf {
  uint8_t  buf[TERM_BUF_SIZE];
  uint32_t totalBytes = 0; // monotonic count of every byte ever tapped, never resets except on reboot
};

static TermRingBuf termBufSerial2, termBufSerial3, termBufSerial5, termBufSerial7;

// Explicit prototypes, positioned after TermRingBuf is defined but before any use - same
// fix, same reason, as the calcChecksum()/OtaStageResult cases elsewhere in this project:
// Arduino's auto-prototype generator hoists a prototype for every function to the very top
// of the concatenated sketch, before this struct would exist yet, which fails to compile.
static void termTap(TermRingBuf &rb, uint8_t b);
static TermRingBuf *termBufForSelector(uint8_t sel);

static void termTap(TermRingBuf &rb, uint8_t b)
{
  rb.buf[rb.totalBytes % TERM_BUF_SIZE] = b;
  rb.totalBytes++;
}

// Called from the existing GPS/GPS2/TM171 read sites - resolves whichever physical port is
// actually being read right now, regardless of current role assignment (auto-detected or
// manually pinned in Board Setup), so the terminal always reflects real wiring.
void termTapPort(HardwareSerial *port, uint8_t b)
{
  if      (port == &Serial2) termTap(termBufSerial2, b);
  else if (port == &Serial5) termTap(termBufSerial5, b);
  else if (port == &Serial7) termTap(termBufSerial7, b);
}

// Serial3/RTK's role is fixed (unlike Serial2/5/7), so no pointer indirection is needed.
void termTapSerial3(uint8_t b)
{
  termTap(termBufSerial3, b);
}

static TermRingBuf *termBufForSelector(uint8_t sel)
{
  switch (sel) {
    case 1: return &termBufSerial2;
    case 2: return &termBufSerial3;
    case 3: return &termBufSerial5;
    case 4: return &termBufSerial7;
    default: return NULL;
  }
}

static HardwareSerial *termPortForSelector(uint8_t sel)
{
  switch (sel) {
    case 1: return &Serial2;
    case 2: return &Serial3;
    case 3: return &Serial5;
    case 4: return &Serial7;
    default: return NULL;
  }
}

static const char *termLabelForSelector(uint8_t sel)
{
  switch (sel) {
    case 1: return "Serial2";
    case 2: return "Serial3 (RTK)";
    case 3: return "Serial5";
    case 4: return "Serial7";
    default: return "?";
  }
}

// Describes what this port is currently doing, for display next to its name in the picker.
static String termCurrentRoleDescription(HardwareSerial *port)
{
  String s = "";
  if (port == SerialGPS)  s += "GPS1 ";
  if (port == SerialGPS2) s += "GPS2 ";
  if (useTM171 && port == SerialImu) s += "TM171 ";
  if (port == &Serial3) s += "RTK ";
  s.trim();
  return (s.length() == 0) ? "unused" : s;
}

// Minimal application/x-www-form-urlencoded value decoder (+  -> space, %XX -> byte). Only
// used here - the other POST handlers in this project only ever parse numeric fields.
static String termUrlDecodeValue(const String &body, const char *key)
{
  String k = String(key) + "=";
  int idx = body.indexOf(k);
  if (idx < 0) return "";
  idx += k.length();
  int end = body.indexOf('&', idx);
  String raw = (end < 0) ? body.substring(idx) : body.substring(idx, end);

  String out;
  out.reserve(raw.length());
  for (unsigned int i = 0; i < raw.length(); i++)
  {
    char c = raw[i];
    if (c == '+')
    {
      out += ' ';
    }
    else if (c == '%' && i + 2 < raw.length())
    {
      char hex[3] = { raw[i + 1], raw[i + 2], 0 };
      out += (char)strtol(hex, NULL, 16);
      i += 2;
    }
    else
    {
      out += c;
    }
  }
  return out;
}

// Extracts the query string (between '?' and the next space) from an HTTP request line, so
// GET /terminal/data?port=1&since=42 can be parsed with the same extractFloat() used for
// POST bodies elsewhere - the key=value&key2=value2 shape is identical either way.
static String termQueryString(const String &requestLine)
{
  int q = requestLine.indexOf('?');
  if (q < 0) return "";
  int end = requestLine.indexOf(' ', q);
  return (end < 0) ? requestLine.substring(q + 1) : requestLine.substring(q + 1, end);
}

static void sendTerminalData(EthernetClient &c, uint8_t sel, uint32_t since)
{
  TermRingBuf *rb = termBufForSelector(sel);
  sendOK(c, "application/json");
  if (!rb)
  {
    c.print("{\"hex\":\"\",\"next\":"); c.print(since); c.println("}");
    return;
  }

  uint32_t total = rb->totalBytes;
  uint32_t start;
  if (since > total)                    start = total;              // e.g. board rebooted since the client last polled
  else if (total - since > TERM_BUF_SIZE) start = total - TERM_BUF_SIZE; // client fell behind, buffer already wrapped
  else                                     start = since;

  const uint32_t MAX_PER_POLL = 1024; // cap response size regardless of how far behind
  if (total - start > MAX_PER_POLL) start = total - MAX_PER_POLL;

  c.print("{\"hex\":\"");
  for (uint32_t i = start; i < total; i++)
  {
    uint8_t b = rb->buf[i % TERM_BUF_SIZE];
    if (b < 16) c.print('0');
    c.print(b, HEX);
  }
  c.print("\",\"next\":"); c.print(total); c.println("}");
}

static void handleTerminalSend(const String &body)
{
  uint8_t sel = (uint8_t)extractFloat(body, "port", 0);
  HardwareSerial *target = termPortForSelector(sel);
  if (!target) return;

  String text = termUrlDecodeValue(body, "text");
  bool newline = body.indexOf("newline=1") >= 0;

  target->print(text);
  if (newline) target->print("\r\n");

  Serial.print("[TERM] sent "); Serial.print(text.length()); Serial.print(" bytes to ");
  Serial.println(termLabelForSelector(sel));
}

static void handleTerminalBaud(const String &body)
{
  uint8_t sel = (uint8_t)extractFloat(body, "port", 0);
  uint32_t baud = (uint32_t)extractFloat(body, "baud", 0);
  HardwareSerial *target = termPortForSelector(sel);
  if (!target || baud < 1200 || baud > 2000000) return;

  target->begin(baud);
  Serial.print("[TERM] "); Serial.print(termLabelForSelector(sel));
  Serial.print(" baud set to "); Serial.println(baud);
}

static void sendTerminalPage(EthernetClient &c)
{
  sendOK(c, "text/html");
  sendHead(c, "AgOpenGPS Serial Terminal", false);
  sendNav(c, 4);

  c.println("<div class='desc' style='color:#888;margin-bottom:10px'>");
  c.println("Raw bytes as they're actually read by the firmware on each port - useful for checking "
             "whether anything is coming out of a sensor's wiring at all, independent of whatever "
             "protocol is supposed to be running on it.");
  c.println("</div>");

  c.println("<div class='row'><label>Port</label><select id='termPort' style='width:220px'>");
  for (uint8_t sel = 1; sel <= 4; sel++)
  {
    HardwareSerial *p = termPortForSelector(sel);
    c.print("<option value='"); c.print(sel); c.print("'>");
    c.print(termLabelForSelector(sel));
    c.print(" \xe2\x80\x94 "); c.print(termCurrentRoleDescription(p));
    c.println("</option>");
  }
  c.println("</select></div>");

  c.println("<div class='row'><label>Baud rate</label><select id='termBaud' style='width:220px'>");
  const uint32_t bauds[] = { 4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600 };
  for (uint8_t i = 0; i < 9; i++)
  {
    c.print("<option value='"); c.print(bauds[i]); c.print("'");
    if (bauds[i] == 115200) c.print(" selected");
    c.print(">"); c.print(bauds[i]); c.println("</option>");
  }
  c.println("</select> <button id='applyBaud' style='width:auto;padding:6px 12px;margin-top:0'>Apply</button></div>");
  c.println("<div class='desc' style='color:#f0a030'>Changing baud rate disrupts normal use of this port until it's changed back or the board reboots.</div>");

  c.println("<pre id='termView' style='background:#0d1117;color:#7ee787;padding:10px;border-radius:6px;height:280px;overflow-y:auto;font-size:.78em;white-space:pre-wrap;word-break:break-all;margin-top:10px'></pre>");

  c.println("<div class='row' style='margin-top:10px'>");
  c.println("<input type='text' id='termSendText' placeholder='text to send' style='flex:1;padding:8px;background:#16213e;color:#eee;border:1px solid #0f3460;border-radius:4px'>");
  c.println("</div>");
  c.println("<label style='display:flex;align-items:center;gap:6px;font-size:.82em;color:#bbb;margin:6px 0'>");
  c.println("<input type='checkbox' id='termNewline' checked style='width:auto'> append newline</label>");
  c.println("<button id='termSendBtn'>&#128228; Send</button>");

  c.println("<script>");
  c.println("let since = 0;");
  c.println("const view = document.getElementById('termView');");
  c.println("function hexToBytes(h) { const a=[]; for (let i=0;i<h.length;i+=2) a.push(parseInt(h.substr(i,2),16)); return a; }");
  c.println("function render(bytes) {");
  c.println("  let s = '';");
  c.println("  for (const b of bytes) s += (b>=32 && b<127) ? String.fromCharCode(b) : (b===10?'\\n':(b===13?'':'.'));");
  c.println("  view.textContent += s;");
  c.println("  if (view.textContent.length > 20000) view.textContent = view.textContent.slice(-20000);");
  c.println("  view.scrollTop = view.scrollHeight;");
  c.println("}");
  c.println("async function poll() {");
  c.println("  const port = document.getElementById('termPort').value;");
  c.println("  try {");
  c.println("    const resp = await fetch(`/terminal/data?port=${port}&since=${since}`);");
  c.println("    const j = await resp.json();");
  c.println("    since = j.next;");
  c.println("    if (j.hex) render(hexToBytes(j.hex));");
  c.println("  } catch (e) {}");
  c.println("  setTimeout(poll, 500);");
  c.println("}");
  c.println("document.getElementById('termPort').addEventListener('change', () => { since = 0; view.textContent = ''; });");
  c.println("document.getElementById('applyBaud').addEventListener('click', async () => {");
  c.println("  const port = document.getElementById('termPort').value;");
  c.println("  const baud = document.getElementById('termBaud').value;");
  c.println("  await fetch('/terminal/baud', { method:'POST', headers:{'Content-Type':'application/x-www-form-urlencoded'}, body:`port=${port}&baud=${baud}` });");
  c.println("});");
  c.println("document.getElementById('termSendBtn').addEventListener('click', async () => {");
  c.println("  const port = document.getElementById('termPort').value;");
  c.println("  const text = encodeURIComponent(document.getElementById('termSendText').value);");
  c.println("  const nl = document.getElementById('termNewline').checked ? 1 : 0;");
  c.println("  await fetch('/terminal/send', { method:'POST', headers:{'Content-Type':'application/x-www-form-urlencoded'}, body:`port=${port}&text=${text}&newline=${nl}` });");
  c.println("  document.getElementById('termSendText').value = '';");
  c.println("});");
  c.println("poll();");
  c.println("</script>");

  c.println("</body></html>");
}

#endif // ARDUINO_TEENSY41
