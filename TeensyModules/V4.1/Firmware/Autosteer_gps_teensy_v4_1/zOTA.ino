// =============================================================
// OTA FIRMWARE UPDATE - via the existing web server, over the network instead of USB
//
// Built on FlasherX's flash primitives (FlashTxx.h/.c, vendored unmodified from
// https://github.com/joepasquariello/FlasherX, public domain). Deliberately NOT using
// FlasherX's own update_firmware() from FXUtil.cpp - that one is interactive, prompting for
// confirmation over the same Stream it's reading the hex file from ("enter N to flash or 0
// to abort"), which has no equivalent over a one-shot HTTP POST. Instead, this is an
// explicit two-step flow:
//
//   GET  /ota          upload form
//   POST /ota/stage    streams the uploaded .hex directly off the TCP connection into a
//                       flash-based staging buffer above the running code - never buffers
//                       the whole file in RAM. Validates the Intel HEX and the FLASH_ID
//                       marker (confirms the image was actually built for this exact
//                       board), then responds with a summary and explicit
//                       Confirm/Cancel buttons. Uploading alone flashes nothing.
//   POST /ota/confirm  copies the staged image into place and reboots into it. Point of no
//                       return for this session - recoverable via USB + the physical
//                       PROGRAM button if something goes wrong, same as any other flash,
//                       but the running firmware cannot undo it once started.
//   POST /ota/cancel   frees the staged buffer, discards it
//
// Only one staging session at a time; state below is static/global, not per-connection.
// =============================================================

#ifdef ARDUINO_TEENSY41

extern "C" {
  #include "FlashTxx.h"
}

// ---- Intel HEX line parsing ----
// Same public-domain parser FlasherX uses (originally Paul Stoffregen's), reproduced here
// with internal linkage since we don't want FXUtil.cpp's interactive update_firmware().
typedef struct {
  char *data;
  unsigned int addr;
  unsigned int code;
  unsigned int num;
  uint32_t base;
  uint32_t min;
  uint32_t max;
  int eof;
  int lines;
} ota_hex_info_t;

// Explicit prototype, positioned after ota_hex_info_t is defined but before any use -
// Arduino's ctags-based auto-prototype generator hoists a prototype for every function to
// the very top of the concatenated sketch, before this typedef exists, which would fail to
// compile ("ota_hex_info_t not declared"). Declaring it here ourselves, in the right place,
// makes the tool skip generating its own broken one instead (same fix as the calcChecksum()
// overloads in the main .ino - see that comment for the general pattern).
static int ota_process_hex_record(ota_hex_info_t *hex);

static int ota_parse_hex_line(const char *theline, char *bytes,
    unsigned int *addr, unsigned int *num, unsigned int *code)
{
  unsigned sum, len, cksum;
  const char *ptr;
  int temp;

  *num = 0;
  if (theline[0] != ':') return 0;
  if (strlen(theline) < 11) return 0;
  ptr = theline + 1;
  if (!sscanf(ptr, "%02x", &len)) return 0;
  ptr += 2;
  if (strlen(theline) < (11 + (len * 2))) return 0;
  if (!sscanf(ptr, "%04x", (unsigned int *)addr)) return 0;
  ptr += 4;
  if (!sscanf(ptr, "%02x", code)) return 0;
  ptr += 2;
  sum = (len & 255) + ((*addr >> 8) & 255) + (*addr & 255) + (*code & 255);
  while (*num != len)
  {
    if (!sscanf(ptr, "%02x", &temp)) return 0;
    bytes[*num] = temp;
    ptr += 2;
    sum += bytes[*num] & 255;
    (*num)++;
    if (*num >= 256) return 0;
  }
  if (!sscanf(ptr, "%02x", &cksum)) return 0;
  if (((sum & 255) + (cksum & 255)) & 255) return 0; // checksum error
  return 1;
}

static int ota_process_hex_record(ota_hex_info_t *hex)
{
  if (hex->code == 0)
  {
    if (hex->base + hex->addr + hex->num > hex->max) hex->max = hex->base + hex->addr + hex->num;
    if (hex->base + hex->addr < hex->min) hex->min = hex->base + hex->addr;
  }
  else if (hex->code == 1) hex->eof = 1;
  else if (hex->code == 2) hex->base = ((hex->data[0] << 8) | hex->data[1]) << 4;
  else if (hex->code == 3) return 1;
  else if (hex->code == 4) hex->base = ((hex->data[0] << 8) | hex->data[1]) << 16;
  else if (hex->code == 5) hex->base = (hex->data[0] << 24) | (hex->data[1] << 16) | (hex->data[2] << 8) | hex->data[3];
  else return 1;
  return 0;
}

// Reads one '\n'/'\r'-terminated line directly off the TCP connection, with a timeout - the
// stock FXUtil.cpp version busy-loops on serial->available() forever, which would hang the
// board indefinitely if a network upload stalls or the browser tab is closed mid-transfer.
static bool ota_read_line_timeout(EthernetClient &client, char *line, int maxbytes, uint32_t timeoutMs)
{
  int nchar = 0;
  uint32_t start = millis();
  while (nchar < maxbytes - 1)
  {
    if (client.available())
    {
      char c = client.read();
      if (c == '\n' || c == '\r')
      {
        if (nchar > 0) break; // end of a non-empty line
        else continue;        // skip a leading CR/LF
      }
      line[nchar++] = c;
      start = millis(); // reset timeout on forward progress
    }
    else if (millis() - start > timeoutMs)
    {
      return false; // stalled
    }
  }
  line[nchar] = 0;
  return true;
}

// ---- Staging state (single session, not per-connection) ----
static uint32_t otaBufferAddr   = 0;
static uint32_t otaBufferSize   = 0;
static uint32_t otaImageSize    = 0; // hex.max - hex.min for the currently-staged image
static bool     otaStaged       = false; // buffer allocated (whether or not still valid)
static bool     otaReadyToFlash = false; // parsed + FLASH_ID-checked successfully

static void otaFreeBuffer()
{
  if (otaStaged)
  {
    firmware_buffer_free(otaBufferAddr, otaBufferSize);
    otaStaged = false;
  }
  otaReadyToFlash = false;
}

struct OtaStageResult
{
  bool     ok;
  int      lines;
  uint32_t bytes;
  uint32_t minAddr, maxAddr;
  String   error;
};

// Same reasoning as ota_process_hex_record() above: explicit prototypes for every function
// that takes/returns OtaStageResult, positioned after the struct is defined, so Arduino's
// auto-prototype generator doesn't hoist a broken one above this point in the file.
static OtaStageResult otaStageFromClient(EthernetClient &client, uint32_t contentLen);
static void sendOtaResultPage(EthernetClient &c, const OtaStageResult &result);

// Streams contentLen bytes of an Intel HEX file directly off the still-open connection into
// a freshly-allocated flash staging buffer, parsing and writing as it goes - never buffers
// the whole upload in RAM. Body bytes must not have been consumed yet when this is called.
static OtaStageResult otaStageFromClient(EthernetClient &client, uint32_t contentLen)
{
  OtaStageResult result = { false, 0, 0, 0, 0, "" };

  otaFreeBuffer();

  if (firmware_buffer_init(&otaBufferAddr, &otaBufferSize) == NO_BUFFER_TYPE)
  {
    result.error = "unable to allocate a flash staging buffer";
    return result;
  }
  otaStaged = true;

  static char line[96];
  static char data[32] __attribute__((aligned(8)));
  ota_hex_info_t hex = { data, 0, 0, 0, 0, 0xFFFFFFFF, 0, 0, 0 };

  uint32_t bytesConsumed = 0;
  const uint32_t LINE_TIMEOUT_MS = 5000;

  while (!hex.eof && bytesConsumed < contentLen)
  {
    if (!ota_read_line_timeout(client, line, sizeof(line), LINE_TIMEOUT_MS))
    {
      result.error = "upload stalled - no data for " + String(LINE_TIMEOUT_MS / 1000) + "s mid-transfer";
      otaFreeBuffer();
      return result;
    }
    // Approximate - doesn't distinguish \n vs \r\n line endings. Only used as a backstop
    // against a malformed/never-ending stream; real termination is the Intel HEX EOF record.
    bytesConsumed += strlen(line) + 1;

    if (!ota_parse_hex_line(line, hex.data, &hex.addr, &hex.num, &hex.code))
    {
      result.error = "bad hex line: " + String(line);
      otaFreeBuffer();
      return result;
    }
    if (ota_process_hex_record(&hex) != 0)
    {
      result.error = "invalid hex record code " + String(hex.code);
      otaFreeBuffer();
      return result;
    }
    if (hex.code == 0) // data record
    {
      uint32_t addr = otaBufferAddr + hex.base + hex.addr - FLASH_BASE_ADDR;
      if (hex.max > (FLASH_BASE_ADDR + otaBufferSize))
      {
        result.error = "image too large for the staging buffer";
        otaFreeBuffer();
        return result;
      }
      int err = flash_write_block(addr, hex.data, hex.num);
      if (err)
      {
        result.error = "flash_write_block error " + String(err);
        otaFreeBuffer();
        return result;
      }
    }
    hex.lines++;
  }

  if (!hex.eof)
  {
    result.error = "upload ended before an Intel HEX EOF record was seen";
    otaFreeBuffer();
    return result;
  }

  if (!check_flash_id(otaBufferAddr, hex.max - hex.min))
  {
    result.error = String("uploaded image is missing the \"") + FLASH_ID + "\" marker - wrong board target, refusing to flash";
    otaFreeBuffer();
    return result;
  }

  result.ok      = true;
  result.lines   = hex.lines;
  result.bytes   = hex.max - hex.min;
  result.minAddr = hex.min;
  result.maxAddr = hex.max;

  otaImageSize    = hex.max - hex.min;
  otaReadyToFlash = true;
  return result;
}

// ---- HTML ----
static void sendOtaPage(EthernetClient &c)
{
  sendOK(c, "text/html");
  sendHead(c, "AgOpenGPS OTA Update", false);
  sendNav(c, 3);

  c.println("<div class='desc' style='color:#f0a030;margin-bottom:12px;font-size:.8em'>");
  c.print("Upload a .hex file built for this exact board (target: "); c.print(FLASH_ID); c.println("). ");
  c.println("The board keeps running its current firmware until you explicitly confirm on the next "
             "screen - uploading alone never flashes anything. If anything looks wrong there, just "
             "don't confirm.");
  c.println("</div>");

  c.println("<input type='file' id='hexfile' accept='.hex' style='width:100%;padding:8px;background:#16213e;color:#eee;border:1px solid #0f3460;border-radius:4px'>");
  c.println("<button id='uploadBtn' style='margin-top:12px'>&#128228; Upload &amp; verify</button>");
  c.println("<p id='otaStatus' class='foot'></p>");

  c.println("<script>");
  c.println("document.getElementById('uploadBtn').addEventListener('click', async () => {");
  c.println("  const f = document.getElementById('hexfile').files[0];");
  c.println("  const s = document.getElementById('otaStatus');");
  c.println("  if (!f) { s.textContent = 'Pick a .hex file first.'; return; }");
  c.println("  s.textContent = 'Uploading ' + f.name + ' (' + f.size + ' bytes)... this can take a minute.';");
  c.println("  try {");
  c.println("    const resp = await fetch('/ota/stage', { method: 'POST', body: f });");
  c.println("    document.open(); document.write(await resp.text()); document.close();");
  c.println("  } catch (e) { s.textContent = 'Upload failed: ' + e; }");
  c.println("});");
  c.println("</script>");

  c.println("</body></html>");
}

static void sendOtaResultPage(EthernetClient &c, const OtaStageResult &result)
{
  sendOK(c, "text/html");
  sendHead(c, "AgOpenGPS OTA Update", false);
  sendNav(c, 3);

  if (result.ok)
  {
    c.println("<div class='waslessBanner wOn'>&#9989; Image verified - not flashed yet</div>");
    c.print("<p>"); c.print(result.lines); c.print(" hex lines, "); c.print(result.bytes);
    c.print(" bytes, address range 0x"); c.print(result.minAddr, HEX);
    c.print(" - 0x"); c.print(result.maxAddr, HEX); c.println("</p>");
    c.print("<p>Target ID \""); c.print(FLASH_ID); c.println("\" found - this image was built for this board.</p>");
    c.println("<form method='POST' action='/ota/confirm'><button style='background:#00c853'>&#9989; Confirm and flash</button></form>");
    c.println("<form method='POST' action='/ota/cancel'><button style='background:#555;margin-top:10px'>Cancel</button></form>");
  }
  else
  {
    c.println("<div class='waslessBanner wOff'>&#10060; Upload rejected - nothing was flashed</div>");
    c.print("<p>"); c.print(result.error); c.println("</p>");
    c.println("<a href='/ota' style='color:#e94560'>&laquo; Back to upload</a>");
  }

  c.println("</body></html>");
}

// Point of no return for this session: copies the staged image into the live code region
// and reboots into it. Response is sent BEFORE calling flash_move(), since that call
// reboots the board partway through and never returns.
static void handleOtaConfirm(EthernetClient &client)
{
  if (!otaReadyToFlash)
  {
    sendOK(client, "text/html");
    client.println("<h1>No verified image staged</h1><p>Upload a .hex file first.</p><a href='/ota'>Back</a>");
    return;
  }

  sendOK(client, "text/html");
  client.println("<!DOCTYPE html><html><body style='background:#1a1a2e;color:#eee;font-family:sans-serif;padding:24px'>");
  client.println("<h1>Flashing now...</h1><p>Do not disconnect power. The board restarts automatically in a few seconds.</p>");
  client.println("</body></html>");
  client.flush();
  delay(200); // let the response actually leave the wire before flash_move() takes over
  client.stop();

  uint32_t bufferAddr = otaBufferAddr;
  uint32_t imageSize  = otaImageSize;
  otaStaged       = false; // buffer is being consumed by flash_move(), not to be freed normally
  otaReadyToFlash = false;

  flash_move(FLASH_BASE_ADDR, bufferAddr, imageSize); // never returns - reboots into new firmware
}

static void handleOtaCancel(EthernetClient &client)
{
  otaFreeBuffer();
  sendRedirect(client, "/ota");
}

// Prints FLASH_ID once at boot. This isn't just a log line - it's what makes check_flash_id()
// work at all: it forces the compiler to embed the "fw_teensy41" string as a literal in this
// build's own flash content, so that THIS build, uploaded as a future OTA image, can itself
// be verified. Keep this line in any future firmware if OTA is expected to keep working.
void otaSetup()
{
  Serial.print("OTA target ID: "); Serial.println(FLASH_ID);
}

#endif // ARDUINO_TEENSY41
