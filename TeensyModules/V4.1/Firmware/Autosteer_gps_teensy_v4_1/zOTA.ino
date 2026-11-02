/* zOTA.ino - firmware update from the web page.
   Ported from renatorgr/Boards, Firmware/v4.5_AiO_WEB/zOTA.ino (written 2026 by Claude (Anthropic) for renatorgr),
   via AOG_CAN_Teensy4.1. Autosteer_gps_teensy_v4_1, 2026: own family marker; interrupts of this board (all UARTs,
   the FEMA pin interrupt, the speed pulse tone timer) quieted before the copy.
   Uses FlasherX by Joe Pasquariello (FlashTxx.c / FlashTxx.h, public domain) for the flash.
   Licence: GNU General Public License v3 or later (file COPYING), WITHOUT ANY WARRANTY.
   SPDX-License-Identifier: GPL-3.0-or-later
*/
// Firmware update from the web page (zWeb.ino), without a USB cable: the browser sends the .hex file made
// by the Arduino IDE ("Sketch > Export Compiled Binary"), the Teensy checks it, keeps it in a free part of
// its flash (the "buffer", 4-6 MB) and only then copies it over the running firmware (FlasherX
// flash_move) and restarts.
//
// Safety:
//   - nothing is written over the running firmware until the whole file has arrived and passed every
//     check: Intel HEX checksums, end-of-file record, starts at the flash base, fits the buffer, contains
//     "fw_teensy41" (built for a Teensy 4.1) and OTA_MARKER (it is this firmware, with this web page, so the
//     next update can also be done from the web page). "Other firmware" on the page skips the marker check.
//   - each flash sector written to the buffer is read back and compared
//   - steering is switched off before receiving, and an update is refused while autosteer is engaged or
//     the tractor is moving (zWeb.ino)
//   - the EEPROM (AgOpenGPS steer settings, network, hydraulics, web page board setup) is kept: FLASH_RESERVE
//     in FlashTxx.h
//   - the only risky moment is the final copy (a few seconds): a power cut THEN leaves the Teensy without
//     firmware, and it must be reprogrammed over USB (the Teensy itself is not damaged).

extern "C" {
#include "FlashTxx.h"
}

#define OTA_BUFFER_ADDR   (FLASH_BASE_ADDR + 0x400000)   // 4 MB into the flash
#define OTA_BUFFER_SIZE   (0x200000)                     // 2 MB (the firmware is ~0.5 MB)
// The firmware family marker: the running firmware refuses a .hex without it (unless "Other firmware" is
// ticked), so the CAN steer-ready firmware cannot be put on this board by mistake, and the other way round.
#define OTA_MARKER        "AOG-T41-AUTOSTEER-GPS-WEB-OTA-1"

// Kept in the image (sent in /api/status, so the linker keeps them)
const char otaMarker[]  = OTA_MARKER;
const char otaFlashId[] = FLASH_ID;
// Build line, also read by the web page from a chosen .hex file before it is sent (version and build time)
const char fwBuildInfo[] = "[fw] " FW_NAME ", built " __DATE__ " " __TIME__;

extern unsigned long _flashimagelen;                     // size of the running firmware (linker)
#ifndef OTA_IMAGE_LEN
#define OTA_IMAGE_LEN     ((uint32_t)&_flashimagelen)
#endif

struct OtaState
{
    uint32_t    base;           // extended linear address (record 04)
    uint32_t    minAddr, maxAddr;
    int32_t     sector;         // buffer sector held in "buf" (-1 none)
    int32_t     lastWritten;    // last buffer sector written to flash (-1 none)
    bool        eof;
    uint32_t    lines, dataBytes;
    const char* err;
    uint16_t    lineLen;
    char        line[600];
    uint8_t     buf[FLASH_SECTOR_SIZE] __attribute__((aligned(4)));
};
static OtaState ota;

static int hexNib(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return -1;
}

static int hexByte(const char* p)
{
    const int h = hexNib(p[0]), l = hexNib(p[1]);
    return (h < 0 || l < 0) ? -1 : (h << 4) | l;
}

// Erase (if needed) one 4 KB sector of the buffer
static bool otaEraseSector(int32_t sector)
{
    const uint32_t addr = OTA_BUFFER_ADDR + (uint32_t)sector * FLASH_SECTOR_SIZE;
    if (flash_sector_not_erased(addr)) eepromemu_flash_erase_sector((void*)addr);
    return !flash_sector_not_erased(addr);
}

// Write the sector held in ota.buf to the buffer and read it back
static bool otaFlushSector()
{
    if (ota.sector < 0) return true;
    const uint32_t addr = OTA_BUFFER_ADDR + (uint32_t)ota.sector * FLASH_SECTOR_SIZE;
    if (!otaEraseSector(ota.sector)) { ota.err = "flash erase failed"; return false; }
    for (uint32_t off = 0; off < FLASH_SECTOR_SIZE; off += 256)      // flash page = 256 bytes
    {
        bool blank = true;
        for (uint32_t i = 0; i < 256; i++) if (ota.buf[off + i] != 0xFF) { blank = false; break; }
        if (!blank) eepromemu_flash_write((void*)(addr + off), ota.buf + off, 256);
    }
    if (memcmp((const void*)addr, ota.buf, FLASH_SECTOR_SIZE) != 0) { ota.err = "flash verify failed"; return false; }
    ota.lastWritten = ota.sector;
    ota.sector = -1;
    return true;
}

// Put one data byte at absolute flash address "a" into the image
static bool otaPutByte(uint32_t a, uint8_t v)
{
    if (a < FLASH_BASE_ADDR || a >= FLASH_BASE_ADDR + OTA_BUFFER_SIZE) { ota.err = "address outside the Teensy 4.1 program flash"; return false; }
    const uint32_t off = a - FLASH_BASE_ADDR;
    const int32_t  sec = (int32_t)(off / FLASH_SECTOR_SIZE);
    if (sec != ota.sector)
    {
        if (sec < ota.sector || sec <= ota.lastWritten) { ota.err = "hex records out of order"; return false; }
        if (!otaFlushSector()) return false;
        // sectors with no data between the last one and this one must be blank in the buffer
        for (int32_t s = ota.lastWritten + 1; s < sec; s++)
            if (!otaEraseSector(s)) { ota.err = "flash erase failed"; return false; }
        ota.sector = sec;
        memset(ota.buf, 0xFF, sizeof(ota.buf));
    }
    ota.buf[off % FLASH_SECTOR_SIZE] = v;
    if (a < ota.minAddr) ota.minAddr = a;
    if (a + 1 > ota.maxAddr) ota.maxAddr = a + 1;
    return true;
}

// One Intel HEX line ":LLAAAATT<data>CC"
static bool otaLine(const char* l, uint16_t n)
{
    if (n == 0) return true;
    if (l[0] != ':' || n < 11) { ota.err = "not an Intel HEX file"; return false; }
    if (ota.eof) return true;                                  // ignore anything after the end record
    const int len = hexByte(l + 1);
    if (len < 0 || n < (uint16_t)(11 + 2 * len)) { ota.err = "bad hex line"; return false; }
    // count, address (2), type, data (len) and the checksum byte must add up to 0
    uint8_t sum = 0;
    for (int i = 0; i < 5 + len; i++)
    {
        const int b = hexByte(l + 1 + 2 * i);
        if (b < 0) { ota.err = "bad hex digit"; return false; }
        sum += (uint8_t)b;
    }
    if (sum != 0) { ota.err = "hex checksum error"; return false; }
    const uint32_t addr = ((uint32_t)hexByte(l + 3) << 8) | (uint32_t)hexByte(l + 5);
    const int      type = hexByte(l + 7);
    ota.lines++;
    switch (type)
    {
        case 0:                                                 // data
            for (int i = 0; i < len; i++)
                if (!otaPutByte(ota.base + addr + i, (uint8_t)hexByte(l + 9 + 2 * i))) return false;
            ota.dataBytes += len;
            return true;
        case 1: ota.eof = true; return true;                    // end of file
        case 4:                                                 // extended linear address
            if (len != 2) { ota.err = "bad hex address record"; return false; }
            ota.base = ((uint32_t)hexByte(l + 9) << 24) | ((uint32_t)hexByte(l + 11) << 16);
            return true;
        case 3: case 5: return true;                            // start address: not needed
        default: ota.err = "unsupported hex record"; return false;
    }
}

// Search the buffer for a string (like FlasherX check_flash_id)
static bool check_flash_id_str(uint32_t buffer, uint32_t size, const char* id)
{
    const size_t n = strlen(id);
    for (uint32_t i = buffer; i + n <= buffer + size; ++i)
        if (memcmp((const void*)i, id, n) == 0) return true;
    return false;
}

// ---------------------------------------------------------------- interface for zWeb.ino
// Can an update be stored now? (the running firmware must end below the buffer)
const char* otaBegin()
{
    memset(&ota, 0, offsetof(OtaState, buf));
    ota.minAddr = 0xFFFFFFFF;
    ota.sector = ota.lastWritten = -1;
    if (FLASH_BASE_ADDR + OTA_IMAGE_LEN + FLASH_SECTOR_SIZE > OTA_BUFFER_ADDR) ota.err = "running firmware too large for the update buffer";
    return ota.err;
}

// Feed the bytes of the .hex file as they arrive
const char* otaFeed(const uint8_t* d, size_t n)
{
    for (size_t i = 0; i < n && !ota.err; i++)
    {
        const char c = (char)d[i];
        if (c == '\n' || c == '\r')
        {
            if (!otaLine(ota.line, ota.lineLen)) break;
            ota.lineLen = 0;
        }
        else if (ota.lineLen < sizeof(ota.line) - 1) ota.line[ota.lineLen++] = c;
        else ota.err = "hex line too long";
    }
    return ota.err;
}

// After the last byte: every check. Returns nullptr when the image may be installed;
// *size = image size in bytes.
const char* otaFinish(bool force, uint32_t* size)
{
    if (!ota.err && ota.lineLen)                                  // last line without a line end
    {
        if (!otaLine(ota.line, ota.lineLen)) ota.err = "file incomplete (cut off in the middle of a line)";
        ota.lineLen = 0;
    }
    if (!ota.err) otaFlushSector();
    if (ota.err) return ota.err;
    if (!ota.eof)                                    return ota.err = "file incomplete (no end record)";
    if (ota.dataBytes < 16384)                       return ota.err = "file too small to be a Teensy firmware";
    if (ota.minAddr != FLASH_BASE_ADDR)              return ota.err = "image does not start at the Teensy 4.1 flash base";
    const uint32_t len = ota.maxAddr - FLASH_BASE_ADDR;
    if (!check_flash_id(OTA_BUFFER_ADDR, len))       return ota.err = "not built for a Teensy 4.1 (fw_teensy41 missing)";
    if (!force && !check_flash_id_str(OTA_BUFFER_ADDR, len, OTA_MARKER))
        return ota.err = "not this firmware (no web update in it) - tick 'Other firmware' to install it anyway";
    *size = len;
    return nullptr;
}

// Copy the checked image over the running firmware and restart. Does not return.
void otaInstall(uint32_t size)
{
    Serial.flush();
    // Quiet the peripherals whose interrupts could touch flash constants during the copy
    NVIC_DISABLE_IRQ(IRQ_ENET);
    NVIC_DISABLE_IRQ(IRQ_USB1);
    NVIC_DISABLE_IRQ(IRQ_LPUART1);    // all eight UARTs: the GNSS, IMU and radio ports are chosen on the web page
    NVIC_DISABLE_IRQ(IRQ_LPUART2);
    NVIC_DISABLE_IRQ(IRQ_LPUART3);
    NVIC_DISABLE_IRQ(IRQ_LPUART4);
    NVIC_DISABLE_IRQ(IRQ_LPUART5);
    NVIC_DISABLE_IRQ(IRQ_LPUART6);
    NVIC_DISABLE_IRQ(IRQ_LPUART7);
    NVIC_DISABLE_IRQ(IRQ_LPUART8);
    NVIC_DISABLE_IRQ(IRQ_GPIO6789);   // pin interrupts (the FEMA pressure input, attachInterrupt)
    NVIC_DISABLE_IRQ(IRQ_PIT);        // IntervalTimer: the speed pulse tone()
    flash_move(FLASH_BASE_ADDR, OTA_BUFFER_ADDR, size);   // erases the buffer afterwards, reboots
    for (;;) {}
}
