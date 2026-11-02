/* zWebJson.h - small JSON writer for the web server.
   Copied unchanged from renatorgr/Boards, Firmware/v4.5_AiO_WEB (Teensy 4.1 firmware for the AiO v4.5 boards).
   Written 2026 by Claude (Anthropic) for renatorgr.
   Licence: GNU General Public License v3 or later (file COPYING), WITHOUT ANY WARRANTY.
   SPDX-License-Identifier: GPL-3.0-or-later
*/
// zWebJson.h
// A small JSON writer into a fixed buffer (zWeb.ino). In a header because the Arduino IDE adds
// function prototypes at the top of the sketch, before any type defined in an .ino file.
#ifndef ZWEBJSON_H
#define ZWEBJSON_H

#include <math.h>
#include <stdio.h>

struct Json
{
    char*  b;
    size_t cap, n;
    bool   first;
    Json(char* buf, size_t size) : b(buf), cap(size), n(0), first(true) { b[0] = '\0'; }
    void raw(const char* s)  { while (*s && n + 1 < cap) b[n++] = *s++; b[n] = '\0'; }
    void ch(char c)          { if (n + 1 < cap) { b[n++] = c; b[n] = '\0'; } }
    void sep()               { if (!first) ch(','); first = false; }
    void key(const char* k)  { sep(); ch('"'); raw(k); raw("\":"); }
    void esc(const char* s)
    {
        ch('"');
        for (; *s; s++)
        {
            const uint8_t c = (uint8_t)*s;
            if (c == '"' || c == '\\') { ch('\\'); ch((char)c); }
            else if (c < 0x20)         { char t[8]; snprintf(t, sizeof(t), "\\u%04x", c); raw(t); }
            else                        ch((char)c);
        }
        ch('"');
    }
    void str(const char* k, const char* v)       { key(k); esc(v); }
    void num(const char* k, double v, int d)     { key(k); if (isnan(v) || isinf(v)) { raw("null"); return; } char t[32]; snprintf(t, sizeof(t), "%.*f", d, v); raw(t); }
    void i(const char* k, long v)                { key(k); char t[16]; snprintf(t, sizeof(t), "%ld", v); raw(t); }
    void u(const char* k, unsigned long v)       { key(k); char t[16]; snprintf(t, sizeof(t), "%lu", v); raw(t); }
    void bo(const char* k, bool v)               { key(k); raw(v ? "true" : "false"); }
    void obj(const char* k)                      { key(k); ch('{'); first = true; }
    void arr(const char* k)                      { key(k); ch('['); first = true; }
    void open()                                  { sep(); ch('{'); first = true; }
    void close()                                 { ch('}'); first = false; }
    void closeArr()                              { ch(']'); first = false; }
    void item(const char* v)                     { sep(); esc(v); }
};

#endif
