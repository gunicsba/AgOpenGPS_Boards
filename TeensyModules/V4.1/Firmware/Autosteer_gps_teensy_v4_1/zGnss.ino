/* zGnss.ino - commands to the GNSS receiver from the web page (GNSS tab), for receivers that take text commands on
   their NMEA port, like the Unicore UM982 / UM980. Autosteer_gps_teensy_v4_1, 2026.
   Licence: GNU General Public License v3 or later (file COPYING), WITHOUT ANY WARRANTY.
   SPDX-License-Identifier: GPL-3.0-or-later
*/
// A command goes to the GNSS port (SerialGPS) the way the NTRIP corrections go, as CR LF <command> CR LF: the
// CR LF in front ends whatever the receiver got before it (a part of a correction message). For a few seconds
// after it, the lines the receiver sends are kept for the web page: its answer ("$command,...,response: OK*..",
// #VERSIONA, $CONFIG ...). The position lines it sends all the time are left out, unless all lines are asked for.

#define GNSS_LINES      60       // lines kept for the web page
#define GNSS_LEN        200      // characters per line (longer ones are cut)
#define GNSS_CMD_MAX    160      // characters per command
#define GNSS_LISTEN_MS  5000     // lines kept for this long after a command

struct GnssLine { uint32_t seq, ms; uint8_t port; bool sent; char text[GNSS_LEN]; };
GnssLine gnssLines[GNSS_LINES];
uint32_t gnssNext = 1;               // sequence number of the next line
uint32_t gnssListenUntil = 0;        // millis
bool     gnssListenAll = false;      // also the periodic lines (GGA, VTG ...)
char     gnssRx[GNSS_LEN];           // the line coming in
uint16_t gnssRxLen = 0;
bool     gnssRxBinary = false;       // not text (RTCM, UBX): not kept

static void gnssKeep(bool sent, const char* text)
{
    GnssLine& l = gnssLines[gnssNext % GNSS_LINES];
    l.seq = gnssNext++;
    l.ms = millis();
    l.port = boardPortNum(SerialGPS);
    l.sent = sent;
    snprintf(l.text, sizeof(l.text), "%s", text);
}

// The lines a receiver sends all the time (position for AgOpenGPS, satellites ...)
static bool gnssPeriodic(const char* s)
{
    if (!strncmp(s, "$KSXT", 5)) return true;
    if (s[0] != '$' || strlen(s) < 6) return false;
    static const char* const types[] = { "GGA", "VTG", "RMC", "GSA", "GSV", "GLL", "ZDA", "GST", "HDT", "HPR", "THS" };
    for (const char* t : types) if (!strncmp(s + 3, t, 3)) return true;
    return false;
}

// Every byte from the GNSS port (loop)
void gnssMonByte(char c)
{
    if (c == '\r' || c == '\n')
    {
        if (gnssRxLen && !gnssRxBinary && (int32_t)(gnssListenUntil - millis()) > 0)
        {
            gnssRx[gnssRxLen] = '\0';
            if (gnssListenAll || !gnssPeriodic(gnssRx)) gnssKeep(false, gnssRx);
        }
        gnssRxLen = 0;
        gnssRxBinary = false;
        return;
    }
    if ((uint8_t)c < 0x20 || (uint8_t)c > 0x7E) gnssRxBinary = true;
    else if (gnssRxLen < GNSS_LEN - 1) gnssRx[gnssRxLen++] = c;
}

// From the web page (autosteer off, cmd checked there): nullptr when sent, else why not
const char* gnssSend(const char* cmd, bool all)
{
    if (!SerialGPS) return "no GNSS port";
    SerialGPS->write("\r\n");
    SerialGPS->write(cmd);
    SerialGPS->write("\r\n");
    gnssKeep(true, cmd);
    gnssListenUntil = millis() + GNSS_LISTEN_MS;
    gnssListenAll = all;
    return nullptr;
}

// The kept lines after sequence number "after", for /api/gnss
void gnssJson(Json& j, uint32_t after)
{
    const uint32_t now = millis();
    j.u("next", gnssNext);
    j.u("port", boardPortNum(SerialGPS));
    j.u("listen", (int32_t)(gnssListenUntil - now) > 0 ? gnssListenUntil - now : 0);
    j.arr("lines");
    const uint32_t first = gnssNext > GNSS_LINES ? gnssNext - GNSS_LINES : 1;
    for (uint32_t s = (after + 1 > first ? after + 1 : first); s < gnssNext; s++)
    {
        const GnssLine& l = gnssLines[s % GNSS_LINES];
        if (l.seq != s) continue;
        if (j.n + GNSS_LEN + 64 > j.cap) break;              // the page asks again for the rest
        j.open(); j.u("s", l.seq); j.u("t", l.ms);
        if (l.sent) { j.bo("tx", true); j.u("p", l.port); }
        j.str("x", l.text); j.close();
    }
    j.closeArr();
}
