/* zNet.h - the network stack (FNET, under NativeEthernet) runs from loop(), not from an interrupt.
   From renatorgr/Boards, Firmware/v4.5_AiO_WEB (Teensy 4.1 firmware for the AiO v4.5 boards), via
   AOG_CAN_Teensy4.1 (the subnet is still set by AgIO, PGN 201); the functions are in zWeb.ino.
   Written 2026 by Claude (Anthropic) for renatorgr.
   Licence: GNU General Public License v3 or later (file COPYING), WITHOUT ANY WARRANTY.
   SPDX-License-Identifier: GPL-3.0-or-later
*/
// zNet.h
// NativeEthernet runs the network stack (fnet_poll: the TCP timers and its web server's accept) from
// a 1 ms IntervalTimer interrupt. FNET's own lock only holds back the Ethernet receive interrupt, not
// that timer, so the timer could change the sockets, NativeEthernet's socket table and the stack's
// memory while loop() was in the middle of a call into them (web page, UDP). It crashed once that way
// (0x0001D542 with Teensyduino 1.57.3: a received TCP segment with a broken data pointer, when a
// browser opened a connection to the web page). FNET is designed to be polled from the main loop: after
// setup() the timer is stopped and netPoll() runs fnet_poll() from loop() and from every
// wait for the network (zWeb.ino). Anything that loops for a long time must call netPoll().
// EthernetClass::_fnet_poll (the timer) is private. It is reached through an explicit template
// instantiation, where C++ does not check access (C++11 [temp.explicit]), so NativeEthernet itself
// is not changed.
#ifndef ZNET_H
#define ZNET_H

#include <NativeEthernet.h>

struct NetPollTimerTag { friend IntervalTimer* netPollTimer(NetPollTimerTag); };
template <IntervalTimer* T> struct NetPollTimerAccess
{
    friend IntervalTimer* netPollTimer(NetPollTimerTag) { return T; }
};
template struct NetPollTimerAccess<&EthernetClass::_fnet_poll>;

void netPollFromLoop();             // at the end of setup(): the library's timer stops (zWeb.ino)
void netPoll();                     // fnet_poll(), once a millisecond at most (zWeb.ino)
void netDelay(uint32_t ms);         // delay() that keeps the network going (zWeb.ino)

#endif
