/*
   SASA CAN Bus Reader (READ ONLY / LISTEN_ONLY)
   Reads steering wheel rotation speed from SASA sensor on CAN3.

   Accepted CAN IDs (both active simultaneously):
     Machine 1: 0x0CFF104D (Extended Frame, J1939/ISOBUS)
       Byte 0-1 : Steering angle (12-bit LE, wraps 0-4095)
       Byte 2-3 : Speed (16-bit LE, centered at 0x5000, scale /20)
     Machine 2: 0x301 (Standard Frame, ~100 Hz)
       Byte 0-1 : Steering angle (12-bit LE, wraps 0-4095)
       Byte 2   : Message counter (not used)
       Speed calculated as angle delta between consecutive messages.

   Hardware: Teensy 4.1 CAN3 (pins CRX3/TX3 on the board)
   Baud    : 250 kbps
   Mode    : LISTEN_ONLY (passive, no ACK / no transmit)
*/

#include <FlexCAN_T4.h>

// Set true to suppress $PANDA output and print raw CAN bytes for debugging
#define SASA_DEBUG false

// CAN IDs to accept - both active at the same time
#define SASA_ID_EXT  0x0CFF104D  // Machine 1 (Extended, J1939)
#define SASA_ID_STD  0x301       // Machine 2 (Standard)

// CAN3 - SASA sensor bus (READ ONLY)
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_256> SASA_CAN;

// Steering wheel rotation speed (signed: positive = one direction, negative = other)
// Machine 1: from dedicated speed bytes. Machine 2: angle delta per cycle.
// Smoothing done in Autosteer.ino.
int16_t canSteeringWheelSpeed = 0;

// Machine 2: accumulate angle change over 20 cycles (~200ms at 100Hz)
// for better resolution before computing speed
#define SASA_M2_WINDOW 20
static int16_t windowAngle = -1;
static uint8_t windowCount = 0;

void CAN_SASA_Setup()
{
    SASA_CAN.begin();
    SASA_CAN.setBaudRate(250000, LISTEN_ONLY);
    SASA_CAN.enableFIFO();
    SASA_CAN.setFIFOFilter(REJECT_ALL);
    SASA_CAN.setFIFOFilter(0, SASA_ID_EXT, EXT);  // Machine 1 (J1939)
    SASA_CAN.setFIFOFilter(1, SASA_ID_STD, STD);  // Machine 2

    Serial.println("CAN3 SASA listener initialized (LISTEN_ONLY @ 250 kbps)");
    Serial.print("  Filters: 0x"); Serial.print(SASA_ID_EXT, HEX); Serial.print(" (EXT)");
    Serial.print(", 0x");         Serial.print(SASA_ID_STD, HEX); Serial.println(" (STD)");
    if (SASA_DEBUG) Serial.println("  SASA_DEBUG ON - raw bytes printing enabled, PANDA suppressed");
}

void CAN_SASA_Read()
{
    CAN_message_t msg;

    while (SASA_CAN.read(msg))
    {
        if ((msg.id == SASA_ID_EXT || msg.id == SASA_ID_STD) && msg.len >= 4)
        {
#if SASA_DEBUG
            // Print all 8 raw bytes in hex + byte-pair interpretations
            Serial.print("SASA [");
            for (uint8_t i = 0; i < 8; i++) {
                if (msg.buf[i] < 0x10) Serial.print('0');
                Serial.print(msg.buf[i], HEX);
                if (i < 7) Serial.print(' ');
            }
            Serial.print("]  B0-1 BE=");
            Serial.print((int16_t)((msg.buf[0] << 8) | msg.buf[1]));
            Serial.print(" LE=");
            Serial.print((int16_t)((msg.buf[1] << 8) | msg.buf[0]));
            Serial.print("  B2-3 BE=");
            Serial.print((int16_t)((msg.buf[2] << 8) | msg.buf[3]));
            Serial.print(" LE=");
            Serial.print((int16_t)((msg.buf[3] << 8) | msg.buf[2]));
            Serial.print("  B4-5 BE=");
            Serial.print((int16_t)((msg.buf[4] << 8) | msg.buf[5]));
            Serial.print(" LE=");
            Serial.println((int16_t)((msg.buf[5] << 8) | msg.buf[4]));
#endif

            int16_t speed = 0;

            if (msg.id == SASA_ID_EXT)
            {
                // Machine 1: bytes 2-3 = signed speed (16-bit LE, centered at 0x5000, /20)
                uint16_t raw16 = (uint16_t)msg.buf[2] | ((uint16_t)msg.buf[3] << 8);
                speed = (int16_t)(((int32_t)raw16 - 0x5000) / 20);
            }
            else // SASA_ID_STD (Machine 2)
            {
                // Machine 2: speed = angle delta over 20-cycle window (~200ms at 100Hz)
                int16_t angle = (int16_t)((uint16_t)msg.buf[0] | ((uint16_t)msg.buf[1] << 8));
                angle &= 0x0FFF;  // 12-bit mask (0-4095)

                if (windowAngle < 0)
                {
                    // First reading: seed the window
                    windowAngle = angle;
                    windowCount = 0;
                }
                else
                {
                    windowCount++;
                    if (windowCount >= SASA_M2_WINDOW)
                    {
                        int16_t delta = angle - windowAngle;
                        // Handle 4096 wrap-around
                        if (delta > 2048)  delta -= 4096;
                        if (delta < -2048) delta += 4096;
                        speed = delta;
                        // Reset window
                        windowAngle = angle;
                        windowCount = 0;
                    }
                    else
                    {
                        // Still accumulating - don't update speed
                        speed = canSteeringWheelSpeed;  // hold previous value
                    }
                }
            }

            // Dead zone: treat very low values as stationary
            if (abs(speed) < 2) speed = 0;

            canSteeringWheelSpeed = speed;
        }
    }
}
