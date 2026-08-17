/*
   SASA CAN Bus Reader (READ ONLY / LISTEN_ONLY)
   Reads steering wheel rotation speed from SASA sensor on CAN3.

   CAN ID : 0x0CFF104D (Extended Frame)
   Byte 0-1 : Steering angle (12-bit LE, wraps 0-4095)
   Byte 2-3 : Steering rotation speed (16-bit LE, centered at 0x5000, scale /20)
   Byte 4-5 : Rolling counter

   Hardware: Teensy 4.1 CAN3 (pins CRX3/TX3 on the board)
   Baud    : 250 kbps
   Mode    : LISTEN_ONLY (passive, no ACK / no transmit)
*/

#include <FlexCAN_T4.h>

// Set true to suppress $PANDA output and print raw CAN bytes for debugging
#define SASA_DEBUG false

// CAN3 - SASA sensor bus (READ ONLY)
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_256> SASA_CAN;

// Steering wheel rotation speed extracted from SASA message (bytes 2-3 LE)
// Signed value: 0 = stationary, positive/negative = turn direction & speed
// Formula: (raw16 - 0x5000) / 20. Smoothing done in Autosteer.ino.
int16_t canSteeringWheelSpeed = 0;

void CAN_SASA_Setup()
{
    SASA_CAN.begin();
    SASA_CAN.setBaudRate(250000, LISTEN_ONLY);
    SASA_CAN.enableFIFO();
    SASA_CAN.setFIFOFilter(REJECT_ALL);
    SASA_CAN.setFIFOFilter(0, 0x0CFF104D, EXT);  // SASA sensor message

    Serial.println("CAN3 SASA listener initialized (LISTEN_ONLY @ 250 kbps)");
    Serial.println("  Filter: 0x0CFF104D (EXT)");
    if (SASA_DEBUG) Serial.println("  SASA_DEBUG ON - raw bytes printing enabled, PANDA suppressed");
}

void CAN_SASA_Read()
{
    CAN_message_t msg;

    while (SASA_CAN.read(msg))
    {
        if (msg.id == 0x0CFF104D && msg.len >= 4)
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

            // Bytes 2-3 = signed rotation speed (16-bit LE, centered at 0x5000, scale /20)
            // Positive = one direction, negative = other, 0 = stationary
            uint16_t raw16 = (uint16_t)msg.buf[2] | ((uint16_t)msg.buf[3] << 8);
            int16_t speed = (int16_t)(((int32_t)raw16 - 0x5000) / 20);

            // Dead zone: treat very low values as stationary
            if (abs(speed) < 2) speed = 0;

            canSteeringWheelSpeed = speed;
        }
    }
}
