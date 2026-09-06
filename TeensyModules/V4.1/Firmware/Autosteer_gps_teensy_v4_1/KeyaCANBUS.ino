// KeyaCANBUS
// Drives a Keya steering motor over CAN. Only active when steerConfig.SteerDriverType == STEER_DRIVER_KEYA.

//Enable	0x23 0x0D 0x20 0x01 0x00 0x00 0x00 0x00
//Disable	0x23 0x0C 0x20 0x01 0x00 0x00 0x00 0x00
//Fast clockwise	0x23 0x00 0x20 0x01 0xFC 0x18 0xFF 0xFF (0xfc18 signed dec is - 1000
//Anti - clockwise	0x23 0x00 0x20 0x01 0x03 0xE8 0x00 0x00 (0x03e8 signed dec is 1000
//Slow clockwise	0x23 0x00 0x20 0x01 0xFE 0x0C 0xFF 0xFF (0xfe0c signed dec is - 500)
//Slow anti - clockwise	0x23 0x00 0x20 0x01 0x01 0xf4 0x00 0x00 (0x01f4 signed dec is 500)

uint64_t KeyaPGN = 0x06000001;

// ---------------------------------------------------------------------------
// Cumulative encoder position from the heartbeat (bytes 0-1). 65535 ticks = 1 motor
// revolution. The hardware counter is a uint16 that wraps in either direction; deltas are
// accumulated into a signed int32 absolute position for the wasless mode (see Autosteer.ino).
// ---------------------------------------------------------------------------
#define KEYA_ENCODER_INVERT 1   // 0 = normal direction | 1 = reversed

int32_t  keyaEncoderRaw  = 0;
uint16_t keyaEncPrev     = 0;
bool     keyaEncInitDone = false;

void keyaUpdateEncoder(uint16_t rawTick)
{
	if (!keyaEncInitDone) {
		keyaEncPrev     = rawTick;
		keyaEncInitDone = true;
		return;
	}
	int16_t delta = (int16_t)(rawTick - keyaEncPrev);
#if KEYA_ENCODER_INVERT
	delta = -delta;
#endif
	keyaEncoderRaw += delta;
	keyaEncPrev     = rawTick;
}

// Set true only for bench debugging - Serial.println() with String concatenation runs every
// 25ms steering cycle while engaged, which allocates on the heap and isn't something we want
// running continuously on a steering controller in the field.
const bool debugKeya = false;

void CAN_Setup() {
	if (steerConfig.SteerDriverType != STEER_DRIVER_KEYA) return;

	Keya_Bus.begin();
	Keya_Bus.setBaudRate(250000);
	// Dedicated bus, zero chat from others. No need for filters.
	if (debugKeya) Serial.println("Initialised Keya CANBUS");
}

void disableKeyaSteer() {
	CAN_message_t KeyaBusSendData;
	KeyaBusSendData.id = KeyaPGN;
	KeyaBusSendData.flags.extended = true;
	KeyaBusSendData.len = 8;
	KeyaBusSendData.buf[0] = 0x23;
	KeyaBusSendData.buf[1] = 0x0c;
	KeyaBusSendData.buf[2] = 0x20;
	KeyaBusSendData.buf[3] = 0x01;
	KeyaBusSendData.buf[4] = 0;
	KeyaBusSendData.buf[5] = 0;
	KeyaBusSendData.buf[6] = 0;
	KeyaBusSendData.buf[7] = 0;
	Keya_Bus.write(KeyaBusSendData);
}

void enableKeyaSteer() {
	CAN_message_t KeyaBusSendData;
	KeyaBusSendData.id = KeyaPGN;
	KeyaBusSendData.flags.extended = true;
	KeyaBusSendData.len = 8;
	KeyaBusSendData.buf[0] = 0x23;
	KeyaBusSendData.buf[1] = 0x0d;
	KeyaBusSendData.buf[2] = 0x20;
	KeyaBusSendData.buf[3] = 0x01;
	KeyaBusSendData.buf[4] = 0;
	KeyaBusSendData.buf[5] = 0;
	KeyaBusSendData.buf[6] = 0;
	KeyaBusSendData.buf[7] = 0;
	Keya_Bus.write(KeyaBusSendData);
	if (debugKeya) Serial.println("Enabled Keya motor");
}

void SteerKeya(int steerSpeed) {
	if (steerSpeed == 0) {
		disableKeyaSteer();
		return;
	}

	int actualSpeed = map(steerSpeed, -255, 255, -995, 998);

	CAN_message_t KeyaBusSendData;
	KeyaBusSendData.id = KeyaPGN;
	KeyaBusSendData.flags.extended = true;
	KeyaBusSendData.len = 8;
	KeyaBusSendData.buf[0] = 0x23;
	KeyaBusSendData.buf[1] = 0x00;
	KeyaBusSendData.buf[2] = 0x20;
	KeyaBusSendData.buf[3] = 0x01;
	if (steerSpeed < 0) {
		KeyaBusSendData.buf[4] = highByte(actualSpeed);
		KeyaBusSendData.buf[5] = lowByte(actualSpeed);
		KeyaBusSendData.buf[6] = 0xff;
		KeyaBusSendData.buf[7] = 0xff;
	}
	else {
		KeyaBusSendData.buf[4] = highByte(actualSpeed);
		KeyaBusSendData.buf[5] = lowByte(actualSpeed);
		KeyaBusSendData.buf[6] = 0x00;
		KeyaBusSendData.buf[7] = 0x00;
	}
	Keya_Bus.write(KeyaBusSendData);
	enableKeyaSteer();

	if (debugKeya) Serial.println("SteerKeya: pwm " + String(steerSpeed) + " -> speed " + String(actualSpeed));
}

void KeyaBus_Receive() {
	if (steerConfig.SteerDriverType != STEER_DRIVER_KEYA) return;

	CAN_message_t KeyaBusReceiveData;
	if (Keya_Bus.read(KeyaBusReceiveData)) {
		// heartbeat 0x07000001
		// 0-1 - Cumulative value of angle (360 deg / circle), high byte first
		// 2-3 - Motor speed, signed int
		// 4-5 - Motor current, byte 4 == 0xFF flags negative current
		// 6-7 - Control_Close (error code) - not yet consumed
		if (KeyaBusReceiveData.id == 0x07000001) {
			uint16_t encTick = ((uint16_t)KeyaBusReceiveData.buf[0] << 8) | (uint16_t)KeyaBusReceiveData.buf[1];
			keyaUpdateEncoder(encTick);

			if (KeyaBusReceiveData.buf[4] == 0xFF) {
				KeyaCurrentSensorReading = (0.95 * KeyaCurrentSensorReading) + (0.05 * (256 - KeyaBusReceiveData.buf[5]) * 20);
			}
			else {
				KeyaCurrentSensorReading = (0.95 * KeyaCurrentSensorReading) + (0.05 * KeyaBusReceiveData.buf[5] * 20);
			}
		}
	}
}
