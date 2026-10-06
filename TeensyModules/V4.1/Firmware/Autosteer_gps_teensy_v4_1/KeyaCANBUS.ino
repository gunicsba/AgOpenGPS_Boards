
// KeyaCANBUS
// Trying to get Keya to steer the tractor over CANBUS

#define lowByte(w) ((uint8_t)((w) & 0xFF))
#define highByte(w) ((uint8_t)((w) >> 8))

//Enable	0x23 0x0D 0x20 0x01 0x00 0x00 0x00 0x00
//Disable	0x23 0x0C 0x20 0x01 0x00 0x00 0x00 0x00
//Fast clockwise	0x23 0x00 0x20 0x01 0xFC 0x18 0xFF 0xFF (0xfc18 signed dec is - 1000
//Anti - clockwise	0x23 0x00 0x20 0x01 0x03 0xE8 0x00 0x00 (0x03e8 signed dec is 1000
//Slow clockwise	0x23 0x00 0x20 0x01 0xFE 0x0C 0xFF 0xFF (0xfe0c signed dec is - 500)
//Slow anti - clockwise	0x23 0x00 0x20 0x01 0x01 0xf4 0x00 0x00 (0x01f4 signed dec is 500)

uint8_t KeyaSteerPGN[] = { 0x23, 0x00, 0x20, 0x01, 0,0,0,0 }; // last 4 bytes change ofc
uint8_t KeyaHeartbeat[] = { 0, 0, 0, 0, 0, 0, 0, 0, };

// templates for matching responses of interest
uint8_t keyaCurrentResponse[] = { 0x60, 0x12, 0x21, 0x01 };

uint64_t KeyaPGN = 0x06000001;

// Allynav motors speak the same command set as Keya, just on different IDs (from Andreas' AllyNavKeyaCANBUS.ino)
#define KEYA_CMD_ID 0x06000001
#define KEYA_HEARTBEAT_ID 0x07000001
#define ALLYNAV_CMD_ID 0x06c73001
#define ALLYNAV_HEARTBEAT_ID 0x04854001

// Motor type and baud are autodetected: step through these until a Keya or Allynav heartbeat shows up
const uint32_t keyaBaudRates[] = { 250000, 500000, 125000, 1000000 };
const uint8_t nrKeyaBaudRates = sizeof(keyaBaudRates) / sizeof(keyaBaudRates[0]);
const uint16_t KEYA_SCAN_STEP_MS = 400;
uint8_t keyaBaudIndex = 0;
uint8_t keyaScanUnknownPrinted = 0;
elapsedMillis keyaScanTimer;

bool keyaDetected = false;
bool isAllynav = false;
uint8_t allynavLastError1 = 0;
uint8_t allynavLastError2 = 0;
elapsedMillis keyaHeartbeatAge;
elapsedMillis keyaStatusTimer;
uint32_t keyaFramesSeen = 0;

const bool debugKeya = true;

void keyaSend(uint8_t data[]) {
	//TODO Use this optimisation function once we're happy things are moving the right way
	CAN_message_t KeyaBusSendData;
	KeyaBusSendData.id = KeyaPGN;
	KeyaBusSendData.flags.extended = true;
	KeyaBusSendData.len = 8;
	memcpy(KeyaBusSendData.buf, data, sizeof(data));
	Keya_Bus.write(KeyaBusSendData);
}

// Poke both motor types with a disable - Allynav wants a command before it starts its heartbeat
void keyaScanPoke() {
	KeyaPGN = KEYA_CMD_ID;
	disableKeyaSteer();
	KeyaPGN = ALLYNAV_CMD_ID;
	disableKeyaSteer();
	KeyaPGN = KEYA_CMD_ID;
}

void keyaScanSetBaud() {
	Keya_Bus.setBaudRate(keyaBaudRates[keyaBaudIndex]);
	keyaScanUnknownPrinted = 0;
	keyaScanTimer = 0;
	if (debugKeya) Serial.println("Keya/Allynav scan: trying " + String(keyaBaudRates[keyaBaudIndex]) + " baud");
	keyaScanPoke();
}

// Andreas' Allynav init: enable, write parameter 0x0132 = 2, disable
void allynavInit() {
	enableKeyaSteer();
	delay(10);

	CAN_message_t msg;
	msg.id = ALLYNAV_CMD_ID;
	msg.flags.extended = true;
	msg.len = 8;
	msg.buf[0] = 0x24;
	msg.buf[1] = 0x01;
	msg.buf[2] = 0x32;
	msg.buf[3] = 0x02;
	msg.buf[4] = 0x00;
	msg.buf[5] = 0x02;
	msg.buf[6] = 0x00;
	msg.buf[7] = 0x00;
	Keya_Bus.write(msg);
	delay(10);

	disableKeyaSteer();
}

void keyaMotorDetected(bool allynav) {
	keyaDetected = true;
	isAllynav = allynav;
	KeyaPGN = allynav ? ALLYNAV_CMD_ID : KEYA_CMD_ID;
	Serial.println(String(allynav ? "Allynav" : "Keya") + " heartbeat detected at " + String(keyaBaudRates[keyaBaudIndex]) + " baud");
	if (allynav) allynavInit();
}

void CAN_Setup() {
	Keya_Bus.begin();
	keyaScanSetBaud();
	// Dedicated bus, zero chat from others. No need for filters
//	CAN_message_t msgV;
//	msgV.id = KeyaPGN;
//	msgV.flags.extended = true;
//	msgV.len = 8;
//	// claim an address. Don't think I need to do this tho
//	// anyway, just pinched this from Claas address. TODO, looks like we can do without, ditch this
//	msgV.buf[0] = 0x00;
//	msgV.buf[1] = 0x00;
//	msgV.buf[2] = 0xC0;
//	msgV.buf[3] = 0x0C;
//	msgV.buf[4] = 0x00;
//	msgV.buf[5] = 0x17;
//	msgV.buf[6] = 0x02;
//	msgV.buf[7] = 0x20;
//	Keya_Bus.write(msgV);
	delay(1000);
	if (debugKeya) Serial.println("Initialised Keya CANBUS");
}

bool isPatternMatch(const CAN_message_t& message, const uint8_t* pattern, size_t patternSize) {
	return memcmp(message.buf, pattern, patternSize) == 0;
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
	//if (debugKeya) Serial.println("Disabled Keya motor");
}

void disableKeyaSteerTEST() {
  CAN_message_t KeyaBusSendData;
  KeyaBusSendData.id = KeyaPGN;
  KeyaBusSendData.flags.extended = true;
  KeyaBusSendData.len = 8;
  KeyaBusSendData.buf[0] = 0x03;
  KeyaBusSendData.buf[1] = 0x0d;
  KeyaBusSendData.buf[2] = 0x20;
  KeyaBusSendData.buf[3] = 0x11;
  KeyaBusSendData.buf[4] = 0;
  KeyaBusSendData.buf[5] = 0;
  KeyaBusSendData.buf[6] = 0;
  KeyaBusSendData.buf[7] = 0;
  Keya_Bus.write(KeyaBusSendData);
  //if (debugKeya) Serial.println("Disabled Keya motor");
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
	int actualSpeed = isAllynav ? map(steerSpeed, -255, 255, -900, 900) : map(steerSpeed, -255, 255, -995, 998);
	if (pwmDrive == 0) {
		disableKeyaSteer();
		//if (debugKeya) Serial.println("pwmDrive zero - disabling");
		return; // don't need to go any further, if we're disabling, we're disabling
	}
	if (debugKeya) Serial.println("told to steer, with " + String(steerSpeed) + " so....");
	if (debugKeya) Serial.println("I converted that to speed " + String(actualSpeed));

	CAN_message_t KeyaBusSendData;
	KeyaBusSendData.id = KeyaPGN;
	KeyaBusSendData.flags.extended = true;
	KeyaBusSendData.len = 8;
	KeyaBusSendData.buf[0] = 0x23;
	KeyaBusSendData.buf[1] = 0x00;
	KeyaBusSendData.buf[2] = 0x20;
	KeyaBusSendData.buf[3] = 0x01;
	if (steerSpeed < 0) {
		KeyaBusSendData.buf[4] = highByte(actualSpeed); // TODO take PWM in instead for speed (this is -1000)
		KeyaBusSendData.buf[5] = lowByte(actualSpeed);
		KeyaBusSendData.buf[6] = 0xff;
		KeyaBusSendData.buf[7] = 0xff;
		if (debugKeya) Serial.println("pwmDrive < zero - clockwise - steerSpeed " + String(steerSpeed));
	}
	else {
		KeyaBusSendData.buf[4] = highByte(actualSpeed);
		KeyaBusSendData.buf[5] = lowByte(actualSpeed);
		KeyaBusSendData.buf[6] = 0x00;
		KeyaBusSendData.buf[7] = 0x00;
		if (debugKeya) Serial.println("pwmDrive > zero - anticlock-clockwise - steerSpeed " + String(steerSpeed));
	}
	Keya_Bus.write(KeyaBusSendData);
	enableKeyaSteer();
}


// Allynav error bits per datasheet (from Andreas), only printed when they change
void checkAllynavErrors(uint8_t error1, uint8_t error2) {
	if (error1 == allynavLastError1 && error2 == allynavLastError2) return;
	allynavLastError1 = error1;
	allynavLastError2 = error2;

	String msg = "";
	// Byte7 (error1)
	if (bitRead(error1, 0)) msg += "MotorNotEnabled ";
	if (bitRead(error1, 1)) msg += "OverVoltage ";
	if (bitRead(error1, 2)) msg += "HWOverCurrent ";
	if (bitRead(error1, 3)) msg += "EEPROM ";
	if (bitRead(error1, 4)) msg += "UnderVoltage ";
	if (bitRead(error1, 5)) msg += "PosDeviation ";
	if (bitRead(error1, 6)) msg += "SWOverCurrent ";
	if (bitRead(error1, 7)) msg += "ControlModeErr ";
	// Byte6 (error2)
	if (bitRead(error2, 0)) msg += "WorkModeFail ";
	if (bitRead(error2, 1)) msg += "SpeedDeviation ";
	if (bitRead(error2, 2)) msg += "OverTemp ";
	if (bitRead(error2, 3)) msg += "EncoderFail ";
	if (bitRead(error2, 4)) msg += "OtherError ";
	if (bitRead(error2, 6)) msg += "CANCommLost ";
	if (bitRead(error2, 7)) msg += "Stall ";

	Serial.println("Allynav status: " + (msg.length() ? msg : String("OK")) + "| raw 0x" + String(error1, HEX) + " 0x" + String(error2, HEX));
}

void KeyaBus_Receive() {
	CAN_message_t KeyaBusReceiveData;
	while (Keya_Bus.read(KeyaBusReceiveData)) {
		keyaFramesSeen++;
		if (KeyaBusReceiveData.id == KEYA_HEARTBEAT_ID || KeyaBusReceiveData.id == ALLYNAV_HEARTBEAT_ID) keyaHeartbeatAge = 0;

		if (!keyaDetected) {
			if (KeyaBusReceiveData.id == KEYA_HEARTBEAT_ID) keyaMotorDetected(false);
			else if (KeyaBusReceiveData.id == ALLYNAV_HEARTBEAT_ID) keyaMotorDetected(true);
			else if (keyaScanUnknownPrinted < 5) {
				// Right baud but unknown IDs - print them so we can see what the motor talks
				keyaScanUnknownPrinted++;
				Serial.print("Keya/Allynav scan: unknown frame 0x");
				Serial.print(KeyaBusReceiveData.id, HEX);
				for (uint8_t i = 0; i < KeyaBusReceiveData.len; i++) {
					Serial.print(" ");
					Serial.print(KeyaBusReceiveData.buf[i], HEX);
				}
				Serial.println();
			}
		}

		// Allynav heartbeat 0x04854001
		// 0-1 - Motor current, 0.1A
		// 2-3 - Motor speed, signed
		// 6-7 - error bits
		if (isAllynav && KeyaBusReceiveData.id == ALLYNAV_HEARTBEAT_ID) {
			// Same scale as Keya (amps * 20) so the AOG current kickout setting means the same for both
			int16_t allynavCurrent = (int16_t)((KeyaBusReceiveData.buf[0] << 8) | KeyaBusReceiveData.buf[1]);
			KeyaCurrentSensorReading = (0.95 * KeyaCurrentSensorReading) + (0.05 * abs(allynavCurrent) * 2);
			KeyaCurrentSensorReading = min(KeyaCurrentSensorReading, 255);
			checkAllynavErrors(KeyaBusReceiveData.buf[7], KeyaBusReceiveData.buf[6]);
		}

		// parse the different message types
		// heartbeat 0x07000001
   // change heartbeat time in the software, default is 20ms
		if (!isAllynav && KeyaBusReceiveData.id == KEYA_HEARTBEAT_ID) {
			// 0-1 - Cumulative value of angle (360 def / circle)
			// 2-3 - Motor speed, signed int eg -500 or 500
			// 4-5 - Motor current, with "symbol" ? Signed I think that means, but it does appear to be a crap int. 1, 2 for 1, 2 amps etc
			//		is that accurate enough for us?
			// 6-7 - Control_Close (error code)
			// TODO Yeah, if we ever see something here, fire off a disable, refuse to engage autosteer or..?
			//KeyaCurrentSensorReading = abs((int16_t)((KeyaBusReceiveData.buf[5] << 8) | KeyaBusReceiveData.buf[4]));
			//if (KeyaCurrentSensorReading > 255) KeyaCurrentSensorReading -= 255;
			// Manual: signed, high byte first. Float + clamp so it can't wrap (int8 wrapped above ~6A)
			int16_t keyaCurrent = (int16_t)((KeyaBusReceiveData.buf[4] << 8) | KeyaBusReceiveData.buf[5]);
			KeyaCurrentSensorReading = (0.95 * KeyaCurrentSensorReading) + (0.05 * abs(keyaCurrent) * 20);
			KeyaCurrentSensorReading = min(KeyaCurrentSensorReading, 255);
			//if (debugKeya) Serial.println("Heartbeat current is " + String(KeyaCurrentSensorReading));
		}

		// response from most commands 0x05800001
		// could have been separate PGNs, but oh no...

		//if (KeyaBusReceiveData.id == 0x05800001) {
		//	// response to current request (this is also in heartbeat)
		//	if (isPatternMatch(KeyaBusReceiveData, keyaCurrentResponse, sizeof(keyaCurrentResponse))) {
		//		// Current is unsigned float in [4]
		//		// set the motor current variable, when you find out what that is
		//		KeyaCurrentSensorReading = KeyaBusReceiveData.buf[4];
		//		if (debugKeya) Serial.println("Returned current is " + KeyaCurrentSensorReading);
		//	}
		//	else if (1 == 0) {
		//		// placeholder for more checks
		//	}
		//}
	}

	if (keyaStatusTimer > 2000) {
		keyaStatusTimer = 0;
		if (keyaDetected) {
			Serial.println(String(isAllynav ? "Allynav" : "Keya") + " @" + String(keyaBaudRates[keyaBaudIndex])
				+ " | frames " + String(keyaFramesSeen) + " | last heartbeat " + String((uint32_t)keyaHeartbeatAge) + "ms ago"
				+ " | current " + String(KeyaCurrentSensorReading / 20.0, 2) + "A | pwmDrive " + String(pwmDrive));
		}
		else {
			Serial.println("Keya/Allynav: no heartbeat yet, scanning (" + String(keyaFramesSeen) + " frames seen)");
		}
	}

	// Nothing heard at this baud yet - move on to the next one
	if (!keyaDetected && keyaScanTimer > KEYA_SCAN_STEP_MS) {
		keyaBaudIndex = (keyaBaudIndex + 1) % nrKeyaBaudRates;
		keyaScanSetBaud();
	}
}
