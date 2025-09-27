
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

uint8_t fjdSteerPGN[] = { 0x23, 0x00, 0x20, 0x01, 0,0,0,0 }; // last 4 bytes change ofc
uint8_t KeyaHeartbeat[] = { 0, 0, 0, 0, 0, 0, 0, 0, };

// templates for matching responses of interest
uint8_t keyaCurrentResponse[] = { 0x60, 0x12, 0x21, 0x01 };

uint64_t fjdPGN = 0x201;

bool fjdON = false;
const bool fjdSafety = false;
const bool debugFjd = false;
elapsedMillis fjdMsgMillis = 0;
elapsedMillis fjdCANdata = 0;
/*
0: 02 == engage trigger cycle see 200h for status!!
1: speed + direction
2: speed + direction
3:  19 / 01
4: Proportional Gain: Range: 4 (04h) .. 125 (7d) default: 25
5: Manual steering activity: Min: 64 Range: 14 .. 5F? High: 3A 3F Max: 64
6: BIT 8: Manual override 01 ON  00 Manual override OFF
7
*/

uint8_t fjdPreviousStatus = 0xAA;
char buffer [60];

void keyaSend(uint8_t data[]) {
	//TODO Use this optimisation function once we're happy things are moving the right way
	CAN_message_t fjdBusSendData;
	fjdBusSendData.id = fjdPGN;
	fjdBusSendData.flags.extended = false;
	fjdBusSendData.len = 8;
	memcpy(fjdBusSendData.buf, data, sizeof(data));
	Keya_Bus.write(fjdBusSendData);
}

void CAN_Setup() {
	Keya_Bus.begin();
	Keya_Bus.setBaudRate(1000000);
	// Dedicated bus, zero chat from others. No need for filters
//	CAN_message_t msgV;
//	msgV.id = fjdPGN;
//	msgV.flags.extended = false;
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
	if (debugFjd) Serial.println("Initialised FJD CANBUS");
}

bool isPatternMatch(const CAN_message_t& message, const uint8_t* pattern, size_t patternSize) {
	return memcmp(message.buf, pattern, patternSize) == 0;
}

void enablefjdSteer()
{
  triggerfjdSteer(true);
}

void disablefjdSteer()
{
  triggerfjdSteer(false);
}

void triggerfjdSteer(bool desired) {
  if(fjdON == desired) return; //we're good
	CAN_message_t fjdBusSendData;
	fjdBusSendData.id = fjdPGN;
	fjdBusSendData.flags.extended = false;
	fjdBusSendData.len = 8;
	fjdBusSendData.buf[0] = 0x02;
	fjdBusSendData.buf[1] = 0x00;
	fjdBusSendData.buf[2] = 0x00;
	fjdBusSendData.buf[3] = 0x19;
	fjdBusSendData.buf[4] = 00;
	fjdBusSendData.buf[5] = 16;
	fjdBusSendData.buf[6] = 00;
	fjdBusSendData.buf[7] = 00;
	Keya_Bus.write(fjdBusSendData);
	//if (debugFjd) Serial.println("Disabled Keya motor");
}

void disablefjdSteerTEST() {
  CAN_message_t fjdBusSendData;
  fjdBusSendData.id = fjdPGN;
  fjdBusSendData.flags.extended = false;
  fjdBusSendData.len = 8;
  fjdBusSendData.buf[0] = 0x03;
  fjdBusSendData.buf[1] = 0x0d;
  fjdBusSendData.buf[2] = 0x20;
  fjdBusSendData.buf[3] = 0x11;
  fjdBusSendData.buf[4] = 0;
  fjdBusSendData.buf[5] = 0;
  fjdBusSendData.buf[6] = 0;
  fjdBusSendData.buf[7] = 0;
  Keya_Bus.write(fjdBusSendData);
  //if (debugFjd) Serial.println("Disabled Keya motor");
}



void SteerKeya(int steerSpeed) {
	int16_t actualSpeed = map(steerSpeed, -255, 255, -5000, 5000); //TODO
	if (pwmDrive == 0) {
		disablefjdSteer();
		//if (debugFjd) Serial.println("pwmDrive zero - disabling");
		return; // don't need to go any further, if we're disabling, we're disabling
	}
	if (debugFjd) Serial.println("told to steer, with " + String(steerSpeed) + " so....");
	if (debugFjd) Serial.println("I converted that to speed " + String(actualSpeed));

	enablefjdSteer();
	CAN_message_t fjdBusSendData;
	fjdBusSendData.id = fjdPGN;
	fjdBusSendData.flags.extended = false;
	fjdBusSendData.len = 8;
	fjdBusSendData.buf[0] = 0x00;
	fjdBusSendData.buf[1] = highByte(actualSpeed);
	fjdBusSendData.buf[2] = lowByte(actualSpeed);
	fjdBusSendData.buf[3] = 0x19;
  fjdBusSendData.buf[4] = 0x00;
  if(fjdSafety) {
    fjdBusSendData.buf[5] = steerConfig.PulseCountMax;
    fjdBusSendData.buf[6] = steerConfig.CurrentSensor;
  } else 
  {
    fjdBusSendData.buf[5] = 0x00;
    fjdBusSendData.buf[6] = 0x00;
  }
  fjdBusSendData.buf[7] = 0x00;

	Keya_Bus.write(fjdBusSendData);
}


void fjdBus_Receive() {
	CAN_message_t fjdBusReceiveData;
	if (Keya_Bus.read(fjdBusReceiveData)) {

    if (fjdBusReceiveData.id == 0x200) //Generic status
    {
      fjdCANdata = 0;
      switch(fjdBusReceiveData.buf[0])
      {
        case 0x13: //error
          triggerfjdSteer(false); //clear error message
          fjdON = false;
          if (steerConfig.CurrentSensor && fjdSafety) 
          {
          	steerSwitch = 1; // reset values like it turned off
					  currentState = 1;
					  previous = 0;
          }
          break;
        case 0x00: //ready
          fjdON = false;
          break;
        case 0x05: //ON also?
        case 0x06: //ON
        case 0x07: //ON also?
        case 0x09: //ON also?
          fjdON = true;
          break;
        case 0x08: //Shutting down
          fjdON = false;
          break;
        default:
          fjdON = false;
          break;

      }
      snprintf(buffer, sizeof(buffer), "FJD: %s (code %02X) from %s ", fjdStatusName(fjdBusReceiveData.buf[0]), fjdBusReceiveData.buf[0], fjdStatusName(fjdPreviousStatus));
      if(fjdPreviousStatus != fjdBusReceiveData.buf[0]) {
        if (debugFjd) sendHardwareMessage(buffer, 1);
        fjdMsgMillis = 0;
      } 
      else {
        if(fjdMsgMillis > 5000) { //Send last status every 5 second
          if (debugFjd) sendHardwareMessage(buffer, 1);
          fjdMsgMillis = 0;
        }
      }
      fjdPreviousStatus = fjdBusReceiveData.buf[0];


    } else if (debugFjd){
            Serial.print(", FJD-Bus"); 
            Serial.print(", MB: "); Serial.print(fjdBusReceiveData.mb);
            Serial.print(", ID: 0x"); Serial.print(fjdBusReceiveData.id, HEX );
            Serial.print(", EXT: "); Serial.print(fjdBusReceiveData.flags.extended );
            Serial.print(", LEN: "); Serial.print(fjdBusReceiveData.len);
            Serial.print(", DATA: ");
            for ( uint8_t i = 0; i < 8; i++ ) 
            {
              Serial.print(fjdBusReceiveData.buf[i]); Serial.print(", ");
            }
  
            Serial.println("");
    }
  } else if(fjdCANdata > 100) {
    if(fjdON) {
      snprintf(buffer, sizeof(buffer), "No CAN data from FJD wheel but it should be ON %s ", fjdStatusName(fjdPreviousStatus));
    }
    else {
      snprintf(buffer, sizeof(buffer), "No CAN data from FJD wheel but it should be OFF %s ", fjdStatusName(fjdPreviousStatus));
    }
    if(fjdMsgMillis > 2000) { //Send last status every 5 second
      sendHardwareMessage(buffer, 2);
      fjdMsgMillis = 0;
    }
    fjdCANdata = 0;
  }
}

// Small map from code → human-readable text
struct FJDStatus { uint8_t code; const char* name; };
static const FJDStatus FJD_STATUS_MAP[] = {
  {0x00, "Ready"},
  {0x03, "ON 3"},
  {0x05, "ON 5"},
  {0x06, "ON 6"},
  {0x07, "ON 7"},
  {0x09, "? 9 ?"},
  {0x08, "Shutting down"},
  {0x13, "Error"},
};

static const char* fjdStatusName(uint8_t code) {
  for (auto &e : FJD_STATUS_MAP) if (e.code == code) return e.name;

   // Fallback: print as hex
  static char buf[20];  // enough for "0xFF\0"
  snprintf(buf, sizeof(buf), "Unknown 0x%02X", code);
  return buf; 
}

void sendHardwareMessage(String message, int seconds) {

          Serial.print("Sending Hardware message!!                  ");
          Serial.println(message);

          uint8_t hardwareMessage[128] = { 0x80, 0x81, 0x7E, 221 };

          int msgLen = message.length();  // UTF-8 byte count (assuming no extended chars)
          int totalLength = 7 + msgLen + 1; // header(7) + message + CRC(1)

          hardwareMessage[4] = msgLen + 2;
          hardwareMessage[5] = seconds; //seconds to display
          hardwareMessage[6] = 0; //color 0 or 1
          
          // Copy message bytes into hardwareMessage[7..]
          message.getBytes(&hardwareMessage[7], msgLen + 1);  // +1 for null-terminator safety

          //checksum
          int16_t CK_A = 0;
          for (uint8_t i = 2; i < 7 + msgLen; i++)
          {
            CK_A = (CK_A + hardwareMessage[i]);
          }
          hardwareMessage[7 + msgLen] = CK_A;  // CRC

          SendUdp(hardwareMessage, totalLength, Eth_ipDestination, portDestination);

}
