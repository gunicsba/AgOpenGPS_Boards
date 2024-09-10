//#include <pcf8574.h>
#include <elapsedMillis.h>
#include <Wire.h>
#include <EEPROM.h> 
#define EEPROM_SIZE 512


#include <ETH.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <Update.h>

#include <AsyncUDP.h>

#include <EthernetUdp.h>
#include <HX711.h>     


// rate control with ESP32

#define InoDescription "RCESP32   12-JUL-2023"
#define InoID 11072    // change to send defaults to eeprom, ddmmy, no leading 0

#define MaxReadBuffer 100 // bytes
#define MaxProductCount 1
#include <esp_task_wdt.h>
#define WDT_TIMEOUT 90

const uint16_t LoopTime = 50;      //in msec = 20hz
uint32_t LoopLast = LoopTime;
const uint16_t SendTime = 200;
uint32_t SendLast = SendTime;


//-----------------------------------------------------------------------------------------------
// Change this number to reset and reload default parameters To EEPROM
#define EEP_Ident 0x5425  
//the default network address
struct ConfigIP {
    uint8_t ipOne = 192;
    uint8_t ipTwo = 168;
    uint8_t ipThree = 5;
};  ConfigIP networkAddress;   //3 bytes
//-----------------------------------------------------------------------------------------------


AsyncUDP UDPcomm;
static bool eth_connected = false;
byte DataEthernet[MaxReadBuffer];
int DataEthernetLen;
IPAddress src_ip;


// ethernet interface ip address
static uint8_t myip[] = { 0,0,0,123 };

// gateway ip address
static uint8_t gwip[] = { 0,0,0,1 };

//DNS- you just need one anyway
static uint8_t myDNS[] = { 8,8,8,8 };

//mask
static uint8_t mask[] = { 255,255,255,0 };

//this is port of this autosteer module
uint16_t portMy = 5123;

//sending back to where and which port
static uint8_t ipDestination[] = { 0,0,0,255 };
uint16_t portDestination = 9999; //AOG port that listens

// ethernet mac address - must be unique on your network
static uint8_t mymac[] = { 0x00,0x00,0x56,0x00,0x00,0x7B };

//Variables for config - 0 is false  
struct Config {
    uint8_t raiseTime = 2;
    uint8_t lowerTime = 4;
    uint8_t enableToolLift = 0;
    uint8_t isRelayActiveHigh = 0; //if zero, active low (default)

    uint8_t user1 = 0; //user defined values set in machine tab
    uint8_t user2 = 0;
    uint8_t user3 = 0;
    uint8_t user4 = 0;

};  Config aogConfig;   //4 bytes

//Program counter reset
void(*resetFunc) (void) = 0;

//ethercard 10,11,12,13 Nano = 10 depending how CS of ENC28J60 is Connected
#define CS_Pin 10
#define NUM_OF_SECTIONS 7 //16 relays max for PCA9685                                                             //<-

/*
* Functions as below assigned to pins
0: -
1 thru 16: Section 1,Section 2,Section 3,Section 4,Section 5,Section 6,Section 7,Section 8,
            Section 9, Section 10, Section 11, Section 12, Section 13, Section 14, Section 15, Section 16,
17,18    Hyd Up, Hyd Down,
19 Tramline,
20: Geo Stop
21,22,23 - unused so far*/
uint8_t pin[] = { 1,2,3,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 };

//read value from Machine data and set 1 or zero according to list
uint8_t relayState[] = { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 };


//hello from AgIO
uint8_t helloFromMachine[] = { 128, 129, 123, 123, 5, 0, 0, 0, 0, 0, 71 };

const uint8_t LOOP_TIME = 200; //5hz
uint32_t lastTime = LOOP_TIME;
uint32_t currentTime = LOOP_TIME;
uint32_t fifthTime = 0;
uint16_t count = 0;

//Comm checks
uint8_t watchdogTimer = 20; //make sure we are talking to AOG
uint8_t serialResetTimer = 0; //if serial buffer is getting full, empty it

bool isRaise = false, isLower = false;

//Communication with AgOpenGPS
int16_t temp, EEread = 0;

//Parsing PGN
bool isPGNFound = false, isHeaderFound = false;
uint8_t pgn = 0, dataLength = 0, idx = 0;
int16_t tempHeader = 0;

//settings pgn
uint8_t PGN_237[] = { 0x80,0x81, 0x7f, 237, 8, 1, 2, 3, 4, 0,0,0,0, 0xCC };
int8_t PGN_237_Size = sizeof(PGN_237) - 1;

//The variables used for storage
uint8_t relayHi = 0, relayLo = 0, tramline = 0, uTurn = 0, hydLift = 0, geoStop = 0;
float gpsSpeed;
uint8_t raiseTimer = 0, lowerTimer = 0, lastTrigger = 0;

uint8_t onLo = 0, offLo = 0, onHi = 0, offHi = 0, mainByte = 0;


bool PCAFound = false;


void setup()
{    
  //set the baud rate
  Serial.begin(38400);

  //watchdog timer
  Serial.println("WDT setup.");
  esp_task_wdt_init(WDT_TIMEOUT, true);
  esp_task_wdt_add(NULL);

  EEPROM.get(0, EEread);              // read identifier

  if (EEread != EEP_Ident)   // check on first start and write EEPROM
  {
      EEPROM.put(0, EEP_Ident);
      EEPROM.put(6, aogConfig);
      EEPROM.put(20, pin);
      EEPROM.put(50, networkAddress);
  }
  else
  {
      EEPROM.get(6, aogConfig);
      EEPROM.get(20, pin);
      EEPROM.get(50, networkAddress);
  }

  // I2C
  Wire.begin(4,5);     // I2C on pins SCL 19, SDA 18
  //Wire.setClock(400000);  //Increase I2C data rate to 400kHz
  sectionSetup();
  Serial.println("Section setup done.");
  Serial.println("");    
  //Eth_Start();
  myip[0] = networkAddress.ipOne;
  myip[1] = networkAddress.ipTwo;
  myip[2] = networkAddress.ipThree;
  gwip[0] = networkAddress.ipOne;
  gwip[1] = networkAddress.ipTwo;
  gwip[2] = networkAddress.ipThree;
  Eth8720_Start();
  Serial.println("Ethernet setup done.");
  //IPAddress(myip[0],myip[1],myip[2],myip[3])  
  UDPcomm.listen( IPAddress(myip), 8888);
  UDPcomm.onPacket([](AsyncUDPPacket packet) {
    src_ip = packet.remoteIP();
    for (int i=0; ((i<packet.length()) && (i<MaxReadBuffer)); i++) {
      DataEthernet[i] = packet.data()[i];
      DataEthernetLen = i+1;
    }
  });

  Serial.println("");
  Serial.println("Finished setup.");
  Serial.println("");
}

void loop()
{
  if (millis() - LoopLast >= LoopTime)
  {
    LoopLast = millis();
    for (int i = 0; i < 7; i++)
    {
      if (bitRead(relayLo, i)) {
        sectionHandler(i, "on", 4096);
      } else {
        sectionHandler(i, "off", 4096);
      }
    }
  }

  if (millis() - SendLast > SendTime)
  {
    SendLast = millis();
    SendData();
  }

  ReceiveData();

  wdt_timer();
}

byte ParseModID(byte ID)
{
  // top 4 bits
  return ID >> 4;
}

byte ParseSenID(byte ID)
{
  // bottom 4 bits
  return (ID & 0b00001111);
}

byte BuildModSenID(byte Mod_ID, byte Sen_ID)
{
  return ((Mod_ID << 4) | (Sen_ID & 0b00001111));
}

bool GoodCRC(byte Data[], byte Length)
{
  byte ck = CRC(Data, Length - 1, 0);
  bool Result = (ck == Data[Length - 1]);
  return Result;
}

byte CRC(byte Chk[], byte Length, byte Start)
{
  byte Result = 0;
  int CK = 0;
  for (int i = Start; i < Length; i++)
  {
    CK += Chk[i];
  }
  Result = (byte)CK;
  return Result;
}


bool State = false;
elapsedMillis BlinkTmr;
elapsedMicros LoopTmr;
byte ReadReset;
uint32_t MaxLoopTime;
int last1 = millis();

void wdt_timer() {
  

  // resetting WDT every 2s, 5 times only
  if (millis() - last1 >= 5000) {
      Serial.println("Resetting WDT...");
      esp_task_wdt_reset();
      last1 = millis();
      }
}
   


String scanI2CDevices(){
  String forReturn="";
  byte error, address;
  int nDevices;
  Serial.println("Scanning...");   /*ESP32 starts scanning available I2C devices*/
  forReturn += "Scanning...\n";
  nDevices = 0;
  for(address = 1; address < 127; address++ ) {   /*for loop to check number of devices on 127 address*/
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0) {   /*if I2C device found*/
      Serial.print("I2C device found at address 0x");/*print this line if I2C device found*/
      forReturn += "I2C device found at address 0x";
      if (address<16) {
        Serial.print("0");
        forReturn += "0";
      }
      Serial.println(address,HEX);  /*prints the HEX value of I2C address*/
      forReturn += String(address, HEX);
      nDevices++;
    }
    else if (error==4) {
      Serial.print("Unknown error at address 0x");
      if (address<16) {
        Serial.print("0");
      }
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0) {
    Serial.println("No I2C devices found\n"); /*If no I2C device attached print this message*/
    forReturn += "No I2C devices found\n";
  }
  else {
    Serial.println("done\n");
  }
  return forReturn;
}
