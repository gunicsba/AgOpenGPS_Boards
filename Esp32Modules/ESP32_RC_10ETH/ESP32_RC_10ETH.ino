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

#define InoDescription "RCESP32   24-SEP-2023"
#define InoID 11073    // change to send defaults to eeprom, ddmmy, no leading 0

#define MaxReadBuffer 100 // bytes
#define MaxProductCount 1
#include <esp_task_wdt.h>
#define WDT_TIMEOUT 90

struct ModuleConfig 
{
  uint8_t ID = 0;
  uint8_t ProductCount = 1;       // up to 2 sensors
  uint8_t IPpart2 = 168;      // ethernet IP address
  uint8_t IPpart3 = 1;
  uint8_t IPpart4 = 50;     // 60 + ID
  uint8_t RelayOnSignal = 0;      // value that turns on relays
  uint8_t FlowOnDirection = 0;  // sets on value for flow valve or sets motor direction
  uint8_t RelayControl = 2;   // 0 - no relays, 1 - RS485, 2 - PCA9555 8 relays, 3 - PCA9555 16 relays, 4 - MCP23017, 5 - Teensy GPIO
  uint8_t MotorDriverType = 0;  // serial port to connect to ESP8266
  uint8_t RelayPins[16];      // pin numbers when GPIOs are used for relay control (5)
  uint8_t LOADCELL_DOUT_PIN[MaxProductCount];
  uint8_t LOADCELL_SCK_PIN[MaxProductCount];
  uint8_t Debounce = 3;     // minimum ms pin change
};

ModuleConfig MDL;

struct SensorConfig 
{
  uint8_t FlowPin = 0;
  uint8_t DirPin = 0;
  uint8_t PWMPin = 0;
  bool MasterOn = false;
  bool FlowEnabled = false;
  float RateError = 0;    // rate error X 1000
  float UPM = 0;        // upm X 1000
  uint16_t pwmSetting = 0;
  uint32_t CommTime = 0;
  byte InCommand = 0;     // command byte from RateController
  byte ControlType = 0;   // 0 standard, 1 combo close, 2 motor, 3 motor/weight, 4 fan
  uint32_t TotalPulses = 0;
  float RateSetting = 0;
  float MeterCal = 0;
  float ManualAdjust = 0;
  bool UseMultiPulses = 0;  // 0 - time for one pulse, 1 - average time for multiple pulses
  float KP = 5;
  float KI = 0;
  float KD = 0;
  byte MinPWM = 5;
  byte MaxPWM = 255;
  byte Deadband = 3;
  byte BrakePoint = 20;
};

SensorConfig Sensor[MaxProductCount];

struct AnalogConfig
{
  int16_t AIN0; // Pressure 0
  int16_t AIN1; // Pressure 1
  int16_t AIN2; 
  int16_t AIN3;
};

AnalogConfig AINs;

AsyncUDP UDPcomm;
static bool eth_connected = false;
uint16_t ListeningPort = 28888;
uint16_t DestinationPort = 29999;
IPAddress DestinationIP(192, MDL.IPpart2, MDL.IPpart3, 255);
IPAddress gateway(192, MDL.IPpart2, MDL.IPpart3, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress LocalIP(192, MDL.IPpart2, MDL.IPpart3, MDL.IPpart4);
static uint8_t LocalMac[] = { 0x00,0x20,0x42,0x01,0x00,0x10 };

// Relays
byte RelayLo = 0; // sections 0-7
byte RelayHi = 0; // sections 8-15
byte PowerRelayLo;
byte PowerRelayHi;

const uint16_t LoopTime = 50;      //in msec = 20hz
uint32_t LoopLast = LoopTime;
const uint16_t SendTime = 200;
uint32_t SendLast = SendTime;

bool AutoOn = true;

uint8_t ErrorCount;
bool ADSfound = false;
const int16_t AdsI2Caddress = 0x48;
//PCF8574 pcf(0x20);//PCF address 0x20 - A2, A1, A0 shorted to GND

uint32_t Analogtime;
uint32_t SaveTime;
int LED_BUILTIN = 2;

HX711 scale[2];
bool ScaleFound[2] = { false,false };
int8_t PWM1_Ch[] = { 0,1 };
int8_t PWM2_Ch[] = { 2,3 };
#define PWM1_Res   8
#define PWM1_Freq  1000
float debug1;

byte DataEthernet[MaxReadBuffer];
int DataEthernetLen;

void setup()
{    
  //watchdog timer
  Serial.println("WDT setup.");
  esp_task_wdt_init(WDT_TIMEOUT, true);
  esp_task_wdt_add(NULL);

  // initial scale pins
  MDL.LOADCELL_DOUT_PIN[0] = 16;
  MDL.LOADCELL_SCK_PIN[0] = 17;

  // default relay pins, RC11
  uint8_t Pins[] = { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 };
  //memcpy(MDL.RelayPins, Pins, 16);

  // default flow pins
  Sensor[0].FlowPin = 14;
  Sensor[0].DirPin = 32;
  Sensor[0].PWMPin = 33;
  
  if (MaxProductCount > 1) {
    Sensor[1].FlowPin = 12;
    Sensor[1].DirPin = 26;
    Sensor[1].PWMPin = 27;
  }

  // default pid
  Sensor[0].MinPWM = 5;
  Sensor[0].MaxPWM = 50;
  Sensor[0].Deadband = 3;
  Sensor[0].BrakePoint = 20;

  if (MaxProductCount > 1) {
    Sensor[1].MinPWM = 5;
    Sensor[1].MaxPWM = 50;
    Sensor[1].Deadband = 3;
    Sensor[1].BrakePoint = 20;
  }

  unsigned long PulseTimeISR0 = micros();
  unsigned long PulseTimeISR1 = micros();

/*  ledcAttachPin(Sensor[0].PWMPin, PWM1_Ch[0]);
  ledcAttachPin(Sensor[1].PWMPin, PWM1_Ch[1]);
  ledcAttachPin(Sensor[0].DirPin, PWM2_Ch[0]);
  ledcAttachPin(Sensor[1].DirPin, PWM2_Ch[1]);
  ledcSetup(PWM1_Ch[0], PWM1_Freq, PWM1_Res);
  ledcSetup(PWM1_Ch[1], PWM1_Freq, PWM1_Res);
  ledcSetup(PWM2_Ch[0], PWM1_Freq, PWM1_Res);
  ledcSetup(PWM2_Ch[1], PWM1_Freq, PWM1_Res);
*/

  // eeprom
  int16_t StoredID;
  EEPROM.begin(EEPROM_SIZE);
  EEPROM.get(100, StoredID);
  if (StoredID == InoID)
  {
    EEPROM.get(110, MDL);

    for (int i = 0; i < MaxProductCount; i++)
    {
      EEPROM.get(200 + i * 80, Sensor[i]);
    } EEPROM.commit();
  }
  else
  {
    EEPROM.put(100, InoID);
    EEPROM.put(110, MDL);

    for (int i = 0; i < MaxProductCount; i++)
    {
      EEPROM.put(200 + i * 80, Sensor[i]);
    } EEPROM.commit();
  }

  if (MDL.ProductCount < 1) MDL.ProductCount = 1;
  if (MDL.ProductCount > MaxProductCount) MDL.ProductCount = MaxProductCount;

  MDL.IPpart4 = MDL.ID + 60;
  if (MDL.IPpart4 > 255) MDL.IPpart4 = 255 - MDL.ID;

  Serial.begin(38400);
  delay(5000);
  Serial.println();
  Serial.println(InoDescription);
  Serial.print("Module ID: ");
  Serial.println(MDL.ID);
  Serial.println();
  
  // I2C
  Wire.begin(4,5);     // I2C on pins SCL 19, SDA 18
  //Wire.setClock(400000);  //Increase I2C data rate to 400kHz

   // PCF8574 setup output and LOW
   if (MDL.RelayControl == 2)
  {
    Serial.println("PCF8574 ...");      
    for (int i = 0; i < 8; i++) { 
      //pinMode(pcf, i, OUTPUT); 
      //digitalWrite(pcf, i, 1); 
    }
  }

  // ADS1115
  Serial.println("Starting ADS ...");
  ErrorCount = 0;
/*
  while (!ADSfound)
  {
    Wire.beginTransmission(AdsI2Caddress);
    Wire.write(0b00000000); //Point to Conversion register
    Wire.endTransmission();
    Wire.requestFrom(AdsI2Caddress, 2);
    ADSfound = Wire.available();
    Serial.print(".");
    delay(500);
    if (ErrorCount++ > 10) break;
  }
*/
  Serial.println("");
  if (ADSfound)
  {
    Serial.println("ADS connected.");
    Serial.println("");
  }
  else
  {
    Serial.println("ADS not found.");
    Serial.println("");
  }

  Serial.print("MDL.ProductCount is");
  Serial.print( MDL.ProductCount);  
  Serial.println("");
  // sensors
  for (int i = 0; i < MDL.ProductCount; i++)
  {
    Serial.print("Setting up sensor ");
    Serial.print(i);
    Serial.println("");
    pinMode(Sensor[i].FlowPin, INPUT_PULLUP);

    switch (i)
    {
    case 0:
      Serial.print("Attaching interrupt ISR0 to GPIO");
      Serial.print(Sensor[i].FlowPin);
      Serial.println("");
      attachInterrupt(digitalPinToInterrupt(Sensor[i].FlowPin), ISR0, FALLING);
      break;
    case 1:
      attachInterrupt(digitalPinToInterrupt(Sensor[i].FlowPin), ISR1, FALLING);
      break;
    }
  }

  // Relay Pins
  if (MDL.RelayControl == 5)
  {
    for (int i = 0; i < 16; i++)
    {
      if (MDL.RelayPins[i] > 0)
      {
        pinMode(MDL.RelayPins[i], OUTPUT);
      }
    }
  }

//  scanI2CDevices();

  // load cell
  for (int i = 0; i < MaxProductCount; i++)
  {
    Serial.print("Initializing scale ");
    Serial.println(i);
    ErrorCount = 0;
    ScaleFound[i] = false;
/*
    if (MDL.LOADCELL_DOUT_PIN[i] > 1 && MDL.LOADCELL_SCK_PIN[i] > 1)
    {
      scale[i].begin(MDL.LOADCELL_DOUT_PIN[i], MDL.LOADCELL_SCK_PIN[i]);
      pinMode(MDL.LOADCELL_DOUT_PIN[i], INPUT_PULLUP);
      while (!ScaleFound[i])
      {
        ScaleFound[i] = scale[i].wait_ready_timeout(1000);
        Serial.print(".");
        delay(500);
        if (ErrorCount++ > 5) break;
      }
    }
*/
    Serial.println("");
    if (ScaleFound[i])
    {
      Serial.println("HX711 found.");
    }
    else
    {
      Serial.println("HX711 not found.");
    }
    Serial.println("");
  }

//  pinMode(LED_BUILTIN, OUTPUT);
  sectionSetup();
  Serial.println("");
  Serial.println("Section setup done.");
  Serial.println("");
    
  //Eth_Start();
  Eth8720_Start();
  Serial.println("Ethernet setup done.");
  // UDP
  //UDPcomm.begin(ListeningPort);
  UDPcomm.listen(LocalIP, ListeningPort);
  UDPcomm.onPacket([](AsyncUDPPacket packet) {
    for (int i=0; ((i<packet.length()) && (i<MaxReadBuffer)); i++) {
      DataEthernet[i] = packet.data()[i];
      DataEthernetLen = i+1;
    }
  });

  pinMode(12, OUTPUT);
  
  Serial.println("");
  Serial.println("Finished setup.");
  Serial.println("");
}

void loop()
{
  if (millis() - LoopLast >= LoopTime)
  {
    LoopLast = millis();

    for (int i = 0; i < MDL.ProductCount; i++)
    {
      Sensor[i].FlowEnabled = (millis() - Sensor[i].CommTime < 4000) &&
        ((Sensor[i].RateSetting > 0 && Sensor[i].MasterOn)
          || ((Sensor[i].ControlType == 4) && (Sensor[i].RateSetting > 0))
          || (!AutoOn && Sensor[i].MasterOn));
//      Serial.print("Current FlowEnabled is : ");
//      Serial.print(Sensor[i].FlowEnabled);
/*      Serial.print("Current RateSetting : ");
      Serial.print(Sensor[i].RateSetting);
      Serial.print("Current MasterOn : ");
      Serial.print(Sensor[i].MasterOn);
      Serial.print("!AutoOn");
      Serial.print(!AutoOn);
      Serial.println("Setting it to true anyways :) ");
      Sensor[i].FlowEnabled=true;*/
    }

    GetUPM();
    CheckRelays();
    AdjustFlow();

    if (AutoOn)
    {
      AutoControl();
    }
    else
    {
      ManualControl();
    }
  }

  if (millis() - SendLast > SendTime)
  {
    SendLast = millis();
    SendData();
  }

  if (millis() - Analogtime > 2)
  {
    Analogtime = millis();
    ReadAnalog();
  }

  if (millis() - SaveTime > 3600000)  // 1 hour
  {
    // save sensor data
    SaveTime = millis();
    EEPROM.put(100, InoID);
   
    for (int i = 0; i < MaxProductCount; i++)
    {
      EEPROM.put(200 + i * 60, Sensor[i]);
    } EEPROM.commit();
  }

  ReceiveData();

  Blink();
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

void AutoControl()
{
  for (int i = 0; i < MDL.ProductCount; i++)
  {
    Sensor[i].RateError = Sensor[i].RateSetting - Sensor[i].UPM;

    switch (Sensor[i].ControlType)
    {
    case 2:
    case 3:
    case 4:
      // motor control
      Sensor[i].pwmSetting = PIDmotor(i);
      break;

    default:
      // valve control
      Sensor[i].pwmSetting = PIDvalve(i);
      break;
    }
  }
}

void ManualControl()
{
  for (int i = 0; i < MDL.ProductCount; i++)
  {
    Sensor[i].pwmSetting = Sensor[i].ManualAdjust;
  }
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
   

void Blink()
{
  if (BlinkTmr > 5000)
  {
    BlinkTmr = 0;
    //    digitalWrite(LED_BUILTIN, State);
    State = !State;
    digitalWrite(12, State);
    Serial.println(".");  // needed to allow PCBsetup to connect

    Serial.print(" Micros: ");
    Serial.print(MaxLoopTime);

    //Serial.print(", Temp: ");
    

    //Serial.print(", dBm: ");
    //Serial.print(", ");
    //Serial.print(WifiTime);
    //
    //Serial.print(", ");
    //Serial.print(debug2);

    //Serial.print("pulsecount");
    //Serial.print(ResetTimerOn);
   // Serial.print( PulseCount[0]);

    //Serial.println("");

    if (ReadReset++ > 10)
    {
      ReadReset = 0;
      MaxLoopTime = 0;
    }
  }
  if (LoopTmr > MaxLoopTime) MaxLoopTime = LoopTmr;
  LoopTmr = 0;
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
