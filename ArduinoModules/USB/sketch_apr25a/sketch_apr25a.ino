#include <TinyGPSPlus.h>

#include <SoftwareSerial.h>

//This example code is in the Public Domain (or CC0 licensed, at your option.)
//By Evandro Copercini - 2018
//
//This example creates a bridge between Serial and Classical Bluetooth (SPP)
//and also demonstrate that SerialBT have the same functionalities of a normal Serial

#include "BluetoothSerial.h"
#include <Pins_Arduino.h>

// The serial connection to the GPS module
SoftwareSerial ss(D2, D3); // Here are the aliased pins 
//  SoftwareSerial SUART(2, 3); //SRX = DPin-2; STX = DPin-3"
TinyGPSPlus gps;
byte *buffer;

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;
bool ledstatus;

void setup() {
  Serial.begin(115200);
  ss.begin(9600);
  SerialBT.begin("ESP32test"); //Bluetooth device name
  Serial.println("The device started, now you can pair it with bluetooth!");
  pinMode(LED_BUILTIN, OUTPUT);
  ledstatus=false;
  digitalWrite(LED_BUILTIN, ledstatus);

}

void loop() {

  while(ss.available() > 0){
    *buffer = ss.read();
//    SerialBT.write(buffer);
    gps.encode( *buffer);
    if(gps.location.isUpdated()){
      Serial.print("Latitude=");
      Serial.print(gps.location.lat());

      Serial.print(" Longitude= "); 
      Serial.println(gps.location.lng(), 6);

      Serial.println(gps.time.value()); 

      // Hour (0-23) (u8)
      Serial.print("Hour = "); 
      Serial.println(gps.time.hour()); 
      // Minute (0-59) (u8)
      Serial.print("Minute = "); 
      Serial.println(gps.time.minute()); 
      // Second (0-59) (u8)
      Serial.print("Second = "); 
      Serial.println(gps.time.second()); 
      // 100ths of a second (0-99) (u8)
      Serial.print("Centisecond = "); 
      Serial.println(gps.time.centisecond()); 
    }

    if (SerialBT.available()) {
      ss.write(SerialBT.read());
    }
 
    ledstatus = !ledstatus;
    digitalWrite(LED_BUILTIN, ledstatus);
    delay(20);

  }
  digitalWrite(LED_BUILTIN, HIGH);

}