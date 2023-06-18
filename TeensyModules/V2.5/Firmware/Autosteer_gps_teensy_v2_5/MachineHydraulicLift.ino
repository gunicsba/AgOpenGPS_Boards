/*
   UDP Hydraulic Lift for Teensy 4.1
   For AgOpenGPS
   11 Feb 2023, Balazs Gunics
   Like all Arduino code - copied from somewhere else :)
   Idea is based on Entropiemaximun & farmerbriantee github repos
   So don't claim it as your own


*/


  ////////////////// User Settings ///////////////////////// 
#define HYDRAULIC_ENABLED true
#define HYDRAULIC_DEBUG false


//This Implementation uses the 3 pins on the Ampseal header to control hydraulics.
//No section control sorry

//These are the pins available on the AIO boards AMPSEAL:
#define HYDRAULIC_LIFT_OR_UP 26 //A12 was: Hyd_up Used to lift up the hydraulics
#define HYDRAULIC_LOWER_OR_DOWN 27//A13 was: Hyd_down Used to lower the hydraulics 
#define HYDRAULIC_TRAMLINE 38 //A14

//

#include <Wire.h>
#include <EEPROM.h>




  //Variables for config - 0 is false  
    struct Config {
        uint8_t raiseTime = 2;
        uint8_t lowerTime = 4;
        uint8_t enableToolLift = 0;
        uint8_t isRelayActiveHigh = 0; //if zero, active low (default)

        uint8_t user1 = 0; //user defined values set in machine tab
        uint8_t user2 = 0;
        uint8_t user3 = 0;
        uint8_t user4 = 0; //0 - disabled , 1 - Pulls the relay and keeps it that way , 2 - pulls and release it after N second

    };  Config hydConfig;   //8 bytes


  /*
    * Functions as below assigned to pins
    0: -
    17,18    Hyd Up, Hyd Down, 
    19 Tramline, 
    20: Geo Stop
    */

    //24 possible pins assigned to these functions
    uint8_t pin[] = { 1,2,3,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 };
 
  //The variables used for storage
  uint8_t tramline = 0, hydLift = 0, hydLiftPrev = 3;
  uint32_t raiseTimer = 0, lowerTimer = 0;

///////////////////////////////

  //reset function
  void(* resetFunc) (void) = 0;

  void HydraulicSetup() {
      if (!HYDRAULIC_ENABLED) return;

      hydraulicConfigEprom(false); //read
      Serial.println("Setting up Hydraulics!");
      pinMode(HYDRAULIC_LIFT_OR_UP, OUTPUT);
      pinMode(HYDRAULIC_LOWER_OR_DOWN, OUTPUT);
      pinMode(HYDRAULIC_TRAMLINE, OUTPUT);
      digitalWrite(HYDRAULIC_LIFT_OR_UP, hydConfig.isRelayActiveHigh);
      digitalWrite(HYDRAULIC_LOWER_OR_DOWN, hydConfig.isRelayActiveHigh);
      digitalWrite(HYDRAULIC_TRAMLINE, hydConfig.isRelayActiveHigh);

      Serial.println("Hydraulics are ready!");
      Serial.println("Active High: ");
      Serial.print(hydConfig.isRelayActiveHigh);
      Serial.println("NOT Active High: ");
      Serial.print(!hydConfig.isRelayActiveHigh);
  }

void hydraulicLoop(uint8_t hydraulicUdpData[UDP_TX_PACKET_MAX_SIZE])
{
    // When ethernet is not running, return directly. parsePacket() will block when we don't
    if (!HYDRAULIC_ENABLED ) return;

    //Check if it's data  && Machine data? //is also checked in autosteer but in case we want to refactor...
    if (hydraulicUdpData[0] != 0x80 || hydraulicUdpData[1] != 0x81 || hydraulicUdpData[2] != 0x7F ) return;
    

    if(hydraulicUdpData[3] == 239) //Hydraulic State
    {
//      Serial.println("Hydraulic loop start!");
    
      //uTurn = hydraulicUdpData[5];
      //gpsSpeed = (float)hydraulicUdpData[6];//actual speed times 4, single uint8_t

      hydLift = hydraulicUdpData[7];
    
      tramline = hydraulicUdpData[8];  //bit 0 is right bit 1 is left

      //relayLo = hydraulicUdpData[11];          // read relay control from AgOpenGPS
      //relayHi = hydraulicUdpData[12];
      //Bit 13 CRC

       prettyPrint239();
       hydraulicExecute();
    }
    else if(hydraulicUdpData[3] == 238) //Hydraulic config
    {
       hydConfig.raiseTime = hydraulicUdpData[5];
       hydConfig.lowerTime = hydraulicUdpData[6];
       //hydConfig.enableToolLift = hydraulicUdpData[7]; //seems to be always 0

       uint8_t sett = hydraulicUdpData[8];  //setting0     
       if (bitRead(sett, 0)) hydConfig.isRelayActiveHigh = 1; else hydConfig.isRelayActiveHigh = 0;
       if (bitRead(sett, 1)) hydConfig.enableToolLift = 1; else hydConfig.enableToolLift = 0;

       hydConfig.user1 = hydraulicUdpData[9];
       hydConfig.user2 = hydraulicUdpData[10];
       hydConfig.user3 = hydraulicUdpData[11];
       hydConfig.user4 = hydraulicUdpData[12];
      //Bit 13 CRC
       hydraulicConfigEprom(true);
       prettyPrint238();
    }
    else if (hydraulicUdpData[3] == 236) //Hydraulic config
    {
        for (uint8_t i = 0; i < 24; i++)
        {
            pin[i] = hydraulicUdpData[i + 5];
        }

        //save in EEPROM and restart
//        EEPROM.put(20, pin);
        //Bit 13 CRC
        //hydraulicConfigEprom(true);
        prettyPrint236();
    }

} //End Loop

void hydraulicConfigEprom(bool write)
{
  if(write) {
    EEPROM.put(100, hydConfig);  
  } else {
    EEPROM.get(100, hydConfig);  
  }
}

void hydraulicExecute()
{

  if(hydLift == 0 || !hydConfig.enableToolLift) {
    triggerPin(HYDRAULIC_LIFT_OR_UP, hydConfig.isRelayActiveHigh ,0);
    triggerPin(HYDRAULIC_LOWER_OR_DOWN, hydConfig.isRelayActiveHigh ,0);
    triggerPin(HYDRAULIC_TRAMLINE, hydConfig.isRelayActiveHigh, 0);
    hydLiftPrev = 0;
    return; //Disabled
  }
  hydraulicTimedPins(); //check what needs to be disabled

  if (tramline == 0)
  {
      triggerPin(HYDRAULIC_TRAMLINE, hydConfig.isRelayActiveHigh, 0);
  }
  else //if (tramline == 1 || tramline == 2 || tramline == 3)
  {
      triggerPin(HYDRAULIC_TRAMLINE, !hydConfig.isRelayActiveHigh, 0);
  }

  if (hydLift == hydLiftPrev) return; //nothing changed nothing to do
  hydLiftPrev = hydLift; 
 
  if (hydLift == 1 && hydConfig.enableToolLift) //raise
  {
      if (hydConfig.user4 == 1) //start lifting we don't care about timing
      {
          triggerPin(HYDRAULIC_LIFT_OR_UP, !hydConfig.isRelayActiveHigh, 0);
          triggerPin(HYDRAULIC_LOWER_OR_DOWN, hydConfig.isRelayActiveHigh, 0);
      }
      else if (hydConfig.user4 == 2) //now we care about timing
      {
          lowerTimer = triggerPin(HYDRAULIC_LOWER_OR_DOWN, hydConfig.isRelayActiveHigh, 0);
          raiseTimer = triggerPin(HYDRAULIC_LIFT_OR_UP, !hydConfig.isRelayActiveHigh, hydConfig.raiseTime);
//          timedPinOff = triggerPin(HYDRAULIC_TIMED_PIN, true, 1);
      }
  }
  else if(hydLift == 2 && hydConfig.enableToolLift) //lower
  {
    if(hydConfig.user4 == 1) //let it down we don't care about timing
    {
      triggerPin(HYDRAULIC_LIFT_OR_UP, hydConfig.isRelayActiveHigh, 0);
      triggerPin(HYDRAULIC_LOWER_OR_DOWN, !hydConfig.isRelayActiveHigh, 0);
      
    }
    else if (hydConfig.user4 == 2) //now we care about timing
    {
        raiseTimer = triggerPin(HYDRAULIC_LIFT_OR_UP, hydConfig.isRelayActiveHigh, 0);
        lowerTimer = triggerPin(HYDRAULIC_LOWER_OR_DOWN, !hydConfig.isRelayActiveHigh, hydConfig.lowerTime);

//        timedPinOff = triggerPin(HYDRAULIC_TIMED_PIN, true, 1);
    }
  }

}


void hydraulicTimedPins() 
{
    if (raiseTimer  && millis() > raiseTimer )  raiseTimer  = triggerPin(HYDRAULIC_LIFT_OR_UP, hydConfig.isRelayActiveHigh, 0);
    if (lowerTimer  && millis() > lowerTimer )  lowerTimer  = triggerPin(HYDRAULIC_LOWER_OR_DOWN, hydConfig.isRelayActiveHigh, 0);
}

int triggerPin(int pin, bool state, int timer) {
    Serial.print("Trigger Pin ");
    Serial.print(pin);
    Serial.print(" - ");
    Serial.print(state);
    Serial.print(" - ");
    Serial.print(timer);
    digitalWrite(pin, state);
    if(timer) return millis() + (1000 * timer);
    return 0;
}


void prettyPrint239()
{
  if(!HYDRAULIC_DEBUG) return;

  Serial.print("\r\nHydraulics - 239 - Machine info:");
  Serial.print(" user4 ");
  Serial.print(hydConfig.user4);
  Serial.print(" hydLift ");
  Serial.print(hydLift);
  Serial.print(" hydLiftPrev ");
  Serial.print(hydLiftPrev);
  Serial.print(" tramLine ");
  Serial.print(tramline);
  Serial.print(" enableToolLift ");
  Serial.print(hydConfig.enableToolLift);
  Serial.print("\r\n");
}

void prettyPrint238()
{
  if(!HYDRAULIC_DEBUG) return;

  Serial.print("\r\n Hydraulics -238 - Config: ");
  Serial.print("Raise Time: ");
  Serial.print(hydConfig.raiseTime);
  Serial.print(" Lower time: ");
  Serial.print(hydConfig.lowerTime);
  Serial.print(" Enable toolift: ");
  Serial.print(hydConfig.enableToolLift);
  Serial.print(" isRelayActiveHigh ");
  Serial.print(hydConfig.isRelayActiveHigh);
  Serial.print(" User1: ");
  Serial.print(hydConfig.user1);
  Serial.print(" User2: ");
  Serial.print(hydConfig.user2);
  Serial.print(" User3: ");
  Serial.print(hydConfig.user3);
  Serial.print(" User4: ");
  Serial.print(hydConfig.user4);
}

void prettyPrint236()
{
    if (!HYDRAULIC_DEBUG) return;
    Serial.print("\r\n Hydraulics - 236 ");
    /* Functions as below assigned to pins
        0: -
        17, 18    Hyd Up, Hyd Down,
        19 Tramline,
        20 : Geo Stop
    */
    Serial.print(" Tramline: ");
    Serial.print(pin[19]);
    Serial.print(" Geo Stop: ");
    Serial.print(pin[20]);
    Serial.print(" Hyd up/down: ");
    Serial.print(pin[17]);
    Serial.print(pin[18]);
    Serial.print("\r\n Sections: ");
    for (int i = 0; i < 17; i++) {
        Serial.print(pin[i]);
    }
}