/*
   UDP Autosteer code for Teensy 4.1
   For AgOpenGPS
   01 Feb 2022
   Like all Arduino code - copied from somewhere else :)
   So don't claim it as your own
*/

////////////////// User Settings /////////////////////////

//How many degrees before decreasing Max PWM
#define LOW_HIGH_DEGREES 3.0

/*  PWM Frequency ->
     490hz (default) = 0
     122hz = 1
     3921hz = 2
*/
#define PWM_Frequency 0

/////////////////////////////////////////////

// if not in eeprom, overwrite
// Bumped from 2400 for the SteerDriverType field added to Setup (Phase 1), and again from
// 2500 for the PressureSensorType field (Phase 3) - each forces a clean re-init on first boot
// after the struct's shape changes. Without the bump, the new trailing byte(s) get read as
// whatever garbage was already sitting in that EEPROM region, not a defined default - harmless
// here since PressureSensorType treats any unrecognized value as generic, but not something to
// rely on. Losing old hydraulic-lift/steer settings on the upgrade itself is accepted.
#define EEP_Ident 2501

//   ***********  Steering driver type  **************888
#define STEER_DRIVER_HYDRAULIC 0   // Cytron / IBT2 / Danfoss-valve PWM
#define STEER_DRIVER_KEYA 1        // Keya CAN motor (see KeyaCANBUS.ino)

//   ***********  Kickout pressure-sensor type (only matters when steerConfig.PressureSensor)
// Replaces the old compile-time JOHNDEERE flag with a runtime, web-UI-selectable choice.
#define PRESSURE_SENSOR_GENERIC   0  // analog voltage, read directly
#define PRESSURE_SENSOR_JOHNDEERE 1  // PWM duty cycle (JD factory sensor)
#define PRESSURE_SENSOR_DANFOSS   2  // pulse frequency mapped to pressure %
#define EEPROM_ADDR_PRESSURE_MAX_HZ 190  // float - frequency (Hz) that reads as 100% for PRESSURE_SENSOR_DANFOSS
float pressureSensorMaxHz = 200.0f;

//   ***********  Wasless (Keya-encoder-as-WAS) mode  **************888
// Active only when SteerDriverType == STEER_DRIVER_KEYA and the AOG "Danfoss" checkbox
// (steerConfig.IsDanfoss) is set - reusing that bit is safe because a real Danfoss valve
// and a Keya motor are never fitted to the same board. When active, steerAngleActual comes
// from Keya's own CAN encoder instead of the ADS1115 WAS, continuously re-zeroed by the
// auto-zero engine below (ported from AIO_Keya_WasKeyaFiltre) while driving straight.
//
// EEPROM layout note: these addresses are placed *after* hydConfig (100-107, see
// MachineHydraulicLift.ino) with a deliberate gap, rather than reusing the 90/84/80
// addresses from the source project - those actually overlap hydConfig's range once both
// the hydraulic-lift feature and this wasless feature exist in the same firmware image,
// which would have the two silently corrupt each other's EEPROM on every save.
#define EEPROM_ADDR_KEYA_TICKS    114   // float - Keya encoder ticks-per-degree calibration
#define KEYA_TICKS_PER_DEG_DEFAULT 24.0f  // 4 motor turns / 60 deg lock-to-lock, per AIO source

float   keyaTicksPerDeg = KEYA_TICKS_PER_DEG_DEFAULT;
int32_t keyaZeroTicks = 0;
bool    wasZeroDone = false;
uint32_t stableStart = 0;
float   azCorrAccum = 0.0f; // sub-tick accumulation for the smooth in-guidance correction mode

// Keya's own CAN encoder position - defined in KeyaCANBUS.ino (compiled after this file,
// forward-declared here so autosteerLoop() can read it)
extern int32_t keyaEncoderRaw;

// Filtered GPS heading - defined in zHandlers.ino (compiled after this file)
extern float emaGpsHdg;

// TM171's yaw in plain degrees - defined in TM171.ino (compiled after this file)
extern float tm171YawDeg;

// Current heading in plain degrees from whichever gyro is active, for the auto-zero engine's
// yaw-rate check below. BNO's own `yaw` global is stored as degrees x10 (see zHandlers.ino),
// so it's converted here rather than compared directly against a plain deg/s threshold.
static float currentYawDeg()
{
  if (useTM171)  return tm171YawDeg;
  if (useBNO08x) return yaw / 10.0f;
  return 0.0f;
}

// Auto-zero tuning parameters - struct defined here, instance lives in zAutoZeroMenu.ino
// (also holds the serial menu ('z' key) used to tune these before the web UI existed)
struct AutoZeroParams {
  float    speedMin;
  float    yawRateMax;
  float    gpsHdgMax;
  uint32_t timeSlowMs;
  uint32_t timeFastMs;
  float    speedSlow;
  float    speedFast;
  uint8_t  useBno;      // 1 = use BNO yaw rate as a stability condition
  uint8_t  useGps;      // 1 = use GPS heading-rate as a stability condition
  float    beta;        // in-guidance correction speed (0.01=slow .. 0.2=fast)
  uint16_t ident;
};
extern AutoZeroParams azParams;

//   ***********  Motor drive connections  **************888
//Connect ground only for cytron, Connect Ground and +5v for IBT2

//Dir1 for Cytron Dir, Both L and R enable for IBT2
#define DIR1_RL_ENABLE  4

//PWM1 for Cytron PWM, Left PWM for IBT2
#define PWM1_LPWM  2

//Not Connected for Cytron, Right PWM for IBT2
#define PWM2_RPWM  3

//--------------------------- Switch Input Pins ------------------------
#define STEERSW_PIN 32
#define WORKSW_PIN 34
#define REMOTE_PIN 37

//Define sensor pin for current or pressure sensor
#define CURRENT_SENSOR_PIN A17
#define PRESSURE_SENSOR_PIN A10
elapsedMicros dutyTime = 0;
float dutyTimeCurrent = 0;
float dutyTimePrev = 0;

// Danfoss pulse-frequency pressure sensor (PRESSURE_SENSOR_DANFOSS)
volatile uint32_t danfossPulseCount = 0;
elapsedMillis danfossWindowTimer = 0;
const uint16_t DANFOSS_WINDOW_MS = 200;
void ISRDanfossPulse() { danfossPulseCount++; }

#define CONST_180_DIVIDED_BY_PI 57.2957795130823

#include <Wire.h>
#include <EEPROM.h>
#include "zADS1115.h"
ADS1115_lite adc(ADS1115_DEFAULT_ADDRESS);     // Use this for the 16-bit version ADS1115

#include <IPAddress.h>
#include "BNO08x_AOG.h"

#ifdef ARDUINO_TEENSY41
// ethernet
#include <NativeEthernet.h>
#include <NativeEthernetUdp.h>
#endif

#ifdef ARDUINO_TEENSY41
//uint8_t Ethernet::buffer[200]; // udp send and receive buffer
uint8_t autoSteerUdpData[UDP_TX_PACKET_MAX_SIZE];  // Buffer For Receiving UDP Data
#endif

//loop time variables in microseconds
const uint16_t LOOP_TIME = 25;  //40Hz
uint32_t autsteerLastTime = LOOP_TIME;
uint32_t currentTime = LOOP_TIME;

const uint16_t WATCHDOG_THRESHOLD = 100;
const uint16_t WATCHDOG_FORCE_VALUE = WATCHDOG_THRESHOLD + 2; // Should be greater than WATCHDOG_THRESHOLD
uint8_t watchdogTimer = WATCHDOG_FORCE_VALUE;

//Heart beat hello AgIO
uint8_t helloFromIMU[] = { 128, 129, 121, 121, 5, 0, 0, 0, 0, 0, 71 };
uint8_t helloFromAutoSteer[] = { 0x80, 0x81, 126, 126, 5, 0, 0, 0, 0, 0, 71 };
int16_t helloSteerPosition = 0;

//fromAutoSteerData FD 253 - ActualSteerAngle*100 -5,6, SwitchByte-7, pwmDisplay-8
uint8_t PGN_253[] = {0x80,0x81, 126, 0xFD, 8, 0, 0, 0, 0, 0,0,0,0, 0xCC };
int8_t PGN_253_Size = sizeof(PGN_253) - 1;

//fromAutoSteerData FD 250 - sensor values etc
uint8_t PGN_250[] = { 0x80,0x81, 126, 0xFA, 8, 0, 0, 0, 0, 0,0,0,0, 0xCC };
int8_t PGN_250_Size = sizeof(PGN_250) - 1;
uint8_t aog2Count = 0;
float sensorReading;
float sensorSample;
elapsedMillis sensorPulseReset;

elapsedMillis gpsSpeedUpdateTimer = 0;

//EEPROM
int16_t EEread = 0;

//Relays
bool isRelayActiveHigh = true;
uint8_t relay = 0, relayHi = 0, uTurn = 0;
uint8_t tram = 0;

//Switches
uint8_t remoteSwitch = 0, workSwitch = 0, steerSwitch = 1, switchByte = 0;

//On Off
uint8_t guidanceStatus = 0;
uint8_t prevGuidanceStatus = 0;
bool guidanceStatusChanged = false;

//speed sent as *10
float gpsSpeed = 0;

//steering variables
float steerAngleActual = 0;
float steerAngleSetPoint = 0; //the desired angle from AgOpen
int16_t steeringPosition = 0; //from steering sensor
float steerAngleError = 0; //setpoint - actual

//pwm variables
int16_t pwmDrive = 0, pwmDisplay = 0;
float pValue = 0;
float errorAbs = 0;
float highLowPerDeg = 0;

//Steer switch button  ***********************************************************************************************************
uint8_t currentState = 1, reading, previous = 0;
uint8_t pulseCount = 0; // Steering Wheel Encoder
bool encEnable = false; //debounce flag
uint8_t thisEnc = 0, lastEnc = 0;

//Variables for settings
struct Storage {
  uint8_t Kp = 40;              // proportional gain
  uint8_t lowPWM = 10;          // band of no action
  int16_t wasOffset = 0;
  uint8_t minPWM = 9;
  uint8_t highPWM = 60;         // max PWM value
  float steerSensorCounts = 30;
  float AckermanFix = 1;        // sent as percent
};  Storage steerSettings;      // 11 bytes

//Variables for settings - 0 is false
struct Setup {
  uint8_t InvertWAS = 0;
  uint8_t IsRelayActiveHigh = 0;    // if zero, active low (default)
  uint8_t MotorDriveDirection = 0;
  uint8_t SingleInputWAS = 1;
  uint8_t CytronDriver = 1;
  uint8_t SteerSwitch = 0;          // 1 if switch selected
  uint8_t SteerButton = 0;          // 1 if button selected
  uint8_t ShaftEncoder = 0;
  uint8_t PressureSensor = 0;
  uint8_t CurrentSensor = 0;
  uint8_t PulseCountMax = 5;
  uint8_t IsDanfoss = 0;
  uint8_t IsUseY_Axis = 0;     //Set to 0 to use X Axis, 1 to use Y avis
  uint8_t SteerDriverType = STEER_DRIVER_HYDRAULIC;  //STEER_DRIVER_HYDRAULIC or STEER_DRIVER_KEYA
  uint8_t PressureSensorType = PRESSURE_SENSOR_GENERIC;  //only used when PressureSensor == 1
}; Setup steerConfig;               // 15 bytes

void steerConfigInit()
{
  if (steerConfig.CytronDriver) 
  {
    pinMode(PWM2_RPWM, OUTPUT);
  }
}

void steerSettingsInit()
{
  // for PWM High to Low interpolator
  highLowPerDeg = ((float)(steerSettings.highPWM - steerSettings.lowPWM)) / LOW_HIGH_DEGREES;
}

void ISRJOHNDEERERISING(){
  attachInterrupt(digitalPinToInterrupt(PRESSURE_SENSOR_PIN), ISRJOHNDEEREFALLING, FALLING);
  dutyTime = 0;
  return;
}

void ISRJOHNDEEREFALLING(){
  attachInterrupt(digitalPinToInterrupt(PRESSURE_SENSOR_PIN), ISRJOHNDEERERISING, RISING);
  dutyTimeCurrent = dutyTime;
  return;
}

// Sets up PRESSURE_SENSOR_PIN for whichever PressureSensorType is configured. Must run after
// steerConfig is loaded from EEPROM (called from autosteerSetup(), below), since it used to
// run off a compile-time flag before the runtime value even existed.
void pressureSensorInit()
{
  detachInterrupt(digitalPinToInterrupt(PRESSURE_SENSOR_PIN));

  if (steerConfig.PressureSensorType == PRESSURE_SENSOR_JOHNDEERE)
  {
    pinMode(PRESSURE_SENSOR_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(PRESSURE_SENSOR_PIN), ISRJOHNDEERERISING, RISING);
  }
  else if (steerConfig.PressureSensorType == PRESSURE_SENSOR_DANFOSS)
  {
    pinMode(PRESSURE_SENSOR_PIN, INPUT);
    danfossPulseCount  = 0;
    danfossWindowTimer = 0;
    attachInterrupt(digitalPinToInterrupt(PRESSURE_SENSOR_PIN), ISRDanfossPulse, RISING);
  }
  else
  {
    pinMode(PRESSURE_SENSOR_PIN, INPUT_DISABLE);
  }
}


void autosteerSetup()
{
  //PWM rate settings. Set them both the same!!!!
  /*  PWM Frequency ->
       490hz (default) = 0
       122hz = 1
       3921hz = 2
  */
  if (PWM_Frequency == 0)
  {
    analogWriteFrequency(PWM1_LPWM, 490);
    analogWriteFrequency(PWM2_RPWM, 490);
  }
  else if (PWM_Frequency == 1)
  {
    analogWriteFrequency(PWM1_LPWM, 122);
    analogWriteFrequency(PWM2_RPWM, 122);
  }
  else if (PWM_Frequency == 2)
  {
    analogWriteFrequency(PWM1_LPWM, 3921);
    analogWriteFrequency(PWM2_RPWM, 3921);
  }

  //keep pulled high and drag low to activate, noise free safe
  pinMode(WORKSW_PIN, INPUT_PULLUP);
  pinMode(STEERSW_PIN, INPUT_PULLUP);
  pinMode(REMOTE_PIN, INPUT_PULLUP);
  pinMode(DIR1_RL_ENABLE, OUTPUT);

  // Disable digital inputs for analog input pins
  pinMode(CURRENT_SENSOR_PIN, INPUT_DISABLE);
  // PRESSURE_SENSOR_PIN mode depends on steerConfig.PressureSensorType, set up below via
  // pressureSensorInit() once steerConfig has actually been loaded from EEPROM.

  //set up communication
  Wire1.end();
  Wire1.begin();
    
  // Check ADC - fatality decided below, once steerConfig is loaded, since a Keya wasless
  // board (steering angle from the CAN encoder, not the ADS1115) doesn't need it.
  bool adcOk = adc.testConnection();
  if (adcOk)
  {
    Serial.println("ADC Connecton OK");
  }
  else
  {
    Serial.println("ADC Connecton FAILED!");
  }

  //50Khz I2C
  //TWBR = 144;   //Is this needed?

  EEPROM.get(0, EEread);              // read identifier

  if (EEread != EEP_Ident)            // check on first start and write EEPROM
  {
    EEPROM.put(0, EEP_Ident);
    EEPROM.put(10, steerSettings);
    EEPROM.put(40, steerConfig);
    EEPROM.put(60, networkAddress);
    hydraulicConfigEprom(true);
  }
  else
  {
    EEPROM.get(10, steerSettings);     // read the Settings
    EEPROM.get(40, steerConfig);
    EEPROM.get(60, networkAddress);
    hydraulicConfigEprom(false);
  }

  if (!adcOk)
  {
    if (steerConfig.SteerDriverType == STEER_DRIVER_KEYA && steerConfig.IsDanfoss)
      Serial.println("ADC not used in Keya wasless mode (SteerDriverType=Keya, IsDanfoss=1) - continuing.");
    else
      Autosteer_running = false;
  }

  steerSettingsInit();
  steerConfigInit();
  pressureSensorInit();

  // Restore the Keya encoder ticks-per-degree mechanical calibration
  {
    float savedTicks = 0.0f;
    EEPROM.get(EEPROM_ADDR_KEYA_TICKS, savedTicks);
    if (!isnan(savedTicks) && !isinf(savedTicks) && savedTicks > 1.0f && savedTicks < 500.0f)
      keyaTicksPerDeg = savedTicks;
    else
      keyaTicksPerDeg = KEYA_TICKS_PER_DEG_DEFAULT;
  }

  // Restore the Danfoss pulse-frequency calibration (frequency that reads as 100% pressure)
  {
    float savedHz = 0.0f;
    EEPROM.get(EEPROM_ADDR_PRESSURE_MAX_HZ, savedHz);
    if (!isnan(savedHz) && !isinf(savedHz) && savedHz > 1.0f && savedHz < 5000.0f)
      pressureSensorMaxHz = savedHz;
    else
      pressureSensorMaxHz = 200.0f;
  }

  wasZeroDone = false; // the zero must be re-established every boot

  if (Autosteer_running)
  {
    Serial.println("Autosteer running, waiting for AgOpenGPS");
    // Autosteer Led goes Red if ADS1115 is found
    digitalWrite(AUTOSTEER_ACTIVE_LED, 0);
    digitalWrite(AUTOSTEER_STANDBY_LED, 1);
  }
  else
  {
    Autosteer_running = false;  //Turn off auto steer if no ethernet (Maybe running T4.0)
//    if(!Ethernet_running)Serial.println("Ethernet not available");
    Serial.println("Autosteer disabled, GPS only mode");   
    return;
  }

  adc.setSampleRate(ADS1115_REG_CONFIG_DR_128SPS); //128 samples per second
  adc.setGain(ADS1115_REG_CONFIG_PGA_6_144V);

  azMenuSetup();   // load auto-zero tuning params from EEPROM (zAutoZeroMenu.ino)
  emaParamsLoad(); // load IMU EMA filter alphas from EEPROM (zWebConfig.ino)

}// End of Setup

void autosteerLoop()
{
#ifdef ARDUINO_TEENSY41
  ReceiveUdp();
#endif

  // Auto-zero tuning serial menu ('z' key in the serial monitor) - blocks the rest of the
  // loop only while actively in the menu. See zAutoZeroMenu.ino.
  if (azMenuLoop()) return;

  //Serial.println("AutoSteer loop");

  // Loop triggers every 100 msec and sends back gyro heading, and roll, steer angle etc
  currentTime = systick_millis_count;

  if (currentTime - autsteerLastTime >= LOOP_TIME)
  {
    autsteerLastTime = currentTime;

    //reset debounce
    encEnable = true;

    //If connection lost to AgOpenGPS, the watchdog will count up and turn off steering
    if (watchdogTimer++ > 250) watchdogTimer = WATCHDOG_FORCE_VALUE;

    //read all the switches
    workSwitch = digitalRead(WORKSW_PIN);  // read work switch

    // Engage: physical button and the tablet's onscreen button are both always live,
    // regardless of AOG's switch-type setting (SteerSwitch/SteerButton/None) - deliberately,
    // not an oversight. This fleet wires momentary buttons only (no toggle switches), and
    // buttons are a known field failure point: if the physical one breaks, the tablet button
    // still fully engages/disengages on its own, and vice versa. Ported from the dual-path
    // model in SteerReadyCAN. steerConfig.SteerSwitch/SteerButton are still parsed from AOG's
    // PGN 251 (can't change AOG's own UI) but no longer consulted here.
    //
    // The two inputs have different shapes and are handled accordingly: the physical button
    // is a momentary press, so it toggles currentState/steerSwitch on each press-edge. The
    // tablet button is a level (guidanceStatus bit 0 directly reflects AOG's desired engage
    // state), so it sets currentState/steerSwitch to match on each change rather than
    // toggling. Both write the same latch, so whichever acted most recently wins, and the
    // other input picks up correctly from there afterward.
    reading = digitalRead(STEERSW_PIN);
    if (reading == LOW && previous == HIGH)
    {
      if (currentState == 1)
      {
        currentState = 0;
        steerSwitch = 0;
      }
      else
      {
        currentState = 1;
        steerSwitch = 1;
      }
    }
    previous = reading;

    if (guidanceStatusChanged)
    {
      if (bitRead(guidanceStatus, 0) == 1)
      {
        steerSwitch   = 0;
        currentState  = 0;
      }
      else
      {
        steerSwitch   = 1;
        currentState  = 1;
      }
    }

    if (steerConfig.ShaftEncoder && pulseCount >= steerConfig.PulseCountMax)
    {
      steerSwitch = 1; // reset values like it turned off
      currentState = 1;
      previous = 0;
    }

    // Pressure sensor?
    if (steerConfig.PressureSensor)
    {
      if (steerConfig.PressureSensorType == PRESSURE_SENSOR_JOHNDEERE)
      {
        if(dutyTimeCurrent > 100 && dutyTimeCurrent < 4500)
        {
          //current dutyTime should be between
          if(abs(dutyTimeCurrent - dutyTimePrev) < 1000) // if it's more than 2000 we jumped...
          {
            sensorSample = abs((double)dutyTimeCurrent-2600)/5; //should make it into a smoother transition around 95 to 5 percent
           sensorReading = (min(abs( ( abs((double)dutyTimePrev-2600)/5 ) - sensorSample),255) * 0.6) + (sensorReading * 0.4);
          } else {
            sensorReading = 0;
          }
          dutyTimePrev = dutyTimeCurrent;
        }
      }
      else if (steerConfig.PressureSensorType == PRESSURE_SENSOR_DANFOSS)
      {
        if (danfossWindowTimer >= DANFOSS_WINDOW_MS)
        {
          noInterrupts();
          uint32_t count = danfossPulseCount;
          danfossPulseCount = 0;
          uint32_t windowMs = danfossWindowTimer;
          danfossWindowTimer = 0;
          interrupts();

          float hz = (float)count * 1000.0f / (float)windowMs;
          sensorSample = constrain((hz / pressureSensorMaxHz) * 255.0f, 0.0f, 255.0f);
          sensorReading = sensorReading * 0.6f + sensorSample * 0.4f;
        }
      }
      else // PRESSURE_SENSOR_GENERIC
      {
        sensorSample = (float)analogRead(PRESSURE_SENSOR_PIN);
        sensorSample *= 0.25;
        sensorReading = sensorReading * 0.6 + sensorSample * 0.4;
      }

      if (sensorReading >= steerConfig.PulseCountMax)
      {
          steerSwitch = 1; // reset values like it turned off
          currentState = 1;
          previous = 0;
      }
    }

    // Current sensor?
    if (steerConfig.CurrentSensor)
    {
      if (steerConfig.SteerDriverType == STEER_DRIVER_KEYA)
      {
        sensorReading = KeyaCurrentSensorReading; // fed by KeyaBus_Receive() heartbeat parsing
      }
      else
      {
        sensorSample = (float)analogRead(CURRENT_SENSOR_PIN);
        sensorSample = (abs(775 - sensorSample)) * 0.5;
        sensorReading = sensorReading * 0.9 + sensorSample * 0.1;
        sensorReading = min(sensorReading, 255);
      }

      if (sensorReading >= steerConfig.PulseCountMax)
      {
          steerSwitch = 1; // reset values like it turned off
          currentState = 1;
          previous = 0;
      }
    }

    remoteSwitch = digitalRead(REMOTE_PIN); //read auto steer enable switch open = 0n closed = Off
    switchByte = 0;
    switchByte |= (remoteSwitch << 2); //put remote in bit 2
    switchByte |= (steerSwitch << 1);   //put steerswitch status in bit 1 position
    switchByte |= workSwitch;

    /*
      #if Relay_Type == 1
        SetRelays();       //turn on off section relays
      #elif Relay_Type == 2
        SetuTurnRelays();  //turn on off uTurn relays
      #endif
    */

    // =================================================================
    // STEERING ANGLE
    //   SteerDriverType == Keya && IsDanfoss  -> Keya CAN encoder, wasless
    //   otherwise                             -> physical WAS via ADS1115
    // =================================================================
    bool wasless = (steerConfig.SteerDriverType == STEER_DRIVER_KEYA) && steerConfig.IsDanfoss;

    if (wasless)
    {
      int32_t deltaTicks = keyaEncoderRaw - keyaZeroTicks;
      float rawAngle = (float)deltaTicks / keyaTicksPerDeg;

      if (steerConfig.InvertWAS) rawAngle = -rawAngle;

      steerAngleActual   = rawAngle;
      helloSteerPosition = (int16_t)(rawAngle * 100.0f);
      steeringPosition   = (int16_t)deltaTicks;

      // Block guidance until the encoder zero has been established at least once
      if (!wasZeroDone) watchdogTimer = WATCHDOG_FORCE_VALUE;
    }
    else
    {
      //get steering position
      if (steerConfig.SingleInputWAS)   //Single Input ADS
      {
        adc.setMux(ADS1115_REG_CONFIG_MUX_SINGLE_0);
        steeringPosition = adc.getConversion();
        adc.triggerConversion();//ADS1115 Single Mode

        steeringPosition = (steeringPosition >> 1); //bit shift by 2  0 to 13610 is 0 to 5v
        helloSteerPosition = steeringPosition - 6800;
      }
      else    //ADS1115 Differential Mode
      {
        adc.setMux(ADS1115_REG_CONFIG_MUX_DIFF_0_1);
        steeringPosition = adc.getConversion();
        adc.triggerConversion();

        steeringPosition = (steeringPosition >> 1); //bit shift by 2  0 to 13610 is 0 to 5v
        helloSteerPosition = steeringPosition - 6800;
      }

      //DETERMINE ACTUAL STEERING POSITION

      //convert position to steer angle. 32 counts per degree of steer pot position in my case
      //  ***** make sure that negative steer angle makes a left turn and positive value is a right turn *****
      if (steerConfig.InvertWAS)
      {
        steeringPosition = (steeringPosition - 6805  - steerSettings.wasOffset);   // 1/2 of full scale
        steerAngleActual = (float)(steeringPosition) / -steerSettings.steerSensorCounts;
      }
      else
      {
        steeringPosition = (steeringPosition - 6805  + steerSettings.wasOffset);   // 1/2 of full scale
        steerAngleActual = (float)(steeringPosition) / steerSettings.steerSensorCounts;
      }
    }

    //Ackerman fix
    if (steerAngleActual < 0) steerAngleActual = (steerAngleActual * steerSettings.AckermanFix);

    // =================================================================
    // WASLESS AUTO-ZERO - continuously re-zeroes the Keya encoder while the tractor is
    // judged to be driving straight, fusing BNO yaw-rate and GPS heading-rate stability.
    // Ported from AIO_Keya_WasKeyaFiltre. Only meaningful (and only runs) in wasless mode.
    // =================================================================
    if (wasless)
    {
      static const float AZ_NEAR_ZERO_DEG    = 2.0f;
      static const float AZ_NEAR_ZERO_FACTOR = 0.3f;

      static float    azLastYaw    = 0.0f;
      static uint32_t azLastTime   = 0;
      static int64_t  azAccum      = 0;
      static uint32_t azCount      = 0;
      static uint32_t dbgLastPrint = 0;
      static uint32_t azCooldown   = 0;
      static bool     azYawInit    = false;
      static float    azLastGpsHdg = 0.0f;
      static bool     azGpsInit    = false;

      uint32_t nowMs = millis();

      bool guidanceActive = (watchdogTimer < WATCHDOG_THRESHOLD);

      // --- Gyro yaw rate [deg/s], from whichever of BNO/TM171 is active ---
      float yawNow = currentYawDeg();
      float yawRate = 0.0f;
      if (!azYawInit) {
        azLastYaw  = yawNow;
        azLastTime = nowMs;
        azYawInit  = true;
      } else {
        float dt = (nowMs - azLastTime) / 1000.0f;
        if (dt < 0.001f) dt = 0.001f;
        float dYaw = yawNow - azLastYaw;
        if (dYaw >  180.0f) dYaw -= 360.0f;
        if (dYaw < -180.0f) dYaw += 360.0f;
        yawRate    = fabsf(dYaw) / dt;
        azLastYaw  = yawNow;
        azLastTime = nowMs;
      }

      // --- Filtered GPS heading rate (emaGpsHdg from zHandlers.ino, x10 deg -> deg) ---
      float gpsHdgDeg  = emaGpsHdg / 10.0f;
      float gpsHdgRate = 0.0f;
      if (!azGpsInit) {
        azLastGpsHdg = gpsHdgDeg;
        azGpsInit    = true;
      } else {
        float dHdg = gpsHdgDeg - azLastGpsHdg;
        if (dHdg >  180.0f) dHdg -= 360.0f;
        if (dHdg < -180.0f) dHdg += 360.0f;
        gpsHdgRate   = fabsf(dHdg);
        azLastGpsHdg = gpsHdgDeg;
      }

      // --- Adaptive thresholds near zero angle (guidance-active only) ---
      float adaptFactor = 1.0f;
      if (guidanceActive) {
        float absAngle = fabsf(steerAngleActual);
        if (absAngle < AZ_NEAR_ZERO_DEG) {
          float ratio = absAngle / AZ_NEAR_ZERO_DEG;
          adaptFactor = AZ_NEAR_ZERO_FACTOR + ratio * (1.0f - AZ_NEAR_ZERO_FACTOR);
        }
      }

      float yawRateMax = azParams.yawRateMax * adaptFactor;
      float gpsHdgMax  = azParams.gpsHdgMax  * adaptFactor;
      bool  gpsOk      = (gpsHdgRate < gpsHdgMax);

      // --- Required stability duration (interpolated by speed) ---
      float azTimeMsF;
      if      (gpsSpeed <= azParams.speedSlow) azTimeMsF = (float)azParams.timeSlowMs;
      else if (gpsSpeed >= azParams.speedFast) azTimeMsF = (float)azParams.timeFastMs;
      else {
        float t = (gpsSpeed - azParams.speedSlow) / (azParams.speedFast - azParams.speedSlow);
        azTimeMsF = (float)azParams.timeSlowMs + t * ((float)azParams.timeFastMs - (float)azParams.timeSlowMs);
      }
      azTimeMsF = constrain(azTimeMsF, 200.0f, 5000.0f);
      uint32_t azTimeMs = (uint32_t)azTimeMsF;

      bool speedOk    = (gpsSpeed > azParams.speedMin);
      bool straightOk = (!azParams.useBno) || (yawRate < yawRateMax);
      bool gpsCapOk   = (!azParams.useGps) || gpsOk;
      bool cooldownOk = (nowMs - azCooldown > 2000);

      if (stableStart > 0 && (nowMs - dbgLastPrint > 5000)) {
        dbgLastPrint = nowMs;
        Serial.print(guidanceActive ? "[AZ-PRECISE] " : "[AZ-FAST] ");
        Serial.print("stable ");
        Serial.print(nowMs - stableStart); Serial.print("/");
        Serial.print(azTimeMs); Serial.print("ms");
        Serial.print(" spd=");  Serial.print(gpsSpeed, 1);
        Serial.print(" gyro="); Serial.print(straightOk ? "OK" : "NOK");
        Serial.print(" yawR="); Serial.print(yawRate, 2);
        Serial.print("/");      Serial.print(yawRateMax, 2);
        Serial.print(" gps=");  Serial.print(gpsCapOk ? "OK" : "NOK");
        Serial.print(" gpsR="); Serial.print(gpsHdgRate, 2);
        Serial.print("/");      Serial.print(gpsHdgMax, 2);
        Serial.print(" adapt="); Serial.print(adaptFactor, 2);
        Serial.print(" angle="); Serial.print(steerAngleActual, 2);
        Serial.print(" enc=");   Serial.println(keyaEncoderRaw);
      }

      if (speedOk && straightOk && gpsCapOk && cooldownOk)
      {
        if (stableStart == 0) {
          stableStart = nowMs;
          azAccum     = 0;
          azCount     = 0;
        }

        azAccum += (int64_t)keyaEncoderRaw;
        azCount++;

        if ((nowMs - stableStart) > azTimeMs && azCount > 0)
        {
          int32_t meanTicks = (int32_t)(azAccum / (int64_t)azCount);

          if (!wasZeroDone)
          {
            keyaZeroTicks = meanTicks;
            wasZeroDone   = true;
            azCorrAccum   = 0.0f;
            Serial.print("[AZ] First zero established (");
            Serial.print(azCount); Serial.print(" samples) zeroTicks=");
            Serial.println(keyaZeroTicks);
          }
          else if (!guidanceActive)
          {
            // FAST mode: jump straight to the new mean
            int32_t oldZero = keyaZeroTicks;
            keyaZeroTicks   = meanTicks;
            azCorrAccum     = 0.0f;
            Serial.print("[AZ-FAST] zero: "); Serial.print(oldZero);
            Serial.print(" -> ");             Serial.println(keyaZeroTicks);
          }
          else
          {
            // PRECISE mode: smooth sub-tick correction while actively steering
            float corrSign = steerConfig.InvertWAS ? -1.0f : 1.0f;
            azCorrAccum += corrSign * azParams.beta * steerAngleActual * keyaTicksPerDeg;
            int32_t corrInt = (int32_t)azCorrAccum;
            if (corrInt != 0) {
              keyaZeroTicks += corrInt;
              azCorrAccum   -= (float)corrInt;
            }
          }

          azAccum     = 0;
          azCount     = 0;
          stableStart = 0;
          azCooldown  = nowMs;
        }
      }
      else
      {
        stableStart = 0;
        azAccum     = 0;
        azCount     = 0;
      }
    }
    // =================================================================

    if (watchdogTimer < WATCHDOG_THRESHOLD)
    {
      //Enable H Bridge for IBT2, hyd aux, etc for cytron. On this board the same Cytron-enable
      //line also drives the steer-button LED backlight ("lock" circuit), so it runs for every
      //SteerDriverType, not just hydraulic - only the actual motor command differs, in motorDrive().
      if (steerConfig.CytronDriver)
      {
        if (steerConfig.IsRelayActiveHigh)
        {
          digitalWrite(PWM2_RPWM, 0);
        }
        else
        {
          digitalWrite(PWM2_RPWM, 1);
        }
      }
      else digitalWrite(DIR1_RL_ENABLE, 1);

      steerAngleError = steerAngleActual - steerAngleSetPoint;   //calculate the steering error
      //if (abs(steerAngleError)< steerSettings.lowPWM) steerAngleError = 0;

      calcSteeringPID();  //do the pid
      motorDrive();       //out to motors the pwm value
      // Autosteer Led goes GREEN if autosteering

      digitalWrite(AUTOSTEER_ACTIVE_LED, 1);
      digitalWrite(AUTOSTEER_STANDBY_LED, 0);
    }
    else
    {
      //we've lost the comm to AgOpenGPS, or just stop request
      //Disable H Bridge for IBT2, hyd aux, etc for cytron - also turns off the steer-button LED
      //backlight via the same lock circuit, for every SteerDriverType (see enable-side comment above)
      if (steerConfig.CytronDriver)
      {
        if (steerConfig.IsRelayActiveHigh)
        {
          digitalWrite(PWM2_RPWM, 1);
        }
        else
        {
          digitalWrite(PWM2_RPWM, 0);
        }
      }
      else digitalWrite(DIR1_RL_ENABLE, 0); //IBT2

      pwmDrive = 0; //turn off steering motor
      if (steerConfig.SteerDriverType == STEER_DRIVER_KEYA) disableKeyaSteer(); //lost comms with AOG - definitely stop steering
      motorDrive(); //out to motors the pwm value
      pulseCount = 0;
      // Autosteer Led goes back to RED when autosteering is stopped
      digitalWrite (AUTOSTEER_STANDBY_LED, 1);
      digitalWrite (AUTOSTEER_ACTIVE_LED, 0);
    }
  } //end of timed loop

  //This runs continuously, outside of the timed loop, keeps checking for new udpData, turn sense
  //delay(1);

  // Speed pulse
  if (gpsSpeedUpdateTimer < 1000)
  {
      if (speedPulseUpdateTimer > 200) // 100 (10hz) seems to cause tone lock ups occasionally
      {
          speedPulseUpdateTimer = 0;

          //130 pp meter, 3.6 kmh = 1 m/sec = 130hz or gpsSpeed * 130/3.6 or gpsSpeed * 36.1111
          //gpsSpeed = ((float)(autoSteerUdpData[5] | autoSteerUdpData[6] << 8)) * 0.1;
          float speedPulse = gpsSpeed * 36.1111;

          //Serial.print(gpsSpeed); Serial.print(" -> "); Serial.println(speedPulse);

          if (gpsSpeed > 0.11) { // 0.10 wasn't high enough
              tone(velocityPWM_Pin, uint16_t(speedPulse));
          }
          else {
              noTone(velocityPWM_Pin);
          }
      }
  }
  else  // if gpsSpeedUpdateTimer hasn't update for 1000 ms, turn off speed pulse
  {
      noTone(velocityPWM_Pin);
  }

  if (encEnable)
  {
    thisEnc = digitalRead(REMOTE_PIN);
    if (thisEnc != lastEnc)
    {
      lastEnc = thisEnc;
      if ( lastEnc) EncoderFunc();
    }
  }

} // end of main loop

int currentRoll = 0;
int rollLeft = 0;
int steerLeft = 0;

#ifdef ARDUINO_TEENSY41
// UDP Receive
void ReceiveUdp()
{
    // When ethernet is not running, return directly. parsePacket() will block when we don't
    if (!Ethernet_running)
    {
        return;
    }

    uint16_t len = Eth_udpAutoSteer.parsePacket();

    // if (len > 0)
    // {
    //  Serial.print("ReceiveUdp: ");
    //  Serial.println(len);
    // }

    // Check for len > 4, because we check byte 0, 1, 3 and 3
    if (len > 4)
    {
        Eth_udpAutoSteer.read(autoSteerUdpData, UDP_TX_PACKET_MAX_SIZE);

        if (autoSteerUdpData[0] == 0x80 && autoSteerUdpData[1] == 0x81 && autoSteerUdpData[2] == 0x7F) //Data
        {
            if (autoSteerUdpData[3] == 0xFE && Autosteer_running)  //254
            {
                gpsSpeed = ((float)(autoSteerUdpData[5] | autoSteerUdpData[6] << 8)) * 0.1;
                gpsSpeedUpdateTimer = 0;

                prevGuidanceStatus = guidanceStatus;

                guidanceStatus = autoSteerUdpData[7];
                guidanceStatusChanged = (guidanceStatus != prevGuidanceStatus);

                //Bit 8,9    set point steer angle * 100 is sent
                steerAngleSetPoint = ((float)(autoSteerUdpData[8] | ((int8_t)autoSteerUdpData[9]) << 8)) * 0.01; //high low bytes

                //Serial.print("steerAngleSetPoint: ");
                //Serial.println(steerAngleSetPoint);

                //Serial.println(gpsSpeed);

                if ((bitRead(guidanceStatus, 0) == 0) || (gpsSpeed < 0.1) || (steerSwitch == 1))
                {
                    watchdogTimer = WATCHDOG_FORCE_VALUE; //turn off steering motor
                }
                else          //valid conditions to turn on autosteer
                {
                    watchdogTimer = 0;  //reset watchdog
                }

                //Bit 10 Tram
                tram = autoSteerUdpData[10];

                //Bit 11
                relay = autoSteerUdpData[11];

                //Bit 12
                relayHi = autoSteerUdpData[12];

                //----------------------------------------------------------------------------
                //Serial Send to agopenGPS

                int16_t sa = (int16_t)(steerAngleActual * 100);

                PGN_253[5] = (uint8_t)sa;
                PGN_253[6] = sa >> 8;

                // heading
                PGN_253[7] = (uint8_t)9999;
                PGN_253[8] = 9999 >> 8;

                // roll
                PGN_253[9] = (uint8_t)8888;
                PGN_253[10] = 8888 >> 8;

                PGN_253[11] = switchByte;
                PGN_253[12] = (uint8_t)pwmDisplay;

                //checksum
                int16_t CK_A = 0;
                for (uint8_t i = 2; i < PGN_253_Size; i++)
                    CK_A = (CK_A + PGN_253[i]);

                PGN_253[PGN_253_Size] = CK_A;

                //off to AOG
                SendUdp(PGN_253, sizeof(PGN_253), Eth_ipDestination, portDestination);

                //Steer Data 2 -------------------------------------------------
                if (steerConfig.PressureSensor || steerConfig.CurrentSensor)
                {
                    if (aog2Count++ > 2)
                    {
                        //Send fromAutosteer2
                        PGN_250[5] = (byte)sensorReading;

                        //add the checksum for AOG2
                        CK_A = 0;

                        for (uint8_t i = 2; i < PGN_250_Size; i++)
                        {
                            CK_A = (CK_A + PGN_250[i]);
                        }

                        PGN_250[PGN_250_Size] = CK_A;

                        //off to AOG
                        SendUdp(PGN_250, sizeof(PGN_250), Eth_ipDestination, portDestination);
                        aog2Count = 0;
                    }
                }

                //Serial.println(steerAngleActual);
                //--------------------------------------------------------------------------
            }

            //steer settings
            else if (autoSteerUdpData[3] == 0xFC && Autosteer_running)  //252
            {
                //PID values
                steerSettings.Kp = ((float)autoSteerUdpData[5]);   // read Kp from AgOpenGPS

                steerSettings.highPWM = autoSteerUdpData[6]; // read high pwm

                steerSettings.lowPWM = (float)autoSteerUdpData[7];   // read lowPWM from AgOpenGPS

                steerSettings.minPWM = autoSteerUdpData[8]; //read the minimum amount of PWM for instant on

                float temp = (float)steerSettings.minPWM * 1.2;
                steerSettings.lowPWM = (byte)temp;

                steerSettings.steerSensorCounts = autoSteerUdpData[9]; //sent as setting displayed in AOG

                // In Keya wasless mode, steerSensorCounts instead scales keyaTicksPerDeg -
                // pure proportional, centered on 100 = KEYA_TICKS_PER_DEG_DEFAULT (24 ticks/deg).
                // The zero point itself is handled separately by the auto-zero engine above.
                if (steerConfig.SteerDriverType == STEER_DRIVER_KEYA && steerConfig.IsDanfoss)
                {
                    keyaTicksPerDeg = KEYA_TICKS_PER_DEG_DEFAULT * ((float)steerSettings.steerSensorCounts / 100.0f);
                    if (keyaTicksPerDeg < 1.0f) keyaTicksPerDeg = 1.0f; // guard against divide-by-zero
                    EEPROM.put(EEPROM_ADDR_KEYA_TICKS, keyaTicksPerDeg);
                }

                steerSettings.wasOffset = (autoSteerUdpData[10]);  //read was zero offset Lo

                steerSettings.wasOffset |= (autoSteerUdpData[11] << 8);  //read was zero offset Hi

                steerSettings.AckermanFix = (float)autoSteerUdpData[12] * 0.01;

                //crc
                //autoSteerUdpData[13];

                //store in EEPROM
                EEPROM.put(10, steerSettings);

                // In Keya wasless mode, AOG sending wasOffset = 0 (its own "re-zero WAS" action)
                // forces an immediate re-zero of the encoder instead of touching an ADC offset.
                if (steerConfig.SteerDriverType == STEER_DRIVER_KEYA && steerConfig.IsDanfoss && steerSettings.wasOffset == 0)
                {
                    keyaZeroTicks = keyaEncoderRaw;
                    wasZeroDone   = true;
                    stableStart   = 0;
                    azCorrAccum   = 0.0f;
                    Serial.print("[AZ] Zero forced from AOG - zeroTicks=");
                    Serial.println(keyaZeroTicks);
                }

                // Re-Init steer settings
                steerSettingsInit();
            }

            else if (autoSteerUdpData[3] == 0xFB)  //251 FB - SteerConfig
            {
                uint8_t sett = autoSteerUdpData[5]; //setting0

                if (bitRead(sett, 0)) steerConfig.InvertWAS = 1; else steerConfig.InvertWAS = 0;
                if (bitRead(sett, 1)) steerConfig.IsRelayActiveHigh = 1; else steerConfig.IsRelayActiveHigh = 0;
                if (bitRead(sett, 2)) steerConfig.MotorDriveDirection = 1; else steerConfig.MotorDriveDirection = 0;
                if (bitRead(sett, 3)) steerConfig.SingleInputWAS = 1; else steerConfig.SingleInputWAS = 0;
                if (bitRead(sett, 4)) steerConfig.CytronDriver = 1; else steerConfig.CytronDriver = 0;
                if (bitRead(sett, 5)) steerConfig.SteerSwitch = 1; else steerConfig.SteerSwitch = 0;
                if (bitRead(sett, 6)) steerConfig.SteerButton = 1; else steerConfig.SteerButton = 0;
                if (bitRead(sett, 7)) steerConfig.ShaftEncoder = 1; else steerConfig.ShaftEncoder = 0;

                steerConfig.PulseCountMax = autoSteerUdpData[6];

                //was speed
                //autoSteerUdpData[7];

                sett = autoSteerUdpData[8]; //setting1 - Danfoss valve etc

                if (bitRead(sett, 0)) steerConfig.IsDanfoss = 1; else steerConfig.IsDanfoss = 0;
                if (bitRead(sett, 1)) steerConfig.PressureSensor = 1; else steerConfig.PressureSensor = 0;
                if (bitRead(sett, 2)) steerConfig.CurrentSensor = 1; else steerConfig.CurrentSensor = 0;
                if (bitRead(sett, 3)) steerConfig.IsUseY_Axis = 1; else steerConfig.IsUseY_Axis = 0;

                //crc
                //autoSteerUdpData[13];

                EEPROM.put(40, steerConfig);

                // Re-Init
                steerConfigInit();

            }//end FB
            else if (autoSteerUdpData[3] == 200) // Hello from AgIO
            {
                if(Autosteer_running)
                {
                int16_t sa = (int16_t)(steerAngleActual * 100);

                helloFromAutoSteer[5] = (uint8_t)sa;
                helloFromAutoSteer[6] = sa >> 8;

                helloFromAutoSteer[7] = (uint8_t)helloSteerPosition;
                helloFromAutoSteer[8] = helloSteerPosition >> 8;
                helloFromAutoSteer[9] = switchByte;

                SendUdp(helloFromAutoSteer, sizeof(helloFromAutoSteer), Eth_ipDestination, portDestination);
                }
                if(useBNO08x || useCMPS || useTM171)
                {
                 SendUdp(helloFromIMU, sizeof(helloFromIMU), Eth_ipDestination, portDestination); 
                }
            }

            else if (autoSteerUdpData[3] == 201)
            {
             //make really sure this is the subnet pgn
             if (autoSteerUdpData[4] == 5 && autoSteerUdpData[5] == 201 && autoSteerUdpData[6] == 201)
             {
              networkAddress.ipOne = autoSteerUdpData[7];
              networkAddress.ipTwo = autoSteerUdpData[8];
              networkAddress.ipThree = autoSteerUdpData[9];
        
              //save in EEPROM and restart
              EEPROM.put(60, networkAddress);
              SCB_AIRCR = 0x05FA0004; //Teensy Reset
              }
            }//end 201

            //whoami
            else if (autoSteerUdpData[3] == 202)
            {
                //make really sure this is the reply pgn
                if (autoSteerUdpData[4] == 3 && autoSteerUdpData[5] == 202 && autoSteerUdpData[6] == 202)
                {
                    IPAddress rem_ip = Eth_udpAutoSteer.remoteIP();

                    //hello from AgIO
                    uint8_t scanReply[] = { 128, 129, Eth_myip[3], 203, 7,
                        Eth_myip[0], Eth_myip[1], Eth_myip[2], Eth_myip[3], 
                        rem_ip[0],rem_ip[1],rem_ip[2], 23 };

                    //checksum
                    int16_t CK_A = 0;
                    for (uint8_t i = 2; i < sizeof(scanReply) - 1; i++)
                    {
                        CK_A = (CK_A + scanReply[i]);
                    }
                    scanReply[sizeof(scanReply)-1] = CK_A;

                    static uint8_t ipDest[] = { 255,255,255,255 };
                    uint16_t portDest = 9999; //AOG port that listens

                    //off to AOG
                    SendUdp(scanReply, sizeof(scanReply), ipDest, portDest);
                }
            }
            else if (autoSteerUdpData[3] == 236 || autoSteerUdpData[3] == 238 || autoSteerUdpData[3] == 239)  //machine data
            {
                //Serial.println("Autosteer got some 236/238/239 machine data forwarding to Hydraulics!");
                hydraulicLoop(autoSteerUdpData);
            }
        } //end if 80 81 7F
    }
}
#endif

#ifdef ARDUINO_TEENSY41
void SendUdp(uint8_t *data, uint8_t datalen, IPAddress dip, uint16_t dport)
{
  Eth_udpAutoSteer.beginPacket(dip, dport);
  Eth_udpAutoSteer.write(data, datalen);
  Eth_udpAutoSteer.endPacket();
}
#endif

//ISR Steering Wheel Encoder
void EncoderFunc()
{
  if (encEnable)
  {
    //Reset counter to 0 if there wasn't any activity for 15 seconds
    if(sensorPulseReset >= 10000 && steerConfig.PulseCountMax >= 3)  pulseCount = 0;
    sensorPulseReset = 0;
    pulseCount++;
    encEnable = false;
  }
}
