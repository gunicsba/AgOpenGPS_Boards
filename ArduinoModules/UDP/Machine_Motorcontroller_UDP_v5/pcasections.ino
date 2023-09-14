#include <Adafruit_PWMServoDriver.h>
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);
#include <Wire.h>

// 1 means +12, -1 means -12, 0 means 0
int8_t positionOpen[]    = {  1,  1,  1,  1,   1,  1,  1,  1 };   
int8_t positionNeutral[] = {  0,  0,  0,  0,   0,  0,  0,  0 };   
int8_t positionClosed[]  = { -1, -1, -1, -1,  -1, -1, -1, -1 };   
#define TIME_RETURN_NEUTRAL 50 //time to return to neutral 0 means never go back to zero, 10 means 1 second

int16_t pinPwmLo[] { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
int16_t pinPwmHi[] { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

#define SERVO_FREQ 3000 //no idea what's the right value :)

uint8_t lastTimeSectionMove[] = { 0,0,0,0, 0,0,0,0 };
bool lastPositionMove[] = { true,true,true,true, true,true,true,true};


void pcasectionsSetup(){
		Serial.println("Starting PCA Controller ...");
		int ErrorCount = 0;
		while (!PCAFound)
		{
			Serial.print(".");
			Wire.beginTransmission(0x40);
			PCAFound = (Wire.endTransmission() == 0);
			ErrorCount++;
			delay(250);
			if (ErrorCount > 5) break;
		}
    if(PCAFound) {
      Serial.println("Found PCA Controller ...");
      pwm.begin();
      pwm.setPWMFreq(SERVO_FREQ);
    } else {
      Serial.println("PCA Controller missing!!!");
    }

    delay(50); 
    switchRelaisOn();
}

void switchRelaisOn() {  //that are the relais, switch all off
  if(!PCAFound) return;
//szakaszok
//a szakasz "off"-ra kapcsol be a művelőút meg "off"-ra elzárva...
    for (count = 0; count < 6; count++) {
      lastPositionMove[count] = true; 
      setSection(count, false); 
    }
//muvelout -> on-ra kell azaz TRUE
    for(count = 6 ; count < 8; count++) {
      lastPositionMove[count] = false; 
      setSection(count, true);
    }
    onLo = onHi = 0;
    offLo = offHi = 0b11111111;
}

void setSection(uint8_t section, bool sectionActive) {
  if(!PCAFound) return;
    if (sectionActive && !lastPositionMove[section]) {
        setPosition(section, positionOpen[count]);
        lastPositionMove[section] = true;
        lastTimeSectionMove[section] = 0;
    }
    else if (!sectionActive && lastPositionMove[section]) {
        setPosition(section, positionClosed[count]);
        lastPositionMove[section] = false;
        lastTimeSectionMove[section] = 0;
    }
}

void returnNeutralPosition() {
  if(!PCAFound) return;
    uint8_t tmp = 0;
    for (count = 0; count < NUM_OF_SECTIONS; count++) {
        tmp = lastTimeSectionMove[count];
        if (tmp != 255) {
            if (tmp < TIME_RETURN_NEUTRAL) {
                tmp++;
            }
            else {
                setPosition(count, positionNeutral[count]);
                tmp = 255;
            }
        }
        lastTimeSectionMove[count] = tmp;
    }
}
String setPosition(int section, int num) {
    String action = "disable";
    if(num == -1) action = "off";
    if(num == 1) action = "on";
    return sectionHandler(section,action, 4096); 
}
String setPosition(int section, String action) {
    return sectionHandler(section,action, 4096); 
}

String sectionHandler(int section, String action, int force) {
    if(force > 4096) force = 4096;

    int pin1 = section * 2;
    int pin2 = pin1+1;

    String forReturn = "SectionHandler: ";
    forReturn += section;
    forReturn += " action: ";
    forReturn += action;
    forReturn += " pin1: ";
    forReturn += pin1;
    forReturn += " pin2: ";
    forReturn += pin2;

    if( strcmp(action.c_str(),"off") == 0 ){
        sectionHandler(pin1, 4096-force, 0); 
        sectionHandler(pin2, 4096 , 0);
        forReturn += " pwm is ";
        forReturn += (4096-force);
        forReturn += ",0 + 4096,0";
    }
    if( strcmp(action.c_str(),"on") == 0){
        sectionHandler(pin1, 4096, 0);
        sectionHandler(pin2, 4096-force, 0 );
        forReturn += " pwm is 4096,0 + ";
        forReturn += (4096-force) ;
    }
    if( strcmp(action.c_str(),"disable") == 0){
        sectionHandler(pin1, 4096, 0);
        sectionHandler(pin2, 4096, 0);
        forReturn += " pwm is 4096,0 + 4096,0";
    }
//    if(section == 0 || section == 7) Serial.println(forReturn); //DEBUG
    return forReturn;
}

void sectionHandler(int pin, int pwm1, int pwm2) {
    pwm.setPWM(pin, pwm1, pwm2); 
}

void setPwmForPin(uint8_t pin, int16_t pwmLo, int16_t pwmHi) {
  //pwm.setPWM(pin1, 0, 4096); //off
  if(pinPwmLo[pin] != pwmLo && pinPwmHi[pin] != pwmHi){
    pwm.setPWM(pin, pwmLo, pwmHi);
    pinPwmLo[pin] = pwmLo;
    pinPwmHi[pin] = pwmHi;
  }
}

/* pwm can be from -255 to 255 */
void setPwmForSection(uint8_t section, short pwm) {
  if(!PCAFound) return;
  uint8_t pin1 = section*2;
  uint8_t pin2 = pin1+1;
  if( pwm < 0) { //we need to turn off 1 direction
    setPwmForPin(pin1, 0, 4096); //off
    setPwmForPin(pin2, 0, pwm*-16);
  } else if ( pwm == 0) {
    setPwmForPin(pin1, 4096, 0); //on
    setPwmForPin(pin2, 4096, 0); //on
    delay(100);
  } else {
    setPwmForPin(pin1, 0, pwm*16);
    setPwmForPin(pin2, 0, 4096); //off
  }

}


