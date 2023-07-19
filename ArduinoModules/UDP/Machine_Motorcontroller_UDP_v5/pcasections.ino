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
		Serial.println("");
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
    switchRelaisOff();
}

void switchRelaisOff() {  //that are the relais, switch all off
  if(!PCAFound) return;
    for (count = 0; count < NUM_OF_SECTIONS; count++) {
        lastPositionMove[count] = false; 
        setSection(count, true); 
        setPosition(count, positionClosed[count]);
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

void setPosition(uint8_t section, int8_t mode) {
  if(!PCAFound) return;
    uint8_t pin1 = section*2;
    uint8_t pin2 = pin1+1;
    Serial.println("");
    Serial.print("Section ");
    Serial.print(section);
    Serial.print(" pin1 ");
    Serial.print(pin1);
    Serial.print(" pin2 ");
    Serial.print(pin2);
    Serial.print(" mode ");
    Serial.print(mode);
    
    switch(mode) {
      case -1:
            setPwmForPin(pin1, 0, 4096); //off
            setPwmForPin(pin2, 4096, 0); //on
        break;
      case 0:
            setPwmForPin(pin1, 4096, 0); //on
            setPwmForPin(pin2, 4096, 0); //on
        break;
      case 1:
          setPwmForPin(pin1, 4096, 0); //on
          setPwmForPin(pin2, 0, 4096); //off
        break;
      default: 
        Serial.println("");
        Serial.print("Not a valid value to set section! ");
        Serial.print("Section ");
        Serial.print(section);
        Serial.print(" value ");
        Serial.print(mode);
        Serial.println("");
    }
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
    Serial.println("");
    Serial.print("Section ");
    Serial.print(section);
    Serial.print(" pin1 ");
    Serial.print(pin1);
    Serial.print(" pin2 ");
    Serial.print(pin2);
    Serial.print(" pwm ");
  if( pwm < 0) { //we need to turn off 1 direction
    setPwmForPin(pin1, 0, 4096); //off
    setPwmForPin(pin2, 0, pwm*-16);
    Serial.print( pwm*-16 );
  } else if ( pwm == 0) {
    setPwmForPin(pin1, 4096, 0); //on
    setPwmForPin(pin2, 4096, 0); //on
    delay(100);
    Serial.print(" 0-0 ");
  } else {
    setPwmForPin(pin1, 0, pwm*16);
    setPwmForPin(pin2, 0, 4096); //off
    Serial.print( pwm*16 );
  }
  Serial.println("");

}


