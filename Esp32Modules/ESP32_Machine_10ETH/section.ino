
#include <Adafruit_PWMServoDriver.h>

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pca1 = Adafruit_PWMServoDriver(0x40);
//bool PCAFound = false;

void sectionSetup(){
    Serial.println("");
    Serial.println("Connect to PCA9685 at address: 0x40");
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
      pca1.begin();
      pca1.setPWMFreq(1600);
      Serial.print("PCA9685 init done ");
    } else {
      Serial.println("PCA Controller missing!!!");
    }

    Serial.println("");
    lofasz();
}

void lofasz() {
  for(int i = 50; i < 256 ; i++){
    pca1.setPWM(15,0,0);
    pca1.setPWM(14, 0, i*8);
    delay(10); 
  }
}

String sectionHandler(int section, String action) {
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
/*
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
*/
    if( strcmp(action.c_str(),"off") == 0 ){
//        sectionHandler(pin1, 0, 4096);
        sectionHandler(pin1, 0, force);
        sectionHandler(pin2, 4096 , 0);
        forReturn += " pwm is 0,";
        forReturn += force;
        forReturn += " 4096,0";
    }
    if( strcmp(action.c_str(),"on") == 0){
        sectionHandler(pin1, 4096, 0);
 //       sectionHandler(pin2, 0, 4096 );
        sectionHandler(pin2, 0, force );
        forReturn += " pwm is 4096,0 + 0,";
        forReturn += force;
    }
    if( strcmp(action.c_str(),"disable") == 0){
        sectionHandler(pin1, 4096, 0);
        sectionHandler(pin2, 4096, 0);
        forReturn += " pwm is 0,0 + 0,0";
    }
//    Serial.println(forReturn); //DEBUG
    return forReturn;
}

void sectionHandler(int pin, int pwm1, int pwm2) {
    pca1.setPWM(pin, pwm1, pwm2); 
}