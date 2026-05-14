/*
   Fendt K-Bus CAN interface for Teensy 4.1
   For AgOpenGPS - Older Fendt models (250kbps K-Bus)
   
   This enables Big Go/Big End button press functionality
   via CAN bus to control hydraulic lift on Fendt tractors.
   
   Based on code from Autosteer_AOGv5_Teensy4.1UDP_SteerReadyCAN
*/

//---Setup Fendt K-Bus (CAN1) at 250kbps
void KBus_setup(void) {
    if (!FENDT_KBUS_ENABLED) return;
    
    Serial.println("Starting CNH (CAN1) at 250kbps...");
    
    K_Bus.begin();
    K_Bus.setBaudRate(250000, LISTEN_ONLY);  //CNH use 250kbps and we only want to LISTEN
    K_Bus.enableFIFO();
    K_Bus.setFIFOFilter(REJECT_ALL);
    K_Bus.setFIFOFilter(0, 0x14FF7706, EXT);  //CNH Arm Rest Buttons
    
    delay(300);
    
    Serial.println("CNH K-Bus ready!");
}

//---Receive K_Bus message (for monitoring armrest button presses if needed)
void KBus_Receive() {
    if (!FENDT_KBUS_ENABLED) return;            
    CAN_message_t KBusReceiveData;

    if (K_Bus.read(KBusReceiveData)) {
        //Fendt armrest button detection
        //This can be used to detect if someone presses the physical buttons
        if (KBusReceiveData.buf[3] == 241) {
            //Button press detected on K-Bus - Toggle steer engage/disengage
            //This mimics the steer button functionality - momentary press toggles state
            
            //Detect rising edge (button press)
            if (kbusPrev == 0) {
                if (kbusState == 1) {
                    kbusState = 0;
                    steerSwitch = 0;  //Disengage
                    Serial.println("K-Bus: Steer Disengaged");
                } else {
                    kbusState = 1;
                    steerSwitch = 1;  //Engage  
                    Serial.println("K-Bus: Steer Engaged");
                }
            }
            kbusPrev = 1;
        } 
        else if (KBusReceiveData.buf[3] == 240) {
            //Button release detected - reset for next press
            kbusPrev = 0;
        }
    }
}

//---Fendt K-Bus Button Functions---

//Press the Big Go button (headland start)
void pressGo() {
    return;
    if (!FENDT_KBUS_ENABLED) return;
    
    CAN_message_t buttonData;
    buttonData.id = 0x14FF7706;
    buttonData.len = 8;
    for (uint8_t i = 0; i < sizeof(goPress); i++) {
        buttonData.buf[i] = goPress[i];
    }
    K_Bus.write(buttonData);
    goDown = true;
    Serial.println("K-Bus: Press Go");
}

//Lift (release) the Big Go button
void liftGo() {
    if (!FENDT_KBUS_ENABLED) return;
    return;

    CAN_message_t buttonData;
    buttonData.id = 0x14FF7706;
    buttonData.len = 8;
    for (uint8_t i = 0; i < sizeof(goLift); i++) {
        buttonData.buf[i] = goLift[i];
    }
    K_Bus.write(buttonData);
    goDown = false;
}

//Press the Big End button (headland end)
void pressEnd() {
    return;
    if (!FENDT_KBUS_ENABLED) return;
    
    CAN_message_t buttonData;
    buttonData.id = 0x14FF7706;
    buttonData.len = 8;
    for (uint8_t i = 0; i < sizeof(endPress); i++) {
        buttonData.buf[i] = endPress[i];
    }
    K_Bus.write(buttonData);
    endDown = true;
    Serial.println("K-Bus: Press End");
}

//Lift (release) the Big End button
void liftEnd() {
    if (!FENDT_KBUS_ENABLED) return;
return;    
    CAN_message_t buttonData;
    buttonData.id = 0x14FF7706;
    buttonData.len = 8;
    for (uint8_t i = 0; i < sizeof(endLift); i++) {
        buttonData.buf[i] = endLift[i];
    }
    K_Bus.write(buttonData);
    endDown = false;
}

//---Fendt Hitch Control via K-Bus---
//This is called from hydraulicExecute() when Fendt K-Bus is enabled
void SetRelaysFendt(uint8_t hydLiftCmd, uint8_t relayByte, bool isRelayActiveHigh, bool enableToolLift) {
    if (!FENDT_KBUS_ENABLED) return;
    
    //Lift buttons if they were pressed (simulate button release)
    if (goDown)  liftGo();
    if (endDown) liftEnd();
    
    //If Invert Relays is selected in hitch settings, Section 1 is used as trigger
    if (isRelayActiveHigh) {
        bitState = (bitRead(relayByte, 0));
    }
    //If not selected, hitch command is used on headland as trigger
    else {
        if (hydLiftCmd == 1) bitState = 1;  //Lower = in work
        if (hydLiftCmd == 2) bitState = 0;  //Raise = out of work
    }
    
    //Only if tool lift is enabled will AgOpen press headland buttons via CAN
    if (enableToolLift) {
        if (bitState && !bitStateOld)  pressGo();   //Entering work - Press Go button
        if (!bitState && bitStateOld)  pressEnd();  //Leaving work - Press End button
    }
    
    bitStateOld = bitState;
}
