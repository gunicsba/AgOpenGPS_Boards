# What do we have here?

A few modifications from me :)

1. Autosteer_gps_teensy_v4_1_hydlift.ino.hex
	This will utilise the extra 3 pins on the ampseal as per:

	//These are the pins available on the AIO boards AMPSEAL:
	#define HYDRAULIC_LIFT_OR_UP 26 //A12 was: Hyd_up Used to lift up the hydraulics
	#define HYDRAULIC_LOWER_OR_DOWN 27//A13 was: Hyd_down Used to lower the hydraulics 
	#define HYDRAULIC_TRAMLINE 38 //A14

	And you can tune the behavior using User4 in the hydlift page:
	uint8_t user4 = 0; //0 - disabled , 1 - Pulls the relay and keeps it that way , 2 - pulls and release it after N second


2. JDautotracKickOff
	For the analog like encoder found on newer JD-s this will start to report the wheel turn speed as % when the pressure sensor is enabled. (tested on 2 tractors) jumper as pressure and steal the 0-5V output of the wheel.
	Requires the #define JOHNDEERE true  (by default its false)

3. Autosteer_gps_teensy_v4_1_lazycurrent
	For the 20A current sensor I try to flatten the sudden spikes when the motor is turned on by doing a 80-20 rolling average.

4. Autosteer_gps_teensy_v4_1.ino_hydlift_and_lazycurrent_avgbno
	Experimental!!
	Using the user1 parameter we try to average out the BNO readings (50Hz -ish currently).
	Setting it to 0 means we have the old behavior. Setting it to 10 means we'll do:
	roll = roll*0.9 + measuredRoll*0.1
	setting it to 20 means:
	roll = roll*0.8 + measuredRoll*0.2



Fendt COM3 stuff from Thibault:

#include <FlexCAN_T4.h>
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_256> K_Bus;

void setup(){
  Serial.begin(115200);
  delay(1000);
  
  K_Bus.begin();
  K_Bus.setBaudRate(100000); // Vitesse Fendt COM3 (100kbps)
  K_Bus.enableFIFO();
  K_Bus.setFIFOFilter(REJECT_ALL);
  K_Bus.setFIFOFilter(0, 0x61F, STD);
  
  Serial.println("--- TEST CAN FENDT COM3 LANCE ---");
}

void loop()
{
  // 1. SEQUENCE DESCENTE (GO)
  Serial.println("Appui sur GO...");
  pressGo();
  delay(300);      // On reste appuye 0.3s
  liftGo();       // On relache
  
  Serial.println("Attente 10 secondes...");
  delay(10000);   // Pause de 10s avant l'autre bouton

  // 2. SEQUENCE MONTEE (END)
  Serial.println("Appui sur END...");
  pressEnd();  
  delay(300);      // On reste appuye 0.3s
  liftEnd();      // On relache

  Serial.println("Attente 10 secondes...");
  delay(10000);   // Pause de 10s avant de recommencer
}

// --- FONCTIONS CORRIGEES AVEC LES TRAMES DE TONY ---

void pressGo()
{
    CAN_message_t msg;
    msg.id = 0x61F;
    msg.len = 8;
    msg.flags.extended = false;
    msg.buf[0] = 0x15;
    msg.buf[1] = 0x33; // Code bouton GO
    msg.buf[2] = 0x1E; // Signature K-Bus (Crucial !)
    msg.buf[3] = 0xCA;
    msg.buf[4] = 0x80; // Status: Pressé
    msg.buf[5] = 0x01; // Action: Press GO
    msg.buf[6] = 0x00;
    msg.buf[7] = 0x00;
    K_Bus.write(msg);
}

void liftGo()
{
    CAN_message_t msg;
    msg.id = 0x61F;
    msg.len = 8;
    msg.flags.extended = false;
    msg.buf[0] = 0x15;
    msg.buf[1] = 0x33; 
    msg.buf[2] = 0x1E; 
    msg.buf[3] = 0xCA;
    msg.buf[4] = 0x00; // Status: Relâché
    msg.buf[5] = 0x02; // Action: Release GO
    msg.buf[6] = 0x00;
    msg.buf[7] = 0x00;
    K_Bus.write(msg);
}

void pressEnd()
{
    CAN_message_t msg;
    msg.id = 0x61F;
    msg.len = 8;
    msg.flags.extended = false;
    msg.buf[0] = 0x15;
    msg.buf[1] = 0x34; // Code bouton END
    msg.buf[2] = 0x1E; 
    msg.buf[3] = 0xCA;
    msg.buf[4] = 0x80; 
    msg.buf[5] = 0x03; // Action: Press END
    msg.buf[6] = 0x00;
    msg.buf[7] = 0x00;
    K_Bus.write(msg);
}

void liftEnd()
{
    CAN_message_t msg;
    msg.id = 0x61F;
    msg.len = 8;
    msg.flags.extended = false;
    msg.buf[0] = 0x15;
    msg.buf[1] = 0x34; 
    msg.buf[2] = 0x1E; 
    msg.buf[3] = 0xCA;
    msg.buf[4] = 0x00; 
    msg.buf[5] = 0x04; // Action: Release END
    msg.buf[6] = 0x00;
    msg.buf[7] = 0x00;
    K_Bus.write(msg);
}

	I think the right value will be around 15-30 needs testing.
