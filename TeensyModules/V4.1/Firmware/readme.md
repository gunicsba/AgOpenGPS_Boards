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

	I think the right value will be around 15-30 needs testing.