#include <avr/EEPROM.h>
#include <phys253_TEST.h>
#include <LiquidCrystal.h>
#include "Robot_Code.h"

// QRD Menu Class
class MenuItem
{
public:
	String    Name;
	uint16_t  Value;
	uint16_t* EEPROMAddress;
	static uint16_t MenuItemCount;
	MenuItem(String name)
	{
		MenuItemCount++;
		EEPROMAddress = (uint16_t*)(2 * MenuItemCount) + 73;
		Name = name;
		Value = eeprom_read_word(EEPROMAddress);
	}
	void Save()
	{
		eeprom_write_word(EEPROMAddress, Value);
	}
};

// IR Menu Class
class IRMenuItem
{
public:
	String    Name;
	uint16_t  Value;
	uint16_t* EEPROMAddress;
	static uint16_t MenuItemCount;
	IRMenuItem(String name)
	{
		MenuItemCount++;
		EEPROMAddress = (uint16_t*)(MenuItemCount) + 56; // offset the EEPROMAddress
		Name = name;
		Value = eeprom_read_word(EEPROMAddress);
	}
	void Save()
	{
		eeprom_write_word(EEPROMAddress, Value);
	}
};

// Main Menu
class MainMenuItem
{
public:
	String Name;
	static uint16_t MenuItemCount;
	MainMenuItem(String name)
	{
		MenuItemCount++;
		Name = name;
	}
	static void Open(int index)
	{
		boolean f = true;
		switch (index) {
		case 0:
			// set pid mode for testing
			mode = modes[modeIndex];
			if (mode == modes[0]) {
				// tape following mode
				pidfn = tapePID;
			} else if (mode == modes[1]) {
				// ir following mode
				pidfn = irPID;
			}
			break;
		case 1:
			QRDMENU();
			break;
		case 2:
			IRMENU();
			break;
		case 3:
			turnForward(val, 80);
			break;
		case 4:
			travel(val, FORWARDS);
			break;
		case 5:
			delay(1000);
			LCD.clear(); LCD.home();
			LCD.print("R U SURE~");
			LCD.setCursor(0, 1); LCD.print("PRESS START");
			while (f) {
				if ( startbutton()) {
					f = false;
					launch(val);
				} else if (stopbutton()) {
					f = false;
				}
			}
			break;
		case 6:
			delay(250);
			armCal();
			LCD.clear(); LCD.home();
			LCD.print("Returning...");
			delay(300);
			break;
		}
	}
};

/* Add the menu items */
uint16_t MenuItem::MenuItemCount = 0;
MenuItem Speed        	  = MenuItem("Speed");
MenuItem ProportionalGain = MenuItem("P-gain");
MenuItem DerivativeGain   = MenuItem("D-gain");
MenuItem IntegralGain     = MenuItem("I-gain");
MenuItem ThresholdVoltage = MenuItem("T-volt");
MenuItem HorThreshold = MenuItem("H-volt");
MenuItem menuItems[]      = {Speed, ProportionalGain, DerivativeGain, IntegralGain, ThresholdVoltage, HorThreshold};

uint16_t IRMenuItem::MenuItemCount = 0;
IRMenuItem IRProportionalGain = IRMenuItem("P-gain");
IRMenuItem IRDerivativeGain   = IRMenuItem("D-gain");
IRMenuItem IRIntegralGain     = IRMenuItem("I-gain");
IRMenuItem IRmenuItems[]      = {IRProportionalGain, IRDerivativeGain, IRIntegralGain};

uint16_t MainMenuItem::MenuItemCount = 0;
MainMenuItem Sensors     = MainMenuItem("Sensors");
MainMenuItem TapePID     = MainMenuItem("Tape PID");
MainMenuItem IRPID       = MainMenuItem("IR PID");
MainMenuItem pivotTest   = MainMenuItem("pivotTest");
MainMenuItem travelTest  = MainMenuItem("travelTest");
MainMenuItem launchTest  = MainMenuItem("launchTest");
MainMenuItem armTest  = MainMenuItem("armTest");
MainMenuItem mainMenu[]  = {Sensors, TapePID, IRPID, pivotTest, travelTest, launchTest, armTest};

void setup()
{
#include <phys253setup.txt>
	//Serial.begin(9600);
	LCD.clear(); LCD.home();

	// set gains
	base_speed = menuItems[0].Value;
	q_pro_gain = menuItems[1].Value;
	q_diff_gain = menuItems[2].Value;
	q_int_gain = menuItems[3].Value;
	q_threshold = menuItems[4].Value;
	h_threshold = menuItems[5].Value;
	ir_pro_gain = IRmenuItems[0].Value;
	ir_diff_gain = IRmenuItems[1].Value;
	ir_int_gain = IRmenuItems[2].Value;

	// set ports 8 to 15 as OUTPUT
	portMode(1, OUTPUT);
	// ensure relays are LOW on start.
	digitalWrite(LAUNCH_F, LOW);

	time_L = millis();
	time_R = millis();
	lastSpeedUp = 0;

	// set servo initial positions
	RCServo2.write(90);
	RCServo0.write(78);

	// default PID loop is QRD tape following
	pidfn = tapePID;

	LCD.print("RC5"); LCD.setCursor(0, 1);
	LCD.print("Press Start.");
	while (!startbutton()) {
		lowerArmPID();
		upperArmPID();
	};
	motor.stop_all();
	LCD.clear();
	MainMenu();
}

void loop()
{
	// Check for menu command
	if (startbutton() && stopbutton()) {
		// Pause motors
		motor.stop_all();
		MainMenu();
		// Restart motors
		motor.speed(LEFT_MOTOR, base_speed);
		motor.speed(RIGHT_MOTOR, base_speed);
	}
	petProcess();
	pidfn();
}

/* Control Loops */
void tapePID() {

	encoderProcess();

	int left_sensor = analogRead(QRD_L);
	int right_sensor = analogRead(QRD_R);
	int error = 0;

	if (left_sensor > q_threshold && right_sensor > q_threshold)
		error = 0; // both sensors on black
	else if (left_sensor > q_threshold && right_sensor <= q_threshold)
		error = -1;	// left sensor on black
	else if (left_sensor <= q_threshold && right_sensor > q_threshold)
		error = 1; // right sensor on black
	else if (left_sensor <= q_threshold && right_sensor <= q_threshold)
	{
		// neither sensor on black. check last error to see which side we are on.
		if ( last_error > 0)
			error = 4;
		else if ( last_error <= 0)
			error = -4;
	}
	if ( !(error == last_error))
	{
		recent_error = last_error;
		to = t;
		t = 1;
	}

	P_error = q_pro_gain * error;
	D_error = q_diff_gain * ((float)(error - recent_error) / (float)(t + to)); // time is present within the differential gain
	net_error = P_error + D_error;

	// prevent adjusting errors from going over actual speed.
	net_error = constrain(net_error, -base_speed, base_speed);

	//if net error is positive, right_motor will be weaker, will turn to the right
	motor.speed(LEFT_MOTOR, base_speed + net_error);
	motor.speed(RIGHT_MOTOR, base_speed - net_error);

	if ( count == 100 ) {
		count = 0;
		LCD.clear(); LCD.home();
		LCD.print(petCount);
		LCD.print(" LQ:"); LCD.print(left_sensor);
		LCD.print(" LM:"); LCD.print(base_speed + net_error);
		LCD.setCursor(0, 1);
		LCD.print("RQ:"); LCD.print(right_sensor);
		LCD.print(" RM:"); LCD.print(base_speed - net_error);
		// LCD.print("LE:"); LCD.print(encount_L); LCD.print(" RE:"); LCD.print(encount_R);
		// LCD.setCursor(0, 1); //LCD.print(s_L); LCD.print(" "); LCD.print(s_R); LCD.print(" ");
		// LCD.print("base:"); LCD.print(base_speed); LCD.print(" ");
		// LCD.print((s_L + s_R) / 2);
	}

	last_error = error;
	count++;
	t++;
}

void irPID() {

	encoderProcess();

	int left_sensor = analogRead(IR_L);
	int right_sensor = analogRead(IR_R);
	int error = right_sensor - left_sensor;
	int average = (left_sensor + right_sensor) >> 3;

	P_error = (ir_pro_gain) * error;
	D_error = ir_diff_gain * (error - last_error);
	I_error += ir_int_gain * error;
	net_error = static_cast<int32_t>(P_error + D_error + I_error) >> 4;
	Serial.print(average); Serial.print(" ");
	Serial.println(net_error);
	//Serial.print(D_error); Serial.print(" ");

	// Limit max error
	net_error = constrain(net_error, -base_speed, base_speed);

	//if net error is positive, right_motor will be stronger, will turn to the left
	motor.speed(LEFT_MOTOR, base_speed + net_error);
	motor.speed(RIGHT_MOTOR, base_speed - net_error);

	if ( count == 100 ) {
		count = 0;
		LCD.clear(); LCD.home();
		LCD.print("L:"); LCD.print(left_sensor);
		LCD.print(" R:"); LCD.print(right_sensor);
		LCD.setCursor(0, 1);
		LCD.print("ERR:"); LCD.print(net_error);
		//LCD.print("LM:"); LCD.print(base_speed + net_error); LCD.print(" RM:"); LCD.print(base_speed - net_error);
	}

	last_error = error;
	count++;
}

/* Helper Functions */
void petProcess() {
	if (checkPet()) {
		pauseDrive();
		petCount++;

		if (petCount == 1) {
			getFirstPet();
			// decrease base speed for the turn
			base_speed = 70;

			// int ql = analogRead(QRD_L);
			// int qr = analogRead(QRD_R);
			// if (ql > q_threshold && qr > q_threshold) {
			// 	error = 0;
			// } else if ( ql > q_threshold && qr <= q_threshold) {
			// 	error = -1;
			// } else if (ql <= q_threshold && qr > q_threshold) {
			// 	error = 1;
			// } else if (ql <= q_threshold && qr <= q_threshold) {
			// 	// neither sensor on black. check last error to see which side we are on.
			// 	if ( last_error > 0)
			// 		error = 4;
			// 	else if ( last_error <= 0)
			// 		error = -4;
			// }

			while (!stopbutton()) {
				LCD.clear(); LCD.home();
				//LCD.print(error); LCD.print(" "); LCD.print(last_error); LCD.print(" "); LCD.print(recent_error);
				LCD.print(petCount); LCD.print(" ");
				LCD.setCursor(0, 1); LCD.print(analogRead(QRD_L)); LCD.print(" "); LCD.print(analogRead(QRD_R));
				delay(200);
			}
		} else if (petCount == 2) {
			hardStop();
			getSecondPet();
			// error = 0;
			// recent_error = error;
			// last_error = 0;

			// int ql = analogRead(QRD_L);
			// int qr = analogRead(QRD_R);
			// if (ql > q_threshold && qr > q_threshold) {
			// 	error = 0;
			// } else if ( ql > q_threshold && qr <= q_threshold) {
			// 	error = -1;
			// } else if (ql <= q_threshold && qr > q_threshold) {
			// 	error = 1;
			// } else if (ql <= q_threshold && qr <= q_threshold) {
			// 	// neither sensor on black. check last error to see which side we are on.
			// 	if ( last_error > 0)
			// 		error = 4;
			// 	else if ( last_error <= 0)
			// 		error = -4;
			// }
			while (!stopbutton()) {
				LCD.clear(); LCD.home();
				//LCD.print("E:"); LCD.print(error); LCD.print(" LE:"); LCD.print(last_error); LCD.print(" RE:"); LCD.print(recent_error);
				LCD.setCursor(0, 1); LCD.print(analogRead(QRD_L)); LCD.print(" "); LCD.print(analogRead(QRD_R));
				delay(200);
			}

		} else if (petCount == 3) {
			// for pausing motors on the ramp.
			motor.speed(LEFT_MOTOR, base_speed);
			motor.speed(RIGHT_MOTOR, base_speed);
			delay(100);
			motor.speed(LEFT_MOTOR, 40);
			motor.speed(RIGHT_MOTOR, 40);

			if (secPet) {
				placeSecondPet();
			}
			delay(800);
			getThirdPet();

			base_speed = 170;

			// error = 1;
			// last_error = 0;
		} else if (petCount == 5) { //TODO: temporarily 5 for 4th pet
			// TODO: implement more elegant switching to ir -in timedPivot
			armCal();
			pivot(3);
			getFourthPet();

			pivotToIR(LEFT, 75); //TODO: Tune IR threshold value
			encount_L = 0;
			encount_R = 0;
			switchMode();
			petCount = 4;
		}

		// speed control
		if ( petCount == 2 ) {
			lastSpeedUp = millis();
		} else if (petCount == 3 ) {
			lastSpeedUp = millis();
		}

	}

	// speed control
	if (petCount == 2 && millis() - lastSpeedUp > 1200) {
		base_speed = 170;
		q_pro_gain = 70;
		q_diff_gain = 15;
		// q_diff_gain = menuItems[2].Value;
	} else if (petCount == 3 && millis() - lastSpeedUp > 1400) {
		base_speed = menuItems[0].Value;
		q_pro_gain = menuItems[1].Value;
	}
}

void switchMode() {
	pidfn = irPID;
}

// Set arm vertical height
void setUpperArm(int V) {
	upperArmV = V;
}

void setLowerArm(int V) {
	lowerArmV = V;
}

// Keep arm vertically in place. Should be run along with PID.
void upperArmPID() {
	int currentV = constrain(analogRead(UPPER_POT), 300, 740);
	int diff = currentV - upperArmV;
	if ( diff <= 20 && diff >= -20) {
		diff = 0;
	}
	// if ( diff  > 0) diff = 255;
	// else if (diff < 0) diff = -255;
	diff = 6 * diff;
	diff = constrain(diff, -255, 255);
	motor.speed(UPPER_ARM, diff);
}

void lowerArmPID() {
	int currentV = constrain(analogRead(LOWER_POT), 330, 610);
	int diff = currentV - lowerArmV;
	if (diff <= 22 && diff >= -22) {
		diff = 0;
	}
	if (diff  > 0) diff = 255;
	else if (diff < 0) diff = -255;

	// diff = 4 * (diff);
	// diff = constrain(diff, -255, 255);
	motor.speed(LOWER_ARM, diff);
}

// Power lower arm upwards till it goes past threshold.
// Caution when using this function.
void raiseLowerArm(int V) {
	lowerArmV = V;
	int currentV = constrain(analogRead(LOWER_POT), 350, 600);
	while (currentV - lowerArmV < 10) {
		currentV = constrain(analogRead(LOWER_POT), 350, 600);
		motor.speed(LOWER_ARM, -255);
	}
	motor.stop(LOWER_ARM);
}

// Stop driving
void pauseDrive() {
	motor.stop(LEFT_MOTOR);
	motor.stop(RIGHT_MOTOR);
}

void hardStop() {
	motor.speed(LEFT_MOTOR, -base_speed);
	motor.speed(RIGHT_MOTOR, -base_speed);
	delay(50);
	pauseDrive();
}

// Stop arm movement
void pauseArms() {
	motor.stop(LOWER_ARM);
	motor.stop(UPPER_ARM);
}

// Power the catapult for a time ms.
void launch(int ms) {
	// start catapult motion (relay on)
	digitalWrite(LAUNCH_F, HIGH);
	delay(ms);
	// stop catapult motion (relay off)
	digitalWrite(LAUNCH_F, LOW);
}

// Checks for the horizontal line that signals a pet to pick up.
boolean checkPet() {
	if (petCount < LAST_TAPE_PET) {
		int e = analogRead(QRD_LINE);
		if ( e > h_threshold && onTape == false) {
			onTape = true;
			return true;
		} else if ( e < q_threshold ) {
			onTape = false;
		}

	} else {

		if (petCount == LAST_TAPE_PET) {
			if (checkRafterPet()) return true;
		} else if ( petCount == LAST_TAPE_PET + 1) {
			if (checkBoxedPet()) return true;
		} else if (petCount == LAST_TAPE_PET + 2) {
			//TODO: SOMETHING COOL WHEN WE FINISH!!!!!!!!!!!!!!!!!
		}
	}
	return false;
}

// Checks if it is time to pick up the pet on the rafter in IR following
boolean checkRafterPet() {
	// TODO: needs tuning
	if ( encount_L >= ENC_RAFTER && encount_R >= ENC_RAFTER ) {
		return true;
	}
	return false;
}

// Check if robot has followed to the end, where the box is
boolean checkBoxedPet() {
	if (digitalRead(FRONT_SWITCH) == LOW) {
		return true;
	}
	return false;
}

// Check if switch on arm is activated
boolean petOnArm() {
	if (digitalRead(HAND_SWITCH) == LOW) {
		return true;
	}
	return false;
}

// Pivot the robot for a specified number of encoder
// counts on both motors.
void pivot(int counts) {
	// flags to check whether motors have stopped or not
	boolean lflag = false, rflag = false;
	// cache starting values
	int pivotCount = abs(counts);
	int pivotEncountStart_L = encount_L;
	int pivotEncountStart_R = encount_R;
	int speed = STABLE_SPEED;

	motor.speed(RIGHT_MOTOR, counts > 0 ? -speed : speed);
	motor.speed(LEFT_MOTOR, counts > 0 ? speed : -speed );
	while (lflag == false || rflag == false) {
		encoderProcess();
		if (encount_L - pivotEncountStart_L >= pivotCount) {
			motor.stop(LEFT_MOTOR);
			lflag = true;
		}
		if (encount_R - pivotEncountStart_R >= pivotCount) {
			motor.stop(RIGHT_MOTOR);
			rflag = true;
		}
	}
}

// Pivot robot a direction d (LEFT/RIGHT) until both QRDs are over threshold
// Will not start checking for tape until after timer
// void pivotToLine(int d, int timer) {
// 	LCD.clear(); LCD.home();
// 	int fast =  STABLE_SPEED + 20;
// 	int speed = fast;
// 	int slow = STABLE_SPEED - 20;
// 	if ( d == LEFT) {
// 		motor.speed(RIGHT_MOTOR, speed);
// 		motor.speed(LEFT_MOTOR, -speed);
// 	} else if ( d == RIGHT) {
// 		motor.speed(RIGHT_MOTOR, -speed);
// 		motor.speed(LEFT_MOTOR, speed);
// 	}
// 	LCD.print("PIVOTING"); // ghetto fix: pivoting works with LCD print???
// 	uint32_t start = millis();
// 	while (true) {
// 		if (millis() - start >= timer) {
// 			LCD.clear(); LCD.home();
// 			LCD.print("time up");
// 			speed = slow;
// 		}

// 		if ((analogRead(QRD_L) >= q_threshold || analogRead(QRD_R) >= q_threshold) && millis() - start >= timer) {
// 			LCD.clear(); LCD.home();
// 			LCD.print("detected");
// 			speed = 255;
// 			if (d == LEFT) {
// 				motor.speed(RIGHT_MOTOR, -speed);
// 				motor.speed(LEFT_MOTOR, speed);
// 				delay(100);
// 			} else if (d == RIGHT) {
// 				motor.speed(RIGHT_MOTOR, speed);
// 				motor.speed(LEFT_MOTOR, -speed);
// 				delay(100);
// 			}
// 			pauseDrive();
// 			return;
// 		}
// 	}
// }
void pivotToLine(int d, int timer) {
	if ( d == LEFT) {
		motor.speed(RIGHT_MOTOR, STABLE_SPEED + 20);
		motor.speed(LEFT_MOTOR, -STABLE_SPEED);
	} else if ( d == RIGHT) {
		motor.speed(RIGHT_MOTOR, -STABLE_SPEED - 20);
		motor.speed(LEFT_MOTOR, STABLE_SPEED);
	}
	uint32_t start = millis();
	while (true) {
		if ((analogRead(QRD_L) >= q_threshold || analogRead(QRD_R) >= q_threshold) && millis() - start >= timer) {
			if (d == LEFT) {
				motor.speed(RIGHT_MOTOR, -STABLE_SPEED);
				motor.speed(LEFT_MOTOR, STABLE_SPEED);
				delay(110);
			} else if (d == RIGHT) {
				motor.speed(RIGHT_MOTOR, STABLE_SPEED);
				motor.speed(LEFT_MOTOR, -STABLE_SPEED);
				delay(110);
			}
			pauseDrive();
			return;
		}
	}
}

void pivotOnLine(int d, int timer, int delayTime) {
	if ( d == LEFT) {
		motor.speed(RIGHT_MOTOR, STABLE_SPEED + 20);
		motor.speed(LEFT_MOTOR, -STABLE_SPEED);
	} else if ( d == RIGHT) {
		motor.speed(RIGHT_MOTOR, -STABLE_SPEED - 20);
		motor.speed(LEFT_MOTOR, STABLE_SPEED);
	}
	uint32_t start = millis();
	while (true) {
		if ((analogRead(QRD_L) >= q_threshold && analogRead(QRD_R) >= q_threshold) && millis() - start >= timer) {
			// if (d == LEFT) {
			// 	motor.speed(RIGHT_MOTOR, -STABLE_SPEED);
			// 	motor.speed(LEFT_MOTOR, STABLE_SPEED);
			// 	delay(110);
			// } else if (d == RIGHT) {
			// 	motor.speed(RIGHT_MOTOR, STABLE_SPEED);
			// 	motor.speed(LEFT_MOTOR, -STABLE_SPEED);
			// 	delay(110);
			// }
			delay(delayTime);
			pauseDrive();
			return;
		}
	}
}

// Pivot in direction until IR_sensor threshold value has been reached for IR_following
void pivotToIR(int d, int threshold) {
	if ( d == LEFT) {
		motor.speed(RIGHT_MOTOR, STABLE_SPEED + 20);  // Pivoting, can also use turn forward?
		motor.speed(LEFT_MOTOR, -STABLE_SPEED);
	} else if ( d == RIGHT) {
		motor.speed(RIGHT_MOTOR, -STABLE_SPEED - 20);
		motor.speed(LEFT_MOTOR, STABLE_SPEED);
	}

	while (true) {
		if ((analogRead(IR_R) >= threshold || analogRead(IR_L) >= threshold)) {
			// Shoule be unnecessary for IR following
			// if (d == LEFT) {
			// 	motor.speed(RIGHT_MOTOR, -STABLE_SPEED);
			// 	motor.speed(LEFT_MOTOR, STABLE_SPEED);
			// 	delay(110);
			// } else if (d == RIGHT) {
			// 	motor.speed(RIGHT_MOTOR, STABLE_SPEED);
			// 	motor.speed(LEFT_MOTOR, -STABLE_SPEED);
			// 	delay(110);
			// }
			pauseDrive();
			return;
		}
	}
}

// Turn robot left (counts < 0) or right (counts > 0) for
// certain amount of encoder counts forward
void turnForward(int counts, int s) {
	int turnCount = abs(counts);
	int turnEncountStart_L = encount_L;
	int turnEncountStart_R = encount_R;

	if (counts < 0) {
		motor.stop(LEFT_MOTOR);
		motor.speed(RIGHT_MOTOR, s);
		while (true) {
			encoderProcess();
			if (encount_R - turnEncountStart_R >= turnCount) {
				motor.stop(RIGHT_MOTOR);
				return;
			}
		}
	} else if (counts > 0) {
		motor.stop(RIGHT_MOTOR);
		motor.speed(LEFT_MOTOR, s);
		while (true) {
			encoderProcess();
			if (encount_L - turnEncountStart_L >= turnCount) {
				motor.stop(LEFT_MOTOR);
				return;
			}
		}
	}
}

// Turn robot left (counts < 0) or right (counts > 0) for
// certain amount of encoder counts backward
void turnBack(int counts) {
	boolean flag = false;
	int turnCount = abs(counts);
	int turnEncountStart_L = encount_L;
	int turnEncountStart_R = encount_R;

	if (counts < 0) {
		motor.speed(RIGHT_MOTOR, -STABLE_SPEED);
		motor.stop(LEFT_MOTOR);
		while (flag == false) {
			if (encount_R - turnEncountStart_R >= turnCount) {
				motor.stop(RIGHT_MOTOR);
				flag = true;
			}
		}
	} else if (counts > 0) {
		motor.stop(RIGHT_MOTOR);
		motor.speed(LEFT_MOTOR, -STABLE_SPEED);
		while (flag == false) {
			if (encount_L - turnEncountStart_L >= turnCount) {
				motor.stop(LEFT_MOTOR);
				flag = true;
			}
		}
	}
}

// Pivot in a direction d for a time t.
void timedPivot(int32_t t) {
	int32_t at = abs(t);
	if ( t < 0) {
		motor.speed(RIGHT_MOTOR, STABLE_SPEED + 20);
		motor.speed(LEFT_MOTOR, -STABLE_SPEED);
	} else if (t > 0) {
		motor.speed(RIGHT_MOTOR, -STABLE_SPEED - 20);
		motor.speed(LEFT_MOTOR, STABLE_SPEED);
	} else {
		pauseDrive();
	}
	delay(at);
	pauseDrive();
}

// Travel in a direction d for a number of counts.
void travel(int counts, int d) {

	boolean lflag = false, rflag = false;
	int travelCount = counts;
	int travelEncountStart_L = encount_L;
	int travelEncountStart_R = encount_R;
	lastTravelTime = millis();

	// TODO: one motor may need a power offset to travel straight
	motor.speed(RIGHT_MOTOR, d == FORWARDS ? STABLE_SPEED : -STABLE_SPEED);
	motor.speed(LEFT_MOTOR, d == FORWARDS ? STABLE_SPEED : -STABLE_SPEED);
	while (lflag == false || rflag == false) {
		encoderProcess();
		if (encount_L - travelEncountStart_L >= travelCount) {
			motor.stop(LEFT_MOTOR);
			lflag = true;
		}
		if (encount_R - travelEncountStart_R >= travelCount) {
			motor.stop(RIGHT_MOTOR);
			rflag = true;
		}
	}
}

// Travel in a direction d for a time t.
void timedTravel( uint32_t t, int d) {
	motor.speed(RIGHT_MOTOR, d == FORWARDS ? STABLE_SPEED : -STABLE_SPEED);
	motor.speed(LEFT_MOTOR, d == FORWARDS ? STABLE_SPEED : -STABLE_SPEED);
	delay(t);
	pauseDrive();
}

// processfn for first IR segment
void rafterProcess() {
	if (checkRafterPet()) {
		petCount++;
		pauseDrive();
		// TODO: rafter pet pickup here
		// getFifthPet();
		armCal();
		pivotToIR(LEFT, 300);  // Might hit rafter
		//processfn = buriedProcess;
	}
}

// processfn for second IR segment
void buriedProcess() {
	if (checkBoxedPet()) {
		petCount++;
		pauseDrive();
		// TODO: buried pet pickup here
		// getSixthPet();
		armCal();
	}
}

// Drop the pet helper function
void dropPet() {
	LCD.clear(); LCD.home(); LCD.print("DR- DR- DR-");
	LCD.setCursor(0, 1); LCD.print("DROP THE PET!");
	RCServo2.write(0);
	delay(450);
	RCServo2.write(90);
}

// Pivot the arm from and to
void pivotArm( int from, int to, int increment) {
	int p = from;
	if (from > to) {
		while ( p > to) {
			p -= increment;
			p = constrain(p, to, 185);
			RCServo0.write(p);
			delay(100);
		}
	} else if ( from < to) {
		while ( p < to ) {
			p += increment;
			p = constrain(p, 0, to);
			RCServo0.write(p);
			delay(100);
		}
	}
}

// Adjust pivot arm position in multiple attempts of pet
void adjustArm(int pivotPosition, int try_num) {
	if (try_num == 1) {
		RCServo0.write(pivotPosition + 5); //TODO: Tune adjust amount
	} else if (try_num == 2) {
		RCServo0.write(pivotPosition - 10);
	}
}

//Function to get first pet
void getFirstPet() {

	boolean flag = false;
	uint32_t timeStart = millis();
	int pivotPosition = 32;
	int c = 0;

	boolean unsuccessful = false;
	int try_num = 0;

	// first stage pickup - pick up pet; checks if pet is picked up,
	// if not, pick up pet again
	RCServo0.write(pivotPosition);
	delay(200);
	while (!flag) {

		upperArmPID();
		lowerArmPID();

		unsigned int dt = millis() - timeStart;

		if ( dt >= 1000 && c == 0 ) {
			setLowerArm(520);
			c++;
		} else if ( dt >= 2000 && c == 1 ) {
			setUpperArm(350);
			c++;
		} else if ( dt >= 4000 && c == 2 ) {
			setUpperArm(720);
			c++;
		} else if ( dt >= 4500 && c == 3) {
			if (petOnArm()) {
				c++;
			} else if (try_num < 2) {
				try_num++;
				adjustArm(pivotPosition, try_num); // Adjust pivot arm during multiple attempts
				// Does not change lower arm position
				// May waste a lot of time potentially.. (~6s each attempt)
				c = 1;
				timeStart = millis() - 1500;
			} else if (try_num >= 2 && !petOnArm()) { //Gives up after three attempts preventing infinite loop
				flag = true;
				unsuccessful = true;
			}
		} else if ( c == 4 ) {
			setLowerArm(590); // See "REDUN" can set lower arm position here?
			c++;
		} else if ( dt >= 7000 && c == 5) {
			flag = true;
		}
	}

	motor.stop_all(); // ensure motors are not being powered

	if (!unsuccessful) {
		placePetCatapult(pivotPosition);
		delay(500);
		// move arm out of catapult's way
		RCServo0.write(70);
		c = 0;
		flag = false;
		timeStart = millis();
		while (!flag) {
			lowerArmPID();
			unsigned int dt = millis() - timeStart;
			if (dt >= 0 && c == 0) {
				// this lower arm height is also the height second pet is picked up from
				setLowerArm(410);
				c++;
			} else if ( dt >= 1000 && c == 1) {
				flag = true;
			}
		}
		pauseArms(); // ensure arms are not powered
		pivotToLine(RIGHT, 2000);
		delay(500);
		launch(85);
		pivotToLine(RIGHT, 1000);
		delay(300);

	} else {
		// while (!stopbutton()) {
		// 	LCD.clear(); LCD.home();
		// 	LCD.print(analogRead(QRD_L)); LCD.print(" "); LCD.print(analogRead(QRD_R));
		// 	delay(200);
		// }

		c = 0;
		flag = false;
		timeStart = millis();
		while (!flag) {
			lowerArmPID();
			unsigned int dt = millis() - timeStart;
			if (dt >= 0 && c == 0) {
				// this lower arm height is also the height second pet is picked up from
				setLowerArm(410);
				c++;
			} else if ( dt >= 1000 && c == 1) {
				flag = true;
			}
		}

	}

	if (analogRead(QRD_L) >= q_threshold && analogRead(QRD_R) < q_threshold)
		pivotOnLine(LEFT, 0, 0);
	else if (analogRead(QRD_R) >= q_threshold && analogRead(QRD_L) < q_threshold)
		pivotOnLine(RIGHT, 0, 0);
	else if (analogRead(QRD_L) < q_threshold && analogRead(QRD_R) < q_threshold)
		pivotOnLine(LEFT, 0, 0);
}

void getSecondPet() {
	boolean flag = false;
	uint32_t timeStart = millis();
	int pivotPosition = 39;
	int c = 0;

	boolean unsuccessful = false;
	int try_num = 0;

	// first stage pickup - pick up pet; checks if pet is picked up,
	// if not, pick up pet again
	RCServo0.write(pivotPosition);
	delay(500);
	while (!flag) {
		upperArmPID();
		lowerArmPID();
		unsigned int dt = millis() - timeStart;

		if ( dt >= 500 && c == 0 ) {
			setUpperArm(350);
			c++;
		} else if ( dt >= 2500 && c == 1 ) {
			setUpperArm(720);
			c++;
		} else if ( dt >= 4500 && c == 2) {
			if (petOnArm()) {
				c++;
			} else if (try_num < 2) {
				try_num++;
				adjustArm(pivotPosition, try_num);
				c = 0;
				timeStart = millis() - 500;
			} else if (try_num >= 2 && !petOnArm()) {
				c = 3;
				timeStart = millis() - 4500;
				// second pet not picked up flag
				unsuccessful = true;
				secPet = false;
			}
		} else if ( c == 3 ) {
			setLowerArm(590);
			c++;
		} else if ( dt >= 6000 && c == 4 ) {
			flag = true;
		}
	}


	motor.stop_all();

	if (!unsuccessful) {
		pivotArm(pivotPosition, 157, 10);
		c = 0;
		flag = false;
		timeStart = millis();
		while (!flag) {
			lowerArmPID();
			upperArmPID();

			unsigned int dt = millis() - timeStart;
			if ( dt >= 500 && c == 0 ) {
				setLowerArm(535);
				c++;
			} else if ( dt >= 1500 && c == 1) {
				pauseArms();
				flag = true;
			}
		}
	}
	LCD.clear(); LCD.home();
	delay(1000);
	if (analogRead(QRD_L) >= q_threshold && analogRead(QRD_R) < q_threshold) {
		pivotOnLine(LEFT, 0, 15);
		LCD.print("left on");
	}
	else if (analogRead(QRD_R) >= q_threshold && analogRead(QRD_L) < q_threshold) {
		pivotOnLine(RIGHT, 0, 0);
		LCD.print("right on");
	}
	else if (analogRead(QRD_L) < q_threshold && analogRead(QRD_R) < q_threshold) {
		pivotOnLine(LEFT, 0, 15);
		LCD.print("just left");
	}
	delay(1000);
	//placePetCatapult(pivotPosition);
}

void placeSecondPet() {
	boolean flag = false;
	int pivotTo = 118;
	int c = 1;
	// TODO: test if pivoting is actually needed
	turnForward(-2, 110);
	motor.speed(LEFT_MOTOR, 40);
	motor.speed(RIGHT_MOTOR, 40);

	setUpperArm(700);
	uint32_t timeStart = millis();
	while (!flag) {
		upperArmPID();
		lowerArmPID();

		unsigned int dt = millis() - timeStart;

		if ( dt >= 750 && c == 1 ) {
			setLowerArm(610);
			c++;
		} else if (dt >= 2000 && c == 2) {
			pivotArm(157, pivotTo, 10);
			delay(500);
			setLowerArm(280);
			c++;
		} else if ( dt >= 3500 && c == 3) {
			setUpperArm(510);
			c++;
		} else if ( dt >=  4500 && c == 4) {
			dropPet();
			c++;
		} else if ( dt >= 5300 && c == 5) {
			setLowerArm(590);
			c++;
		} else if ( dt >= 6300 && c == 6) {
			setUpperArm(710);
			c++;
		} else if ( dt >= 7700 && c == 7) {
			flag = true;
		}
	}
	pauseArms();
}

void getThirdPet() {
	boolean flag = false;
	int pivotPosition = 26;
	int c = 0;

	boolean unsuccessful = false;
	int try_num = 0;

	// RCServo0.write(pivotPosition);
	pivotArm(118, pivotPosition, 8);
	uint32_t timeStart = millis();

	// first stage pickup - pick up pet; checks if pet is picked up,
	// if not, pick up pet again
	while (!flag) {

		upperArmPID();
		lowerArmPID();
		unsigned int dt = millis() - timeStart;

		if ( dt >= 600 && c == 0 ) {
			setLowerArm(510);
			c++;
		} else if ( dt >= 1600 && c == 1 ) {
			setUpperArm(385);
			c++;
		} else if ( dt >= 3600 && c == 2 ) {
			setUpperArm(720);
			c++;
		} else if ( dt >= 5600 && c == 3) {
			if (petOnArm()) {
				c++;
			} else {
				c = 1;
				timeStart = millis() - 1500;
			}
		} else if ( c == 4) {
			setLowerArm(590); // See "REDUN" can set lower arm position here?
			c++;
		} else if ( dt >= 6600 && c == 5) {
			flag = true;
		}
	}

	// pivot arm to correct location
	pivotArm(pivotPosition, 115, 10); //TODO: tune

	flag = false;
	c = 0;
	timeStart = millis();
	//move upper/lower arm to correct position for drop;
	while (!flag) {
		upperArmPID();
		lowerArmPID();

		unsigned int dt = millis() - timeStart;
		// TODO: just copy place second pet drop loop once that is working.
		if ( dt >= 500 && c == 0 ) {
			setLowerArm(350);
			c++;
		} else if ( dt >= 1500 && c == 1) {
			setUpperArm(520);
			c++;
		} else if ( dt >= 2500 && c == 2) {
			dropPet();
			delay(200);
			setUpperArm(700);
			setLowerArm(590);
			c++;
		} else if (dt >= 4000 && c == 3) {
			flag = true;
		}
	}
	pauseArms();
}

void getFourthPet() {
	boolean flag = false;
	timedPivot(300); // TODO tune: makes pet pick up easier
	uint32_t timeStart = millis();
	int pivotPosition = 110; //TODO: tune -pet will be on left side of robot
	int c = 0;

	// first stage pickup - pick up pet; checks if pet is picked up,
	// if not, pick up pet again
	RCServo0.write(pivotPosition);
	delay(200);
	while (!flag) {
		upperArmPID();
		lowerArmPID();

		unsigned int dt = millis() - timeStart;

		if ( dt >= 1000 && c == 0 ) {
			setLowerArm(490); //TODO: tune
			c++;
		} else if ( dt >= 2000 && c == 1 ) {
			setUpperArm(350);
			c++;
		} else if ( dt >= 4000 && c == 2 ) {
			setUpperArm(720);
			c++;
		} else if ( dt >= 6000 && c == 3) {
			if (petOnArm()) {
				c++;
			} else {
				c = 1;
				timeStart = millis() - 1500;
			}
		} else if ( c == 4) {
			setLowerArm(590);
			c++;
		} else if ( dt >= 7500 && c == 5) {
			flag = true;
		}
	}
	placePetCatapult(pivotPosition);
	flag = false;
	delay(500);
	RCServo0.write(80);
	timeStart = millis();
	while (!flag) {
		lowerArmPID();
		unsigned int dt = millis() - timeStart;
		if (dt >= 0 && c == 3) {
			setLowerArm(480);
			c++;
		} else if ( dt >= 1000 && c == 4) {
			flag = true;
		}
	}
	pivot(2); //TODO: tune to lineup catapult
	delay(500);
	//launch(130); //TODO: tune to get enough distance

}

void getFifthPet() {
	boolean flag = false;
	uint32_t timeStart = millis();
	int pivotPosition = 37; //TODO: tune
	int c = 0;

	// first stage pickup - pick up pet; checks if pet is picked up,
	// if not, pick up pet again
	RCServo0.write(pivotPosition);
	delay(200);
	while (!flag) {
		upperArmPID();
		lowerArmPID();

		unsigned int dt = millis() - timeStart;
		// TODO: may be able to set lower upper arm at same time as pivot
		if ( dt >= 1000 && c == 0 ) {
			setLowerArm(530); //TODO: tune
			c++;
		} else if ( dt >= 2000 && c == 1 ) {
			setUpperArm(650); //TODO: tune
			c++;
		} else if ( dt >= 4000 && c == 2 ) {
			setUpperArm(720);
			c++;
		} else if ( dt >= 6000 && c == 3) {
			if (petOnArm()) {
				c++;
			} else {
				c = 1;
				timeStart = millis() - 1500;
			}
		} else if ( c == 4) {
			setLowerArm(590);
			c++;
		} else if ( dt >= 7500 && c == 5) {
			flag = true;
		}
	}
	placePetCatapult(pivotPosition);
	flag = false;
	c = 0;
	delay(500);

	// move arm out of catapult's way
	RCServo0.write(80);
	timeStart = millis();
	while (!flag) {
		lowerArmPID();
		unsigned int dt = millis() - timeStart;
		if (dt >= 0 && c == 0) {
			setLowerArm(480);
			c++;
		} else if ( dt >= 1000 && c == 1) {
			flag = true;
		}
	}
	motor.stop_all();
	timedPivot(-2400); //TODO: tune pivot towards the left or will hit stand
	delay(500);
	launch(150); //TODO: tune to get enough distance
}

void getSixthPet() {
	boolean flag = false;
	//timedPivot(400); //TODO pivot first
	uint32_t timeStart = millis();
	int pivotPosition = 37; //TODO: tune
	int c = 0;

	// first stage pickup - pick up pet; checks if pet is picked up,
	// if not, pick up pet again
	RCServo0.write(pivotPosition);
	delay(200);
	while (!flag) {
		upperArmPID();
		lowerArmPID();

		unsigned int dt = millis() - timeStart;
		// TODO: may be able to set lower upper arm at same time as pivot
		if ( dt >= 800 && c == 0 ) {
			setLowerArm(650);
			setLowerArm(545); //TODO: tune
			c++;
		} else if ( dt >= 1300 && c == 1 ) {
			setUpperArm(360);
			c++;
		} else if ( dt >= 2300 && c == 2 ) {
			setUpperArm(715);
			c++;
		} else if ( dt >= 4000 ) {
			if (petOnArm()) {
				flag = true;
				delay(1000);
			} else {
				// TODO: change arm pickup position
				pivotPosition += 10; // sweep to left
				RCServo0.write(pivotPosition);

				c = 1;
				timeStart = millis() - 800;
			}
		}
	}
	placePetCatapult(pivotPosition);
	flag = false;
	delay(500);
	RCServo0.write(80);
	timeStart = millis();

	while (!flag) {
		lowerArmPID();
		unsigned int dt = millis() - timeStart;
		if (dt >= 0 && c == 3) {
			setLowerArm(480);
			c++;
		} else if ( dt >= 1000 && c == 4) {
			flag = true;
		}
	}
	timedPivot(1800); //TODO: tune pivot towards the left..
	delay(500);
	launch(85); //TODO: tune

}

// Place pet in catapult from pivot arm's position 'pivotFrom'
void placePetCatapult(int pivotFrom) {
	int c = 0;
	pivotArm(pivotFrom, 157, 10);
	uint32_t timeStart = millis();

	while (true) {
		lowerArmPID();
		upperArmPID();

		unsigned int dt = millis() - timeStart;
		if ( dt >= 250 && c == 0 ) {
			setLowerArm(480);
			c++;
		}
		else if ( dt >= 1600 && c == 1) {
			dropPet();
			c++;
		} else if (dt >= 2200 && c == 2) {
			setLowerArm(580);
			c++;
		} else if (dt >= 3200 && c == 3 ) {
			pauseArms(); // ensure arms stop moving
			return;
		}
	}
}

// testing arm calibration code
void armCal() {
	int a;
	int s = 90; // temp
	int c = 0;

	while (!stopbutton()) {
		// temporary arm calibration code
		int selection = map(knob(6), 0 , 1023, 0, 3);

		if (selection == 0) {
			a = map(knob(7), 0 , 1023, 0 , 184);
		} else if (selection == 1) {
			a = map(knob(7), 0, 1023, 330, 590); // lower arm
		} else if ( selection == 2) {
			a = map(knob(7), 0 , 1023, 300, 720); // higher arm
		}

		if ( c >= 100) {
			c = 0;
			LCD.clear(); LCD.home();
			if (selection == 0)
				LCD.print("PIVOT ARM:");
			else if (selection == 1) {
				LCD.print("LOWER ARM: "); LCD.print(analogRead(LOWER_POT));
			}
			else if (selection == 2) {
				LCD.print("UPPER ARM:"); LCD.print(analogRead(UPPER_POT));
			}

			LCD.setCursor(0, 1); LCD.print(a); LCD.print("? S:");
			if (selection == 0)
				LCD.print(s);
			else if (selection == 1)
				LCD.print(lowerArmV);
			else if (selection == 2)
				LCD.print(upperArmV);
		}

		if (startbutton()) {
			delay(200);
			if (selection == 0) {
				s = a;
				RCServo0.write(s);
			} else if (selection == 1) {
				setLowerArm(a);
			} else if (selection == 2) {
				setUpperArm(a);
			}
		}

		if (digitalRead(FRONT_SWITCH) == LOW) {
			dropPet();
		}

		// move arm
		upperArmPID();
		lowerArmPID();
		c++;
	}
}

/* ISRs */

// Encoder ISRs. S == speed
void encoderProcess() {
	if (digitalRead(1) == HIGH && !ecL) {
		encount_L++;
		ecL = true;
	} else if (digitalRead(1) == LOW) {
		ecL = false;
	}
	if (digitalRead(2) == HIGH && !ecR) {
		encount_R++;
		ecR = true;
	} else if (digitalRead(2) == LOW) {
		ecR = false;
	}
}

void LE() {
	encount_L++;
}

void RE() {
	encount_R++;
}

void LES() {
	encount_L++;
	int ct = millis() - time_L;
	// filter out speeds less than 10 ms
	s_L = ct > 10 ? ct : s_L;
	time_L = millis();
}

void RES() {
	encount_R++;
	int ct = millis() - time_R;
	s_R = ct > 10 ? ct : s_R;
	time_R = millis();
}

/* Menus */
void QRDMENU()
{
	LCD.clear(); LCD.home();
	LCD.print("Entering submenu");
	delay(500);

	while (true)
	{
		/* Show MenuItem value and knob value */
		int menuIndex = knob(6) * (MenuItem::MenuItemCount) >> 10;
		LCD.clear(); LCD.home();
		LCD.print(menuItems[menuIndex].Name); LCD.print(" "); LCD.print(menuItems[menuIndex].Value);
		LCD.setCursor(0, 1);
		LCD.print("Set to ");
		if (menuIndex == 0 || menuIndex == 1 || menuIndex == 2) {
			LCD.print(knob(7) >> 2);
		} else {
			LCD.print(knob(7));
		}

		LCD.print("?");

		delay(100);

		/* Press start button to save the new value */
		if (startbutton())
		{
			delay(100);
			int val = knob(7); // cache knob value to memory
			if (menuIndex == 0) {
				val = val >> 2;
				LCD.clear(); LCD.home();
				LCD.print("Speed set to "); LCD.print(val);
				delay(250);
			} else if (menuIndex == 1 || menuIndex == 2) {
				val = val >> 2;
			}

			menuItems[menuIndex].Value = val;
			menuItems[menuIndex].Save();
			delay(250);
		}


		/* Press stop button to exit menu */
		if (stopbutton())
		{
			delay(100);
			if (stopbutton())
			{
				LCD.clear(); LCD.home();
				LCD.print("Leaving menu");
				// Set values after exiting menu
				base_speed = menuItems[0].Value;
				q_pro_gain = menuItems[1].Value;
				q_diff_gain = menuItems[2].Value;
				q_int_gain = menuItems[3].Value;
				q_threshold = menuItems[4].Value;
				h_threshold = menuItems[5].Value;
				delay(500);
				return;
			}
		}
	}
}

void IRMENU()
{
	LCD.clear(); LCD.home();
	LCD.print("Entering submenu");
	delay(500);

	while (true)
	{
		/* Show IRMenuItem value and knob value */
		int menuIndex = knob(6) * (IRMenuItem::MenuItemCount) >> 10;
		LCD.clear(); LCD.home();
		LCD.print(IRmenuItems[menuIndex].Name); LCD.print(" "); LCD.print(IRmenuItems[menuIndex].Value);
		LCD.setCursor(0, 1);
		LCD.print("Set to "); LCD.print(knob(7)); LCD.print("?");
		delay(100);

		/* Press start button to save the new value */
		if (startbutton())
		{
			delay(100);
			int val = knob(7); // cache knob value to memory
			IRmenuItems[menuIndex].Value = val;
			IRmenuItems[menuIndex].Save();
			delay(250);
		}

		/* Press stop button to exit menu */
		if (stopbutton())
		{
			delay(100);
			if (stopbutton())
			{
				LCD.clear(); LCD.home();
				LCD.print("Leaving menu");
				// set values
				ir_pro_gain = IRmenuItems[0].Value;
				ir_diff_gain = IRmenuItems[1].Value;
				ir_int_gain = IRmenuItems[2].Value;
				delay(500);
				return;
			}
		}
	}
}

void MainMenu() {
	LCD.clear(); LCD.home();
	LCD.print("Entering Main");
	delay(500);

	while (true)
	{
		/* Display submenu or pid mode */
		int menuIndex = knob(6) * (MainMenuItem::MenuItemCount) >> 10;
		LCD.clear(); LCD.home();
		if (menuIndex == 0) {
			// mode switching handling
			if (mode == "qrd") {
				LCD.print("LQ:"); LCD.print(analogRead(QRD_L)); LCD.print(" RQ:"); LCD.print(analogRead(QRD_R));
				LCD.setCursor(0, 1); LCD.print("HQ:"); LCD.print(analogRead(QRD_LINE));
			}  else if (mode == "ir") {
				LCD.print("L:"); LCD.print(analogRead(IR_L)); LCD.print(" R:"); LCD.print(analogRead(IR_R));
			} else {
				LCD.print("Error: no mode");
			}
			modeIndex = (knob(7) * (sizeof(modes) / sizeof(*modes))) >> 10; // sizeof(a)/sizeof(*a) gives length of array
			LCD.setCursor(9, 1);
			LCD.print(modes[modeIndex]); LCD.print("?");
		} else if (menuIndex == 3) {
			// pivot test menu option
			LCD.print(mainMenu[menuIndex].Name);
			LCD.setCursor(0, 1);
			//val = (knob(7) >> 3) - 64;
			val = (knob(7) >> 2 ) - 128;
			LCD.print(val); LCD.print("?");
		} else if ( menuIndex == 4) {
			// travel test menu option
			LCD.print(mainMenu[menuIndex].Name);
			LCD.setCursor(0, 1);
			val = map(knob(7), 0, 1023, 0, 3069);
			LCD.print(val); LCD.print("?");
		} else if (menuIndex == 5) {
			// launch catapult test menu option
			LCD.print(mainMenu[menuIndex].Name);
			LCD.setCursor(0, 1);
			val = knob(7) >> 1;
			LCD.print(val); LCD.print("?");
		} else {
			// generic submenu handling
			LCD.print(mainMenu[menuIndex].Name);
			LCD.setCursor(0, 1);
			LCD.print("Start to Select.");
		}

		/* Press start button to enter submenu / switch pid modes */
		if (startbutton())
		{
			LCD.clear(); LCD.home();
			MainMenuItem::Open(menuIndex);
		}

		/* Press stop button to exit menu */
		if (stopbutton())
		{
			delay(100);
			if (stopbutton())
			{
				LCD.clear(); LCD.home();
				LCD.print("Leaving menu");
				delay(500);
				// reset variables and counters
				base_speed = menuItems[0].Value;
				onTape = false;
				petCount = 0;
				count = 0;
				t = 1;
				last_error = 0;
				recent_error = 0;
				I_error = 0;
				LCD.clear();
				return;
			}
		}
		delay(150);
	}
}
