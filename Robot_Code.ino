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
		case 7:
			strategySelection();
			break;
		case 8:
			dropPetCtrl(val);
			LCD.clear(); LCD.home(); LCD.print("Turning...");
			delay(300);
			dropPetCtrl(STOP);
			break;
		case 9:
			qrdRead();
			break;
		case 10:
			travel(2, BACKWARDS);
			delay(300);
			turnForward(-13, 90);
			delay(300);
			travel(5, BACKWARDS);
			delay(300);
			turnBack(-4, 90);
			delay(300);
			while (!stopbutton()) {
			}
			LCD.clear(); LCD.home();
			LCD.print("Returning...");
			delay(500);
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
MainMenuItem strategy = MainMenuItem("strategy");
MainMenuItem hand = MainMenuItem("hand motor");
MainMenuItem qrdTest = MainMenuItem("QRDs");
MainMenuItem parallelPark = MainMenuItem("Driver Test");
MainMenuItem mainMenu[]  = {Sensors, TapePID, IRPID, pivotTest, travelTest, launchTest, armTest, strategy, hand, qrdTest, parallelPark};

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
	fullRun = eeprom_read_word(FULLRUN_EEPROM);

	// set ports 8 to 15 as OUTPUT
	portMode(1, OUTPUT);
	// ensure relays/digital outputs are LOW on start.
	digitalWrite(LAUNCH_F, LOW);
	digitalWrite(HAND_UP, LOW);
	digitalWrite(HAND_DOWN, LOW);

	lastSpeedUp = 0;

	// set servo initial positions
	RCServo2.write(90);
	RCServo0.write(85);

	// default PID loop is QRD tape following
	pidfn = tapePID;

	LCD.print("RC7"); LCD.setCursor(0, 1);
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
		// LCD.print(petCount);
		// LCD.print(" LQ:"); LCD.print(left_sensor);
		// LCD.print(" LM:"); LCD.print(base_speed + net_error);
		// LCD.setCursor(0, 1);
		// LCD.print("RQ:"); LCD.print(right_sensor);
		// LCD.print(" RM:"); LCD.print(base_speed - net_error);
		LCD.print("LE:"); LCD.print(encount_L); LCD.print(" RE:"); LCD.print(encount_R);
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

			if (fullRun) { // will just tape follow normally if strategy is last three
				getFirstPet();

				// decrease base speed for the turn
				base_speed = 70;

				// initial tape following conditions
				last_error = 0;
				recent_error = 0;
				t = 1;
				to = 0;

			}
			// Don't need this part of the loop
		} else if (petCount == 2) {

			if (fullRun) {

				//need to lift
				if (petOnArm) {

				}
				// recent_error = 0;
				// last_error = 0;
				// t = 1;
				// to = 0;

				// while (!stopbutton()) {
				// 	LCD.clear(); LCD.home();
				// 	//LCD.print("E:"); LCD.print(error);
				// 	LCD.print("LE:"); LCD.print(last_error); LCD.print(" RE:"); LCD.print(recent_error);
				// 	LCD.setCursor(0, 1); LCD.print(analogRead(QRD_L)); LCD.print(" "); LCD.print(analogRead(QRD_R));
				// 	delay(200);
				// }
			}

		} else if (petCount == 3) {

			if (fullRun) {
				// for pausing motors on the ramp.
				motor.speed(LEFT_MOTOR, base_speed);
				motor.speed(RIGHT_MOTOR, base_speed);
				delay(100);
				motor.speed(LEFT_MOTOR, 40);
				motor.speed(RIGHT_MOTOR, 40);

				if (petOnArm()) {
					placeSecondPet();
				}
				delay(800);
				getThirdPet();

				base_speed = 170;

				// reset tape following conditions
				t = 1;
				to = 0;
				recent_error = 0;
				last_error = -1;
			} else {
				base_speed = 170;
			}
			encount_L = 0;
			encount_R = 0;

		} else if (petCount == 4)  { //Slows down after detecting top of ramp
			base_speed = 60;

		} else if (petCount == LAST_TAPE_PET) { //TODO: temporarily 5 for 4th pet
			// TODO: implement more elegant switching to ir -in timedPivot
			// armCal();
			getFourthPet();

			encount_L = 0;
			encount_R = 0;
			switchMode();

			//RCServo0.write(35); //For Fifth Pet Pickup
			//Adjust Lower/Upper Arm here?
		} else if (petCount == LAST_TAPE_PET + 1) { //Enters loop when over encoder count or petOnArm

			fastPivot(7);
			delay(500);
			if (fourthPet) {
				launch(150);
				delay(500);
			}

			if (petOnArm()) {
				launchFifthPet();
			}
			fastPivot(-7); //Pivots to Right to avoid rafter
			RCServo0.write(90); // Prevents arm from hitting zipline, may also need to adjust lower/upper arm
			delay(250);
			int c = 0;
			boolean flag = false;
			uint32_t timeStart = millis();
			while (!flag) {
				upperArmPID();
				uint16_t dt = millis() - timeStart;
				if (c == 0) {
					setUpperArm(580);
					c++;
				} else if (dt >= 1200 && c == 1) {
					flag = true;
				}
			}

		} else if (petCount == LAST_TAPE_PET + 2) {

			travel(2, BACKWARDS);
			delay(300);
			turnForward(-13, 90);
			delay(300);
			travel(5, BACKWARDS);
			delay(300);
			turnBack(-4, 90);
			delay(300);
			armCal();
			getSixthPet();
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
	}
	// else if (petCount == 3 && millis() - lastSpeedUp > 1400) {
	// 	base_speed = menuItems[0].Value;
	// 	q_pro_gain = menuItems[1].Value;
	// }
}

void switchMode() {
	pidfn = irPID;
}

// Set arm vertical height
void setUpperArm(int V) {
	upperArmV = V;
}

void setLowerArm(int V) {
	lowerArmV = V + 15;
}

// Keep arm vertically in place. Should be run along with PID.
void upperArmPID() {
	int currentV = constrain(analogRead(UPPER_POT), 300, 740);
	int diff = currentV - upperArmV;
	if ( diff <= 25 && diff >= -25) {
		diff = 0;
	}
	if ( diff  > 0) diff = 255;
	else if (diff < 0) diff = -255;
	// diff = 6 * diff;
	// diff = constrain(diff, -255, 255);
	motor.speed(UPPER_ARM, diff);
}

void lowerArmPID() {
	int currentV = constrain(analogRead(LOWER_POT), 330, 645);
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
	if (petCount < LAST_TAPE_PET - 1) {
		int e = analogRead(QRD_LINE);
		if ( e >= h_threshold && onTape == false) {
			onTape = true;
			return true;
		} else if ( e < q_threshold ) {
			onTape = false;
		}

	} else {
		if (petCount == LAST_TAPE_PET - 1) {
			int e = analogRead(QRD_LINE);
			// for the fourth pet, the tape will only trigger when left encoder has surpassed
			// right encoder by 10 so we know that the turn has been made by the robot.
			if ( e >= h_threshold && onTape == false && (encount_L - encount_R) > 11) {
				onTape = true;
				return true;
			} else if ( e < q_threshold ) {
				onTape = false;
			}
		}
		else if (petCount == LAST_TAPE_PET) {
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
	LCD.clear(); LCD.home();
	LCD.print("E:"); LCD.print(encount_L); LCD.print(" "); LCD.print(encount_R);
	LCD.setCursor(0, 1); LCD.print(pivotEncountStart_L); LCD.print(" "); LCD.print(pivotEncountStart_R);
}

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
	int reading = analogRead(IR_R);
	int lastreading = reading;
	if ( d == LEFT) {
		motor.speed(RIGHT_MOTOR, STABLE_SPEED + 20);  // Pivoting, can also use turn forward?
		motor.speed(LEFT_MOTOR, -STABLE_SPEED);
	} else if ( d == RIGHT) {
		motor.speed(RIGHT_MOTOR, -STABLE_SPEED - 20);
		motor.speed(LEFT_MOTOR, STABLE_SPEED);
	}

	while (true) {
		reading = analogRead(IR_R);
		if ( ((reading + lastreading) >> 1) >= threshold || analogRead(IR_L) >= threshold) {
			pauseDrive();
			return;
		}
		lastreading = reading;
	}
}

void fastPivot(int counts) {
	// flags to check whether motors have stopped or not
	boolean lflag = false, rflag = false;
	// cache starting values
	int pivotCount = abs(counts);
	int pivotEncountStart_L = encount_L;
	int pivotEncountStart_R = encount_R;
	int speed = 90;

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
	LCD.clear(); LCD.home();
	LCD.print("E:"); LCD.print(encount_L); LCD.print(" "); LCD.print(encount_R);
	LCD.setCursor(0, 1); LCD.print(pivotEncountStart_L); LCD.print(" "); LCD.print(pivotEncountStart_R);
}

// Turns robot on one wheel until either IR sensors pass the threshold
void turnToIR(int d, int threshold) {
	if (d == LEFT) {
		motor.stop(LEFT_MOTOR);
		motor.speed(RIGHT_MOTOR, STABLE_SPEED);
	} else if (d == RIGHT) {
		motor.stop(RIGHT_MOTOR);
		motor.speed(LEFT_MOTOR, STABLE_SPEED);
	}
	while (true) {
		if ((analogRead(IR_R) >= threshold || analogRead(IR_L) >= threshold)) {
			pauseDrive();
			return;
		}
	}
}

// Turn robot left (counts < 0) or right (counts > 0) for
// certain amount of encoder counts forward at speed s
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
void turnBack(int counts, int s) {
	int turnCount = abs(counts);
	int turnEncountStart_L = encount_L;
	int turnEncountStart_R = encount_R;
	int count = 0;

	if (counts < 0) {
		motor.speed(RIGHT_MOTOR, -s);
		motor.stop(LEFT_MOTOR);
		while (true) {
			encoderProcess();
			if (encount_R - turnEncountStart_R >= turnCount) {
				motor.stop(RIGHT_MOTOR);
				return;
			}
		}
	} else if (counts > 0) {
		motor.stop(RIGHT_MOTOR);
		motor.speed(LEFT_MOTOR, -s);
		while (true) {
			encoderProcess();
			if (encount_L - turnEncountStart_L >= turnCount) {
				motor.stop(LEFT_MOTOR);
				return;
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
	LCD.clear(); LCD.home();
	LCD.print("E:"); LCD.print(encount_L); LCD.print(" "); LCD.print(encount_R);
	LCD.setCursor(0, 1); LCD.print(travelEncountStart_L); LCD.print(" "); LCD.print(travelEncountStart_R);
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

// Tentative drop pet function for the new motor setup
// Time it takes to bring magnet up and down needs to be tested
void dropPet() {
	LCD.clear(); LCD.home(); LCD.print("DR- DR- DR-");
	LCD.setCursor(0, 1); LCD.print("DROP THE PET!");
	int duration = HAND_DURATION;
	uint32_t timeStart = millis();
	while (millis() - timeStart <= duration) {
		digitalWrite(HAND_UP, HIGH);
	}
	digitalWrite(HAND_UP, LOW);
	delay(200);
	timeStart = millis();
	setUpperArm(720);
	while (millis() - timeStart <= 1000) {
		upperArmPID();
	}
	pauseArms();
	timeStart = millis();
	while (millis() - timeStart <= duration) {
		digitalWrite(HAND_DOWN, HIGH);
	}
	digitalWrite(HAND_DOWN, LOW);
}

// Direct control of the hand motor
void dropPetCtrl(int com) {
	switch (com) {
	case RAISE:
		digitalWrite(HAND_UP, HIGH);
		break;
	case LOWER:
		digitalWrite(HAND_DOWN, HIGH);
		break;
	case STOP:
		digitalWrite(HAND_UP, LOW);
		digitalWrite(HAND_DOWN, LOW);
		break;
	}
}

// Drop the pet helper function
void dropPetServo() {
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
void adjustArm(int pivotPosition, int tries, int increment) {
	if (tries == 1) {
		RCServo0.write(pivotPosition + increment);
	} else if (tries == 2) {
		RCServo0.write(pivotPosition - (2 * increment));
	}
}

//Function to get first pet
void getFirstPet() {

	boolean flag = false, unsuccessful = false, found = false;
	int pivotPosition = 50;
	int pivotIncrement = 15;
	int c = 0, try_num = 0;
	int t1 = 1000, t2 = t1, t3 = t2 + 800, t4 = t3 + 1300, t5 = t4 + 1750;

	RCServo0.write(pivotPosition);
	delay(300);
	setUpperArm(400);
	uint32_t timeStart = millis();

	while (!flag) {

		upperArmPID();
		lowerArmPID();

		uint16_t dt = millis() - timeStart;

		if ( dt >= t1 && c == 0 ) {
			// check if pet is somewhere near hand
			if (petOnArm()) {
				found = true;
				try_num = 0;
				pivotIncrement = 7;
			}
			c++;
		} else if ( dt >= t2 && c == 1 ) {
			setUpperArm(500);
			c++;
		} else if ( dt >= t3 && c == 2 ) {
			if (petOnArm()) {
				c++;
			} else if (try_num < 2) {
				adjustArm(pivotPosition, try_num, pivotIncrement);
				try_num++;
				c = (found == true) ? 1 : 0;
				setUpperArm(400);
				timeStart = millis();
			} else if (try_num >= 2 && !petOnArm()) {
				c = 4;
				unsuccessful = true;
				timeStart = millis() - t4;
			}
		} else if ( c == 3 ) {
			setUpperArm(MAX_UPPER);
			c++;
		} else if ( dt >= t4 && c == 4 ) {
			setLowerArm(650);
			c++;
		} else if ( dt >= t5 && c == 5 ) {
			flag = true;
		}
	}

	motor.stop_all(); // ensure motors are not being powered

	if (!unsuccessful) {
		placePetCatapult(pivotPosition);
		delay(500);
		pivotToLine(RIGHT, 2200);
		// move arm out of catapult's way
		RCServo0.write(70);
		c = 0;
		flag = false;
		t1 = 500; t2 = t1 + 1000;
		timeStart = millis();
		while (!flag) {
			lowerArmPID();
			uint16_t dt = millis() - timeStart;
			if (dt >= t1 && c == 0) {
				// this lower arm height is also the height second pet is picked up from
				setLowerArm(555);
				c++;
			} else if (dt >= t2 && c == 1) {
				flag = true;
			}
		}
		pauseArms(); // ensure arms are not powered

		delay(500);
		launch(85);
		pivotToLine(RIGHT, 1000);
		delay(300);

	} else {
		RCServo0.write(70);
		// c = 0;
		// flag = false;
		// timeStart = millis();
		// while (!flag) {
		// 	lowerArmPID();
		// 	unsigned int dt = millis() - timeStart;
		// 	if (dt >= 500 && c == 0) {
		// 		// this lower arm height is also the height second pet is picked up from
		// 		setLowerArm(570);
		// 		c++;
		// 	} else if ( dt >= 1500 && c == 1) {
		// 		setUpperArm(460);
		// 		c++;
		// 	} else if (dt >= 2700 && c == 2) {
		// 		flag = true;
		// 	}
		// }
	}

	setArmSecondPet();

	// if (analogRead(QRD_L) >= q_threshold && analogRead(QRD_R) < q_threshold)
	// 	pivotOnLine(LEFT, 0, 0);
	// else if (analogRead(QRD_R) >= q_threshold && analogRead(QRD_L) < q_threshold)
	// 	pivotOnLine(RIGHT, 0, 0);
	// else if (analogRead(QRD_L) < q_threshold && analogRead(QRD_R) < q_threshold)
	// 	pivotOnLine(LEFT, 0, 0);
}

// Setting arm for second pet pickup
void setArmSecondPet() {
	int c = 0;
	int pivotPosition = 35;
	int t1 = 500, t2 = t1 + 1000, t3 = t2 + 1200;
	RCServo0.write(pivotPosition);
	uint32_t timeStart = millis();
	while (true) {
		lowerArmPID();
		upperArmPID();
		unsigned int dt = millis() - timeStart;
		if (dt >= t1 && c == 0) {
			setLowerArm(517);
			c++;
		} else if ( dt >= t2 && c == 1) {
			setUpperArm(408);
			c++;
		} else if (dt >= t3 && c == 2) {
			return;
		}
	}
}

// This function is not being used
void getSecondPet() {
	boolean flag = false;
	boolean unsuccessful = false;
	uint32_t timeStart = millis();
	int pivotPosition = 39;
	int c = 0;
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
			setUpperArm(490);
			c++;
		} else if ( dt >= 2500 && c == 1 ) {
			setUpperArm(705);
			c++;
		} else if ( dt >= 4500 && c == 3) {
			if (petOnArm()) {
				c++;
			} else if (try_num < 2) {
				try_num++;
				adjustArm(pivotPosition, try_num, 5);
				c = 0;
				timeStart = millis() - 500;
			} else if (try_num >= 2 && !petOnArm()) {
				c = 3;
				timeStart = millis() - 4500;
				// second pet not picked up flag
				unsuccessful = true;
				secPet = false;
			}
		} else if ( c == 4 ) {
			setLowerArm(590);
			c++;
		} else if ( dt >= 6000 && c == 5 ) {
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
	int pivotTo = 118;
	int c = 1;
	int t1 = 1200, t2 = t1 + 1000, t3 = t2 + 800, t4 = t3 + 2000;
	int t5 = 1200, t6 = t5 + 1000;
	// TODO: test if pivoting is actually needed
	turnForward(-2, 110);
	motor.speed(LEFT_MOTOR, 40);
	motor.speed(RIGHT_MOTOR, 40);
	setLowerArm(610);

	uint32_t timeStart = millis();
	while (true) {
		upperArmPID();
		lowerArmPID();

		uint16_t dt = millis() - timeStart;

		if ( dt >= t1 && c == 1 ) {
			setUpperArm(MAX_UPPER);
			c++;
		} else if (dt >= t2 && c == 2) {
			//pivotArm(30, pivotTo, 10);
			RCServo0.write(pivotTo);
			c++;
		} else if ( dt >= t3 && c == 3) {
			setLowerArm(350);
			setUpperArm(510);
			c++;
		} else if ( dt >= t4 && c == 4) {
			pauseArms();
			dropPet();
			timeStart = millis();
			c++;
		} else if ( c == 5) {
			setLowerArm(600);
			c++;
		} else if ( dt >= t5 && c == 6) {
			setUpperArm(MAX_UPPER);
			c++;
		} else if ( dt >= t6 && c == 7) {
			return;
		}
	}
	pauseArms();
}

void getThirdPet() {
	boolean flag = false, unsuccessful = false, found = false;
	int pivotPosition = 40;
	int pivotIncrement = 20;
	int c = 0;
	int try_num = 0;
	int t1 = 500, t2 = t1 + 1000, t3 = t2 + 1000, t4 = t3 + 800, t5 = t4 + 1500, t6 = t5 + 1000;
	RCServo0.write(pivotPosition);
	uint32_t timeStart = millis();

	while (!flag) {

		upperArmPID();
		lowerArmPID();
		uint16_t dt = millis() - timeStart;

		if ( dt >= t1 && c == 0 ) {
			setLowerArm(600);
			c++;
		} else if ( dt >= t2 && c == 1 ) {
			setUpperArm(415);
			c++;
		} else if ( dt >= t3 && c == 2 ) {
			if (petOnArm()) {
				found = true;
				try_num = 0;
				pivotIncrement = 10;
			}
			c++;
		} else if (dt >= t3 && c == 3) {
			setUpperArm(500);
			c++;
		} else if ( dt >= t4 && c == 4) {
			if (petOnArm()) {
				c++;
			} else if (try_num < 2) {
				adjustArm(pivotPosition, try_num, pivotIncrement);
				try_num++;
				c = (found == true) ? 3 : 2;
				setUpperArm(415);
				timeStart = millis() - t2;
			} else if (try_num >= 2 && !petOnArm()) {
				c = 5;
				unsuccessful = true;
				timeStart = millis() - t4;
			}
		} else if ( c == 5) {
			if (!unsuccessful)
				setUpperArm(MAX_UPPER);
			else if (unsuccessful)
				setUpperArm(600);
			c++;
		} else if ( dt >= t5 && c == 6 ) {
			setLowerArm(MAX_LOWER); // See "REDUN" can set lower arm position here?
			c++;
		} else if ( dt >= t6 && c == 7 ) {
			flag = true;
		}
	}

	if (!unsuccessful) {
		// pivot arm to correct location
		RCServo0.write(110);

		flag = false;
		c = 0;
		t1 = 500; t2 = t1 + 1000; t3 = t2 + 1000; t4 = 2000;
		timeStart = millis();
		//move upper/lower arm to correct position for drop;
		while (!flag) {
			upperArmPID();
			lowerArmPID();

			uint16_t dt = millis() - timeStart;
			// TODO: just copy place second pet drop loop once that is working.
			if ( dt >= t1 && c == 0 ) {
				setLowerArm(370);
				c++;
			} else if ( dt >= t2 && c == 1) {
				setUpperArm(520);
				c++;
			} else if ( dt >= t3 && c == 2) {
				pauseArms();
				dropPet();
				c++;
				timeStart = millis();
			} else if (c == 3) {
				RCServo0.write(80);
				setUpperArm(500);
				setLowerArm(MAX_LOWER);
				c++;
			} else if ( dt >= t4 && c == 4 ) {
				flag = true;
			}
		}
	} else {
		flag = false;
		c = 0;
		t1 = 500; t2 = t1 + 1500;
		timeStart = millis();
		// move arm out of way
		while (!flag) {
			upperArmPID();
			lowerArmPID();

			uint16_t dt = millis() - timeStart;
			// TODO: just copy place second pet drop loop once that is working.
			if ( dt >= t1 && c == 0 ) {
				RCServo0.write(80);
				setUpperArm(500);
				setLowerArm(MAX_LOWER);
				c++;
			} else if ( dt >= t2 && c == 1 ) {
				flag = true;
			}
		}
	}
	pauseArms();
}

void getFourthPet() {
	boolean flag = false, unsuccessful = false, found = false;
	LCD.clear(); LCD.home();
	LCD.print("L:"); LCD.print(encount_L);
	LCD.print(" R:"); LCD.print(encount_R);
	int pivotPosition = 85, pivotIncrement = 15; //TODO: tune -pet will be on left side of robot
	int c = 0;
	int try_num = 0;
	int t1 = 500, t2 = t1 + 1000, t3 = t2 + 1000, t4 = t3 + 800, t5 = t4 + 1000, t6 = t5 + 1000;
	RCServo0.write(pivotPosition);

	uint32_t timeStart = millis();
	while (!flag) {
		upperArmPID();
		lowerArmPID();

		uint16_t dt = millis() - timeStart;

		if ( dt >= t1 && c == 0 ) {
			setLowerArm(505);
			c++;
		} else if ( dt >= t2 && c == 1 ) {
			setUpperArm(400);
			c++;
		} else if ( dt >= t3 && c == 2 ) {
			if (petOnArm()) {
				found = true;
				try_num = 0;
				pivotIncrement = 8;
			}
			c++;
		} else if ( dt >= t3 && c == 3) {
			setUpperArm(500);
			c++;
		} else if ( dt >= t4 && c == 4) {
			if (petOnArm()) {
				c++;
			} else if (try_num < 2) {
				adjustArm(pivotPosition, try_num, pivotIncrement);
				try_num++;
				c = (found == true) ? 3 : 2;
				setUpperArm(400);
				timeStart = millis() - t2;
			} else if (try_num >= 2 && !petOnArm()) {
				c = 5;
				unsuccessful = true;
				fourthPet = false;
				timeStart = millis() - t4;
			}
		} else if ( c == 5) {
			setUpperArm(MAX_UPPER);
			c++;
		} else if (dt >= t5 && c == 6) {
			pauseArms();
			setLowerArm(MAX_LOWER);
			c++;
		} else if ( dt >= t6 && c == 7) {
			flag = true;

		}
	}

	pauseArms();
	if (!unsuccessful) {
		placePetCatapult(pivotPosition);
		delay(500);
	}
	travel(9, FORWARDS);
	turnForward(5, STABLE_SPEED);
	RCServo0.write(50);

	// get fifth pet arm position
	c = 0;
	flag = false;
	t1 = 500; t2 = t1 + 1200, t3 = t2 + 1000;
	timeStart = millis();
	while (!flag) {
		lowerArmPID();
		upperArmPID();
		uint16_t dt = millis() - timeStart;

		if (dt >= t1 && c == 0) {
			// this lower arm height is also the height the fifth pet will be picked up from
			setLowerArm(529);
			c++;
		} else if (dt >= t2 && c == 1) {
			// this lower arm height is also the height second pet is picked up from
			setUpperArm(579);
			c++;
		} else if ( dt >= t3 && c == 2) {
			flag = true;
		}
	}
	pauseArms(); // ensure arms are not powered
}

void launchFifthPet() {
	int c = 0;
	boolean flag = false;
	uint32_t timeStart = millis();
	while (!flag) {
		lowerArmPID();
		upperArmPID();
		uint16_t dt = millis() - timeStart;
		if (c == 0) {
			setLowerArm(MAX_LOWER);
			setUpperArm(MAX_UPPER);
			c++;
		} else if (dt >= 1500 && c == 1) {
			flag = true;
		}
	}

	//Adjust Lower/Upper Arm for catapult placing
	placePetCatapult(35);
	delay(500);
	// move arm out of catapult's way
	RCServo0.write(70);
	c = 0;
	flag = false;
	timeStart = millis();
	while (!flag) {
		lowerArmPID();
		unsigned int dt = millis() - timeStart;
		if (dt >= 500 && c == 0) {
			setLowerArm(550);
			c++;
		} else if (dt >= 1500 && c == 1) {
			flag = true;
		}
	}
	pauseArms(); // ensure arms are not powered

	delay(500);
	launch(150);
	delay(300);
}

void getSixthPet() {

	boolean flag = false;
	int pivotPosition = 35;
	int pivotIncrement = 10;
	int c = 0;
	int try_num = 0;
	int t1 = 1000, t2 = t1 + 1000, t3 = t2 + 800, t4 = t3 + 800, t5 = t4 + 1000, t6 = t5 + 1000;

	RCServo0.write(pivotPosition);
	delay(200);
	setLowerArm(550);
	uint32_t timeStart = millis();
	while (!flag) {
		upperArmPID();
		lowerArmPID();

		uint16_t dt = millis() - timeStart;
		if ( dt >= t1 && c == 0 ) {
			setUpperArm(500);
			//Change Lower Arm Position according to try_num?
			c++;
		} else if ( dt >= t2 && c == 1 ) {
			RCServo0.write(pivotPosition - pivotIncrement);
			c++;
		} else if ( dt >= t3 && c == 2 ) {
			RCServo0.write(pivotPosition + pivotIncrement);
			c++;
		} else if ( dt >= t4 && c == 3) {
			if (try_num < 2) {
				c = 1;
				timeStart = millis() - 2000;
			} else {
				c++;
			}
		} else if ( dt >= t4 && c == 4 ) { //Checks switch after because foam will trigger switch
			setUpperArm(MAX_UPPER);
		} else if (dt >= t5 && c == 5) {
			setLowerArm(MAX_LOWER);
			c++;
		} else if (dt >= t6 && c == 6) {
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
		pivot(-12); // Pivot to the Left
		delay(500);
		//launch(150); //TODO: tune Launch value
	}
}

// Place pet in catapult from pivot arm's position 'pivotFrom'
void placePetCatapult(int pivotFrom) {
	int c = 1;
	pauseArms();
	pivotArm(pivotFrom, 163, 6);
	delay(250);
	uint32_t timeStart = millis();
	while (true) {
		lowerArmPID();
		upperArmPID();

		uint32_t dt = millis() - timeStart;
		if ( dt >= 0 && c == 1 ) {
			setLowerArm(538);
			LCD.clear(); LCD.home();
			LCD.print("placing pet");
			digitalWrite(HAND_UP, HIGH);
			c++;
		} else if (dt >= HAND_DURATION && c == 2) {
			setLowerArm(640);
			setUpperArm(MAX_UPPER);
			digitalWrite(HAND_UP, LOW);
			c++;
		} else if ( c == 3) {
			c++;
			digitalWrite(HAND_DOWN, HIGH);
			timeStart = millis();
		} else if (dt >= HAND_DURATION && c == 4 ) {
			digitalWrite(HAND_DOWN, LOW);
			pauseArms(); // ensure arms stop moving
			return;
		}
	}
}

void qrdRead() {
	while (!stopbutton()) {
		LCD.clear(); LCD.home();
		LCD.print("LQ:"); LCD.print(analogRead(QRD_L)); LCD.print(" RQ:"); LCD.print(analogRead(QRD_R));
		LCD.setCursor(0, 1); LCD.print("HQ:"); LCD.print(analogRead(QRD_LINE));
		delay(200);

	}
	LCD.clear(); LCD.home();
	LCD.print("Resturning...");
	delay(500);
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
			a = map(knob(7), 0, 1023, 350, 640); // lower arm
		} else if ( selection == 2) {
			a = map(knob(7), 0 , 1023, 300, 705); // higher arm
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

void strategySelection()
{
	delay(500);
	while (!stopbutton())
	{
		LCD.clear(); LCD.home();
		int selection = map(knob(6), 0 , 1023, 0, 2);

		LCD.print("Strategy: ");
		LCD.print(fullRun ? "Full" : "Top");

		LCD.setCursor(0, 1);

		if (selection == 0) {
			LCD.print("Full?");
		} else if (selection == 1) {
			LCD.print("Top?");
		}

		if (startbutton()) {
			delay(100);

			if (selection == 0) {
				fullRun = true;
				eeprom_write_word(FULLRUN_EEPROM, true);
			} else if (selection == 1) {
				fullRun = false;
				eeprom_write_word(FULLRUN_EEPROM, false);
			}
		}
		delay(100);
	}
	LCD.clear(); LCD.home(); LCD.print("Returning...");
	delay(600);
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
		} else if (menuIndex == 8) {
			LCD.print("Hand Control");
			LCD.setCursor(0, 1);
			val = map(knob(7), 0 , 1023, 7, 9);
			LCD.print(val == 7 ? "LOWER?" : "RAISE?");
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
