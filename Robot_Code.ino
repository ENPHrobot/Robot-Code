#include <avr/EEPROM.h>
#include <Interrupts.h>
#include <phys253.h>
#include <LiquidCrystal.h>

// Sensor Ports
#define IR_L 0
#define IR_R 1
#define QRD_L 2
#define QRD_R 5
#define LEFT_MOTOR 0
#define RIGHT_MOTOR 1

// Constants
#define STABLE_SPEED 75
#define FORWARDS 3
#define BACKWARDS 4
#define LEFT 5
#define RIGHT 6

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
		switch (index) {
		case 0:
			// TODO: can add something here for sensors later like a syscheck.
			break;
		case 1:
			QRDMENU();
			break;
		case 2:
			IRMENU();
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
MenuItem menuItems[]      = {Speed, ProportionalGain, DerivativeGain, IntegralGain, ThresholdVoltage};

uint16_t IRMenuItem::MenuItemCount = 0;
IRMenuItem IRProportionalGain = IRMenuItem("P-gain");
IRMenuItem IRDerivativeGain   = IRMenuItem("D-gain");
IRMenuItem IRIntegralGain     = IRMenuItem("I-gain");
IRMenuItem IRThreshold		  = IRMenuItem("Threshold");
IRMenuItem IRmenuItems[]      = {IRProportionalGain, IRDerivativeGain, IRIntegralGain, IRThreshold};

uint16_t MainMenuItem::MenuItemCount = 0;
MainMenuItem Sensors      = MainMenuItem("Sensors");
MainMenuItem TapePID      = MainMenuItem("Tape PID");
MainMenuItem IRPID        = MainMenuItem("IR PID");
MainMenuItem mainMenu[]   = {Sensors, TapePID, IRPID};

/* Instantiate variables */
int value = 0;
int count = 0;
int average;
int difference;
int left_sensor;
int right_sensor;
volatile int encount_L = 0;
volatile int encount_R = 0;
int pivotCount = 0;
int pivotEncountStart_L;
int pivotEncountStart_R;
uint32_t lastPivotTime;
int travelCount = 0;
int travelEncountStart_L;
int travelEncountStart_R;
uint32_t lastTravelTime;

int error;
int last_error = 0;
int recent_error = 0;
int D_error;
int I_error = 0;
int32_t P_error;
int32_t net_error;
int t = 1;
int to;

// QRD vars
int q_pro_gain;
int q_diff_gain;
int q_int_gain;
int q_threshold;
int base_speed;

// IR vars

void setup()
{
#include <phys253setup.txt>
	//Serial.begin(9600);
	LCD.clear(); LCD.home();

	base_speed = menuItems[0].Value;
	q_pro_gain = menuItems[1].Value;
	q_diff_gain = menuItems[2].Value;
	q_int_gain = menuItems[3].Value;
	q_threshold = menuItems[4].Value;

	LCD.print("RC1"); LCD.setCursor(0, 1);
	LCD.print("Press Start.");
	while (!startbutton()) {};
	LCD.clear();
}

void loop()
{
	if (startbutton() && stopbutton()) {
		// Pause motors
		motor.speed(LEFT_MOTOR, 0);
		motor.speed(RIGHT_MOTOR, 0);
		MainMenu();
		// Set values after exiting menu
		base_speed = menuItems[0].Value;
		q_pro_gain = menuItems[1].Value;
		q_diff_gain = menuItems[2].Value;
		q_int_gain = menuItems[3].Value;
		q_threshold = menuItems[4].Value;
		// Restart motors
		motor.speed(LEFT_MOTOR, base_speed);
		motor.speed(RIGHT_MOTOR, base_speed);
	}

	// PID control
	left_sensor = analogRead(QRD_L);
	right_sensor = analogRead(QRD_R);

	if (left_sensor > q_threshold && right_sensor > q_threshold)
		error = 0; // both sensors on black
	else if (left_sensor > q_threshold && right_sensor < q_threshold)
		error = -1;	// left sensor on black
	else if (left_sensor < q_threshold && right_sensor > q_threshold)
		error = 1; // right sensor on black
	else if (left_sensor < q_threshold && right_sensor < q_threshold)
	{
		// neither sensor on black. check last error to see which side we are on.
		if ( last_error > 0)
			error = 5;
		else if ( last_error < 0)
			error = -5;
	}

	if ( !(error == last_error))
	{
		recent_error = last_error;
		to = t;
		t = 1;
	}

	P_error = q_pro_gain * error;
	D_error = q_diff_gain * ((float)(error - recent_error) / (float)(t + to)); // time is present within the differential gain
	I_error += q_int_gain * error;
	net_error = P_error + D_error + I_error;

	// prevent adjusting errors from going over actual speed.
	if (net_error > base_speed)
		net_error = base_speed;
	if (net_error < -base_speed)
		net_error = -base_speed;

	//if net error is positive, right_motor will be stronger, will turn to the left
	motor.speed(LEFT_MOTOR, base_speed + net_error);
	motor.speed(RIGHT_MOTOR, base_speed - net_error);

	if ( count == 100 ) {
		count = 0;
		LCD.clear(); LCD.home();
		LCD.print("LQ:"); LCD.print(left_sensor);
		LCD.print(" LM:"); LCD.print(base_speed + net_error);
		LCD.setCursor(0, 1);
		LCD.print("RQ:"); LCD.print(right_sensor);
		LCD.print(" RM:"); LCD.print(base_speed - net_error);
	}

	last_error = error;
	count++;
	t++;
}

/* Helper Functions */

// Stop driving
void pauseDrive() {
	motor.speed(LEFT_MOTOR, 0);
	motor.speed(RIGHT_MOTOR, 0);
}

// Pivot the robot for a certain number of encoder
// counts on both. UNTESTED.
void pivot(int counts)
{
	pivotCount = counts;
	pivotEncountStart_L = encount_L;
	pivotEncountStart_R = encount_R;
	lastPivotTime = millis();
	if (counts < 0) {
		motor.speed(RIGHT_MOTOR, STABLE_SPEED);
		motor.speed(LEFT_MOTOR, -STABLE_SPEED);
		attachTimerInterrupt(2000, pivotCheck);
	} else if (counts > 0) {
		motor.speed(RIGHT_MOTOR, -STABLE_SPEED);
		motor.speed(LEFT_MOTOR, STABLE_SPEED);
		attachTimerInterrupt(2000, pivotCheck);
	}
}

// Pivot in a direction d for a time t.
void timedPivot(uint32_t t, int d) {
	if ( d == LEFT) {
		motor.speed(RIGHT_MOTOR, STABLE_SPEED);
		motor.speed(LEFT_MOTOR, -STABLE_SPEED);
	} else if (d == RIGHT) {
		motor.speed(RIGHT_MOTOR, -STABLE_SPEED);
		motor.speed(LEFT_MOTOR, STABLE_SPEED);
	}
	delay(t);
	pauseDrive(); //TODO: change to use a timer interrupt
}

// Travel in a direction d for a number of counts
void travel(int counts, int d) {
	travelCount = counts;
	travelEncountStart_L = encount_L;
	travelEncountStart_R = encount_R;
	lastTravelTime = millis();
	// TODO: one motor may need a power offset to travel straight
	motor.speed(RIGHT_MOTOR, d == FORWARDS ? STABLE_SPEED : -STABLE_SPEED);
	motor.speed(LEFT_MOTOR, d == FORWARDS ? STABLE_SPEED : -STABLE_SPEED);
	attachTimerInterrupt(2000, travelCheck);
}

void timedTravel( uint32_t t, int d) {
	motor.speed(RIGHT_MOTOR, d == FORWARDS ? STABLE_SPEED : -STABLE_SPEED);
	motor.speed(LEFT_MOTOR, d == FORWARDS ? STABLE_SPEED : -STABLE_SPEED);
	delay(t);
	pauseDrive(); //TODO: change to use a timer interrupt
}

/* Timer ISRs */

void pivotCheck()
{
	if (encount_L - pivotEncountStart_L >= pivotCount) {
		motor.speed(LEFT_MOTOR, 0);
	}
	if (encount_R - pivotEncountStart_R >= pivotCount) {
		motor.speed(RIGHT_MOTOR, 0);
	}
	if (encount_L - pivotEncountStart_L >= pivotCount
	        && encount_R - pivotEncountStart_R >= pivotCount) {
		detachTimerInterrupt();
		lastPivotTime -= millis();
	}
}

void travelCheck()
{
	if (encount_L - travelEncountStart_L >= travelCount) {
		motor.speed(LEFT_MOTOR, 0);
	}
	if (encount_R - travelEncountStart_R >= travelCount) {
		motor.speed(RIGHT_MOTOR, 0);
	}
	if (encount_L - travelEncountStart_L >= travelCount
	        && encount_R - travelEncountStart_R >= travelCount) {
		detachTimerInterrupt();
		lastTravelTime -= millis();
	}
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
		LCD.print("Set to "); LCD.print(menuIndex != 0 ? knob(7) : knob(7) >> 2); LCD.print("?");
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
		/* Show MainMenuItem value and knob value */
		int menuIndex = knob(6) * (MainMenuItem::MenuItemCount) >> 10;
		LCD.clear(); LCD.home();
		if (menuIndex == 0) {
			LCD.print ("LQ:"); LCD.print(analogRead(QRD_L)); LCD.print(" RQ:"); LCD.print(analogRead(QRD_R));
		} else {
			LCD.print(mainMenu[menuIndex].Name);
			LCD.setCursor(0, 1);
			LCD.print("Start to Select.");
		}
		delay(100);

		/* Press start button to enter submenu */
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
				return;
			}
		}
	}
}