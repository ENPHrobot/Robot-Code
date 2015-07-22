#ifndef Robot_Code_h
#define Robot_Code_h

/* Sensor Ports */
// Analog Ports
enum {
	IR_L = 0,
	IR_R = 1,
	UPPER_POT = 2,
	LOWER_POT = 3,
	QRD_L = 4,
	QRD_R = 5,
	QRD_LINE = 6
};

// Digital Read Ports
enum {
	ENC_L = INT1,
	ENC_R = INT2,
	HAND_SWITCH = 6,
	FRONT_SWITCH = 7
};

// Digital Write Ports
enum {
	LAUNCH_F = 8
};

// Motor Ports
enum {
	LEFT_MOTOR = 0,
	RIGHT_MOTOR = 2,
	UPPER_ARM = 3,
	LOWER_ARM = 1
};

// Constants
enum {
	STABLE_SPEED = 70,
	FORWARDS = 3,
	BACKWARDS = 4,
	LEFT = 5,
	RIGHT = 6,
	ENC_RAFTER = 60
};

/* Functions */

void tapePID();
void irPID();
void setArmVert(int V);
void armVertControl();
void pauseDrive();
void launch(int ms);
boolean checkPet();
void pivot(int counts);
void speedControl();
void switchMode();

/* Instantiate variables */
volatile int encount_L = 0;
volatile int encount_R = 0;
volatile int s_L = 0;
volatile int s_R = 0;
volatile int time_L;
volatile int time_R;
int count = 0;
uint32_t lastPivotTime;
uint32_t lastTravelTime;
int modeIndex = 0;
int val;
String modes[] = {"qrd", "ir"};
String mode = modes[modeIndex];

void empty() {};
int upperArmV = 740;
int lowerArmV = 550;
int petCount = 0;
boolean onTape = false;
int lastSpeedUp;
void (*pidfn)();
void (*processfn)() = empty;

int last_error = 0;
int recent_error = 0;
int D_error;
int I_error = 0;
int32_t P_error;
int32_t net_error;
int t = 1;
int to = 0;

// QRD vars
int q_pro_gain;
int q_diff_gain;
int q_int_gain;
int q_threshold;
int h_threshold;
int base_speed;

// IR vars
int ir_pro_gain;
int ir_diff_gain;
int ir_int_gain;

#endif