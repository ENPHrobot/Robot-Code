#ifndef Robot_Code_h
#define Robot_Code_h

/* Sensor Ports */
// Analog Ports
enum {
	IR_R = 0,
	IR_L = 1,
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
	FRONT_SWITCH = 6,
	HAND_SWITCH = 7
};

// Digital Write Ports
enum {
	LAUNCH_F = 8
};

// Motor Ports
enum {
	LEFT_MOTOR = 0,
	RIGHT_MOTOR = 1,
	UPPER_ARM = 3,
	LOWER_ARM = 2
};

// Constants
enum {
	STABLE_SPEED = 60,
	FORWARDS = 3,
	BACKWARDS = 4,
	LEFT = 5,
	RIGHT = 6,
	ENC_RAFTER = 10
};

#define SERVO_0_Pin (35)
#define SERVO_1_Pin (31)
#define SERVO_2_Pin (34)

/* Functions */

void tapePID();
void irPID();
void pauseDrive();
void pauseArms();
void launch(int ms);
void pivot(int counts);
void pivotToLine(int d);
void speedControl();
void switchMode();
void buriedProcess();
void rafterProcess();
void timedTravel( uint32_t t, int d);
void travel(int counts, int d);
void timedPivot(uint32_t t, int d);
void turnBack(int counts);
void turnForward(int counts);
void lowerArmPID();
void upperArmPID();
void setLowerArm(int V);
void setUpperArm(int V);
void RES();
void LES();
void RE();
void LE();
void armCal();
void getFirstPet();
void getSecondPet();
void getThirdPet();
void getFourthPet();
void getFifthPet();
void getSixthPet();
void placeSecondPet();
void placePetCatapult();
void pivotArm(int from, int to);
void adjustArm(int pos, int);
void dropPet();
boolean checkPet();
boolean petOnArm();
boolean checkBoxedPet();
boolean checkRafterPet();

/* Instantiate variables */
int encount_L = 0;
int encount_R = 0;
volatile int s_L = 0;
volatile int s_R = 0;
volatile int time_L;
volatile int time_R;
int count = 0;
int modeIndex = 0;
int val;
String modes[] = {"qrd", "ir"};
String mode = modes[modeIndex];

void empty() {};
int upperArmV = 720;
int lowerArmV = 560;
int petCount = 0;
boolean onTape = false;
boolean ecL = false;
boolean ecR = false;
boolean secPet = true;
uint32_t lastSpeedUp;
uint32_t lastTravelTime;
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

Servo RCServo0;
Servo RCServo2;

#endif