#ifndef Robot_Code_h
#define Robot_Code_h

/* Sensor Ports */
// Analog Ports
enum {
	IR_L = 0,
	IR_R = 1,
	ARM_POT = 2,
	QRD_L = 3,
	QRD_R = 4,
	QRD_LINE = 5
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
	LAUNCH_F = 8,
	LAUNCH_B = 10
};

// Motor Ports
enum {
	LEFT_MOTOR = 0,
	RIGHT_MOTOR = 2,
	ARM_MOTOR = 3
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
void (*pidfn)();
void empty() {};
void (*processfn)() = empty;
int armControlV = 550; //TODO this initial value should be tuned after potentiometer is mounted onto arm.
int petCount = 0;
boolean onTape = false;

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
int base_speed;

// IR vars
int ir_pro_gain;
int ir_diff_gain;
int ir_int_gain;

boolean forSpeedControl = false;

#endif