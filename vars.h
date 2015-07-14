/* Sensor Ports */
// Analog Ports
#define IR_L 0
#define IR_R 1
#define ARM_POT 2
#define QRD_L 3
#define QRD_R 5

// Digital Ports
#define ENC_L 0
#define ENC_R 1

// Motor Ports
#define LEFT_MOTOR 0
#define RIGHT_MOTOR 1
#define CATAPULT 2
#define ARM_MOTOR 3

// Constants
#define STABLE_SPEED 75
#define FORWARDS 3
#define BACKWARDS 4
#define LEFT 5
#define RIGHT 6

/* Instantiate variables */
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
int modeIndex = 0;
String modes[] = {"qrd", "ir"};
String mode = modes[modeIndex];
void (*pidfn)(); // default PID loop is QRD tape following
int armControlV = 0; //TODO this initial value should be tuned after potentiometer is mounted onto arm.

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
int ir_pro_gain;
int ir_diff_gain;
int ir_int_gain;
int ir_threshold;