#include <Servo.h>
#include <Encoder.h>

// DEFINE ***************************************************
// CONFIGURATION

///
#define SPEEDY
#ifdef SPEEDY
  #define MAX_SPEED 255
  #define TICK_TIME_STATE_MACHINE 6  //(ms)
  int TICK_TIME_IR_POLLING[] = {5000, 4000, 5000, 4000, 3500, 3000}; //(mS)
  int TICK_TIME_IR_POLLING_SIZE = 6;
  int sequence=0;
  int distance_L_max = 110;//80;     // (mm)
  int distance_FL_max = 160;    // (mm)
  int distance_R_max = 110;//80;     // (mm)
  int distance_FR_max = 160;    // (mm)
  int distance_F_max = 160;//80;     // (mm)
  const int timeout_FL = 2000;     // (ms)
  
  const int timeout_L  = 1500;     // (ms)
  const int timeout_FR = 2000;     // (ms)
  const int timeout_R  = 1500;     // (ms)
  const int timeout_F =  2000;     // (ms)
  #define SPEED_BUF_SIZE 3
  #define SPEED_NORMAL  190
  #define SPEED_HIGH    255
#else
  #define MAX_SPEED 100
  #define TICK_TIME_STATE_MACHINE 10  //(ms)
  int TICK_TIME_IR_POLLING[] = {5000, 4000, 5000, 4000, 3500, 3000}; //(mS)
  int TICK_TIME_IR_POLLING_SIZE = 6;
  int sequence=0;
  int distance_L_max = 110;//80;     // (mm)
  int distance_FL_max = 150;    // (mm)
  int distance_R_max = 110;//80;     // (mm)
  int distance_FR_max = 150;    // (mm)
  const int timeout_FL = 2000;     // (ms)
  const int timeout_L  = 1500;     // (ms)
  const int timeout_FR = 2000;     // (ms)
  const int timeout_R  = 1500;     // (ms)
  #define SPEED_BUF_SIZE 3
  #define SPEED_NORMAL  80
  #define SPEED_HIGH    100
#endif

#define TICK_TIME_IR_SERVO 100      //(ms)
///#define TICK_TIME_IR_POLLING 10000


#define TIMEOUT_RACE    130000  //3min (180000) 2,5min(150000ms) Manga 1/2 //4min(240000ms) Manga 3/4   

#define   azimute_LEFT  0
#define   azimute_RIGHT 1

#define   turn_LEFT 0
#define   turn_RIGHT 1

// Cinematic
// Wheel diameter = 76mm; Perimeter = 239
// Robot Curving diameter = 165mm; Perimeter = 518mm
#define ROBOT_CURVING_PERIMETER 518 //534
#define WHEEL_PERIMETER 210 // (mm) 239
#define WHEEL_IMP_ROT 1430  //(mm)
#define WHEEL_DIST 167      //(mm) 170
#define K  1.71032178 //2.11765 //(360/WHEEL_PERIMETER)

#define DEBUG1 true
#define DEBUG2 true
#define DEBUG3 false
#define DEBUG5 false
#if DEBUG1
#define PRINTS(s)   { Serial.print(F(s)); }
#define PRINT(s,v)  { Serial.print(F(s)); Serial.print(v); }
#define PRINTSLN(s)   { Serial.println(F(s)); }
#define PRINTLN(s,v)  { Serial.print(F(s)); Serial.println(v); }
#define PRINTX(s,v) { Serial.print(F(s)); Serial.print(v, HEX); }
#else
#define PRINTS(s)
#define PRINT(s,v)
#define PRINTSLN(s)
#define PRINTLN(s,v)
#define PRINTX(s,v)
#endif
#if DEBUG2
#define PRINTS2(s)   { Serial.print(F(s)); }
#define PRINT2(s,v)  { Serial.print(F(s)); Serial.print(v, 4); }
#define PRINTSLN2(s)   { Serial.println(F(s)); }
#define PRINTLN2(s,v)  { Serial.print(F(s)); Serial.println(v, 4); }
#define PRINTX2(s,v) { Serial.print(F(s)); Serial.print(v, HEX); }
#else
#define PRINTS2(s)
#define PRINT2(s,v)
#define PRINTSLN2(s)
#define PRINTLN2(s,v)
#define PRINTX2(s,v)
#endif
#if DEBUG3
#define PRINTS3(s)   { Serial.print(F(s)); }
#define PRINT3(s,v)  { Serial.print(F(s)); Serial.print(v); }
#define PRINTSLN3(s)   { Serial.println(F(s)); }
#define PRINTLN3(s,v)  { Serial.print(F(s)); Serial.println(v); }
#define PRINTX3(s,v) { Serial.print(F(s)); Serial.print(v, HEX); }
#else
#define PRINTS3(s)
#define PRINT3(s,v)
#define PRINTSLN3(s)
#define PRINTLN3(s,v)
#define PRINTX3(s,v)
#endif
#if DEBUG4
#define PRINTS4(s)   { Serial.print(F(s)); }
#define PRINT4(s,v)  { Serial.print(F(s)); Serial.print(v); }
#define PRINTSLN4(s)   { Serial.println(F(s)); }
#define PRINTLN4(s,v)  { Serial.print(F(s)); Serial.println(v); }
#define PRINTX4(s,v) { Serial.print(F(s)); Serial.print(v, HEX); }
#else
#define PRINTS4(s)
#define PRINT4(s,v)
#define PRINTSLN4(s)
#define PRINTLN4(s,v)
#define PRINTX4(s,v)
#endif
#if DEBUG5  //MOTORS
#define PRINTS5(s)   { Serial.print(F(s)); }
#define PRINT5(s,v)  { Serial.print(F(s)); Serial.print(v); }
#define PRINTSLN5(s)   { Serial.println(F(s)); }
#define PRINTLN5(s,v)  { Serial.print(F(s)); Serial.println(v); }
#define PRINTX5(s,v) { Serial.print(F(s)); Serial.print(v, HEX); }
#else
#define PRINTS5(s)
#define PRINT5(s,v)
#define PRINTSLN5(s)
#define PRINTLN5(s,v)
#define PRINTX5(s,v)
#endif
#define Debug_PIN3    3

// Motor Driver
#define motorB_1A     8   // Pin  7 of L293
#define motorB_2A     9   // Pin  2 of L293
#define motorA_1A     6   // Pin  7 of L293
#define motorA_2A     7   // Pin  2 of L293
#define MOTOR_BREAK 2
#define ML 0    //Motor Left
#define MR 1    //MOtor Right

// Encoders
///#define ENCODER_PULSE_PER_REV   //Depend on wheel diameter
/***** PIN 20 INT IS NOT WORKING!!! *****/
#define ENCODER_LEFT_CHA_PIN  21//18//21
#define ENCODER_LEFT_CHB_PIN  3//19//20
#define ENCODER_RIGHT_CHA_PIN 18//20//19
#define ENCODER_RIGHT_CHB_PIN 19//21//18

// Ultrasound pins
#define NUM_SENSORS 4
/*#define trigPin_FL  45
  #define echoPin_FL  44
  #define trigPin_L   43
  #define echoPin_L   42
  #define trigPin_FR  A15
  #define echoPin_FR  A14
  #define trigPin_R   A13
  #define echoPin_R   A12
*/
#define trigPin_FR  A15//45   Swaped FL<->FR Crossed sensor
#define echoPin_FR  A14//44
#define trigPin_L   A13//43
#define echoPin_L   A12//42
#define trigPin_FL  43//A15
#define echoPin_FL  42//A14
#define trigPin_R   45//A13
#define echoPin_R   44//A12
#define trigPin_F   49
#define echoPin_F   48

// IR floor sensor
#define IR_SENSOR_NUM_SENSORS 3
const unsigned char IR_FLOOR_PIN[IR_SENSOR_NUM_SENSORS] = {A0, A1, A2};
const int IR_FLOOR_TH[IR_SENSOR_NUM_SENSORS]  = {400, 400, 400};  // Treshold. Higher value is dark

#define IR_FLOOR_PIN_S0 A0
#define IR_FLOOR_PIN_S1 A1
#define IR_FLOOR_PIN_S2 A2
#define IR_SENSOR_NUM_READINGS 10
#define IR_FLOOR_S0_TH  400   //Higher values -> Higher reflection
#define IR_FLOOR_S1_TH  400   //Higher values -> Higher reflection
#define IR_FLOOR_S2_TH  400   //Higher values -> Higher reflection

// BUTTONS START/STATE_STOP
#define BUTTON_START_PIN   4
#define BUTTON_STOP_PIN    2

// LED
#define LED_PIN 50

// END DEFINE ***************************************************

// GLOBAL VAR
const double pi = 3.14159;
const double pix2 = 6.28318;
int i_tmp = 0;
int i_tmp1 = 0;
int i_tmp2 = 0;

int turning = turn_LEFT;


int button_start = 0;

//Collin position
double robot_xPosition = 0;
double robot_yPosition = 0;
double robot_theta = 0;

// IR Swiper
#define servo_pin     12
#define receiver_pin  46
Servo ir_servo;
int servo_old_pos = 89;
int servo_current_pos = 90;
int servo_ang;
int IR_read;

const int servo_millis_tick = 200;
unsigned long servo_last_millis;

//Motors
int speed_bufL[SPEED_BUF_SIZE];
int speed_bufR[SPEED_BUF_SIZE];
int speed_L;
int speed_R;
// Motor encoders
int n_pulse_rotate;
long myEnc_L_oldpos = 0;
long myEnc_R_oldpos = 0;
long myEnc_L_currpos;
long myEnc_R_currpos;

long mainEnc_L_oldpos = 0;
long mainEnc_R_oldpos = 0;
long mainEnc_L_currpos;
long mainEnc_R_currpos;

// Ultra Sound VAR
int distance_L;
int distance_FL;
int distance_R;
int distance_FR;
int distance_F;

// Encoders
Encoder myEnc_R(ENCODER_RIGHT_CHA_PIN, ENCODER_RIGHT_CHB_PIN);  //   avoid using pins with LEDs attached
Encoder myEnc_L(ENCODER_LEFT_CHA_PIN,  ENCODER_LEFT_CHB_PIN);  //   avoid using pins with LEDs attached

unsigned long prevMillis_tick_mouse_state_machine = 0;
unsigned long prevMillis_tick_IR_servo = 0;
unsigned long prevMillis_tick_IR_poling = TICK_TIME_IR_POLLING[sequence]; //
///unsigned long prevMillis_tick_IR_poling = TICK_TIME_IR_POLLING; //

// GLOBAL VAR END ***************************************************



int pwmOut(int speed_in, int motor, int motor_stop = 1);

enum State_enum { STATE_STOP, STATE_FORWARD, STATE_ROTATE_RIGHT, STATE_ROTATE_LEFT, STATE_ABORT, STATE_ROTATE_TETA_RIGHT,
                  STATE_ROTATE_TETA_LEFT, STATE_IR_TOWER_DETECTED, STATE_TOWER_ARRIVE, STATE_TIMEOUT_RACE, STATE_BEGIN_RETURN,
                  STATE_ROTATE_IR_SCAN, STATE_ROTATE_180
                };
enum State_journey_enum { STATEJ_FIND_IR_TOWER, STATEJ_FIND_BEGINING, STATEJ_TIMEOUT_RACE
                        };


enum Sensors_enum {NONE, SENSOR_RIGHT, SENSOR_LEFT, BOTH};

int mouse_state_table[] = {NONE, SENSOR_RIGHT, SENSOR_RIGHT, SENSOR_RIGHT, SENSOR_RIGHT,
                           SENSOR_RIGHT, BOTH, SENSOR_RIGHT, SENSOR_LEFT, BOTH, BOTH, BOTH, SENSOR_LEFT, BOTH, BOTH, BOTH,
                           SENSOR_LEFT, BOTH, SENSOR_RIGHT, SENSOR_LEFT, SENSOR_RIGHT, BOTH, SENSOR_RIGHT, SENSOR_LEFT, SENSOR_LEFT, SENSOR_LEFT 
                          };
// L  LF F RF R
// 0  0  0  0  NONE
// 0  0  0  1  SENSOR_RIGHT
// 0  0  1  0  PREF RIGHT
// 0  0  1  1  RIGHT
// 0  1  0  0  PREF LEFT
// 0  1  0  1  PREF RIGHT
//   0  1  1  0  FRONT
// 0 0  1  1  1  PREF RIGHT
// 0 1  0  0  0
// 0 1  0  0  1
// 0 1  0  1  0 ?
// 0 1  0  1  1
// 0 1  1  0  0
// 0 1  1  0  1
// 0  1  1  1  0
// 0 1  1  1  1
// 1 0  0  0  0  NONE
// 0  0  0  1  SENSOR_RIGHT
// 0  0  1  0  PREF RIGHT
// 0  0  1  1  RIGHT
// 0  1  0  0  PREF LEFT
// 0  1  0  1  PREF RIGHT
// 0  1  1  0  FRONT
// 0  1  1  1  PREF RIGHT
// 1  0  0  0
// 1  0  0  1
// 1  0  1  0 ?
// 1  0  1  1
// 1  1  0  0
// 1  1  0  1
// 1  1  1  0
// 1  1  1  1

int mouse_state_table_entry;

int cnt_motors_right = 0;
int cnt_motors_left = 0;

void mouse_state_machine_run(uint8_t sensors);
void motors_stop();
void motors_forward();
void motors_right();
void motors_left();
uint8_t SonarScan();

uint8_t mouse_state = STATE_ABORT;
uint8_t last_mouse_state = mouse_state;

uint8_t journey_state = STATEJ_FIND_IR_TOWER;
uint8_t last_journey_state = journey_state;

// Timing
unsigned long race_time;

// GLOBAL VAR END
