/* 
  Created by Valentin Betbeze
  03 Nov. 2022
  Description:  Firmware of the 5-Axis Robotic Arm.

 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <assert.h>
#include <Servo.h>

#define INPUT_SIZE      20

/*----------------- MACROS ----------------*/
#define PI_DEG          180
#define PI2_DEG         90
#define PI4_DEG         45
#define PI8_DEG         22.5
#define DEG(x)          (180.0 * x / M_PI)
#define RAD(x)          (M_PI * x / 180.0)

// Motor data pins
#define PIN_M1          7
#define PIN_M2          10
#define PIN_M2s         11
#define PIN_M3          8
#define PIN_M4          9
#define PIN_M5          12

/* Motor pulse width range
 Mx_min refers to the pulse width on the motor x, in microseconds, 
 corresponding to the minimum (0 degree) angle on the servo.
 Similarly, Mx_max refers to the maximum angle (180 degree).
 See https://reference.arduino.cc/reference/en/libraries/servo/attach/
 These values have been determined empirically, and are unique to each
 motor.
 */
#define M1_PW_MIN       410
#define M1_PW_MAX       2370
#define M2_PW_MIN       500
#define M2_PW_MAX       2410
#define M2s_PW_MIN      600
#define M2s_PW_MAX      2450
#define M3_PW_MIN       400
#define M3_PW_MAX       2330
#define M4_PW_MIN       380
#define M4_PW_MAX       2300
#define M5_PW_MIN       544
#define M5_PW_MAX       2500

// Motor maximum angular range in degree
#define RANGE_M1        155
#define RANGE_M2        PI_DEG
#define RANGE_M3        165       // 165 deg is the M3 angle at which the upper arm rests
                                  // on the lower arm in a planar contact configuration
#define RANGE_M4        PI_DEG
#define RANGE_M5        PI_DEG

// Initial motor angles, in degree
#define ANGLE_INIT_M1   0
#define ANGLE_INIT_M2   PI_DEG
#define ANGLE_INIT_M3   RANGE_M3
#define ANGLE_INIT_M4   PI2_DEG
#define ANGLE_INIT_M5   PI_DEG

// Robot geometric parameters
#define ANGLE_INIT_LA   90        // Lower Arm initial angle, in degree
#define ANGLE_INIT_UA   30        // Upper Arm initial angle, in degree
#define INVR_TRANS_LA   1.65      // Lower Arm transmission inverse ratio
#define INVR_TRANS_UA   2         // Upper Arm transmission inverse ratio
#define HEIGHT_SHOULDER 136       // Shoulder's axis height (ref: ground), in mm
#define LENGTH_LA       150       // Lower Arm length, in mm
#define M3M4            25.56     // Interaxial distance between the axis of M3 and M4, in mm
#define LENGTH_UA       186.76    // Upper Arm length, in mm
#define LENGTH_WRIST    35        // Wrist length, in mm

// Robot dynamic parameters
#define ACC             160       // acceleration in deg/s²
#define TF              3000      // full duration of a displacement in ms


/*------------ DATA STRUCTURES ------------*/
typedef struct {
  uint8_t forward;
  int64_t t1;
  int64_t t2;
  uint8_t angle_t0;
  uint8_t angle_t1;
  uint8_t angle_t2;
  uint8_t angle_tf;
  int32_t delta;
}  motor_dynamics_t ;

typedef struct  {
  int16_t x;      // mm
  int16_t y;      // mm
  int16_t z;      // mm
  int16_t yaw;    // deg
  int16_t pitch;  // deg
  // roll not available on current robot version
} cartesian_position_t;


/*----------- GLOBAL VARIABLES ------------*/
Servo M1;
Servo M2;
Servo M2s;
Servo M3;
Servo M4;
Servo M5;

motor_dynamics_t dynamics_M1;
motor_dynamics_t dynamics_M2;
motor_dynamics_t dynamics_M3;
motor_dynamics_t dynamics_M4;
motor_dynamics_t dynamics_M5;

cartesian_position_t cartesian_pos;

uint8_t motor_id = 0;
bool single_motor_mode    = false;
bool cartesian_mode       = false;
bool single_motor_running = false;
bool five_axis_running    = false;


/*---------- FUNCTION PROTOTYPES ----------*/
/* @brief Parse an input for cartesian coordinate control of the effector.
 Parameters:
 Return 1 if errors are encountered, else 0.
 */
static uint8_t parse_cartesian_input(cartesian_position_t *pos, const char *input, const size_t len);

/* Initialise the servo motor angular position. Use only in setup().
 */
static void initialise_robot(void);

/* Provides the motor angles in degree required to reach the cartesian 
 position given as input. The function directly updates the angles for 
 each motor in their respective struct <motor_dynamics_t> (angle_tf).
 */
static void get_angles_from_cartesian(cartesian_position_t *pos);

/* Computation of the required motor_dynamics_t parameters for a 
 given distance with regards to desired effector speed and acceleration.
 @warning: The final angle `stopAngle` must be determined and assigned to
 the motor_dynamics_t variable prior to using the function.
 */
static void compute_dynamics(motor_dynamics_t *dyn, const uint8_t current_angle);

/* Computes the angle at a time t between t0 and tf so that the motor 
accelerates and decelerates while heading to stopAngle.
*/
static uint8_t compute_angle(const motor_dynamics_t *dyn, const int64_t t);

/* Resets the robot to its idle configuration
 */
static uint8_t reset_robot(void);

/* Determines if the delta of the polynomial equation is >= 0,
 meaning the given position is within the work enveloppe.
 */
static uint8_t is_delta_positive(void);


/*-------------- PROGRAM BODY -------------*/
void setup()
{
  Serial.begin(9600);
  initialise_robot();
  Serial.print("\tFor single motor control enter 'motor'\n"
               "\tFor cartesian position control enter 'cartesian'\n"
               "\tTo reset the robot enter 'reset'\n");
}

void loop()
{
  uint32_t t0 = millis();
  // Operate the robot until a user input is detected
  while (!Serial.available()) {
    /* Runs only if delta is positive or equal to zero, practically
     meaning that the acceleration is high enough so that the motor(s) 
     can complete the entire distance within the time TF.
     */  
    if (is_delta_positive()) {
      uint32_t t = millis() - t0;
      if (single_motor_running) {
        switch (motor_id) {
          case 1: 
            M1.write(compute_angle(&dynamics_M1, t));
            break;
          case 2:
            M2.write(compute_angle(&dynamics_M2, t));
            M2s.write(compute_angle(&dynamics_M2, t));
            break;
          case 3: 
            M3.write(compute_angle(&dynamics_M3, t));
            break;
          case 4: 
            M4.write(compute_angle(&dynamics_M4, t));
            break;
          case 5:
            M5.write(compute_angle(&dynamics_M5, t));
            break;
          default:
            break;
        }
      } 
      else if (five_axis_running) {
        M1.write(compute_angle(&dynamics_M1, t));
        M2.write(compute_angle(&dynamics_M2, t));
        M2s.write(compute_angle(&dynamics_M2, t));
        M3.write(compute_angle(&dynamics_M3, t));
        M4.write(compute_angle(&dynamics_M4, t));
        M5.write(compute_angle(&dynamics_M5, t));
      }
    }
  }
  /* An input has been detected, the robot is idle while the input
   is being processed.
   */
  single_motor_running = false;
  five_axis_running = false;
  // Get input
  char c = '\0', input[INPUT_SIZE + 1] = {0}; // Using +1 to account for the null indicator
  uint8_t i = 0;
	while (i < INPUT_SIZE && (c = (char)Serial.read()) != '\n') {
	    if (isalnum(c) || ispunct(c)) {
	      input[i++] = c;
	    }
	}
  input[i] = '\0';
	size_t len = strlen(input);
  Serial.print("> " + String(input) + "\n");

  // Parse input
  if (!strcmp(input, "motor")) {
    single_motor_mode = true;
    cartesian_mode = false;
    Serial.print("Success: 'Single motor mode' accessed. Format: ID.VALUE\n");
  }
  else if (!strcmp(input, "cartesian")) {
    single_motor_mode = false;
    cartesian_mode = true;
    Serial.print("Success: 'Cartesian mode' accessed. Format: X.Y.Z.YAW.PITCH\n");
  }
  else if (!strcmp(input, "reset")) {
    single_motor_mode = false;
    cartesian_mode = false;
    if (reset_robot() == 0) {
      Serial.print("Success: The robot has been reset.\n");
    }
  }
  // If input is not a mode, then it is a value
  else {
    /* The number (ID as in Servo M<ID>) of the motor must be entered as 
      well as the desired position to reach, expressed in percentage of the  
      total allowed displacement of the given motor.

      Exemples: 
      "M1.050" will move the motor M1 to its middle position, hence 90°.
      "M3.100" will move the motor M3 to its maximum position, here 0°. This 
      is due to how the motor is positionned in the robot, thus the use of %
      instead of angles. Conversion is made below.
      */
    if (single_motor_mode) {
      // Parse input
      if (!isdigit(input[0])) {
        Serial.print("Error: The given motor id is invalid. Please use digits only.\n");
        return;
      }
      motor_id = (int)(input[0] - '0');
      char *buffer = (char *)calloc(len, sizeof(char));
      uint8_t start_read_value = 0;
      for (uint8_t i = 0; i < len; i++) {
        if (input[i] == '.') {
          start_read_value = i + 1;
        }
        else if (start_read_value) {
          buffer[i - start_read_value] = input[i];
        }
      }
      buffer[len - start_read_value] = '\0';
      int percent = atoi(buffer);
      free(buffer);
      // Compute dynamics parameters
      switch (motor_id) {
        case 1:
          dynamics_M1.angle_tf = map(percent, 0, 100, 0, RANGE_M1);
          compute_dynamics(&dynamics_M1, M1.read());
          break;
        case 2:
          dynamics_M2.angle_tf = map(percent, 0, 100, RANGE_M2, 0);
          compute_dynamics(&dynamics_M2, M2.read());
          break;
        case 3:
          dynamics_M3.angle_tf = map(percent, 0, 100, RANGE_M3, 0);
          compute_dynamics(&dynamics_M3, M3.read());
          break;
        case 4:
          dynamics_M4.angle_tf = map(percent, 0, 100, RANGE_M4, 0);
          compute_dynamics(&dynamics_M4, M4.read());
          break;
        case 5:
          dynamics_M5.angle_tf = map(percent, 0, 100, RANGE_M5, 0);
          compute_dynamics(&dynamics_M5, M5.read());
          break;
        default:
          Serial.print("Error: The given motor id does not exist.\n");
          break;
      }
      single_motor_running = true;
    }
    else if (cartesian_mode) {
      // Format: X.Y.Z.YAW.PITCH
      if (parse_cartesian_input(&cartesian_pos, input, INPUT_SIZE)) {
        return;
      }
      Serial.println("x: " + String(cartesian_pos.x));
      Serial.println("y: " + String(cartesian_pos.y));
      Serial.println("z: " + String(cartesian_pos.z));
      Serial.println("yaw: " + String(cartesian_pos.yaw));
      Serial.println("pitch: " + String(cartesian_pos.pitch));
      // Compute dynamic parameters
      get_angles_from_cartesian(&cartesian_pos);
      compute_dynamics(&dynamics_M1, M1.read());
      compute_dynamics(&dynamics_M2, M2.read());
      compute_dynamics(&dynamics_M3, M3.read());
      compute_dynamics(&dynamics_M4, M4.read());
      compute_dynamics(&dynamics_M5, M5.read());
      five_axis_running = true;
      Serial.println("M1: " + String(dynamics_M1.angle_tf));
      Serial.println("M2: " + String(dynamics_M2.angle_tf));
      Serial.println("M3: " + String(dynamics_M3.angle_tf));
      Serial.println("M4: " + String(dynamics_M4.angle_tf));
      Serial.println("M5: " + String(dynamics_M5.angle_tf));
    }
    else {
      Serial.print("Error: Unknown command.\n");
    }

    if (!is_delta_positive()) {
      Serial.print("Error: Acceleration is too low. Please increase `ACC`.\n");
    }
  }
}


/*--------- FUNCTION DEFINITIONS ----------*/
static uint8_t parse_cartesian_input(cartesian_position_t *pos, const char input[], const size_t len)
{
  if (pos == NULL) {
    Serial.print("Error: (parse_cartesian_input) pos pointer is NULL.\n");
    assert(pos);
  }
  char *buf = (char *)calloc(len, sizeof(char));
  uint8_t coord_index = 0;
  // Using +1 to account for the null indicator
  for (size_t j = 0, i = 0; i < len + 1; i++) {
    if (input[i] == '.' || input[i] == '\0') {
      buf[j] = '\0';
      // Assign, erase buffer and go to next coordinate
      switch (coord_index) {
        case 0: pos->x     = (int16_t)atoi(buf); break;
        case 1: pos->y     = (int16_t)atoi(buf); break;
        case 2: pos->z     = (int16_t)atoi(buf); break;
        case 3: pos->yaw   = (int16_t)atoi(buf); break;
        case 4: pos->pitch = (int16_t)atoi(buf); break;
        default: break;
      }
      if (input[i] == '\0') break;
      memset(buf, 0, len);
      coord_index++;
      j = 0;
    }
    else {
        buf[j++] = input[i];
    }
  }
  free(buf);
  return 0;
}

static void initialise_robot(void)
{
  M1.attach(PIN_M1, M1_PW_MIN, M1_PW_MAX);
  M2.attach(PIN_M2, M2_PW_MIN, M2_PW_MAX);
  M2s.attach(PIN_M2s, M2s_PW_MIN, M2s_PW_MAX);
  M3.attach(PIN_M3, M3_PW_MIN, M3_PW_MAX);
  M4.attach(PIN_M4, M4_PW_MIN, M4_PW_MAX);
  M5.attach(PIN_M5, M5_PW_MIN, M5_PW_MAX);

  M1.write(ANGLE_INIT_M1);
  M2.write(ANGLE_INIT_M2);
  M2s.write(ANGLE_INIT_M2);
  M3.write(ANGLE_INIT_M3);
  M4.write(ANGLE_INIT_M4);
  M5.write(ANGLE_INIT_M5);

  Serial.print("-------------------------------------------------\n"
               "Success: The robot has been initialized.\n");
}

static void get_angles_from_cartesian(cartesian_position_t *pos)
{
  if (pos == NULL) {
    Serial.print("Error: (get_angles_from_cartesian) pos pointer is NULL.\n");
    assert(pos);
  }
  // Get M5 coordinates
  const float EFFECTOR_PROJ_XY = LENGTH_WRIST * sin(RAD(pos->pitch));
  pos->x -= EFFECTOR_PROJ_XY * cos(RAD(pos->yaw));
  pos->y -= EFFECTOR_PROJ_XY * sin(RAD(pos->yaw));
  pos->z -= LENGTH_WRIST * cos(RAD(pos->pitch));
  // Compute parameters
  const float M2M5_XY = sqrt(pow(pos->x, 2) + pow(pos->y, 2));
  const float M2M5 =  sqrt(pow(M2M5_XY, 2) + pow(pos->z - HEIGHT_SHOULDER, 2));
  const float ALPHA_P = acos((pow(LENGTH_LA, 2) + pow(M2M5, 2) - pow(LENGTH_UA, 2)) / (2 * M2M5 * LENGTH_LA));
  const float ALPHA = (3 * M_PI / 2) - asin((pos->z - HEIGHT_SHOULDER) / M2M5) - ALPHA_P;
  const float BETA = acos((pow(LENGTH_LA, 2) + pow(LENGTH_UA, 2) - pow(M2M5, 2)) / (2 * LENGTH_LA * LENGTH_UA));
  const float PHI = atan(float(pos->x) / float(pos->y));
  const float EPSILON = ALPHA - acos(M3M4 / LENGTH_UA) - BETA;
  const float V_X5 = cos(RAD(pos->yaw))*cos(PHI) - sin(RAD(pos->yaw))*sin(PHI);
  const float V_Y5 = cos(EPSILON)*(cos(RAD(pos->yaw))*sin(PHI) + sin(RAD(pos->yaw))*cos(PHI)) - cos(RAD(pos->pitch))*sin(EPSILON);
  const float V_Z5 = sin(EPSILON)*(cos(RAD(pos->yaw))*sin(PHI) + sin(RAD(pos->yaw))*cos(PHI)) + cos(RAD(pos->pitch))*cos(EPSILON);
  // Compute joint angles
  const uint8_t ANGLE_M1 = round(RANGE_M1 / 2 - DEG(PHI));
  const uint8_t ANGLE_M2 = round(RANGE_M2 - INVR_TRANS_LA * (DEG(ALPHA) - ANGLE_INIT_LA));
  const uint8_t ANGLE_M3 = round(PI_DEG - INVR_TRANS_UA * (DEG(BETA) - ANGLE_INIT_UA));
  const uint8_t ANGLE_M4 = round(RANGE_M4 - DEG(atan(V_X5 / V_Z5)));
  const uint8_t ANGLE_M5 = round(RANGE_M5 - DEG(atan(V_Y5 / sqrt(pow(V_X5, 2) + pow(V_Z5, 2)))) );
  // If within the work enveloppe, assign the angles
  if (ANGLE_M1 >= 0 && ANGLE_M1 <= RANGE_M1
      && ANGLE_M2 >= 0 && ANGLE_M2 <= RANGE_M2
      && ANGLE_M3 >= 0 && ANGLE_M3 <= RANGE_M3
      && ANGLE_M4 >= 0 && ANGLE_M4 <= RANGE_M4
      && ANGLE_M5 >= 0 && ANGLE_M5 <= RANGE_M5) {
    dynamics_M1.angle_tf = ANGLE_M1;
    dynamics_M2.angle_tf = ANGLE_M2;
    dynamics_M3.angle_tf = ANGLE_M3;
    dynamics_M4.angle_tf = ANGLE_M4;
    dynamics_M5.angle_tf = ANGLE_M5;
  }
  else {
    Serial.print("Error: (get_angles_from_cartesian) Position is outside of work enveloppe.\n");
  }
}

static void compute_dynamics(motor_dynamics_t *dyn, const uint8_t current_angle)
{
  if (dyn == NULL) {
    Serial.print("Error: (compute_dynamics) dyn pointer is NULL.\n");
    assert(dyn);
  }
  dyn->angle_t0 = current_angle;
  if (dyn->angle_t0 < dyn->angle_tf) dyn->forward = 1;
  else dyn->forward = 0;

  const byte DISTANCE = sqrt(pow(dyn->angle_tf - dyn->angle_t0, 2));
  dyn->delta = pow(TF * ACC / 1000.0, 2) - 4 * DISTANCE * ACC;

  const int SPEED = 0.5 * (TF * ACC / 1000.0 - pow(dyn->delta, 0.5));
  dyn->t1 = 1000 * SPEED / ACC;     // ms
  dyn->t2 = TF - dyn->t1;           // ms

  const int SIGN = (dyn->angle_tf - dyn->angle_t0) / DISTANCE;
  dyn->angle_t1 = dyn->angle_t0 + SIGN * ACC * pow(dyn->t1 / 1000.0, 2) / 2;
  dyn->angle_t2 = dyn->angle_t1 + SIGN * SPEED * (dyn->t2 - dyn->t1) / 1000.0;
}

static uint8_t compute_angle(const motor_dynamics_t *dyn, const int64_t t)
{
  if (dyn == NULL) {
    Serial.print("Error: (compute_angle) dyn pointer is NULL.\n");
    assert(dyn);
  }
  uint8_t angle;
  if (t <= dyn->t1) {
    angle = dyn->forward ? (pow(t / 1000.0, 2) * ACC / 2 + dyn->angle_t0) :
                           (-pow(t / 1000.0, 2) * ACC / 2 + dyn->angle_t0);
  }
  else if (t > dyn->t1 && t <= dyn->t2) {
    const int B = dyn->angle_t1 - (dyn->angle_t2 - dyn->angle_t1) * dyn->t1 / (dyn->t2 - dyn->t1);
    angle = t * (dyn->angle_t2 - dyn->angle_t1) / (dyn->t2 - dyn->t1) + B;
  }
  else if (t > dyn->t2 && t <= TF) {
    angle = dyn->forward ? (-pow((TF - t) / 1000.0, 2) * ACC / 2 + dyn->angle_tf) :
                           (pow((t - TF) / 1000.0, 2) * ACC / 2 + dyn->angle_tf);
  }
  else {
    angle = dyn->angle_tf;
  }
  return angle;
}

static uint8_t reset_robot(void)
{
  // Compute dynamic parameters
  dynamics_M1.angle_tf = ANGLE_INIT_M1;
  dynamics_M2.angle_tf = ANGLE_INIT_M2;
  dynamics_M3.angle_tf = ANGLE_INIT_M3;
  dynamics_M4.angle_tf = ANGLE_INIT_M4;
  dynamics_M5.angle_tf = ANGLE_INIT_M5;
  compute_dynamics(&dynamics_M1, M1.read());
  compute_dynamics(&dynamics_M2, M2.read());
  compute_dynamics(&dynamics_M3, M3.read());
  compute_dynamics(&dynamics_M4, M4.read());
  compute_dynamics(&dynamics_M5, M5.read());
  // Check if dynamics parameters are valid
  if (!is_delta_positive()) {
    Serial.print("Error: (reset_robot) Acceleration is too low. Please increase `ACC`.\n");
    return 1;
  }
  // Run the motors
  long t = 0;
  long t0 = millis();
  do {
    t = millis() - t0;
    M1.write(compute_angle(&dynamics_M1, t));
    M2.write(compute_angle(&dynamics_M2, t));
    M2s.write(compute_angle(&dynamics_M2, t));
    M3.write(compute_angle(&dynamics_M3, t));
    M4.write(compute_angle(&dynamics_M4, t));
    M5.write(compute_angle(&dynamics_M5, t));
  } while (t < TF);

  return 0;
}

static uint8_t is_delta_positive(void)
{
  if (dynamics_M1.delta >= 0
      && dynamics_M2.delta >= 0
      && dynamics_M3.delta >= 0
      && dynamics_M4.delta >= 0
      && dynamics_M5.delta >= 0) {
      return 1;
  }
  return 0;
}
