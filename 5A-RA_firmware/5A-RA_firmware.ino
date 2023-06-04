/* 
  Created by Valentin Betbeze
  03 Nov. 2022
  Description:  Firmware of the 5-Axis Robotic Arm.

  @warning:     Motor 4 is set to constant 90°.
                To be modified w/ effector orientation update
 */

#include <math.h>
#include <assert.h>
#include <Servo.h>

#define PI_DEG          180
#define PI2_DEG         90
#define PI4_DEG         45
#define PI8_DEG         22.5

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
#define M4_PW_MAX       2400
#define M5_PW_MIN       390
#define M5_PW_MAX       2370
// Motor maximum angular range in degree
#define RANGE_M1        PI_DEG    
#define RANGE_M2        PI_DEG
#define RANGE_M3        165       // 165 deg is the M3 angle at which the upper arm rests
                                  // on the lower arm in a planar contact configuration
#define RANGE_M4        PI_DEG
#define RANGE_M5        PI_DEG
// Initial motor angles, in degree
#define ANGLE_INIT_M1   PI2_DEG
#define ANGLE_INIT_M2   PI_DEG
#define ANGLE_INIT_M3   RANGE_M3
#define ANGLE_INIT_M4   PI2_DEG
#define ANGLE_INIT_M5   120       // Makes the effector positioned horizontally (ref: ground)

// Robot physical parameters
#define ANGLE_INIT_LA   54        // Lower Arm initial angle, in degree
#define ANGLE_INIT_UA   30        // Upper Arm initial angle, in degree
#define INVR_TRANS_LA   1.65      // Lower Arm transmission inverse ratio
#define INVR_TRANS_UA   2         // Upper Arm transmission inverse ratio
#define HEIGHT_SHOULDER 151       // Shoulder's axis height (ref: ground), in mm
#define LENGTH_LA       150       // Lower Arm length, in mm
#define LENGTH_UA       186.76    // Upper Arm length, in mm
#define LENGTH_WRIST    35        // Wrist length, in mm

// Robot dynamic parameters
#define ACC             320       // acceleration in deg/s²
#define TF              1500      // full duration of a displacement in ms


/*------------ DATA STRUCTURES ------------*/

typedef struct {
  bool forward;
  int t1;
  int t2;
  byte angle_t0;
  byte angle_t1;
  byte angle_t2;
  byte angle_tf;
  int delta;
}  motor_dynamics_t ;

typedef struct  {
  int x;
  int y;
  int z;
} cartesian_position_t;

typedef struct {
  int radius;
  int theta;
  int phi;
} polar_position_t;


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
polar_position_t polar_pos;

bool single_motor_mode = false;
bool cartesian_mode = false;
bool polar_mode = false;
bool single_motor_running = false;
bool five_axis_running = false;
unsigned char motor_id = '\0';


/*---------- FUNCTION PROTOYPES -----------*/

/* Initialise the servo motor angular position. Use only in setup().
 */
static void initialise_robot(void);

/* Provides the motor angles in degree required to reach the cartesian 
 position given as input. The function directly updates the angles for 
 each motor in their respective struct <motor_dynamics_t> (angle_tf). 
 LA = Lower Arm
 UA = Upper Arm
 */
static void get_angles_from_cartesian(const cartesian_position_t *pos);

/* Provides the motor angles in degree required to reach the polar 
 position given as input. The function directly updates the angles for 
 each motor in their respective struct <motor_dynamics_t> (angle_tf). 
 LA = Lower Arm
 UA = Upper Arm
 */
static void get_angles_from_polar(const polar_position_t *pos);

/* Computation of the required motor_dynamics_t parameters for a 
 given distance with regards to desired effector speed and acceleration.
 @warning: The final angle `stopAngle` must be determined and assigned to
 the motor_dynamics_t variable prior to using the function.
 */
static void compute_dynamics(motor_dynamics_t *dyn, const byte current_angle);

/* Computes the angle at a time t between t0 and TF so that the motor 
accelerates and decelerates at a rate <ACC> while heading to stopAngle.
*/
static byte compute_angle(const motor_dynamics_t *dyn, const long t);

/* Resets the robot to its idle configuration
 */
static void reset_robot(void);

/* Determines if the delta of the polynomial equation is >= 0,
 meaning the given position is within the work enveloppe.
 */
static bool is_delta_positive(void);


/*-------------- PROGRAM MAIN -------------*/

void setup() {
  Serial.begin(9600);
  initialise_robot();
  Serial.print("-------------------------------------------------\n"
              "The robot is ready for use.\n"
              "\tFor single motor control enter 'motor'\n"
              "\tFor cartesian position control enter 'cartesian'\n"
              "\tFor polar position control enter 'polar'\n"
              "\tTo reset the robot enter 'reset'\n");
}

void loop() {
  uint32_t t0 = millis();
  // Operate the robot
  while (Serial.available() == 0) {
    /* Runs only if delta is positive or equal to zero, practically
     meaning that the acceleration is high enough so that the motor(s) 
     can complete the entire distance within the time TF.
     */  
    if (is_delta_positive()) {
      uint32_t t = millis() - t0;

      if (single_motor_running) {
        switch (motor_id) {
          case '1': 
            M1.write(compute_angle(&dynamics_M1, t));
            break;
          case '2':
            M2.write(compute_angle(&dynamics_M2, t));
            M2s.write(compute_angle(&dynamics_M2, t));
            break;
          case '3': 
            M3.write(compute_angle(&dynamics_M3, t));
            break;
          case '4': 
            M4.write(compute_angle(&dynamics_M4, t));
            break;
          case '5':
            M5.write(compute_angle(&dynamics_M5, t));
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

  single_motor_running = false;
  five_axis_running = false;
  String input = Serial.readString();
  // Mode selection
  if (input == "motor\n") {
    single_motor_mode = true;
    cartesian_mode = false;
    polar_mode = false;
    Serial.println("Single motor mode accessed. Format: MX.%%%");
  }
  else if (input == "cartesian\n") {
    single_motor_mode = false;
    cartesian_mode = true;
    polar_mode = false;
    Serial.println("Cartesian mode accessed. Format: SXXX.YYY.ZZZ");
  }
  else if (input == "polar\n") {
    single_motor_mode = false;
    cartesian_mode = false;
    polar_mode = true;
    Serial.println("Polar mode accessed. Format: RRR.STTT.PPP");
  }
  else if (input == "reset\n") {
    single_motor_mode = false;
    cartesian_mode = false;
    polar_mode = false;
    reset_robot();
    Serial.println("The robot has been reset.");
  }
  // If 'input' is not a mode entry, then it is a value entry
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
      Serial.print("> " + input);
      motor_id = input[1];
      String percent_buf = "";
      for (int i = 3; i < 6; i++) percent_buf += input[i];
      int percent = percent_buf.toInt();

      switch (motor_id) {
        case '1':
          dynamics_M1.angle_tf = map(percent, 0, 100, 0, RANGE_M1);
          compute_dynamics(&dynamics_M1, M1.read());
          break;
        case '2':
          dynamics_M2.angle_tf = map(percent, 0, 100, RANGE_M2, 0);
          compute_dynamics(&dynamics_M2, M2.read());
          break;
        case '3':
          dynamics_M3.angle_tf = map(percent, 0, 100, RANGE_M3, 0);
          compute_dynamics(&dynamics_M3, M3.read());
          break;
        case '4':
          dynamics_M4.angle_tf = map(percent, 0, 100, RANGE_M4, 0);
          compute_dynamics(&dynamics_M4, M4.read());
          break;
        case '5':
          dynamics_M5.angle_tf = map(percent, 0, 100, 0, RANGE_M5);
          compute_dynamics(&dynamics_M5, M5.read());
          break;
      }
      single_motor_running = true;
    }
    // Format: SXXX.YYY.ZZZ
    else if (cartesian_mode) {
      
      String position_buf = "";
      for (int i = 0; i < 4; i++) position_buf += input[i];
      cartesian_pos.x = position_buf.toInt();
      position_buf = "";
      for (int i = 5; i < 8; i++) position_buf += input[i];
      cartesian_pos.y = position_buf.toInt();
      position_buf = "";
      for (int i = 9; i < 12; i++) position_buf += input[i];
      cartesian_pos.z = position_buf.toInt();

      get_angles_from_cartesian(&cartesian_pos);
      compute_dynamics(&dynamics_M1, M1.read());
      compute_dynamics(&dynamics_M2, M2.read());
      compute_dynamics(&dynamics_M3, M3.read());
      compute_dynamics(&dynamics_M4, M4.read());
      compute_dynamics(&dynamics_M5, M5.read());

      Serial.print("> Input position:");
      Serial.print("\tx=" + String(cartesian_pos.x));
      Serial.print("\ty=" + String(cartesian_pos.y));
      Serial.print("\tz=" + String(cartesian_pos.z) + '\n');
      five_axis_running = true;    
    }
    //Format: RRR.STTT.PPP
    else if (polar_mode) {
      
      String position_buf = "";
      for (int i = 0; i < 3; i++) position_buf += input[i];
      polar_pos.radius = position_buf.toInt();
      position_buf = "";
      for (int i = 4; i < 8; i++) position_buf += input[i];
      polar_pos.theta = position_buf.toInt();
      position_buf = "";
      for (int i = 9; i < 12; i++) position_buf += input[i];
      polar_pos.phi = position_buf.toInt();

      get_angles_from_polar(&polar_pos);
      compute_dynamics(&dynamics_M1, M1.read());
      compute_dynamics(&dynamics_M2, M2.read());
      compute_dynamics(&dynamics_M3, M3.read());
      compute_dynamics(&dynamics_M4, M4.read());
      compute_dynamics(&dynamics_M5, M5.read());

      Serial.print("> Input position:");
      Serial.print("\tr=" + String(polar_pos.radius));
      Serial.print("\ttheta=" + String(polar_pos.theta));
      Serial.print("\tphi=" + String(polar_pos.phi) + '\n');
      five_axis_running = true;
    } 
    else {
      Serial.println("Error(loop): Unknown command");
    }

    if (!is_delta_positive()) {
      Serial.println("Error(loop): Too low acceleration");
    }
  }
}


/*--------- FUNCTION DEFINITIONS ----------*/

static void initialise_robot(void) {
  M1.attach(7, M1_PW_MIN, M1_PW_MAX);
  /*
  M2.attach(10, M2_PW_MIN, M2_PW_MAX);
  M2s.attach(11, M2s_PW_MIN, M2s_PW_MAX);
  M3.attach(8, M3_PW_MIN, M3_PW_MAX);
  M4.attach(9, M4_PW_MIN, M4_PW_MAX);
  M5.attach(12, M5_PW_MIN, M5_PW_MAX);
  */

  M1.write(ANGLE_INIT_M1);
  M2.write(ANGLE_INIT_M2);
  M2s.write(ANGLE_INIT_M2);
  M3.write(ANGLE_INIT_M3);
  M4.write(ANGLE_INIT_M4);
  M5.write(ANGLE_INIT_M5);
}

static void get_angles_from_cartesian(const cartesian_position_t *pos) {
  if (pos == NULL) {
    Serial.print("Error(get_angles_from_cartesian): pos pointer is NULL");
    assert(pos);
  }
  const float HYPOT_XY = sqrt(pow(pos->x, 2) + pow(pos->y, 2));
  const float HYPOT = sqrt(pow(HYPOT_XY - LENGTH_WRIST, 2) + pow(pos->z - HEIGHT_SHOULDER, 2));

  const float ALPHA1 = asin((pos->z - HEIGHT_SHOULDER) / HYPOT);
  const float ALPHA2 = acos((pow(LENGTH_LA, 2) + pow(HYPOT, 2) - pow(LENGTH_UA, 2)) / (2 * HYPOT * LENGTH_LA));
  const float ALPHA = (PI * (PI_DEG - ANGLE_INIT_LA) / PI_DEG - ALPHA1 - ALPHA2) * INVR_TRANS_LA;

  const float BETA1 = acos((pow(LENGTH_LA, 2) + pow(LENGTH_UA, 2) - pow(HYPOT, 2)) / (2 * LENGTH_LA * LENGTH_UA));
  const float BETA = (BETA1 - (ANGLE_INIT_UA * PI / PI_DEG)) * INVR_TRANS_UA;

  const float GAMMA = atan(float(pos->x) / float(pos->y)) + PI / 2;

  const float THETA1 = acos((pow(HYPOT, 2) + pow(LENGTH_UA, 2) - pow(LENGTH_LA, 2)) / (2 * HYPOT * LENGTH_UA));
  const float THETA2 = -ALPHA1 + PI / 2;
  const float THETA = PI - THETA1 - THETA2 - PI * (float)18 / PI_DEG;

  const byte ANGLE_M1 = byte(round(PI_DEG * GAMMA / PI));
  const byte ANGLE_M2 = byte(RANGE_M2 - round(PI_DEG * ALPHA / PI));
  const byte ANGLE_M3 = byte(RANGE_M3 - round(PI_DEG * BETA / PI));
  const byte ANGLE_M4 = PI2_DEG; // @note: Motor is disabled. To be modified w/ effector orientation update
  const byte ANGLE_M5 = byte(RANGE_M5 - round(PI_DEG * THETA / PI));

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
    Serial.println("Error(get_angles_from_cartesian): End position is outside of work enveloppe");
  }
}

static void get_angles_from_polar(const polar_position_t *pos) {
  if (pos == NULL) {
    Serial.print("Error(get_angles_from_polar): pos pointer is NULL");
    assert(pos);
  }
  const float ALPHA1 = PI / 2 - pos->phi;
  const float ALPHA2 = acos((pow(LENGTH_LA, 2) + pow(pos->radius, 2) - pow(LENGTH_UA, 2)) / (2 * pos->radius * LENGTH_LA));
  const float ALPHA = ((pos->theta - ANGLE_INIT_LA) * PI / PI_DEG - ALPHA2 + PI / 2) * INVR_TRANS_LA;

  const float BETA1 = acos((pow(LENGTH_LA, 2) + pow(LENGTH_UA, 2) - pow(pos->radius, 2)) / (2 * LENGTH_LA * LENGTH_UA));
  const float BETA = (BETA1 - (ANGLE_INIT_UA * PI / PI_DEG)) * INVR_TRANS_UA;

  const float THETA1 = acos((pow(pos->radius, 2) + pow(LENGTH_UA, 2) - pow(LENGTH_LA, 2)) / (2 * pos->radius * LENGTH_UA));
  const float THETA2 = -1 * ALPHA1 + PI / 2;
  const float THETA = PI - THETA1 - THETA2 - PI * (float)18 / PI_DEG;

  byte ANGLE_M1 = pos->phi;
  byte ANGLE_M2 = byte(RANGE_M2 - round(PI_DEG * ALPHA / PI));
  byte ANGLE_M3 = byte(RANGE_M3 - round(PI_DEG * BETA / PI));
  byte ANGLE_M4 = PI2_DEG; // @note same as get_angles_from_cartesian()
  byte ANGLE_M5 = byte(RANGE_M5 - round(PI_DEG * THETA / PI));

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
    Serial.println("Error(get_angles_from_polar): End position is outside of work enveloppe");
  }
}

static void compute_dynamics(motor_dynamics_t *dyn, const byte current_angle) {
  if (dyn == NULL) {
    Serial.print("Error(compute_dynamics): dyn pointer is NULL");
    assert(dyn);
  }
  dyn->angle_t0 = current_angle;
  if (dyn->angle_t0 < dyn->angle_tf) dyn->forward = true;
  else dyn->forward = false;

  const byte DISTANCE = sqrt(pow(dyn->angle_tf - dyn->angle_t0, 2));
  dyn->delta = pow(TF * ACC / 1000.0, 2) - 4 * DISTANCE * ACC;

  const int SPEED = 0.5 * (TF * ACC / 1000.0 - pow(dyn->delta, 0.5));
  dyn->t1 = 1000 * SPEED / ACC;  // ms
  dyn->t2 = TF - dyn->t1;           //ms

  const int SIGN = (dyn->angle_tf - dyn->angle_t0) / DISTANCE;
  dyn->angle_t1 = dyn->angle_t0 + SIGN * ACC * pow(dyn->t1 / 1000.0, 2) / 2;
  dyn->angle_t2 = dyn->angle_t1 + SIGN * SPEED * (dyn->t2 - dyn->t1) / 1000.0;
}

static byte compute_angle(const motor_dynamics_t *dyn, const long t) {
  if (dyn == NULL) {
    Serial.print("Error(compute_angle): dyn pointer is NULL");
    assert(dyn);
  }
  byte angle;
  if (t <= dyn->t1) {
    angle = (dyn->forward == true) ? (pow(t / 1000.0, 2) * ACC / 2 + dyn->angle_t0) :
                                    (-pow(t / 1000.0, 2) * ACC / 2 + dyn->angle_t0);
  }
  else if (t > dyn->t1 && t <= dyn->t2) {
    const int B = dyn->angle_t1 - (dyn->angle_t2 - dyn->angle_t1) * dyn->t1 / (dyn->t2 - dyn->t1);
    angle = t * (dyn->angle_t2 - dyn->angle_t1) / (dyn->t2 - dyn->t1) + B;
  }
  else if (t > dyn->t2 && t <= TF) {
    angle = (dyn->forward == true) ? (-pow((TF - t) / 1000.0, 2) * ACC / 2 + dyn->angle_tf) :
                                      (pow((t - TF) / 1000.0, 2) * ACC / 2 + dyn->angle_tf);
  }
  else {
    angle = dyn->angle_tf;
  }
  return angle;
}

static void reset_robot(void) {
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
  }
  while (t > TF);
}

static bool is_delta_positive(void) {
  if (dynamics_M1.delta >= 0
      && dynamics_M2.delta >= 0
      && dynamics_M3.delta >= 0
      && dynamics_M4.delta >= 0
      && dynamics_M5.delta >= 0) {
      return true;
  }
  return false;
}
