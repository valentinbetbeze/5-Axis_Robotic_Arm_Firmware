/* 
  Created by Valentin Betbeze
  Date: 03 Nov. 2022
  Description:  Control firmware of my custom 5-Axis Robotic Arm.
                Includes single motor control (manual mode) as well
                as cartesian control of the effector.
  
  Note: Will need to be modified if to be used on a different machine.
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <assert.h>
#include <stdint.h>
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
#define M4_PW_MAX       2430
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
#define HEIGHT_SHOULDER 151       // Shoulder's axis height (ref: ground), in mm
#define LENGTH_LA       150       // Lower Arm length, in mm
#define M3M4            25.56     // Interaxial distance between the axis of M3 and M4, in mm
#define LENGTH_UA       186.76    // Upper Arm length, in mm
#define LENGTH_WRIST    43        // Wrist length, in mm

// Speeds
#define LOW_SPEED       45        // in mm/s
#define HIGH_SPEED      90        // in mm/s
#define SMM_TRAJ_DUR    3000      // Single motor mode trajectory duration, in ms


/*------------ DATA STRUCTURES ------------*/
typedef struct {
  int16_t x;          // mm
  int16_t y;          // mm
  int16_t z;          // mm
  int16_t yaw;        // deg
  int16_t pitch;      // deg
  // roll not available on current robot version
} cartesian_coord_t;

// Linear segment between 2 cartesian coordinates locations.
typedef struct {
  cartesian_coord_t coord;
  uint16_t avg_speed; // in mm/s
  int duration;   // in ms
} trajectory_t;

typedef struct {
  uint8_t forward;
  int64_t t1;
  uint8_t angle_t0;
  uint8_t angle_t1;
  uint8_t angle_tf;
  uint16_t acc;
} motor_dynamics_t;


/*----------- GLOBAL VARIABLES ------------*/

static enum {
  NONE,
  SINGLE_MOTOR_MODE,
  CARTESIAN_MODE,
  SEQUENCE_MODE,
} state = NONE;

uint8_t running = 0;
uint8_t motor_id = 0;

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

/* Effector dependent! Update with new effector length or make script to take
 LENGTH_WRIST into consideration
 */
#define INITIAL_COORD   {.x = 10, .y = 0, .z = 282, .yaw = 0, .pitch = 0}
cartesian_coord_t current_coord;
trajectory_t cartesian_traj = {.avg_speed = LOW_SPEED};
trajectory_t square[] = {
  {.coord = {-70, 100, 250, 90, 90}, .avg_speed = LOW_SPEED},
  {.coord = {-70, 100, 350, 90, 90}, .avg_speed = LOW_SPEED},
  {.coord = {70, 100, 350, 90, 90},  .avg_speed = LOW_SPEED},
  {.coord = {70, 100, 250, 90, 90},  .avg_speed = LOW_SPEED},
  {.coord = {-70, 100, 250, 90, 90}, .avg_speed = LOW_SPEED},
};


/*---------- FUNCTION PROTOTYPES ----------*/

/* 
  @brief Initialize the servo motor angular position. 
  @warning Use only in setup().
*/
static void initialize_robot(void);

/* 
  @brief Parse a cartesian coordinate user input and loads it in memory.

  @param input Input string.
  @param len Size of the string.
  @param coord Pointer to memory location on where to load the parsed coordinates.

  @returns 1 if errors are encountered; else 0.
*/
static uint8_t parse_cartesian_input(const char input[], const size_t len, cartesian_coord_t *coord);

/* 
  @brief Provide the motor angles in degree required to reach the cartesian 
         position given as input. The function directly updates the angles for 
         each motor in their respective struct <motor_dynamics_t> (angle_tf).

  @param coord Pointer to the cartesian coordinates.

  @returns 1 if the coordinates are beyond the work enveloppe; else O.
*/
static uint8_t get_angles_from_cartesian(const cartesian_coord_t *coord);

/* 
  @brief Compute the total duration of a trajectory, in ms.

  @param traj Pointer to the trajectory data.

  @returns 1 if errors are encountered; else 0.
*/
static uint8_t compute_trajectory_duration(trajectory_t *traj);

/* 
  @brief Compute the dynamics parameters of a motor for a specific
         distance. The smallest possible acceleration is taken, so as to
         have the smoothest effector displacements.

  @param duration Duration of the displacement.
  @param current_angle Current motor angle (use motor.read()).
  @param dyn Pointer to the dynamics data of the motor.

  @warning The final angle `stopAngle` must be determined and assigned to
           the motor_dynamics_t variable prior to using the function.
*/
static void compute_motor_dynamics(const int duration, const int current_angle, motor_dynamics_t *dyn);

/* 
  @brief Compute the motor angle to feed to a motor at a given time 
         between the beginning and the end of its course. This function 
         allows the motor to accelerate and deccelerate in a controlled way.

  @param dyn Pointer to the dynamics data of the motor.
  @param duration Duration of the displacement.
  @param t The current time at which to compute the new angle.
  
  @returns The new angle to feed to the motor.
*/
static uint8_t compute_angle(const motor_dynamics_t *dyn, const int duration, const int64_t t);

/*
  @brief Update the position of the robot using the dynamics parameters of
         each motor (global variables).

  @param duration The trajectory duration, in ms.
  @param i The time in ms. t = 0 at the start of the trajectory, t = duration
           at the end of trajectory.
  
  @returns 1 if the position has been reached; else 0.
*/
static uint8_t update_robot_position(const int duration, int64_t t);

/*
  @brief Reset the robot to its idle position.
*/
static void reset_robot(void);

/*
  @brief Operate the robot along a sequence of trajectories. A delay
         can be added in between each trajectory.

  @param path Sequence of trajectories.
  @param npoints Number of trajectories.
  @param dwell_time Delay between each trajectory, in ms.

  @returns -1 if an error is encountered;
           0 if the path is not complete;
           1 if the path is complete
*/
static int8_t follow_path(trajectory_t path[], const uint32_t npoints, const uint16_t dwell_time);


/*-------------- PROGRAM BODY -------------*/
void setup()
{
  Serial.begin(9600);
  initialize_robot();
  current_coord = INITIAL_COORD; 
  Serial.print("Commands:\n"
               "\t'reset'     : Always use before turning the robot off.\n"
               "\t'motor'     : Single motor control\n"
               "\t'cartesian' : Cartesian effector control\n"
               "\t'sequence'  : Run a pre-determined sequence\n");
}

void loop()
{
  // While no user input detected, operate the robot
  int64_t t0 = millis();
  while (!Serial.available()) {
    int64_t t = millis() - t0;

    switch (state) {
      case SINGLE_MOTOR_MODE:
        if (!running || t > SMM_TRAJ_DUR) {
          running = 0;
          break;
        }
        switch (motor_id) {
          case 1: 
            M1.write(compute_angle(&dynamics_M1, SMM_TRAJ_DUR, t));
            break;
          case 2:
            M2.write(compute_angle(&dynamics_M2, SMM_TRAJ_DUR, t));
            M2s.write(compute_angle(&dynamics_M2, SMM_TRAJ_DUR, t));
            break;
          case 3: 
            M3.write(compute_angle(&dynamics_M3, SMM_TRAJ_DUR, t));
            break;
          case 4: 
            M4.write(compute_angle(&dynamics_M4, SMM_TRAJ_DUR, t));
            break;
          case 5:
            M5.write(compute_angle(&dynamics_M5, SMM_TRAJ_DUR, t));
            break;
          default:
            break;
        }
        break;

      case CARTESIAN_MODE:
        if (!running) break;
        else if (update_robot_position(cartesian_traj.duration, t)) {
          running = 0;
          current_coord = cartesian_traj.coord;
        }
        break;

      case SEQUENCE_MODE: {
        int8_t err = follow_path(square, sizeof(square) / sizeof(square[0]), 1000);
        if (err == - 1) {
          Serial.print("Warning: The robot is going to reset automatically in 5 seconds...\n");
          for (int8_t j = 5; j > 0; j--) {
            delay(1000);
            Serial.print("..." + String(j - 1) + '\n');
          }
          reset_robot();
          state = NONE;
        }
        else if (err == 1) {
          state = NONE;
        }
      } break;

      default:
        break;
    }
  }

  // Get input
  running = 0;
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
    state = SINGLE_MOTOR_MODE;
    Serial.print("Success: 'Single motor mode' accessed. Format: ID.VALUE\n");
  }
  else if (!strcmp(input, "cartesian")) {
    state = CARTESIAN_MODE;
    Serial.print("Success: 'Cartesian mode' accessed. Format: X.Y.Z.YAW.PITCH\n");
  }
  else if (!strcmp(input, "reset")) {
    state = NONE;
    reset_robot();
  }
  else if (!strcmp(input, "sequence")) {
    state = SEQUENCE_MODE;
  }
  else { // If input is not a mode, then it may be a value
    switch (state) {
      case SINGLE_MOTOR_MODE: {
        /* The number (ID as in Servo M<ID>) of the motor must be entered as 
          well as the desired position to reach, expressed in percentage of the  
          total allowed displacement of the given motor.

          Exemples: 
          "M1.050" will move the motor M1 to its middle position, hence 90°.
          "M3.100" will move the motor M3 to its maximum position, here 0°. This 
          is due to how the motor is positionned in the robot, thus the use of %
          instead of angles. Conversion is made below.
          */
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
            compute_motor_dynamics(SMM_TRAJ_DUR, M1.read(), &dynamics_M1);
            break;
          case 2:
            dynamics_M2.angle_tf = map(percent, 0, 100, RANGE_M2, 0);
            compute_motor_dynamics(SMM_TRAJ_DUR, M2.read(), &dynamics_M2);
            break;
          case 3:
            dynamics_M3.angle_tf = map(percent, 0, 100, RANGE_M3, 0);
            compute_motor_dynamics(SMM_TRAJ_DUR, M3.read(), &dynamics_M3);
            break;
          case 4:
            dynamics_M4.angle_tf = map(percent, 0, 100, RANGE_M4, 0);
            compute_motor_dynamics(SMM_TRAJ_DUR, M4.read(), &dynamics_M4);
            break;
          case 5:
            dynamics_M5.angle_tf = map(percent, 0, 100, RANGE_M5, 0);
            compute_motor_dynamics(SMM_TRAJ_DUR, M5.read(), &dynamics_M5);
            break;
          default:
            Serial.print("Error: The given motor id does not exist.\n");
            break;
        }
        running = 1;
      } break;

      case CARTESIAN_MODE: {
        cartesian_traj.coord = {0};
        if (parse_cartesian_input(input, INPUT_SIZE, &cartesian_traj.coord)) {
          return;
        }
        if (get_angles_from_cartesian(&cartesian_traj.coord)) {
          return;
        }
        if (compute_trajectory_duration(&cartesian_traj)) {
          return;
        }
        compute_motor_dynamics(cartesian_traj.duration, M1.read(), &dynamics_M1);
        compute_motor_dynamics(cartesian_traj.duration, M2.read(), &dynamics_M2);
        compute_motor_dynamics(cartesian_traj.duration, M3.read(), &dynamics_M3);
        compute_motor_dynamics(cartesian_traj.duration, M4.read(), &dynamics_M4);
        compute_motor_dynamics(cartesian_traj.duration, M5.read(), &dynamics_M5);
        running = 1;
      } break;
      default:
        Serial.print("Error: Unknown command.\n");
        break;
    }
  }
}


/*--------- FUNCTION DEFINITIONS ----------*/
static void initialize_robot(void)
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

static uint8_t parse_cartesian_input(const char input[], const size_t len, cartesian_coord_t *coord)
{
  if (coord == NULL) {
    Serial.print("Error: (parse_cartesian_input) coord pointer is NULL.\n");
    assert(coord);
  }
  char *buf = (char *)calloc(len, sizeof(char));
  if (buf == NULL) {
    Serial.print("Error: (parse_cartesian_input) Memory allocation failed.\n");
    free(buf);
    return 1;
  }
  // Parse input (Using +1 to account for the null indicator)
  uint8_t coord_index = 0;
  for (size_t j = 0, i = 0; i < len + 1; i++) {
    if (input[i] == '.' || input[i] == '\0') {
      buf[j] = '\0';
      // Assign, erase buffer and go to next coordinate
      switch (coord_index++) {
        case 0: coord->x     = (int16_t)atoi(buf); break;
        case 1: coord->y     = (int16_t)atoi(buf); break;
        case 2: coord->z     = (int16_t)atoi(buf); break;
        case 3: coord->yaw   = (int16_t)atoi(buf); break;
        case 4: coord->pitch = (int16_t)atoi(buf); break;
        default: break;
      }
      if (input[i] == '\0') break;
      memset(buf, 0, len);
      j = 0;
    }
    else {
        buf[j++] = input[i];
    }
  }
  free(buf);
  return 0;
}

static uint8_t get_angles_from_cartesian(const cartesian_coord_t *coord)
{
  if (coord == NULL) {
    Serial.print("Error: (get_angles_from_cartesian) coord pointer is NULL.\n");
    assert(coord);
  }
  // Get M5 coordinates
  const float EFFECTOR_PROJ_XY = LENGTH_WRIST * sin(RAD(coord->pitch));
  const float M5_x = coord->x - EFFECTOR_PROJ_XY * cos(RAD(coord->yaw));
  const float M5_y = coord->y - EFFECTOR_PROJ_XY * sin(RAD(coord->yaw));
  const float M5_z = coord->z - LENGTH_WRIST * cos(RAD(coord->pitch));
  // Compute parameters
  const float M2M5_XY = sqrt(pow(M5_x, 2) + pow(M5_y, 2));
  const float M2M5 =  sqrt(pow(M2M5_XY, 2) + pow(M5_z - HEIGHT_SHOULDER, 2));
  const float ALPHA_P = acos((pow(LENGTH_LA, 2) + pow(M2M5, 2) - pow(LENGTH_UA, 2)) / (2 * M2M5 * LENGTH_LA));
  const float ALPHA = (float)(3.0 * M_PI / 2.0) - asin((M5_z - HEIGHT_SHOULDER) / M2M5) - ALPHA_P;
  const float BETA = acos((pow(LENGTH_LA, 2) + pow(LENGTH_UA, 2) - pow(M2M5, 2)) / (2 * LENGTH_LA * LENGTH_UA));
  const float PHI = atan(float(M5_x / M5_y));
  const float EPSILON = ALPHA - acos(M3M4 / LENGTH_UA) - BETA;
  const double V_X5 = cos(RAD(coord->yaw))*cos(PHI) - sin(RAD(coord->yaw))*sin(PHI);
  const double V_Y5 = cos(EPSILON)*(cos(RAD(coord->yaw))*sin(PHI) + sin(RAD(coord->yaw))*cos(PHI)) - cos(RAD(coord->pitch))*sin(EPSILON);
  const double V_Z5 = sin(EPSILON)*(cos(RAD(coord->yaw))*sin(PHI) + sin(RAD(coord->yaw))*cos(PHI)) + cos(RAD(coord->pitch))*cos(EPSILON);
  // Compute joint angles
  const int16_t ANGLE_M1 = round(PI2_DEG + DEG(PHI));
  const int16_t ANGLE_M2 = round(PI_DEG - INVR_TRANS_LA * (DEG(ALPHA) - ANGLE_INIT_LA));
  const int16_t ANGLE_M3 = round(PI_DEG - INVR_TRANS_UA * (DEG(BETA) - ANGLE_INIT_UA));
  const int16_t ANGLE_M4 = round(PI2_DEG - DEG(atan(V_X5 / V_Z5)));
  const int16_t ANGLE_M5 = (V_Z5 >= 0.0) ? PI_DEG - DEG(atan(V_Y5 / sqrt(pow(V_X5, 2) + pow(V_Z5, 2)))) :
                                           DEG(atan(V_Y5 / sqrt(pow(V_X5, 2) + pow(V_Z5, 2))));                                  
  // Check if the position can be reached
  if (ANGLE_M1 < 0 || ANGLE_M1 > RANGE_M1
      || ANGLE_M2 < 0 || ANGLE_M2 > RANGE_M2
      || ANGLE_M3 < 0 || ANGLE_M3 > RANGE_M3
      || ANGLE_M4 < 0 || ANGLE_M4 > RANGE_M4
      || ANGLE_M5 < 0 || ANGLE_M5 > RANGE_M5) {
    Serial.print("Error: (get_angles_from_cartesian) Position cannot be reached, due to...\n");
    if (ANGLE_M1 < 0 || ANGLE_M1 > RANGE_M1) Serial.print("Error: M1 = " + String(ANGLE_M1) + "°\n");
    if (ANGLE_M2 < 0 || ANGLE_M2 > RANGE_M2) Serial.print("Error: M2 = " + String(ANGLE_M2) + "°\n");
    if (ANGLE_M3 < 0 || ANGLE_M3 > RANGE_M3) Serial.print("Error: M3 = " + String(ANGLE_M3) + "°\n");
    if (ANGLE_M4 < 0 || ANGLE_M4 > RANGE_M4) Serial.print("Error: M4 = " + String(ANGLE_M4) + "°\n");
    if (ANGLE_M5 < 0 || ANGLE_M5 > RANGE_M5) Serial.print("Error: M5 = " + String(ANGLE_M5) + "°\n");
    return 1;
  }
  dynamics_M1.angle_tf = ANGLE_M1;
  dynamics_M2.angle_tf = ANGLE_M2;
  dynamics_M3.angle_tf = ANGLE_M3;
  dynamics_M4.angle_tf = ANGLE_M4;
  dynamics_M5.angle_tf = ANGLE_M5;
  return 0;
}

static uint8_t compute_trajectory_duration(trajectory_t *traj)
{
  if (traj == NULL) {
    Serial.print("Error: (compute_trajectory_duration) traj pointer is NULL.\n");
    assert(traj);
  }
  if (traj->avg_speed != LOW_SPEED && traj->avg_speed != HIGH_SPEED) {
    Serial.print("Error: (compute_trajectory_duration) Speed value is incorrect."
                                                      "Choose either LOW_SPEED or HIGH_SPEED.\n");
    return 1;
  }
  // Compute distance to cover (mm)
  const float EFFECTOR_PROJ_XY_CUR = LENGTH_WRIST * sin(RAD(current_coord.pitch));
  const float EFFECTOR_PROJ_XY_NEW = LENGTH_WRIST * sin(RAD(traj->coord.pitch));

  const int16_t M5_X_CUR = current_coord.x - EFFECTOR_PROJ_XY_CUR * cos(RAD(current_coord.yaw));
  const int16_t M5_Y_CUR = current_coord.y - EFFECTOR_PROJ_XY_CUR * sin(RAD(current_coord.yaw));
  const int16_t M5_Z_CUR = current_coord.z - LENGTH_WRIST * cos(RAD(current_coord.pitch));

  const int16_t M5_X_NEW = traj->coord.x - EFFECTOR_PROJ_XY_NEW * cos(RAD(traj->coord.yaw));
  const int16_t M5_Y_NEW = traj->coord.y - EFFECTOR_PROJ_XY_NEW * sin(RAD(traj->coord.yaw));
  const int16_t M5_Z_NEW = traj->coord.z - LENGTH_WRIST * cos(RAD(traj->coord.pitch));

  const int16_t DIST_X = M5_X_NEW - M5_X_CUR;
  const int16_t DIST_Y = M5_Y_NEW - M5_Y_CUR;
  const int16_t DIST_Z = M5_Z_NEW - M5_Z_CUR;
  const uint16_t DIST = sqrt(pow(DIST_X, 2) + pow(DIST_Y, 2) + pow(DIST_Z, 2));
  // Compute the duration of the displacement (ms)
  traj->duration = (1000 * DIST) / traj->avg_speed;
  return 0;
}

static void compute_motor_dynamics(const int duration, const int current_angle, motor_dynamics_t *dyn)
{
  if (dyn == NULL) {
    Serial.print("Error: (compute_motor_dynamics) dyn pointer is NULL.\n");
    assert(dyn);
  }
  dyn->angle_t0 = current_angle;
  dyn->angle_t0 < dyn->angle_tf ? dyn->forward = 1 : dyn->forward = 0;

  // Compute the minimal acceleration (we want the movements to be as smooth as possible)
  const uint8_t ANGULAR_DISTANCE = abs(dyn->angle_tf - dyn->angle_t0);
  const float MAX_SPEED = 2 * (float)ANGULAR_DISTANCE / ((float)duration / 1000.0);
  dyn->acc = (float)(2 * MAX_SPEED) / (float)(duration / 1000.0);
  dyn->t1 = duration / 2.0;   // ms

  const int8_t SIGN = (dyn->angle_tf - dyn->angle_t0) / ANGULAR_DISTANCE;
  dyn->angle_t1 = dyn->angle_t0 + SIGN * dyn->acc * pow(dyn->t1 / 1000.0, 2) / 2;
}

static uint8_t compute_angle(const motor_dynamics_t *dyn, const int duration, const int64_t t)
{
  if (dyn == NULL) {
    Serial.print("Error: (compute_angle) dyn pointer is NULL.\n");
    assert(dyn);
  }
  uint8_t angle;
  if (t <= dyn->t1) {
    angle = dyn->forward ? (pow(t / 1000.0, 2) * dyn->acc / 2 + dyn->angle_t0) :
                           (-pow(t / 1000.0, 2) * dyn->acc / 2 + dyn->angle_t0);
  }
  else if (t <= duration) {
    angle = dyn->forward ? (-pow((duration - t) / 1000.0, 2) * dyn->acc / 2 + dyn->angle_tf) :
                           (pow((t - duration) / 1000.0, 2) * dyn->acc / 2 + dyn->angle_tf);
  }
  else {
    angle = dyn->angle_tf;
  }
  return angle;
}

static uint8_t update_robot_position(const int duration, int64_t t)
{
  if (t > duration) {
    return 1;
  }
  M1.write(compute_angle(&dynamics_M1, duration, t));
  M2.write(compute_angle(&dynamics_M2, duration, t));
  M2s.write(compute_angle(&dynamics_M2, duration, t));
  M3.write(compute_angle(&dynamics_M3, duration, t));
  M4.write(compute_angle(&dynamics_M4, duration, t));
  M5.write(compute_angle(&dynamics_M5, duration, t));
  return 0;
}

static void reset_robot(void)
{
  trajectory_t trajectory = {
    .coord = INITIAL_COORD,
    .avg_speed = LOW_SPEED,
    .duration = SMM_TRAJ_DUR
  };
  dynamics_M1.angle_tf = ANGLE_INIT_M1;
  dynamics_M2.angle_tf = ANGLE_INIT_M2;
  dynamics_M3.angle_tf = ANGLE_INIT_M3;
  dynamics_M4.angle_tf = ANGLE_INIT_M4;
  dynamics_M5.angle_tf = ANGLE_INIT_M5;
  compute_motor_dynamics(trajectory.duration, M1.read(), &dynamics_M1);
  compute_motor_dynamics(trajectory.duration, M2.read(), &dynamics_M2);
  compute_motor_dynamics(trajectory.duration, M3.read(), &dynamics_M3);
  compute_motor_dynamics(trajectory.duration, M4.read(), &dynamics_M4);
  compute_motor_dynamics(trajectory.duration, M5.read(), &dynamics_M5);

  int64_t t = 0, t0 = millis();
  while (!update_robot_position(trajectory.duration, t)) {
    t = millis() - t0;
  }
  current_coord = trajectory.coord;
  Serial.print("Success: The robot has been reset.\n");
}

static int8_t follow_path(trajectory_t path[], const uint32_t npoints, const uint16_t dwell_time)
{
  static uint8_t cur_traj_index = 0;
  static int64_t t_origin = millis();
  static bool parameters_determined = false;
  // Compute parameters
  if (!parameters_determined) {
    if (get_angles_from_cartesian(&path[cur_traj_index].coord)) {
      return -1;
    }
    compute_trajectory_duration(&path[cur_traj_index]);
    compute_motor_dynamics(path[cur_traj_index].duration, M1.read(), &dynamics_M1);
    compute_motor_dynamics(path[cur_traj_index].duration, M2.read(), &dynamics_M2);
    compute_motor_dynamics(path[cur_traj_index].duration, M3.read(), &dynamics_M3);
    compute_motor_dynamics(path[cur_traj_index].duration, M4.read(), &dynamics_M4);
    compute_motor_dynamics(path[cur_traj_index].duration, M5.read(), &dynamics_M5);
    parameters_determined = true;
    t_origin = millis();
  }
  // Check the progression of the robot on the path
  int64_t t = millis() - t_origin;
  if (update_robot_position(path[cur_traj_index].duration, t)) {
    current_coord = path[cur_traj_index].coord;
    parameters_determined = false;
    if (++cur_traj_index >= npoints) {
      Serial.print("Success: The path has been completed.\n");
      cur_traj_index = 0;
      return 1;
    }
    delay(dwell_time);
  }
  return 0;
}
