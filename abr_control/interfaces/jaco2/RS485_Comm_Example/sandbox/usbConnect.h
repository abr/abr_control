#include "kinova-api/KinovaTypes.h"
#include "kinova-api/Kinova.API.CommLayerUbuntu.h"
#include "kinova-api/Kinova.API.UsbCommandLayerUbuntu.h"

#include <hiredis/hiredis.h>

#include <iostream>
#include <dlfcn.h> // Ubuntu dynamic load
#include <unistd.h>
#include <time.h>
#include <signal.h>

#include <stdio.h>
#include <stdlib.h>
#include <cstdlib>
#include <vector>
#include <cmath>

// need to go through and add namespace throughout usbConnect code
using namespace std;

/** Global Constants */
const int KINOVA_DOF = 6;
const float KINOVA_ANGLE_OFFSETS[] = {270.0, 180.0, 180.0, 270.0, 180.0, 90.0};
const float KINOVA_ANGLE_SIGN[] = {1.0, -1.0, 1.0, 1.0, 1.0, 1.0};

const float PI=3.141592653589793;

/** Global variable representing the robot's current joint ANGLES.
 * Note that these values are stored in Kinova degrees and using
 * the Kinova's sign convention.
 */
AngularPosition GLOBAL_Kinova_Joint_Angles;

/** Global variable representing the robot's current joint VELOCITIES.
 * Note that these values also use the Kinova's sign convention.
 */
AngularPosition GLOBAL_Kinova_Joint_Velocities;

/** Global variable representing the robot's desired TRAJECTORY. We'll
 * use this to control the fingers.
 */
TrajectoryPoint GLOBAL_Kinova_Trajectory_POINT;

/** Global data structure storing the joint torques currently being
 * imposed upon the robot. Uses the Kinova sign convention.
 */
float GLOBAL_Kinova_Joint_Torques[COMMAND_SIZE];

/** Global constants for REDIS host and port. */
static const std::string REDIS_HOST = "127.0.0.1";
static const int REDIS_PORT = 6379;

/** Global constants for REDIS keys. */
static const std::string JOINT_ANGLES_KEY  = "scl::robot::jaco2::sensors::q";
static const std::string JOINT_ANGLES_DES_KEY  = "scl::robot::jaco2::sensors::q_des";
static const std::string JOINT_VELOCITIES_KEY = "scl::robot::jaco2::sensors::dq";
static const std::string JOINT_TORQUES_KEY = "scl::robot::jaco2::actuators::fgc";

/** Global REDIS interface variables */
redisContext *GLOBAL_Redis_Context;
redisReply *GLOBAL_Redis_Reply;

/** Global variables for REDIS I/O */
static char GLOBAL_Redis_Buffer[1024];
float GLOBAL_Redis_Temp[KINOVA_DOF];
float GLOBAL_Redis_q[KINOVA_DOF];
float GLOBAL_Redis_dq[KINOVA_DOF];

/** This needs to be volatile to support proper exception handling. */
static volatile int glob_exit_condition = 1;

//Functions
void intCtrlCHandler(int sig);
int getRedisOperateMode();
bool initializeRedis();
bool convertKinovaDegreesToSCLRadians(int arg_joint_index,
    float arg_angle_in_degrees, float& ret_angle_in_radians);
bool convertSCLRadiansToKinovaDegrees(int arg_joint_index,
    float arg_angle_in_radians, float& ret_angle_in_degrees);
void sendCurrentTempToRedis(std::string key);
void loadCurrentTempWithRedisContents(std::string key);
bool sendJointAnglesToRedis();
bool sendJointVelocitiesToRedis();
bool readJointAnglesFromRedis();
bool readJointTorquesFromRedis();
void setRedisFgcWritingFlagFalse();
bool getRedisFgcWritingFlag();
int connectToArm();
