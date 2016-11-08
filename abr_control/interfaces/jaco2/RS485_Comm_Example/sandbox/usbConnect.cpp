#include "usbConnect.h"

/* IMPORTANT NOTES
 * ----------------
 * 0. The Kinova's sign convention is such that the second joint angle is flipped from
 *    the direction that SCL has it (so joint angles, velocities, and torques will have
 *    one of the values negated).
 *
 * 1. All REDIS communication should be in SI units consistent with SCL usage
 *    (i.e. all joint angles should be converted to SCL radians and all torques
 *    and velocities should be properly sign-adjusted BEFORE writing these values
 *    to the REDIS key/value store).
 *
 * 2. Joint Angle/Velocity indexing starts at 1 for the Kinova Jaco (for instance,
 *    Actuators.Actuator1 represents the first (normally 0th index) joint angle.
 *    Moreover, there is no array-based storage for this; the joint angle is stored
 *    via individual constants that are fields within the Actuator type.
 *
 * 3. There is a REDIS key called OptionCode that governs the behavior of this app
 *    in the following way:
 *
 *    i.  If OptionCode is 0: This application should relay the joint angles of the
 *        robot (stored currently) to REDIS (after converting).
 *
 *    ii. If OptionCode is 1: This application should simply act as a mirror of the SCL
 *        robot (i.e. read the joint angles from REDIS, convert them, and command the robot
 *        to update its position to the new joint angles).
 *
 *    iii. If the OptionCode is 2: This application should support torque control - send
 *         the robot's current state (joint angles AND joint velocities) to REDIS
 *         and read torques from REDIS, sending the read torque command to the robot.
 */

/** Function: intCtrlCHandler
 * --------------------------
 * CTRL+c signal handler that causes the main execution loop to exit
 * in a graceful manner and allow the robot to return to its home
 * joint angle configuration.
 */
void intCtrlCHandler(int sig)
{
  if (glob_exit_condition == 0){
    exit(1);
  }
  glob_exit_condition = 0;
}

/** Function: getRedisOptionCode
 * ----------------------------------------------------------
 * Get the OptionCode currently stored in redis and return it.
 */
int getRedisOperateMode()
{
  GLOBAL_Redis_Reply = (redisReply *) redisCommand(GLOBAL_Redis_Context, "GET scl::robot::jaco2::operate_mode");
  if(NULL == GLOBAL_Redis_Reply)
  { return -1;  }
  int returnValue = atoi(GLOBAL_Redis_Reply->str);
  cout << "Read Option Code as: " << returnValue << flush << endl;
  freeReplyObject(GLOBAL_Redis_Reply);
  return returnValue;
}

/** Function: initializeRedis
 * --------------------------
 * Initialize a REDIS context and connect to server. Returns true on
 * a successful connect and false on an unsuccessful connect.
 */
bool initializeRedis()
{
  GLOBAL_Redis_Reply = NULL;
  GLOBAL_Redis_Context = redisConnect(REDIS_HOST.c_str(), REDIS_PORT);
  if (GLOBAL_Redis_Context->err) {
    cerr << "Error: " <<  GLOBAL_Redis_Context->errstr << endl;
    return false;
  } else {
    cout << "REDIS Connection Successful.\n" << endl;
    return true;
  }
  redisCommand(GLOBAL_Redis_Context, "SET a 1");
}

/** Function: convertKinovaDegreesToSCLRadians
 * -------------------------------------------
 * This function accepts an integer representing the joint index (indexed
 * from [0] to [KINOVA-DOF - 1]), a float representing the angle in degrees
 * to convert, and a float reference where the converted angle in radians
 * should be stored if the conversion is successful.
 *
 * Return Value: Boolean indicating success/failure of the conversion
 * (failure if the angle argument is NaN, success if the return value
 * argument was updated successfully). Note that if the return value is
 * false, then ret_angle_in_radians is set to zero.
 */
bool convertKinovaDegreesToSCLRadians(int arg_joint_index,
    float arg_angle_in_degrees, float& ret_angle_in_radians)
{
  if(isnan(arg_angle_in_degrees)) {
    ret_angle_in_radians = 0;
    return false;
  } else {
    float offset_angle = KINOVA_ANGLE_OFFSETS[arg_joint_index] - arg_angle_in_degrees;
    ret_angle_in_radians = ((M_PI)/180) * KINOVA_ANGLE_SIGN[arg_joint_index] * offset_angle;
    return true;
  }
}

/** Function: convertSCLRadiansToKinovaDegrees
 * -------------------------------------------
 * This function accepts an integer representing the joint index (indexed
 * from [0] to [KINOVA-DOF - 1]), a float representing the angle in radians
 * to convert, and a float reference where the converted angle in degrees
 * should be stored if the conversion is successful.
 *
 * Return Value: Boolean indicating success/failure of the conversion
 * (failure if the angle argument is NaN, success if the return value
 * argument was updated successfully). Note that if the return value is
 * false, then ret_angle_in_radians is set to zero.
 */
bool convertSCLRadiansToKinovaDegrees(int arg_joint_index,
    float arg_angle_in_radians, float& ret_angle_in_degrees)
{
  if(isnan(arg_angle_in_radians)) {
    ret_angle_in_degrees = 0;
    return false;
  } else {
    float scaled_angle = (180/(M_PI)) * arg_angle_in_radians * KINOVA_ANGLE_SIGN[arg_joint_index];
    ret_angle_in_degrees = KINOVA_ANGLE_OFFSETS[arg_joint_index] - scaled_angle;
    return true;
  }
}

/** Function: sendCurrentTempToRedis
 * ---------------------------------
 * Helper method which writes an array of floats to REDIS. Before this method
 * is called, one should set the GLOBAL_Redis_Buffer variable to contain the
 * desired values to write. The function accepts an argument that is the REDIS
 * key to which values should be written.
 */
void sendCurrentTempToRedis(std::string key)
{
    static char tmp_str[1024];
    sprintf(tmp_str, "%f %f %f %f %f %f", GLOBAL_Redis_Temp[0],
     GLOBAL_Redis_Temp[1], GLOBAL_Redis_Temp[2], GLOBAL_Redis_Temp[3],
      GLOBAL_Redis_Temp[4], GLOBAL_Redis_Temp[5]);
    GLOBAL_Redis_Reply = (redisReply *) redisCommand(GLOBAL_Redis_Context,
        "SET %s %s", key.c_str(), tmp_str);
    freeReplyObject(GLOBAL_Redis_Reply);
}

/** Function: loadCurrentTempWithRedisContents
 * -------------------------------------------
 * Helper method which updates the GLOBAL_Redis_Buffer variable to contain the
 * values stores under the particular REDIS key specified in the funciton argument.
 * The caller should take care to preserve the contents existing in the global, or
 * make sure that they are no longer needed, because this method simply overwrites
 * the existing contents.
 *
 * Moreover, the method returns true if there are no problems reading in the values,
 * but if all 6 values could not be read, it returns false.
 */
void loadCurrentTempWithRedisContents(std::string key)
{
    GLOBAL_Redis_Reply = (redisReply *) redisCommand(GLOBAL_Redis_Context,
        "GET %s", key.c_str());
    sscanf(GLOBAL_Redis_Reply->str, "%f %f %f %f %f %f", &GLOBAL_Redis_Temp[0],
        &GLOBAL_Redis_Temp[1], &GLOBAL_Redis_Temp[2], &GLOBAL_Redis_Temp[3],
        &GLOBAL_Redis_Temp[4], &GLOBAL_Redis_Temp[5]);
    freeReplyObject(GLOBAL_Redis_Reply);
}

/** Function: sendJointAnglesToRedis
 * ----------------------------------
 * Sends the joint angles currently stored in the joint angles global to redis,
 * first converting them and checking that the values are not too large. If either
 * there is an error in the conversion or if the values are too large, then nothing
 * is set to REDIS and the method returns false. If everything is successful, then
 * the method return true.
 */
bool sendJointAnglesToRedis()
{
  // Determine if all Conversions Succeed
  bool flag = true;
  flag = flag && convertKinovaDegreesToSCLRadians(0,
      GLOBAL_Kinova_Joint_Angles.Actuators.Actuator1, GLOBAL_Redis_Temp[0]);
  flag = flag && convertKinovaDegreesToSCLRadians(1,
      GLOBAL_Kinova_Joint_Angles.Actuators.Actuator2, GLOBAL_Redis_Temp[1]);
  flag = flag && convertKinovaDegreesToSCLRadians(2,
      GLOBAL_Kinova_Joint_Angles.Actuators.Actuator3, GLOBAL_Redis_Temp[2]);
  flag = flag && convertKinovaDegreesToSCLRadians(3,
      GLOBAL_Kinova_Joint_Angles.Actuators.Actuator4, GLOBAL_Redis_Temp[3]);
  flag = flag && convertKinovaDegreesToSCLRadians(4,
      GLOBAL_Kinova_Joint_Angles.Actuators.Actuator5, GLOBAL_Redis_Temp[4]);
  flag = flag && convertKinovaDegreesToSCLRadians(5,
      GLOBAL_Kinova_Joint_Angles.Actuators.Actuator6, GLOBAL_Redis_Temp[5]);
  if(!flag) {
    cerr << "Some conversion has failed. Cannot write joint angles." << endl;
    return false;
  }
  // Determine if any Joint Angles are too Large
  if(fabs(GLOBAL_Redis_Temp[0]) > 20 || fabs(GLOBAL_Redis_Temp[1]) > 20 || fabs(GLOBAL_Redis_Temp[2]) > 20 ||
      fabs(GLOBAL_Redis_Temp[3]) > 20 || fabs(GLOBAL_Redis_Temp[4]) > 20 || fabs(GLOBAL_Redis_Temp[5]) > 20)
  {
    cerr << "Some joint angles were too large. Cannot write joint angles" << endl;
    return false;
  } else {
    // If the values are fine, send them to REDIS
    sendCurrentTempToRedis(JOINT_ANGLES_KEY);
    return true;
  }
}

/** Function: sendJointVelocitiesToRedis
 * ----------------------------------
 * Sends the joint velocities currently stored in the joint angles global to redis,
 * first converting them and [TODO!]{checking that the values are not too large}. If
 * the values are too large, then nothing is set to REDIS and the method returns false.
 * If everything is successful, then the method return true.
 */
bool sendJointVelocitiesToRedis()
{
  // Load velocities into temp variable
  GLOBAL_Redis_Temp[0] = ((M_PI/180) * -1) * KINOVA_ANGLE_SIGN[0] * GLOBAL_Kinova_Joint_Velocities.Actuators.Actuator1;
  GLOBAL_Redis_Temp[1] = ((M_PI/180) * -1) * KINOVA_ANGLE_SIGN[1] * GLOBAL_Kinova_Joint_Velocities.Actuators.Actuator2;
  GLOBAL_Redis_Temp[2] = ((M_PI/180) * -1) * KINOVA_ANGLE_SIGN[2] * GLOBAL_Kinova_Joint_Velocities.Actuators.Actuator3;
  GLOBAL_Redis_Temp[3] = ((M_PI/180) * -1) * KINOVA_ANGLE_SIGN[3] * GLOBAL_Kinova_Joint_Velocities.Actuators.Actuator4;
  GLOBAL_Redis_Temp[4] = ((M_PI/180) * -1) * KINOVA_ANGLE_SIGN[4] * GLOBAL_Kinova_Joint_Velocities.Actuators.Actuator5;
  GLOBAL_Redis_Temp[5] = ((M_PI/180) * -1) * KINOVA_ANGLE_SIGN[5] * GLOBAL_Kinova_Joint_Velocities.Actuators.Actuator6;
  // TODO: Add a check to determine if velocities being sent are too large
  sendCurrentTempToRedis(JOINT_VELOCITIES_KEY);
  return true;
}

/** Function: readJointAnglesFromRedis
 * -----------------------------------
 * Reads the joint angles currently stored in REDIS and writing them to the state,
 * first converting them and checking that the values are not too large. If either
 * there is an error in the conversion or if the values are too large, then the state
 * is not set and the method returns false. If everything is successful, then
 * the method returns true.
 */
bool readJointAnglesFromRedis()
{
  loadCurrentTempWithRedisContents(JOINT_ANGLES_KEY);
  // Determine if any Joint Angles are too Large
  if(fabs(GLOBAL_Redis_Temp[0]) > 20 || fabs(GLOBAL_Redis_Temp[1]) > 20 || fabs(GLOBAL_Redis_Temp[2]) > 20 ||
      fabs(GLOBAL_Redis_Temp[3]) > 20 || fabs(GLOBAL_Redis_Temp[4]) > 20 || fabs(GLOBAL_Redis_Temp[5]) > 20)
  {
    cerr << "Some joint angles read were too large. Did not update state." << endl;
    return false;
  }

  // Determine if all Conversions Succeed
  bool flag = true;
  flag = flag && convertSCLRadiansToKinovaDegrees(0,
      GLOBAL_Redis_Temp[0], GLOBAL_Kinova_Joint_Angles.Actuators.Actuator1);
  flag = flag && convertSCLRadiansToKinovaDegrees(1,
      GLOBAL_Redis_Temp[1], GLOBAL_Kinova_Joint_Angles.Actuators.Actuator2);
  flag = flag && convertSCLRadiansToKinovaDegrees(2,
      GLOBAL_Redis_Temp[2], GLOBAL_Kinova_Joint_Angles.Actuators.Actuator3);
  flag = flag && convertSCLRadiansToKinovaDegrees(3,
      GLOBAL_Redis_Temp[3], GLOBAL_Kinova_Joint_Angles.Actuators.Actuator4);
  flag = flag && convertSCLRadiansToKinovaDegrees(4,
      GLOBAL_Redis_Temp[4], GLOBAL_Kinova_Joint_Angles.Actuators.Actuator5);
  flag = flag && convertSCLRadiansToKinovaDegrees(5,
      GLOBAL_Redis_Temp[5], GLOBAL_Kinova_Joint_Angles.Actuators.Actuator6);
  if(!flag) {
    cerr << "Some conversion has failed. Did not update state." << endl;
    return false;
  } else {
    return true;
  }
}

/** Function: readJointTorquesFromRedis
 * -----------------------------------
 * Reads the joint torques currently stored in REDIS and writing them to the command,
 * first converting them and checking that the values are not too large. If the values
 * are too large, then the torque command is not set and the method returns false.
 * If everything is successful, then the method returns true.
 */
bool readJointTorquesFromRedis()
{
  loadCurrentTempWithRedisContents(JOINT_TORQUES_KEY);
  // Determine if any Joint Torques are too Large
  if(fabs(GLOBAL_Redis_Temp[0]) > 200 || fabs(GLOBAL_Redis_Temp[1]) > 200 || fabs(GLOBAL_Redis_Temp[2]) > 200 ||
      fabs(GLOBAL_Redis_Temp[3]) > 200 || fabs(GLOBAL_Redis_Temp[4]) > 200 || fabs(GLOBAL_Redis_Temp[5]) > 200)
  {
    cerr << "Some joint torques read were too large. Did not update command." << endl;
    return false;
  }
  int id = 0;
  for(int id = 0; id < KINOVA_DOF; ++id) {
    GLOBAL_Kinova_Joint_Torques[id] = (-1 * KINOVA_ANGLE_SIGN[id]) * GLOBAL_Redis_Temp[id];
  }
  return true;
}

void setRedisFgcWritingFlagFalse() {
  GLOBAL_Redis_Reply = (redisReply *) redisCommand(GLOBAL_Redis_Context, "SET scl::robot::jaco2::fgc_command_enabled 0");
  freeReplyObject(GLOBAL_Redis_Reply);
}

bool getRedisFgcWritingFlag() {
  GLOBAL_Redis_Reply = (redisReply *) redisCommand(GLOBAL_Redis_Context, "GET scl::robot::jaco2::fgc_command_enabled");
  int returnValue = atoi(GLOBAL_Redis_Reply->str);
  if(returnValue == 1) return true;
  return false;
}

int connectToArm()
{
  std::cout<<"\n Kinova redis driver"
      <<"\n Operates the kinova using commands provided from a redis interface."
      <<"\n Requires the following redis keys to work:"
      <<"\n  scl::robot::jaco2::sensors::q (say '0 0 0 0 0 0') "
      <<"\n  scl::robot::jaco2::sensors::dq (say '0 0 0 0 0 0') "
      <<"\n  scl::robot::jaco2::actuators::fgc (say '0 0 0 0 0 0') "
      <<"\n  scl::robot::jaco2::operate_mode {0: q-reader, 1: q-command, 2: fgc-command}"
      <<"\n  scl::robot::jaco2::fgc_command_enabled {0, 1}"
      << endl;

  // Install Signal Handler
  signal(SIGINT, intCtrlCHandler);

  //Handle for the library's command layer.
  void * commandLayer_handle;

  // Establish function pointers for the functions that we need.
  int (*MyInitAPI)();
  int (*MyCloseAPI)();
  int (*MyGetAngularCommand)(AngularPosition &);
  int (*MyGetAngularPosition)(AngularPosition &);
  int (*MyGetAngularVelocity)(AngularPosition &);
  int (*MyGetAngularForce)(AngularPosition &Response);
  int (*MyGetAngularForceGravityFree)(AngularPosition &Response);
  int (*MyGetDevices)(KinovaDevice devices[MAX_KINOVA_DEVICE], int &result);
  int (*MySetActiveDevice)(KinovaDevice device);
  int (*MySendBasicTrajectory)(TrajectoryPoint command);
  int (*MySendAdvanceTrajectory)(TrajectoryPoint command);
  int (*MyEraseAllTrajectories)();
  int(*MySetGravityType)(GRAVITY_TYPE Type);
  int(*MySendAngularTorqueCommand)(float Command[COMMAND_SIZE]);
  int(*MyGetAngularTorqueCommand)(float Command[COMMAND_SIZE]);
  int(*MySetGravityOptimalZParam)(float Command[GRAVITY_PARAM_SIZE]);
  int(*MySetTorqueControlType)(TORQUECONTROL_TYPE type);
  int(*MySetTorqueVibrationController)(float value);
  int(*MySwitchTrajectoryTorque)(GENERALCONTROL_TYPE);
  int(*MySetTorqueSafetyFactor)(float factor);
  int(*MyMoveHome)();

  // Load the library.
  commandLayer_handle = dlopen("kinova-api/Kinova.API.USBCommandLayerUbuntu.so",RTLD_NOW|RTLD_GLOBAL);

  // Load the functions from the library (Under Windows, use GetProcAddress).
  MyInitAPI = (int (*)()) dlsym(commandLayer_handle,"InitAPI");
  if(MyInitAPI == NULL) {cerr << "Initialization Failed" << endl; exit(1);}
  MyCloseAPI = (int (*)()) dlsym(commandLayer_handle,"CloseAPI");
  if(MyCloseAPI == NULL) {cerr << "Initialization Failed" << endl; exit(1);}
  MyGetAngularCommand = (int (*)(AngularPosition &)) dlsym(commandLayer_handle,"GetAngularCommand");
  if(MyGetAngularCommand == NULL) {cerr << "Initialization Failed" << endl; exit(1);}
  MyGetAngularPosition = (int (*)(AngularPosition &)) dlsym(commandLayer_handle,"GetAngularPosition");
  if(MyGetAngularPosition == NULL) {cerr << "Initialization Failed" << endl; exit(1);}
  MyGetAngularVelocity = (int (*)(AngularPosition &)) dlsym(commandLayer_handle,"GetAngularVelocity");
  if(MyGetAngularVelocity == NULL) {cerr << "Initialization Failed" << endl; exit(1);}
  MyGetAngularForce = (int (*)(AngularPosition &Response)) dlsym(commandLayer_handle,"GetAngularForce");
  if(MyGetAngularForce == NULL) {cerr << "Initialization Failed" << endl; exit(1);}
  MyGetAngularForceGravityFree = (int (*)(AngularPosition &Response)) dlsym(commandLayer_handle,"GetAngularForceGravityFree");
  if(MyGetAngularForceGravityFree == NULL) {cerr << "Initialization Failed" << endl; exit(1);}
  MyGetDevices = (int (*)(KinovaDevice devices[MAX_KINOVA_DEVICE], int &result)) dlsym(commandLayer_handle,"GetDevices");
  if(MyGetDevices == NULL) {cerr << "Initialization Failed" << endl; exit(1);}
  MySetActiveDevice = (int (*)(KinovaDevice devices)) dlsym(commandLayer_handle,"SetActiveDevice");
  if(MySetActiveDevice == NULL) {cerr << "Initialization Failed" << endl; exit(1);}
  MySendBasicTrajectory = (int (*)(TrajectoryPoint)) dlsym(commandLayer_handle,"SendBasicTrajectory");
  if(MySendBasicTrajectory == NULL) {cerr << "Initialization Failed" << endl; exit(1);}
  MySendAdvanceTrajectory = (int (*)(TrajectoryPoint)) dlsym(commandLayer_handle,"SendAdvanceTrajectory");
  if(MySendAdvanceTrajectory == NULL) {cerr << "Initialization Failed" << endl; exit(1);}
  MyEraseAllTrajectories = (int (*)()) dlsym(commandLayer_handle,"EraseAllTrajectories");
  if(MyEraseAllTrajectories == NULL) {cerr << "Initialization Failed" << endl; exit(1);}
  MySetGravityType = (int(*)(GRAVITY_TYPE Type)) dlsym(commandLayer_handle, "SetGravityType");
  if(MySetGravityType == NULL) {cerr << "Initialization Failed" << endl; exit(1);}
  MySendAngularTorqueCommand = (int(*)(float Command[COMMAND_SIZE])) dlsym(commandLayer_handle, "SendAngularTorqueCommand");
  if(MySendAngularTorqueCommand == NULL) {cerr << "Initialization Failed" << endl; exit(1);}
  MyGetAngularTorqueCommand = (int(*)(float Command[COMMAND_SIZE])) dlsym(commandLayer_handle, "GetAngularTorqueCommand");
  if(MyGetAngularTorqueCommand == NULL) {cerr << "Initialization Failed" << endl; exit(1);}
  MySetGravityOptimalZParam = (int(*)(float Command[GRAVITY_PARAM_SIZE])) dlsym(commandLayer_handle, "SetGravityOptimalZParam");
  if(MySetGravityOptimalZParam == NULL) {cerr << "Initialization Failed" << endl; exit(1);}
  MySetTorqueVibrationController = (int(*)(float)) dlsym(commandLayer_handle, "SetTorqueVibrationController");
  if(MySetTorqueVibrationController == NULL) {cerr << "Initialization Failed" << endl; exit(1);}
  MySetTorqueControlType = (int(*)(TORQUECONTROL_TYPE)) dlsym(commandLayer_handle, "SetTorqueControlType");
  if(MySetTorqueControlType == NULL) {cerr << "Initialization Failed" << endl; exit(1);}
  MySetTorqueSafetyFactor = (int(*)(float)) dlsym(commandLayer_handle, "SetTorqueSafetyFactor");
  if(MySetTorqueSafetyFactor == NULL) {cerr << "Initialization Failed" << endl; exit(1);}
  MySwitchTrajectoryTorque = (int(*)(GENERALCONTROL_TYPE)) dlsym(commandLayer_handle, "SwitchTrajectoryTorque");
  if(MySwitchTrajectoryTorque == NULL) {cerr << "Initialization Failed" << endl; exit(1);}
  MyMoveHome = (int(*)()) dlsym(commandLayer_handle, "MoveHome");
  if(MyMoveHome == NULL) {cerr << "Initialization Failed" << endl; exit(1);}

  // Print status message
  cout << "Initialization Process Completed." << endl << endl;

  // Run a basic test to determine if initialization worked.

  AngularPosition DataCommand;
  int result = (*MyInitAPI)();
  int resultComm = MyGetAngularCommand(DataCommand);
  cout << "Initialization's Result :" << result << endl;
  cout << "Communication Result :" << resultComm << endl;
  // If the API is initialized and the communication with the robot is working
  if (result == 1 && resultComm == 1)
  {
    cout << "Basic API Test Worked." << flush << endl;
  } else {
    cout << "Basic API Test Failed." << flush << endl;
    exit(1);
  }

  /****************************************/
  /** Begin execution of the main program */
  /****************************************/

  // Set to position mode
  MySwitchTrajectoryTorque(POSITION);
  cout << "Set to Position Mode" << flush << endl;

  // Move to home position
  MyMoveHome();
  cout << "Moved to Home Position" << flush << endl;

  // Initialize REDIS and set the option code
  if(!initializeRedis()) {exit(1);}
  int operateCode = getRedisOperateMode();
  cout << "Entering Execution Loop..." << endl;

  if(0 > operateCode)
  {
    std::cout<<"\n ERROR : the redis key scl::robot::jaco2::operate_mode does not exist. Returned operate code: "<<operateCode;
    std::cout<<"\n scl::robot::jaco2::operate_mode 0 : Joint angle read mode (does not command robot)";
    std::cout<<"\n scl::robot::jaco2::operate_mode 1 : Joint angle command mode (commands joint traj to robot)";
    std::cout<<"\n scl::robot::jaco2::operate_mode 2 : Joint torque mode";
    std::cout<<"\n"<<std::endl;
    return -1;
  }

  /*if(operateCode == 0)
  {
    // Loop Indefinitely
    while(glob_exit_condition)
    {
      MyGetAngularPosition(GLOBAL_Kinova_Joint_Angles);
      if(!sendJointAnglesToRedis()) {exit(1);}
    }
  }*/
  
  /*else if(operateCode == 1)
  {
    // Initialize Struct for Joint-Space Trajectory
    TrajectoryPoint point_to_send;
    point_to_send.InitStruct();
    point_to_send.Position.Type = ANGULAR_POSITION;

    // Loop Indefinitely
    while(glob_exit_condition)
    {
      if(!readJointAnglesFromRedis()) {exit(1);}
      point_to_send.Position.Actuators = GLOBAL_Kinova_Joint_Angles.Actuators;
      MySendAdvanceTrajectory(point_to_send);
      // "MyEraseAllTrajectories(); MySendBasicTrajectory(point_to_send);" might also work /
      usleep(50000);//Sleep for 50ms
    }
  }*/
  
  /*else if(operateCode == 2)
  {
    // Set the Optimal parameters obtained from the identification sequence
    float OptimalParam[OPTIMAL_Z_PARAM_SIZE] = {1.26626, 0.0348745, -0.018775, -1.16841, 0.00879105, 0.571748, 0.00526145, 0.187221, -0.00482375, -0.00532718, 0.252576, 0.103216, 0.537339, 0.0634819, -0.0882852, 0.0112307};
    // Set the gravity mode to Manual input
    MySetGravityOptimalZParam(OptimalParam);
    // Set gravity type to optimal
    MySetGravityType(OPTIMAL);

    cout << "Gravity Optimal Parameters Set" << flush << endl;

    // Set the torque control type to Direct Torque Control
    int ret_status;
    ret_status = MySwitchTrajectoryTorque(TORQUE);
    cout << "Switching to torque control mode. Status :" << ret_status << endl;
    ret_status = MySetTorqueControlType(DIRECTTORQUE);
    cout << "Setting torque control type. Status :" << ret_status << endl;

    // Set the safety factor off
    MySetTorqueSafetyFactor(1);
    // Set the vibration controller off
    MySetTorqueVibrationController(0);

    cout << "Finally switching to Direct Torque Control" << flush << endl;
    for(int i = 0; i < 6; i++) GLOBAL_Kinova_Joint_Torques[i] = 0;
    setRedisFgcWritingFlagFalse();

    // Wait until torques have been populated

    long long loopCounter = 0;
    while(glob_exit_condition)
    {
      // Update Joint Angles/Velocities in Program State
      MyGetAngularPosition(GLOBAL_Kinova_Joint_Angles);
      MyGetAngularVelocity(GLOBAL_Kinova_Joint_Velocities);

      // Update Joint Angles/Velocities in REDIS Key-Value Cache
      sendJointAnglesToRedis();
      sendJointVelocitiesToRedis();

      // Fetch Torques
      if(getRedisFgcWritingFlag())
      {	readJointTorquesFromRedis();	
        cout << "fgc writing flag is TRUE" << endl;
      }
      else // Send zero torques..
      { for(int i = 0; i < 6; i++) GLOBAL_Kinova_Joint_Torques[i] = 0;  
        cout << "fgc writing flag is FALSE" << endl;
      }

      MySendAngularTorqueCommand(GLOBAL_Kinova_Joint_Torques);
      loopCounter++;
    }
  }*/
  else
  { std::cout<<"\n ERROR : Found invalid op-code in redis. Terminating.\n\n"; }

  MySwitchTrajectoryTorque(POSITION);
  MyMoveHome();
  cout << endl << "Moved Home." << endl;
  result = (*MyCloseAPI)();
  dlclose(commandLayer_handle);
  cout << "Closed the API." << endl;
  exit(0);
}
