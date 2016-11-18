#ifndef KINOVA_TYPE_H_
#define KINOVA_TYPE_H_

/**
 * @file KinovaTypes.h
 * @brief This file contains all data structures and all data type(enum and typedef) that you'll need to use this API.
 */

/**
 * @brief Size of the @link ControlsModeMap @endlink array in the structure @link JoystickCommand @endlink.
 */
#define JOYSTICK_BUTTON_COUNT 16

/**
 * @brief Max size of the advance retract trajectory that is stored in the @link ClientConfigurations @endlink.
 */
#define NB_ADVANCE_RETRACT_POSITION		20

/**
 * @brief This is the size of the data array stored in a @link SystemError @endlink object.
 */
#define ERROR_DATA_COUNT_MAX 	50

/**
 * @brief The robot's firmware has several software layers. This describes how many layer there is in the firmware.
 */
#define ERROR_LAYER_COUNT 7

/**
 * @brief This represents the max count of protection zones that can be stored in the robot's memory.
 */
#define LEGACY_CONFIG_NB_ZONES_MAX		10

/**
 * @brief This is the size of the array Points in a @link ZoneShape @endlink .
 */
#define LEGACY_CONFIG_NB_POINTS_COUNT   8

/**
 * @brief This is the size of the array Mapping contained in the structure @link ControlMappingCharts @endlink .
 */
#define CONTROL_MAPPING_COUNT 6

/**
 * @brief This is the size of the arrays ModeControlsA and ModeControlsB contained in the structure @link ControlMapping @endlink .
 */
#define MODE_MAP_COUNT 6

/**
 * @brief This is the size of the array ControlSticks contained in the structure @link ControlsModeMap @endlink .
 */
#define STICK_EVENT_COUNT 6

/**
 * @brief This is the size of the array ControlButtons contained in the structure @link ControlsModeMap @endlink .
 */
#define BUTTON_EVENT_COUNT 26

/**
 * @brief This is the size of all strings stored in the robot's firmware.
 */
#define STRING_LENGTH 20

/**
 * @brief This is the max finger count in a robot. (Jaco has 3 fingers and Mico has 2 fingers)
 */
#define JACO_FINGERS_COUNT 3

/**
 * @brief This is an error code. It means that the file you are trying to interact with does not exist
 * or is corrupted. Either way, the OS does not recognise it.
 */
#define ERROR_UNKNOWFILE 5001

/**
 * @brief This is an error code. It means that there was a memory related error. Most of the time it is because
 * the system does not have enough memory.
 */
#define ERROR_MEMORY 5002

/**
 * @brief This is an error code. It means that the function has some problem reading a file. Most of the time
 * it is because the user don't have the privileges to do it.
 */
#define ERROR_FILEREADING 5003

/**
 * @brief This represents the size of a memory page used to program the robot.
 */
const unsigned short PAGE_SIZE = 2048;

/**
 * @brief This represents the size of a page's address
 */
const int ADDRESS_PAGE_SIZE = 4;

/**
 * @brief This represents the quantity of USB packet stored in a memory page.
 */
const unsigned short PACKET_PER_PAGE_QTY = 40;

/**
 * @brief That represents the data's size of each USB packet during firmware update.
 */
const int PAGEPACKET_SIZE = 52;

/**
 * @brief That represents the size of a USB packet's header.
 */
const int USB_HEADER_SIZE = 8;

/**
 * @brief That represents the data's size of a normal USB packet.
 */
const int USB_DATA_SIZE = 56;

/**
 * @brief That represents the type of a position. If used during a trajectory, the type of position
 * will change the behaviour of the robot. For example if the position type is CARTESIAN_POSITION,
 * then the robot's end effector will move to that position using the inverse kinematics. But
 * if the type of position is CARTESIAN_VELOCITY then the robot will use the values as velocity command.
 */
enum POSITION_TYPE
{
	NOMOVEMENT_POSITION = 0,    /*!< Used for initialisation. */
	CARTESIAN_POSITION = 1,     /*!< A cartesian position described by a translation X, Y, Z and an orientation ThetaX, thetaY and ThetaZ. */
	ANGULAR_POSITION = 2,       /*!< An angular position described by a value for each actuator. */
	RETRACTED = 3,              /*!< The robotic arm is in retracted mode. It may be anywhere between the HOME position and the RETRACTED position. */
	PREDEFINED1 = 4,   			/*!< The robotic arm is moving to the pre defined position #1. */
	PREDEFINED2 = 5,   			/*!< The robotic arm is moving to the pre defined position #2. */
	PREDEFINED3 = 6, 			/*!< The robotic arm is moving to the pre defined position #3. */
	CARTESIAN_VELOCITY = 7,     /*!< A velocity vector used for velocity control. */
	ANGULAR_VELOCITY = 8,       /*!< Used for initialisation. */
	PREDEFINED4 = 9,  			/*!< The robotic arm is moving to the pre defined position #4. */
	PREDEFINED5 = 10,  			/*!< The robotic arm is moving to the pre defined position #5. */
	ANY_TRAJECTORY = 11,        /*!< Not used. */
	TIME_DELAY = 12,            /*!< The robotic arm is on time delay. */
};



/**
 * @brief That represents the type of port from a peripheral.
 */
enum PORT_TYPE
{
	PERIPHERAL_PORT_ANY = 0,                      /*!< generic port. */
	PERIPHERAL_PORT_CAN_INTERNAL = 1,             /*!< Internal CAN port. */
	PERIPHERAL_PORT_PORT_CAN_EXTERNAL = 2,        /*!< External CAN port. */
	PERIPHERAL_PORT_PORT_SPI_0 = 3,               /*!< SPI 0 port. */
	PERIPHERAL_PORT_PORT_SPI_1 = 4,               /*!< SPI 1 port. */
	PERIPHERAL_PORT_PORT_USB = 5,                 /*!< USB port. */
	PERIPHERAL_PORT_PORT_UART_0 = 6,              /*!< UART 0 port. */
	PERIPHERAL_PORT_PORT_UART_1 = 7,              /*!< UART 1 port. */
	PERIPHERAL_PORT_PORT_UART_2 = 8,              /*!< UART 2 port. */
	PERIPHERAL_PORT_PORT_VIRTUAL = 9,             /*!< Virtual port. */
};

/**
 * @brief That represents the type of a peripheral.
 */
enum PERIPHERAL_TYPE
{
	PERIPHERAL_TYPE_NONE = 0,                      /*!< Unknown type. */
	PERIPHERAL_TYPE_ANY = 1,                       /*!< Abstract peripheral. internal use only.*/
	PERIPHERAL_TYPE_UNKNOWN = 2,                   /*!< Unknown peripheral. internal use only.*/
	PERIPHERAL_TYPE_ACTUATOR_GENERIC = 100,        /*!< A generic rotary actuator. */
	PERIPHERAL_TYPE_ACTUATOR_BIG_19NM = 101,       /*!< A big 19 Newton*Meter rotary actuator. */
	PERIPHERAL_TYPE_ACTUATOR_BIG_37NM = 102,       /*!< A big 37 Newton*Meter rotary actuator. */
	PERIPHERAL_TYPE_ACTUATOR_SMALL_7NM = 103,      /*!< A small 7 Newton*Meter rotary actuator. */
	PERIPHERAL_TYPE_LINEAR_ACTUATOR_GENERIC = 200, /*!< A generic linear actuator. */
	PERIPHERAL_TYPE_LINEAR_ACTUATOR_120N = 201,    /*!< A 120 Newton(finger) linear actuator. */
	PERIPHERAL_TYPE_JOYSTICK = 300,                /*!< A generic joystick. */
	PERIPHERAL_TYPE_VIRTUAL_JOYSTICK = 301,        /*!< A virtual joystick. This is mainly used by the API. */
	PERIPHERAL_TYPE_KINOVA_JOYSTICK_3AXIS = 302,   /*!< A kinovian joystick. */
	PERIPHERAL_TYPE_UNIVERSAL_INTERFACE_V2 = 303,  /*!< A universal interface V2. */
	PERIPHERAL_TYPE_CAN_INTERFACE = 400            /*!< A CAN interface on the main board. */
};

/**
 * @brief That indicates how the end effector will be used.
 */
enum HAND_MODE
{
	HAND_NOMOVEMENT, /*!< Fingers will not move. */
	POSITION_MODE,   /*!< Fingers will move using position control. */
	VELOCITY_MODE,   /*!< Fingers will move using velocity control. */
};

/**
 * @brief That indicates if the robot will be left handed or right handed.
 */
enum ArmLaterality
{
	RIGHTHAND, /*!< Right handed */
	LEFTHAND,  /*!< Left handed */
};

enum TORQUECONTROL_TYPE
{
	DIRECTTORQUE = 0,
	IMPEDANCEANGULAR = 1,
	IMPEDANCECARTESIAN = 2
};

enum GENERALCONTROL_TYPE
{
	POSITION, /*!< General position control. That include velocity control. */
	  TORQUE, /*!< General torque control. */
};

/**
 * @brief This represents a type of controller. A controller is an entity that can send control
 * commands to the robot.
 */
enum Controller
{
	     THREE_AXIS_JOYSTICK = 0, /*!< A three axis joystick controller. */
	       TWO_AXIS_JOYSTICK = 1, /*!< A two axis joystick controller. */
	                     API = 2, /*!< The kinova API. */
	              EASY_RIDER = 3, /*!< The easy rider controller. */
	     UNIVERSAL_INTERFACE = 4, /*!< The kinova universal interface controller. */
	EXTERNAL_CUSTOMINTERFACE = 5, /*!< An external custom interface controller. */
	                    NONE = 6, /*!< No interface. */
	            OLED_DISPLAY = 7  /*!< An OLED display. */
};

/**
 * @brief This represents a type of control. For now, there is 3 type of control, the
 * cartesian control, the angular control or the force control.
 */
enum CONTROL_TYPE
{
	CONTROL_TYPE_CARTESIAN = 0, /*!< Cartesian control. (translation and orientation) */
	CONTROL_TYPE_ANGULAR = 1,    /*!< Angular control. (joint by joint) */

	CONTROL_TYPE_UNKNOWN = 9999,    /*!< Unknown control type. */
};

/**
 * @brief That describes a control module of the robotical arm's firmware.
 */
enum CONTROL_MODULE
{
	/**
	 * @brief No control module selected. The robotical arm cannot moves.
	 */
	CONTROL_MODULE_NONE,

	/**
	 * @brief Angular velocity control mode. Values sent to the actuators are velocity. Unit: degree / second
	 */
	CONTROL_MODULE_ANGULAR_VELOCITY,

	/**
	 * @brief Angular position control mode. Values sent to the actuators are position. Unit: degree
	 */
	CONTROL_MODULE_ANGULAR_POSITION,

	/**
	 * @brief Cartesian velocity control mode. values sent to the end effector are velocity. translation Unit: meter / second,
	 * orientation unit: RAD / second.
	 */
	CONTROL_MODULE_CARTESIAN_VELOCITY,

	/**
	 * @brief Cartesian position control mode. values sent to the actuators are velocity. translation Unit: meter,
	 * orientation unit: RAD.
	 */
	CONTROL_MODULE_CARTESIAN_POSITION,

	/**
	 * @brief Retract control mode. This manage movement between the READY(HOME) and the RETRACTED position.
	 * This can be angular or cartesian position control.
	 */
	CONTROL_MODULE_RETRACT,

	/**
	 * @brief Not used for now.
	 */
	CONTROL_MODULE_TRAJECTORY,

	/**
	 * @brief This manages the pre programmed position(GOTO). This is position control.
	 */
	CONTROL_MODULE_PREDEFINED,

	/**
	 * @brief This manages the time delay during a trajectory.
	 */
	CONTROL_MODULE_TIMEDELAY,
};

/**
 * @brief This describes the retract type the robotical arm.
 */
enum RETRACT_TYPE
{
	/**
	 * @brief The robotical arm was in a normal custom position and is going toward the READY position.
	 */
	 RETRACT_TYPE_NORMAL_TO_READY = 0,

	 /**
	  * @brief The robotical arm is in READY position and is waiting for a command.
	  */
	   RETRACT_TYPE_READY_STANDBY = 1,

	 /**
	  * @brief The robotical arm was in READY position and is going toward the RETRACTED position.
	  */
	RETRACT_TYPE_READY_TO_RETRACT = 2,

	/**
	  * @brief The robotical arm was in RETRACT position and is waiting for a command.
	  */
	 RETRACT_TYPE_RETRACT_STANDBY = 3,

	 /**
	  * @brief The robotical arm was in RETRACT position and is going toward the READY position.
	  */
	RETRACT_TYPE_RETRACT_TO_READY = 4,

	/**
	 * @brief The robotical arm is initialized and is anywhere in the workspace but it is not retracted, in READY position or between.
	 */
	  RETRACT_TYPE_NORMAL_STANDBY = 5,

	  /**
	  * @brief The robotical arm is not initialized.
	  */
	 RETRACT_TYPE_NOT_INITIALIZED = 6,

	 /**
	  * @brief An error has occured during the retract process.
	  */
	 RETRACT_ERROR = 25000
};

/** @brief This data structure holds values in an angular(joint by joint) control context. As an example struct could contains position, temperature, torque, ...
 *  \struct AngularInfo KinovaTypes.h "Definition"
 */
struct AngularInfo
{
	/**
	 * As an example if the current control mode is angular position the unit will be degree but if the control mode is angular velocity
	 * then the unit will be degree per second.
	 * @brief This is the value related to the first actuator. Unit depends on the context it's been used.
	 */
	float Actuator1;

	/**
	 * As an example if the current control mode is angular position the unit will be degree but if the control mode is angular velocity
	 * then the unit will be degree per second.
	 * @brief This is the value related to the second actuator. Unit depends on the context it's been used.
	 */
	float Actuator2;

	/**
	 * As an example if the current control mode is angular position the unit will be degree but if the control mode is angular velocity
	 * then the unit will be degree per second.
	 * @brief This is the value related to the third actuator. Unit depends on the context it's been used.
	 */
	float Actuator3;

	/**
	 * As an example if the current control mode is angular position the unit will be degree but if the control mode is angular velocity
	 * then the unit will be degree per second.
	 * @brief This is the value related to the actuator #4. Unit depends on the context it's been used.
	 */
	float Actuator4;

	/**
	 * As an example if the current control mode is angular position the unit will be degree but if the control mode is angular velocity
	 * then the unit will be degree per second.
	 * @brief This is the value related to the actuator #5. Unit depends on the context it's been used.
	 */
	float Actuator5;

	/**
	 * As an example if the current control mode is angular position the unit will be degree but if the control mode is angular velocity
	 * then the unit will be degree per second.
	 * @brief This is the value related to the actuator #6. Unit depends on the context it's been used.
	 */
	float Actuator6;


	/**
	 * This method will initialises all the values to 0
	 */
	void InitStruct()
	{
		Actuator1 = 0.0f;
		Actuator2 = 0.0f;
		Actuator3 = 0.0f;
		Actuator4 = 0.0f;
		Actuator5 = 0.0f;
		Actuator6 = 0.0f;
	}
};

/** @brief This data structure holds values in an cartesian control context.
 *  \struct CartesianInfo KinovaTypes.h "Definition"
 */
struct CartesianInfo
{
	/**
	 * As an example if the current control mode is cartesian position the unit will be meters but if the control mode is cartesian velocity
	 * then the unit will be meters per second.
	 * @brief This is the value related to the translation along the X axis. Unit depends on the context it's been used.
	 */
	float X;

	/**
	 * As an example if the current control mode is cartesian position the unit will be meters but if the control mode is cartesian velocity
	 * then the unit will be meters per second.
	 * @brief This is the value related to the translation along the Y axis. Unit depends on the context it's been used.
	 */
	float Y;

	/**
	 * As an example if the current control mode is cartesian position the unit will be meters but if the control mode is cartesian velocity
	 * then the unit will be meters per second.
	 * @brief This is the value related to the translation along the Z axis. Unit depends on the context it's been used.
	 */
	float Z;

	/**
	 * This is the value related to the orientation around the X axis. Depending on the context it's been used, the unit of the value will change.
	 * As an example if the current control mode is cartesian position the unit will be RAD but if the control mode is cartesian velocity
	 * then the unit will be RAD per second.
	 * @brief This is the value related to the orientation around the X axis. Unit depends on the context it's been used.
	 */
	float ThetaX;

	/**
	 * As an example if the current control mode is cartesian position the unit will be RAD but if the control mode is cartesian velocity
	 * then the unit will be RAD per second.
	 * @brief This is the value related to the orientation around the Y axis. Unit depends on the context it's been used.
	 */
	float ThetaY;

	/**
	 * As an example if the current control mode is cartesian position the unit will be RAD but if the control mode is cartesian velocity
	 * then the unit will be RAD per second.
	 * @brief This is the value related to the orientation around the Z axis. Unit depends on the context it's been used.
	 */
	float ThetaZ;


	/**
	 * This method will initialises all the values to 0
	 */
	void InitStruct()
	{
		X = 0.0f;
		Y = 0.0f;
		Z = 0.0f;
		ThetaX = 0.0f;
		ThetaY = 0.0f;
		ThetaZ = 0.0f;
	}
};

/**
 * If the robot is a Jaco, the 3 finger values will be filled but if the robot is a Mico, the finger 3 will not be filled.
 * @brief This data structure holds the values of the robot's sensors.
 *  \struct SensorsInfo KinovaTypes.h "Definition"
 */
struct SensorsInfo
{
	/**
	 * @brief This is the main power supply voltage. (24 V) Unit is V.
	 */
	float Voltage;

	/**
	 * @brief This is the main power supply's current. Unit is A.
	 */
	float Current;

	/**
	 * @brief This is the value read by the acceleration sensor on the X axis. Unit is G.
	 */
	float AccelerationX;

	/**
	 * @brief This is the value read by the acceleration sensor on the Y axis. Unit is G.
	 */
	float AccelerationY;

	/**
	 * @brief This is the value read by the acceleration sensor on the Z axis. Unit is G.
	 */
	float AccelerationZ;

	/**
	 * @brief This is the value read by the temperature sensor on the actuator 1. Unit is C°.
	 */
	float ActuatorTemp1;

	/**
	 * @brief This is the value read by the temperature sensor on the actuator 2. Unit is C°.
	 */
	float ActuatorTemp2;

	/**
	 * @brief This is the value read by the temperature sensor on the actuator 3. Unit is C°.
	 */
	float ActuatorTemp3;

	/**
	 * @brief This is the value read by the temperature sensor on the actuator 4. Unit is C°.
	 */
	float ActuatorTemp4;

	/**
	 * @brief This is the value read by the temperature sensor on the actuator 5. Unit is C°.
	 */
	float ActuatorTemp5;

	/**
	 * @brief This is the value read by the temperature sensor on the actuator 6. Unit is C°.
	 */
	float ActuatorTemp6;

	/**
	 * @brief This is the value read by the temperature sensor on the finger 1. Unit is C°.
	 */
	float FingerTemp1;

	/**
	 * @brief This is the value read by the temperature sensor on the finger 2. Unit is C°.
	 */
	float FingerTemp2;

	/**
	 * @brief This is the value read by the temperature sensor on the finger 3. Unit is C°.
	 */
	float FingerTemp3;


	/**
	 * This method will initialises all the values to 0
	 */
	void InitStruct()
	{
		Voltage       = 0.0f;
		Current       = 0.0f;
		AccelerationX = 0.0f;
		AccelerationY = 0.0f;
		AccelerationZ = 0.0f;
		ActuatorTemp1 = 0.0f;
		ActuatorTemp2 = 0.0f;
		ActuatorTemp3 = 0.0f;
		ActuatorTemp4 = 0.0f;
		ActuatorTemp5 = 0.0f;
		ActuatorTemp6 = 0.0f;
		FingerTemp1   = 0.0f;
		FingerTemp2   = 0.0f;
		FingerTemp3   = 0.0f;
	}

};

/** @brief This data structure holds the values of the robot's fingers.
 *  \struct FingersPosition KinovaTypes.h "Definition"
 */
struct FingersPosition
{
	/**
	 * @brief This is the value of the finger #1. The units will depends on the context it's been used.
	 */
	float Finger1;

	/**
	 * @brief This is the value of the finger #2. The units will depends on the context it's been used.
	 */
	float Finger2;

	/**
	 * @brief This is the value of the finger #3. The units will depends on the context it's been used.
	 */
	float Finger3;

	/**
	 * This method will initialises all the values to 0
	 */
	void InitStruct()
	{
		Finger1       = 0.0f;
		Finger2       = 0.0f;
		Finger3       = 0.0f;
	}
};

/** Coordinates holds the cartesian parts
 *  of the position and Fingers holds contains the value of the fingers. As an example, if an instance
 *  of the CartesianPosition is used in a cartesian velocity control context, the values in the struct
 *  will be velocity.
 *  @brief This data structure holds the values of a cartesian position.
 *  \struct CartesianPosition KinovaTypes.h "Definition"
 */
struct CartesianPosition
{
	/**
	 * @brief This contains values regarding the cartesian information.(end effector).
	 */
	CartesianInfo Coordinates;

	/**
	 * @brief This contains value regarding the fingers.
	 */
	FingersPosition Fingers;


	/**
	 * This method will initialises all the values to 0
	 */
	void InitStruct()
	{
		Coordinates.InitStruct();
		Fingers.InitStruct();
	}
};

/** @brief This data structure holds the values of an angular(actuators) position.
 *  \struct AngularPosition KinovaTypes.h "Definition"
 */
struct AngularPosition
{
	/**
	 * @brief This contains value regarding the actuators.
	 */
	AngularInfo Actuators;

	/**
	 * @brief This contains value regarding the actuators.
	 */
	FingersPosition Fingers;

	/**
	 * This method will initialises all the values to 0
	 */
	void InitStruct()
	{
		Actuators.InitStruct();
		Fingers.InitStruct();
	}
};

/** @brief This data structure represents all limitation that can be applied to a control context.
 *
 *  Depending on the context, units and behaviour can change. See each parameter for more informations.
 *
 *  \struct Limitation KinovaTypes.h "Definition"
 */
struct Limitation
{
	/**
	 * @brief In a cartesian context, this represents the translation velocity but in an angular context, this represents the velocity of the actuators 1, 2 and 3.
	 */
	float speedParameter1;

	/**
	 * @brief In a cartesian context, this represents the orientation velocity but in an angular context, this represents the velocity of the actuators 4, 5 and 6.
	 */
	float speedParameter2;
	float speedParameter3;           /*!< Not used for now. */
	float forceParameter1;           /*!< Not used for now. */
	float forceParameter2;           /*!< Not used for now. */
	float forceParameter3;           /*!< Not used for now. */
	float accelerationParameter1;    /*!< Not used for now. */
	float accelerationParameter2;    /*!< Not used for now. */
	float accelerationParameter3;    /*!< Not used for now. */

	void InitStruct()
	{
		speedParameter1 = 0.0f;
		speedParameter2 = 0.0f;
		speedParameter3 = 0.0f;
		forceParameter1 = 0.0f;
		forceParameter2 = 0.0f;
		forceParameter3 = 0.0f;
		accelerationParameter1 = 0.0f;
		accelerationParameter2 = 0.0f;
		accelerationParameter3 = 0.0f;
	}
};

/**
 *  @brief This data structure represents an abstract position built by a user. Depending on the control type the Cartesian information, the angular information or both will be used.
 *  \struct UserPosition KinovaTypes.h "Definition"
 */
struct UserPosition
{
	/**
	 * @brief The type of this position.
	 */
	POSITION_TYPE Type;

	/**
	 * @brief This is used only if the type of position is TIME_DELAY. It represents the delay in second.
	 */
	float Delay;

	/**
	 * @brief Cartesian information about this position.
	 */
    CartesianInfo CartesianPosition;

    /**
	 * @brief Angular information about this position.
	 */
	AngularInfo Actuators;

	/**
	 * @brief Mode of the gripper.
	 */
	HAND_MODE HandMode;

	/**
	 * @brief fingers information about this position.
	 */
	FingersPosition Fingers;

	void InitStruct()
	{
		Type = CARTESIAN_POSITION;
		Delay = 0.0f;
		CartesianPosition.InitStruct();
		Actuators.InitStruct();
		HandMode = POSITION_MODE;
		Fingers.InitStruct();
	}
};

/** @brief This data structure represents a point of a trajectory. It contains the position a limitation that you can applied.
 *  \struct TrajectoryPoint KinovaTypes.h "Definition"
 */
struct TrajectoryPoint
{
	/**
	 * @brief Position information that described this trajectory point.
	 */
	UserPosition Position;

	/**
	 * @brief A flag that indicates if the limitation are active or not (1 is active 0 is not).
	 */
	int LimitationsActive;

	/**
	* @brief A flag that indicates if the tracjetory's synchronization is active. (1 is active 0 is not). ONLY AVAILABLE IN ANGULAR CONTROL.
	*/
	int SynchroType;

	/**
	 * @brief Limitation applied to this point if the limitation flag is active.
	 */
	Limitation Limitations;

	void InitStruct()
	{
		Position.InitStruct();
		LimitationsActive = 0;
		Limitations.InitStruct();
		SynchroType = 0;
		Limitations.InitStruct();
	}
};

/** @brief This data structure represents the informations regarding the robot's trajectory's FIFO.
 *  \struct TrajectoryFIFO KinovaTypes.h "Definition"
 */
struct TrajectoryFIFO
{
	/**
	 * @brief This tells you how many trajectory point are still stored in the robot.
	 */
	unsigned int TrajectoryCount;

	/**
	 * @brief This is the usage of the trajectory FIFO.
	 */
	float UsedPercentage;

	/**
	 * @brief This is the size of the trajectory FIFO.
	 */
	unsigned int MaxSize;
};

/** @brief This data structure represents the informations regarding the singularities surrounding the end effector.
 * It is not used for now but will be in the future.
 * \struct SingularityVector KinovaTypes.h "Definition"
 */
struct SingularityVector
{
	int TranslationSingularityCount;
	int OrientationSingularityCount;
	float TranslationSingularityDistance;
	float OrientationSingularityDistance;
	CartesianInfo RepulsionVector;
};

/**
 * @brief This is a virtual representation of a 6-axis joystick.
 */
struct JoystickCommand
{
	/**
	 * @brief This array contains the state of all the buttons. (1 = PRESSED, 0 = RELEASED)
	 */
	short ButtonValue[JOYSTICK_BUTTON_COUNT];

	/**
	 * @brief That holds the behaviour of the stick when it is inclined from left to right. (value between -1 and 1 inclusively)
	 * 2 functionalities can be mapped with this value, there is an event when the value is negative and there is one when it is positive.
	 */
	float InclineLeftRight;

	/**
	 * @brief That holds the behaviour of the stick when it is inclined forward and backward. (value between -1 and 1 inclusively)
	 * 2 functionalities can be mapped with this value, there is an event when the value is negative and there is one when it is positive.
	 */
    float InclineForwardBackward;

    /**
	 * @brief That holds the behaviour of the stick when it is rotated clockwork and counter clockwork. (value between -1 and 1 inclusively)
	 * 2 functionalities can be mapped with this value, there is an event when the value is negative and there is one when it is positive.
	 */
    float Rotate;

    /**
	 * @brief That holds the behaviour of the stick when it is moved from left to right. (value between -1 and 1 inclusively)
	 * 2 functionalities can be mapped with this value, there is an event when the value is negative and there is one when it is positive.
	 */
    float MoveLeftRight;

    /**
	 * @brief That holds the behaviour of the stick when it is moved forward and backward. (value between -1 and 1 inclusively)
	 * 2 functionalities can be mapped with this value, there is an event when the value is negative and there is one when it is positive.
	 */
    float MoveForwardBackward;

    /**
	 * @brief That holds the behaviour of the stick when it is pushed and pulled. (value between -1 and 1 inclusively)
	 * 2 functionalities can be mapped with this value, there is an event when the value is negative and there is one when it is positive.
	 */
	float PushPull;

	void InitStruct()
	{
		for(int i = 0; i < JOYSTICK_BUTTON_COUNT; i++)
		{
			ButtonValue[i] = 0;
		}

		InclineLeftRight = 0.0f;
		InclineForwardBackward = 0.0f;
		Rotate = 0.0f;
		MoveLeftRight = 0.0f;
		MoveForwardBackward = 0.0f;
		PushPull = 0.0f;
	}
};

/**
 * @brief This structure holds informations relative to the client.
 * It is mostly used for rehab clients. As an example, if you need to modify the max velocity or the retract position,
 * it will be done here. The easiest way to modify the client configuration is to get the current one by calling the function
 * GetClientConfigurations, modify all the parameters you need and send the structure back to the robot by calling the function
 * SetClientConfigurations(). Note that some of the parameters are read only. That means that even if you modify them and send
 * the structure to the robot, they will not be modified.
 */
struct ClientConfigurations
{
	/**
	 * @brief This is the ID of the client. The string must ends with a EndOfString character(\0).
	 */
    char ClientID[STRING_LENGTH];

    /**
	 * @brief This is the name of the client. The string must ends with a EndOfString character(\0).
	 */
    char ClientName[STRING_LENGTH];

    /**
	 * @brief This is the name of the organization. The string must ends with a EndOfString character(\0).
	 */
	char Organization[STRING_LENGTH];

	/**
	 * @brief This is where you can store the serial number of the robot. The string must ends with a EndOfString character(\0).
	 */
    char Serial[STRING_LENGTH];

    /**
	 * @brief This is where you can store the model number of the robot. The string must ends with a EndOfString character(\0).
	 */
    char Model[STRING_LENGTH];

    /**
	 * @brief That tells you if the robot is left handed or right handed. If you modify this parameter, the change will take
	 * effect on the next REBOOT.
	 */
	ArmLaterality Laterality;

	/**
	 * @brief This is the max translation(X, Y and Z) velocity of the robot's end effector.
	 */
    float MaxTranslationVelocity;

    /**
	 * @brief This is the max orientation(ThetaX, ThetaY and ThetaZ) velocity of the robot's end effector.
	 */
	float MaxOrientationVelocity;

	/**
	 * @brief This is the max translation acceleration of the robot's end effector.
	 */
    float MaxTranslationAcceleration;

    /**
	 * @brief This is the max orientation acceleration of the robot's end effector.
	 */
	float MaxOrientationAcceleration;

	/**
	 * @brief Not used for now.
	 */
	float MaxForce;

	/**
	 * @brief This is the sensibility of the controller. The value is a % of the command received by the controller.
	 * Higher is the value and higher is the sensibility. Higher is the sensibility, quicker the end effector will
	 * reach the desired velocity for the same joystick command.
	 */
    float Sensibility;

    /**
	 * @brief This value is used only when the drinking mode is active. This add a an offset to the translation
	 * Y value. It is the height of a glass when you drink.
	 */
	float DrinkingHeight;

	/**
	 * @brief That flag tells you if the advance retract feature is active. (0 = not active, 1 = active)
	 * The advance retract mode let you decide the trajectory between the READY(home) position and the
	 * retracted position.
	 */
	int ComplexRetractActive;

	/**
	 * @brief This value is the angle between the second carbon link(between actuator 2 and 3) and the X-Y plane.
	 */
	float RetractedPositionAngle;

	/**
	 * @brief This tells you how many positions there is in the advance retract trajectory.
	 */
	int RetractedPositionCount;
	UserPosition RetractPositions[NB_ADVANCE_RETRACT_POSITION];

	/**
	 * @brief This value is used only when the drinking mode is active. This add a an offset to the translation
	 * X axis based on the end effector frame. It is the diameter of a glass when you drink.
	 */
	float DrinkingDistance;

	/**
	 * @brief This is a flag to invert finger 2 and 3. It is mostly used in rehab for left handed robot.(0 = normal, 1 = inverted)
	 */
	int Fingers2and3Inverted;

	/**
	 * @brief This value is used only when the drinking mode is active. This add a an offset to the translation
	 * Z axis based on the end effector frame. It is the distance between the end effector and the glass when you drink.
	 */
	float DrinkingLenght;

	/**
	 * @brief It is a flag that indicates if the GOTO(pre programmed) position are deleted when the robot goes in READY position.
	 */
	int DeletePreProgrammedPositionsAtRetract;

	/**
	 * @brief Not used for now.
	 */
	int EnableFlashErrorLog;

	/**
	 * @brief Not used for now.
	 */
	int EnableFlashPositionLog;

	int RobotConfigSelect;

	int TorqueSensorsEnable;

	/**
	 * @brief Not used for now.
	 */
	int Expansion[196];
};

/**
 * @brief This is the list of available feature that can be mapped with a controller through the mappign system.
 * Every list of mode that a mapping contains is mapped with one of these features. The default value is
 * CF_NoFunctionality.
 */
enum  ControlFunctionalityTypeEnum
{
	/**
	 * @brief Default value, represents nothing.
	 */
	CF_NoFunctionality = 0,

	/**
	 * @brief Virtually turn on and off the joystick.
	 */
	CF_Disable_EnableJoystick = 1,

	/**
	 * @brief Home the robot if the is initialized and anywhere in the workspace except between the READY and RETRACTED position.
	 * Go to RETRACTED position if the robot is in READY position and go to READY position if the robot is in
	 * RETRACTED position.
	 * is in READY mode.
	 */
	CF_Retract_ReadyToUse = 2,

	/**
	 * @brief Not used for now.
	 */
	CF_Change_TwoAxis_ThreeAxis = 3,

	/**
	 * @brief Put the robotical arm in the drinking mode.
	 */
	CF_Change_DrinkingMode = 4,

	/**
	 * @brief Iterate mode in the list A.
	 */
	CF_Cycle_ModeA_list = 5,

	/**
	 * @brief Iterate mode in the list B.
	 */
	CF_Cycle_ModeB_list = 6,

	/**
	 * @brief Divide the velocity by 2.
	 */
	CF_DecreaseSpeed = 7,

	/**
	 * @brief Double the speed.
	 */
	CF_IncreaseSpeed = 8,

	/**
	 * @brief Move the robotical arm's end position to the GOTO position 1.
	 */
	CF_Goto_Position1 = 9,

	/**
	 * @brief Move the robotical arm's end position to the GOTO position 2.
	 */
	CF_Goto_Position2 = 10,

	/**
	 * @brief Move the robotical arm's end position to the GOTO position 3.
	 */
	CF_Goto_Position3 = 11,

	/**
	 * @brief Move the robotical arm's end position to the GOTO position 4.
	 */
	CF_Goto_Position4 = 12,

	/**
	 * @brief Move the robotical arm's end position to the GOTO position 5.
	 */
	CF_Goto_Position5 = 13,

	/**
	 * @brief Store the current cartesian position into the GOTO position 1.
	 */
	CF_RecordPosition1 = 14,

	/**
	 * @brief Store the current cartesian position into the GOTO position 2.
	 */
	CF_RecordPosition2 = 15,

	/**
	 * @brief Store the current cartesian position into the GOTO position 3.
	 */
	CF_RecordPosition3 = 16,

	/**
	 * @brief Store the current cartesian position into the GOTO position 4.
	 */
	CF_RecordPosition4 = 17,

	/**
	 * @brief Store the current cartesian position into the GOTO position 5.
	 */
	CF_RecordPosition5 = 18,

	/**
	 * @brief Move the robotical arm's end effector along the X axis toward the positive values.
	 * If the robotical arm is in angular control, this will move the actuator 1 counterclockwise.
	 */
	CF_X_Positive = 19,

	/**
	 * @brief Move the robotical arm's end effector along the X axis toward the negative values.
	 * If the robotical arm is in angular control, this will move the actuator 1 clockwise.
	 */
	CF_X_Negative = 20,

	/**
	 * @brief Move the robotical arm's end effector along the Y axis toward the positive values.
	 * If the robotical arm is in angular control, this will move the actuator 2 counterclockwise.
	 */
	CF_Y_Positive = 21,

	/**
	 * @brief Move the robotical arm's end effector along the Y axis toward the negative values.
	 * If the robotical arm is in angular control, this will move the actuator 2 clockwise.
	 */
	CF_Y_Negative = 22,

	/**
	 * @brief Move the robotical arm's end effector along the Z axis toward the positive values.
	 * If the robotical arm is in angular control, this will move the actuator 3 counterclockwise.
	 */
	CF_Z_Positive = 23,

	/**
	 * @brief Move the robotical arm's end effector along the Z axis toward the negative values.
	 * If the robotical arm is in angular control, this will move the actuator 3 clockwise.
	 */
	CF_Z_Negative = 24,

	/**
	 * @brief Rotate the robotical arm's end effector around the X axis counterclockwise.
	 * If the robotical arm is in angular control, this will move the actuator 4 counterclockwise.
	 */
	CF_R_Positive = 25,

	/**
	 * @brief Rotate the robotical arm's end effector around the X axis clockwise.
	 * If the robotical arm is in angular control, this will move the actuator 4 clockwise.
	 */
	CF_R_Negative = 26,

	/**
	 * @brief Rotate the robotical arm's end effector around the Y axis counterclockwise.
	 * If the robotical arm is in angular control, this will move the actuator 5 counterclockwise.
	 */
	CF_U_Positive = 27,

	/**
	 * @brief Rotate the robotical arm's end effector around the X axis clockwise.
	 * If the robotical arm is in angular control, this will move the actuator 5 clockwise.
	 */
	CF_U_Negative = 28,

	/**
	 * @brief Rotate the robotical arm's end effector around the Z axis counterclockwise.
	 * If the robotical arm is in angular control, this will move the actuator 6 counterclockwise.
	 */
	CF_V_Positive = 29,

	/**
	 * @brief Rotate the robotical arm's end effector around the Z axis clockwise.
	 * If the robotical arm is in angular control, this will move the actuator 6 clockwise.
	 */
	CF_V_Negative = 30,

	/**
	 * @brief Not used for now.
	 */
	CF_OpenHandOneFingers = 31,

	/**
	 * @brief Not used for now.
	 */
	CF_CloseHandOneFingers = 32,

	/**
	 * @brief Open fingers 1 and 2 of the hand.
	 */
	CF_OpenHandTwoFingers = 33,

	/**
	 * @brief Close fingers 1 and 2 of the hand.
	 */
	CF_CloseHandTwoFingers = 34,

	/**
	 * @brief Open fingers 1, 2 and 3 of the hand.
	 */
	CF_OpenHandThreeFingers = 35,

	/**
	 * @brief Close fingers 1, 2 and 3 of the hand.
	 */
	CF_CloseHandThreeFingers = 36,

	/**
	 * @brief Put the robotical arm in angular control mode.
	 */
	CF_ForceAngularVelocity = 37,

	/**
	 * @brief Turn ON/OFF the force control if the feature is available.
	 */
	CF_ForceControlStatus = 38,

	CF_Trajectory = 39,

	/**
	 * @brief Orient the end effector toward the positive X Axis.
	 */
	CF_AutomaticOrientationXPlus = 40,

	/**
	 * @brief Orient the end effector toward the negative X Axis.
	 */
	CF_AutomaticOrientationXMinus = 41,

	/**
	 * @brief Orient the end effector toward the positive Y Axis.
	 */
	CF_AutomaticOrientationYPlus = 42,

	/**
	 * @brief Orient the end effector toward the negative Y Axis.
	 */
	CF_AutomaticOrientationYMinus = 43,

	/**
	 * @brief Orient the end effector toward the positive Z Axis.
	 */
	CF_AutomaticOrientationZPlus = 44,

	/**
	 * @brief Orient the end effector toward the negative Z Axis.
	 */
	CF_AutomaticOrientationZMinus = 45,

	/**
	 * @brief Move the robot along the advance GOTO position 1.
	 */
	CF_AdvanceGOTO_1 = 46,

	/**
	 * @brief Clear the advance GOTO's trajectory 1.
	 */
	CF_AdvanceGOTO_Clear_1 = 47,

	/**
	 * @brief Add a point to the advance GOTO's trajectory 1.
	 */
	CF_AdvanceGOTO_Add_1 = 48,
};

/**
 * @brief This is an event from a controller's stick. Each variable of the struct can be mapped with a ControlFunctionalityTypeEnum.
 */
struct StickEvents
{
	/**
	 * @brief This represents the negative value of the event. As an example, if you incline the stick to the left
	 * it will trigger that event.
	 */
    unsigned char Minus;

    /**
	 * @brief This represents the positive value of the event. As an example, if you incline the stick to the right
	 * it will trigger that event.
	 */
	unsigned char Plus;
};

/**
 * @brief This is an event from a controller's button. Each variable of the struct can be mapped with a ControlFunctionalityTypeEnum.
 */
struct ButtonEvents
{
	/**
	 * @brief Represents a single CLICK event.(PRESS and RELEASE)
	 */
	unsigned char OneClick;

	/**
	 * @brief Not used for now.
	 */
    unsigned char TwoClick;

    /**
	 * @brief Represents a PRESS and HOLD for 1 second event .
	 */
	unsigned char HoldOneSec;

	/**
	 * @brief Represents a PRESS and HOLD for 2 second event.
	 */
    unsigned char HoldTwoSec;

    /**
	 * @brief Represents a PRESS and HOLD for 3 second event.
	 */
	unsigned char HoldThreeSec;

	/**
	 * @brief Represents a PRESS and HOLD for 4 second event.
	 */
	unsigned char HoldFourSec;

	/**
	 * @brief Represents a PRESS and HOLD event.
	 */
	unsigned char HoldDown;
};

/**
 * @brief Indicates the type of controller.
 */
enum ControlMappingMode
{
	/**
	 * Represents a 1-axis controller.
	 */
    OneAxis,

    /**
	 * Represents a 2-axis controller.
	 */
    TwoAxis,

    /**
	 * Represents a 3-axis controller.
	 */
    ThreeAxis,

    /**
	 * Represents a 6-axis controller.
	 */
	SixAxis
};

/**
 * @brief Represents one mode map of a control mapping.
 * Each control mapping has 2 list of mode map.
 */
struct ControlsModeMap
{
	/**
	 * @brief A flag that indicates if we can perform movement in more than one direction at a time.
	 */
	int DiagonalsLocked;

	/**
	 * @brief Not use for now.
	 */
	int Expansion;

	/**
	 * @brief All events from the stick of the controller.
	 */
	StickEvents ControlSticks[STICK_EVENT_COUNT];

	/**
	 * @brief All events from the buttons of the controller.
	 */
	ButtonEvents ControlButtons[BUTTON_EVENT_COUNT];
};

/**
 * @brief This represents a group of functionalities mapped to some events triggered by a specific controller.
 *
 * As an example, the kinova 3-axis joystick has its own control mapping. This API also has its own control mapping. Note that
 * since list A and list B cannot be used at the same time in the same control mapping, it implies that either one of the variable
 * can have a >= 0 value.
 */
struct ControlMapping
{
	/**
	 * @brief List A's element count. If this value exceeds MODE_MAP_COUNT, we got a problem.
	 */
	int NumOfModesA;

	/**
	 * @brief List B's element count. If this value exceeds MODE_MAP_COUNT, we got a problem.
	 */
	int NumOfModesB;

	/**
	 * @brief This is the actual index of the active mode map in the list A.
	 * If the list A is currently unused, this value will be -1.
	 */
	int ActualModeA;

	/**
	 * @briefThis is the actual index of the active mode map in the list B.
	 * If the list B is currently unused, this value will be -1.
	 */
	int ActualModeB;

	/**
	 * @brief That indicates what kind of controller is in use.
	 */
	ControlMappingMode Mode;

	/**
	 * @brief This is the mode map list A. By default, on the 3-axis kinova joystick, it corresponds to the modes accessible with
	 * the left button on the top of the stick.
	 */
	ControlsModeMap ModeControlsA[MODE_MAP_COUNT];

	/**
	 * @brief This is the mode map list B. By default, on the 3-axis kinova joystick, it corresponds to the modes accessible with
	 * the right button on the top of the stick.
	 */
	ControlsModeMap ModeControlsB[MODE_MAP_COUNT];
};

/**
 * @brief This structure holds all the control mapping of the system. It is the entry point if you want to use the mapping system.
 */
struct ControlMappingCharts
{
	/**
	 * @brief This tells you how many control mapping we got in the charts. it cannot exceeds CONTROL_MAPPING_COUNT.
	 */
	int NumOfConfiguredMapping;

	/**
	 * @brief This is the active control mapping.
	 */
	int ActualControlMapping;

	/**
	 * @brief This is the list of all control mapping stored in the charts.
	 */
    ControlMapping Mapping[CONTROL_MAPPING_COUNT];
};

/**
 * @brief That represents the type of an error. It is used mostly for identification.
 */
enum errorLoggerType
{
	/**
	 * @brief Default value.
	 */
	ERROR_NOTINITIALIZED,

	/**
	 * @brief An error from the system's first software layer. It is very low level stuff.
	 */
	keos_err1,

	/**
	 * @brief An error from the system's second software layer. It is very low level stuff.
	 */
	keos_err2,

	/**
	 * @brief An error from the system's third software layer. It is low level stuff.
	 */
	keos_err3,

	/**
	 * @brief Not used for now.
	 */
	User_err_start_marker,

	/**
	 * @brief Indicates that one of the actuator's temperature has been over the temperature limit.
	 */
	errorlog_Actuator_Temperature,

	/**
	 * @brief Indicates that the actuator that was in temperature error but is now ok.
	 */
	errorlog_Actuator_TemperatureOK,

	/**
	 * @brief Indicates that one of the finger's temperature has been over the temperature limit.
	 */
	errorlog_Finger_Temperature,

	/**
	 * @brief Indicates that one of the finger's temperature was over the temperature limit but is now ok.
	 */
	errorlog_Finger_TemperatureOK,

	/**
	 * @brief Indicates that the voltage is below the minimum value.
	 */
	errorlog_voltage,

	/**
	 * @brief Indicate that the voltage was in error but is now ok.
	 */
	errorlog_voltageOK,

	/**
	 * @brief That indicates the one of the finger's current has been over the current limit while closing.
	 */
	errorlog_current_FingersClosing,

	/**
	 * @brief That indicates the one of the finger's current has been over the current limit while opening.
	 */
	errorlog_current_FingersOpening,

	/**
	 * @brief That indicates the one of the finger's current was in error but is now ok.
	 */
	errorlog_current_FingersOK,

	/**
	 * @brief That indicates the one of the actuators's current has been over the current limit.
	 */
	errorlog_current_Actuators,

	/**
	 * @brief That indicates the one of the actuators was in current error but is now ok.
	 */
	errorlog_current_ActuatorsOK,

	/**
	 * @brief The system did not detect enough hardware to virtually build a JACO or a MICO.
	 */
	errorLog_RobotStatus_Build_Incomplete,

	/**
	 * @brief Not used for now
	 */
	errorLogger_END
};

/**
 * @brief This represents a system error. Every error generated by the system is logged in the robot's flash memory.
 */
struct SystemError
{
	/**
	 * @brief Error's header. Not used for now.
	 */
	unsigned int ErrorHeader;

	/**
	 * @brief The error's type.
	 */
	errorLoggerType ErrorType;

	/**
	 * @brief The firmware's code version.
	 */
	int FirmwareVersion;

	/**
	 * @brief The Keos's code version. Keos is a software layer that contains low level stuff.
	 */
	int KeosVersion;

	/**
	 * @brief Not used for now.
	 */
	unsigned int SystemTime;

	/**
	 * @brief Internal use only.
	 */
	bool LayerErrorStatus[ERROR_LAYER_COUNT];

	/**
	 * @brief Internal use only.
	 */
	int LifeTime;

	/**
	 * @brief internal use only.
	 */
	int DataCount;

	/**
	 * @brief internal use only.
	 */
	unsigned int Data[ERROR_DATA_COUNT_MAX];

};

/**
 * @brief This represents a group of limitations that can be applied to a trajectory point.
 */
struct ZoneLimitation
{
	/**
	 * @brief The first speed parameter. Used in angular control, it is the velocity of the actuator 1, 2 and 3 and if used in cartesian
	 * control, it is the translation velocity.
	 */
	float speedParameter1;

	/**
	 * @brief The second speed parameter. Used in angular control, it is the velocity of the actuator 4, 5 and 6 and if used in cartesian
	 * control, it is the orientation velocity.
	 */
	float speedParameter2;

	/**
	 * @brief Not used for now.
	 */
	float speedParameter3;

	/**
	 * @brief Not used for now.
	 */
	float forceParameter1;

	/**
	 * @brief Not used for now.
	 */
	float forceParameter2;

	/**
	 * @brief Not used for now.
	 */
	float forceParameter3;

	/**
	 * @brief Not used for now.
	 */
	float accelerationParameter1;

	/**
	 * @brief Not used for now.
	 */
	float accelerationParameter2;

	/**
	 * @brief Not used for now.
	 */
	float accelerationParameter3;
};

/**
 * @brief This represents the type of a 3d shape.
 */
enum ShapeType
{
	/**
	 * Not used for now.
	 */
	PrismSquareBase_X = 0,

	/**
	 * Not used for now.
	 */
	PrismSquareBase_Y = 1,

	/**
	 * A rectangular prism.
	 */
	PrismSquareBase_Z = 2,

	/**
	 * Not used for now.
	 */
	PrismTriangularBase_X = 3,

	/**
	 * Not used for now.
	 */
	PrismTriangularBase_Y = 4,

	/**
	 * Not used for now.
	 */
	PrismTriangularBase_Z = 5,

	/**
	 * Not used for now.
	 */
    Pyramid = 6
};

/**
 * @brief This structure contains informations about the torque and the force of the robotical arm.
 */
struct ForcesInfo
{
	/**
	 * @brief That contains the torque of the actuator 1.
	 */
	float Actuator1;

	/**
	 * @brief That contains the torque of the actuator 2.
	 */
	float Actuator2;

	/**
	 * @brief That contains the torque of the actuator 3.
	 */
	float Actuator3;

	/**
	 * @brief That contains the torque of the actuator 4.
	 */
	float Actuator4;

	/**
	 * @brief That contains the torque of the actuator 5.
	 */
	float Actuator5;

	/**
	 * @brief That contains the torque of the actuator 6.
	 */
	float Actuator6;

	/**
	 * @brief That contains the force applied by the robotical arm on the X axis.
	 */
	float X;

	/**
	 * @brief That contains the force applied by the robotical arm on the Y axis.
	 */
	float Y;

	/**
	 * @brief That contains the force applied by the robotical arm on the Z axis.
	 */
	float Z;

	/**
	 * @brief That contains the force applied by the robotical arm around the X axis.
	 */
	float ThetaX;

	/**
	 * @brief That contains the force applied by the robotical arm around the Y axis.
	 */
	float ThetaY;

	/**
	 * @brief That contains the force applied by the robotical arm around the Z axis.
	 */
	float ThetaZ;
};

/**
 * @brief This structure holds various informations but mostly it is flag status.
 */
struct QuickStatus
{
	/**
	 * @brief This flag's value is 1 if the finger #1 is initialized.
	 */
	unsigned char Finger1Status;

	/**
	 * @brief This flag's value is 1 if the finger #1 is initialized.
	 */
	unsigned char Finger2Status;

	/**
	 * @brief This flag's value is 1 if the finger #1 is initialized.
	 */
	unsigned char Finger3Status;

	/**
	 * @brief This is the retract state. this value is directly associated with the enum RETRACT_TYPE.
	 */
	unsigned char RetractType;

	/**
	 * @brief This is a flag that indicates if the advance retract is on. (0 = basic retract - 1 = advance retract)
	 */
	unsigned char RetractComplexity;

	/**
	 * @brief This is a flag that indicates if the control is on. (0 = ON - 1 = OFF)
	 */
	unsigned char ControlEnableStatus;

	/**
	 * @brief This flag indicates the active control module. This value is directly associated with the enum CONTROL_MODULE.
	 */
	unsigned char ControlActiveModule;

	/**
	 * @brief This is a flag that indicates the actual control frame. (0 = fixed - 1 = rotating frame)
	 */
	unsigned char ControlFrameType;

	/**
	 * @brief This is a flag that indicates the actual control frame. (0 = fixed - 1 = rotating frame)
	 */
	unsigned char CartesianFaultState;

	/**
	 * @brief This is a flag that indicates if the force control is ON. (0 = ON - 1 = OFF)
	 */
	unsigned char ForceControlStatus;

	/**
	 * @brief This is a flag that indicates if the current limitation is on. (0 = ON - 1 = OFF)
	 */
	unsigned char CurrentLimitationStatus;

	/**
	 * @brief This tells you if the robotical arm is a JACO or a MICO((0 = JACO - 1 = MICO))
	 */
	unsigned char RobotType;

	/**
	 * @brief Not used for now
	 */
	unsigned char RobotEdition;

	/**
	 * That tells if torque sensors are available or not.
	 */
	unsigned char TorqueSensorsStatus;
};

/**
 * @brief Structure that represents a finger from the end effector's tool.
 */
struct Finger
{
	/**
	 * @brief ID of the finger.
	 */
	char ID[STRING_LENGTH];

	/**
	 * @brief Actual command of the finger.
	 */
	float ActualCommand;

	/**
	 * @brief Actual velocity of the finger.
	 */
	float ActualSpeed;

	/**
	 * @brief Actual force of the finger.
	 */
	float ActualForce;

	/**
	 * @brief Actual acceleration of the finger.
	 */
	float ActualAcceleration;

	/**
	 * @brief Actual current of the finger.
	 */
	float ActualCurrent;

	/**
	 * @brief Actual position of the finger.
	 */
	float ActualPosition;

	/**
	 * @brief Not used for now.
	 */
	float ActualAverageCurrent;

	/**
	 * @brief Actual temperature of the finger.
	 */
	float ActualTemperature;

	/**
	 * @brief Not used for now.
	 */
	int CommunicationErrors;

	/**
	 * @brief Not used for now.
	 */
	int OscillatorTuningValue;

	/**
	 * @brief Not used for now.
	 */
	float CycleCount;

	/**
	 * @brief Not used for now.
	 */
	float RunTime;

	/**
	 * @brief Not used for now.
	 */
	float PeakMaxTemp;

	/**
	 * @brief Not used for now.
	 */
	float PeakMinTemp;

	/**
	 * @brief Not used for now.
	 */
	float PeakCurrent;

	/**
	 * @brief This is the max velocity of the finger.
	 */
	float MaxSpeed;

	/**
	 * @brief This is the max force of the finger.
	 */
	float MaxForce;

	/**
	 * @brief This is the max acceleration of the finger.
	 */
	float MaxAcceleration;

	/**
	 * @brief This is the max current of the finger.
	 */
	float MaxCurrent;

	/**
	 * @brief This is the max position of the finger.
	 */
	float MaxAngle;

	/**
	 * @brief This is the min position of the finger.
	 */
	float MinAngle;

	/**
	 * @brief Not used for now.
	 */
	unsigned int DeviceID;

	/**
	 * @brief Code version of the finger's firmware.
	 */
	unsigned int CodeVersion;

	/**
	 * @brief A flag that indicates if the finger is initialized or not.
	 */
	unsigned short IsFingerInit;

	/**
	 * @brief Index of the finger.
	 */
	unsigned short Index;

	/**
	 * @brief Address of the finger.
	 */
	unsigned short FingerAddress;

	/**
	 * @brief A flag that indicates if the finger is connected.
	 */
	unsigned short IsFingerConnected;
};

/**
 * @brief Structure that represents the robotical arm's gripper.
 */
struct Gripper
{
	/**
	 * @brief Model of the gripper.
	 */
	char Model[STRING_LENGTH];

	/**
	 * @brief The 3 fingers of the gripper.
	 */
	Finger Fingers[JACO_FINGERS_COUNT];
};

/**
 * @brief Represents the 3D shape of a protection zone.
 */
struct ZoneShape
{
	/**
	 * @brief This is the geometric type of shape.
	 */
	ShapeType shapeType;

	/**
	 * @brief Not used for now.
	 */
	int Expansion1;

	/**
	 * @brief The points that describe the shape.
	 */
	CartesianInfo Points[LEGACY_CONFIG_NB_POINTS_COUNT];
};

/**
 * @brief that represents a protection zone.
 */
struct Zone
{
	/**
	 * @brief ID of the zone.
	 */
	int ID;

	/**
	 * @brief Not used for now
	 */
	int Expansion1;

	/**
	 * @brief Geometric shape of the protection zone.
	 */
	ZoneShape zoneShape;

	/**
	 * @brief Limitation to apply inside the protection zone. As an example, if you want the zone to be am unaccessible zone,
	 * you need to limit the velocity to 0.
	 */
	ZoneLimitation zoneLimitation;

	/**
	 * @brief Not used for now.
	 */
	int Expansion2;
};

/**
 * @brief This structure represents the complete list of protection zone of the robotical arm.
 */
struct ZoneList
{
	/**
	 * @brief This is the active zone count.
	 */
	int NbZones;

	/**
	 * @brief Not used for now.
	 */
	int Expansion1;

	/**
	 * @brief This is the list of zone itself.
	 */
	Zone Zones[LEGACY_CONFIG_NB_ZONES_MAX];

};

/**
 * @brief This structure holds system status flags.
 */
struct SystemStatus
{
	/**
	 * @brief That tells if the joystick is active. (0 = active - 1 = not active)
	 */
    unsigned int JoystickActive;

    /**
	 * @brief That tells if the joystick is active. (0 = active - 1 = not active)
	 */
    unsigned int RetractStatus;

    /**
	 * @brief That tells if the drinking mode is active. (0 = active - 1 = not active)
	 */
    unsigned int DrinkingMode;

    /**
	 * @brief That tells if the robotical arm is right handed or left handed. (0 = RIGHTHANDED - 1 = LEFTHANDED)
	 */
    unsigned int ArmLaterality;

    /**
     * @brief A flag that indicates if the translation mode is currently active.(Based on the actual mapping).
     */
    unsigned int TranslationActive;

    /**
	 * @brief A flag that indicates if the orientation mode is currently active.(Based on the actual mapping).
	 */
    unsigned int RotationActive;

    /**
	 * @brief A flag that indicates if the translation mode is currently active.(Based on the actual mapping).
	 */
    unsigned int FingersActive;

    /**
	 * @brief A warning flag that indicates a general overcharge on the robotical arm.
	 */
    unsigned int WarningOverchargeForce;

    /**
	 * @brief A warning flag that indicates an overcharge on the fingers of the robotical arm.
	 */
    unsigned int WarningOverchargeFingers;

    /**
	 * @brief A warning flag that indicates a low voltage on the robotical arm.
	 */
    unsigned int WarningLowVoltage;

    /**
     * @brief A flag that indicates that a major error has occured.
     */
    unsigned int MajorErrorOccured;
};

/**
 * @brief This is structure hold almost all information of the robotical arm.
 */
struct GeneralInformations
{
	/**
	 * @brief not used for now
	 */
	double TimeAbsolute;

	/**
	 * @brief Time in second since the last boot.
	 */
	double TimeFromStartup;

	/**
	 * @brief Not used for now.
	 */
	unsigned int IndexStartup;

	/**
	 * @brief not used for now.
	 */
	int ExpansionLong1;

	/**
	 * @brief Not used for now.
	 */
	float TimeStampSavings;

	/**
	 * @brief Not used for now.
	 */
	float ExpansionFloat;

	/**
	 * @brief Main supply voltage.(24 V) Unit is V.
	 */
	float SupplyVoltage;

	/**
	 * @brief Current consumed on the main supply. Unit is A.
	 */
	float TotalCurrent;

	/**
	 * @brief Power consumed on the main supply. Unit is W.
	 */
	float Power;

	/**
	 * @brief Average power consumed on the main supply. Unit is W.
	 */
	float AveragePower;

	/**
	 * @brief Acceleration X sensor's value. Unit is G.
	 */
	float AccelerationX;

	/**
	 * @brief Acceleration Y sensor's value. Unit is G.
	 */
	float AccelerationY;

	/**
	 * @brief Acceleration Z sensor's value. Unit is G.
	 */
	float AccelerationZ;

	/**
	 * @brief Not used for now.
	 */
	float SensorExpansion1;

	/**
	 * @brief Not used for now.
	 */
	float SensorExpansion2;

	/**
	 * @brief Not used for now.
	 */
	float SensorExpansion3;

	/**
	 * @brief Firmware's code version. (Version.Major.Minor)
	 */
	unsigned int CodeVersion;

	/**
	 * @brief Firmware's code revision.
	 */
	unsigned int CodeRevision;

	/**
	 * @brief Not used for now.
	 */
	unsigned short Status;

	/**
	 * @brief The active controller. Example : 3-axis joystick, API, universal interface, ...
	 */
	unsigned short Controller;

	/**
	 * @brief Not used for now.
	 */
	unsigned short ControlMode;

	/**
	 * @brief Not used for now.
	 */
	unsigned short HandMode;

	/**
	 * @brief Connected actuator count
	 */
	unsigned short ConnectedActuatorCount;

	/**
	 * @brief Type of the actual position.
	 */
	unsigned short PositionType;

	/**
	 *	@brief Not in used now.
	 */
	unsigned short ErrorsSpiExpansion1;

	/**
	 *	@brief Not in used now.
	 */
	unsigned short ErrorsSpiExpansion2;

	/**
	 *	@brief Communication error from the main SPI.
	 */
	unsigned short ErrorsMainSPICount;

	/**
	 *	@brief Communication error from the external SPI.
	 */
	unsigned short ErrorsExternalSPICount;

	/**
	 *	@brief Communication error from the main internal communication bus.
	 */
	unsigned short ErrorsMainCANCount;

	/**
	 *	@brief Communication error from the main external communication bus.
	 */
	unsigned short ErrorsExternalCANCount;

	/**
	 * @brief System status.
	 */
	SystemStatus ActualSystemStatus;

	/**
	 * @brief The actual angular and cartesian position information
	 */
	UserPosition Position;

	/**
	 * @brief The actual angular and cartesian command information
	 */
	UserPosition Command;

	/**
	 * @brief The actual angular and cartesian current information
	 */
	UserPosition Current;

	/**
	 * @brief The actual angular and cartesian force information
	 */
	UserPosition Force;

	/**
	 * @brief Actua limitation applied on the robotical arm.
	 */
	ZoneLimitation ActualLimitations;

	/**
	 * @brief Actual control increment from the actuators.
	 */
	float ControlIncrement[6];

	/**
	 * @brief Actual control increment from the fingers
	 */
	float FingerControlIncrement[3];

	/**
	 * @brief Actual joystick command received by the robotical arm
	 */
	JoystickCommand ActualJoystickCommand;

	/**
	 * @brief An array of all the connected peripheral.
	 */
	unsigned int PeripheralsConnected[4];

	/**
	 * @brief An array of all the connected peripheral's ID.
	 */
	unsigned int PeripheralsDeviceID[4];

	/**
	 * @brief An array of all the connected peripheral.
	 */
	float ActuatorsTemperatures[6];

	/**
	 * @brief An array that contains the fingers's temperature.
	 */
	float FingersTemperatures[3];

	/**
	 * @brief Not used for now.
	 */
	float FutureTemperatures[3];

	/**
	 * @brief An array that contains communication errors of all actuators.
	 */
	int ActuatorsCommErrors[6];

	/**
	 * @brief An array that contains communication errors of all fingers.
	 */
	int FingersCommErrors[3];

	/**
	 * @brief Not used for now.
	 */
	int ExpansionLong2;

	/**
	 * @brief Not used for now.
	 */
	double ControlTimeAbsolute;

	/**
	 * @brief Control time since the last boot. The control time all the time the the robotical arm receive commands
	 * from any controller.
	 */
	double ControlTimeFromStartup;

	/**
	 * Not used for now
	 */
	unsigned char ExpansionsBytes[192];
};

/** @brief This data structure holds acceleration values(X, Y, Z) in an angular(joint by joint) control context.
 *  \struct AngularAcceleration KinovaTypes.h "Definition"
 */
struct AngularAcceleration
{
	/**
	 * Acceleration on X axis of the joint #1. Unit is G.
	 * @brief Acceleration on X axis of the joint #1. Unit is G.
	 */
	float Actuator1_X;

	/**
	 * Acceleration on Y axis of the joint #1. Unit is G.
	 * @brief Acceleration on Y axis of the joint #1. Unit is G.
	 */
	float Actuator1_Y;

	/**
	 * Acceleration on Z axis of the joint #1. Unit is G.
	 * @brief Acceleration on Z axis of the joint #1. Unit is G.
	 */
	float Actuator1_Z;

	/**
	 * Acceleration on X axis of the joint #2. Unit is G.
	 * @brief Acceleration on X axis of the joint #2. Unit is G.
	 */
	float Actuator2_X;

	/**
	 * Acceleration on Y axis of the joint #2. Unit is G.
	 * @brief Acceleration on Y axis of the joint #2. Unit is G.
	 */
	float Actuator2_Y;

	/**
	 * Acceleration on Z axis of the joint #2. Unit is G.
	 * @brief Acceleration on Z axis of the joint #2. Unit is G.
	 */
	float Actuator2_Z;

	/**
	 * Acceleration on X axis of the joint #3. Unit is G.
	 * @brief Acceleration on X axis of the joint #3. Unit is G.
	 */
	float Actuator3_X;

	/**
	 * Acceleration on Y axis of the joint #3. Unit is G.
	 * @brief Acceleration on Y axis of the joint #3. Unit is G.
	 */
	float Actuator3_Y;

	/**
	 * Acceleration on Z axis of the joint #3. Unit is G.
	 * @brief Acceleration on Z axis of the joint #3. Unit is G.
	 */
	float Actuator3_Z;

	/**
	 * Acceleration on X axis of the joint #4. Unit is G.
	 * @brief Acceleration on X axis of the joint #4. Unit is G.
	 */
	float Actuator4_X;

	/**
	 * Acceleration on Y axis of the joint #4. Unit is G.
	 * @brief Acceleration on Y axis of the joint #4. Unit is G.
	 */
	float Actuator4_Y;

	/**
	 * Acceleration on Z axis of the joint #4. Unit is G.
	 * @brief Acceleration on Z axis of the joint #4. Unit is G.
	 */
	float Actuator4_Z;

	/**
	 * Acceleration on X axis of the joint #5. Unit is G.
	 * @brief Acceleration on X axis of the joint #5. Unit is G.
	 */
	float Actuator5_X;

	/**
	 * Acceleration on Y axis of the joint #5. Unit is G.
	 * @brief Acceleration on Y axis of the joint #5. Unit is G.
	 */
	float Actuator5_Y;

	/**
	 * Acceleration on Z axis of the joint #5. Unit is G.
	 * @brief Acceleration on Z axis of the joint #5. Unit is G.
	 */
	float Actuator5_Z;

	/**
	 * Acceleration on X axis of the joint #6. Unit is G.
	 * @brief Acceleration on X axis of the joint #6. Unit is G.
	 */
	float Actuator6_X;

	/**
	 * Acceleration on Y axis of the joint #6. Unit is G.
	 * @brief Acceleration on Y axis of the joint #6. Unit is G.
	 */
	float Actuator6_Y;

	/**
	 * Acceleration on Z axis of the joint #6. Unit is G.
	 * @brief Acceleration on Z axis of the joint #6. Unit is G.
	 */
	float Actuator6_Z;

	/**
	 * This method will initialises all the values to 0
	 */
	void InitStruct()
	{
		Actuator1_X = 0.0f;
		Actuator1_Y = 0.0f;
		Actuator1_Z = 0.0f;
		Actuator2_X = 0.0f;
		Actuator2_Y = 0.0f;
		Actuator2_Z = 0.0f;
		Actuator3_X = 0.0f;
		Actuator3_Y = 0.0f;
		Actuator3_Z = 0.0f;
		Actuator4_X = 0.0f;
		Actuator4_Y = 0.0f;
		Actuator4_Z = 0.0f;
		Actuator5_X = 0.0f;
		Actuator5_Y = 0.0f;
		Actuator5_Z = 0.0f;
		Actuator6_X = 0.0f;
		Actuator6_Y = 0.0f;
		Actuator6_Z = 0.0f;
	}
};

/** @brief This data structure holds information that describes an abstract peripheral.
 *  \struct PeripheralInfo KinovaTypes.h "Definition"
 */
struct PeripheralInfo
{
	/**
	 * @brief Handle to the peripheral. Internal use only.
	 */
	unsigned int Handle;

	/**
	 * @brief Type of peripheral.
	 */
	unsigned int Type;

	/**
	 * @brief Port's type of the peripheral.
	 */
	unsigned int Port;

	/**
	 * @brief Address of the peripheral.
	 */
	unsigned int Address;

	/**
	 * @brief The code's version of the peripheral.
	 */
	unsigned int CodeVersion;
};

/** @brief This data structure holds information that describes how you define the gravity.
 */
enum GRAVITY_TYPE
{
	MANUAL_INPUT = 0,           /*!< You manually input the gravity. */
	OPTIMAL = 1,                /*!< You set the gravity via an automatic routine.*/
};

enum ROBOT_TYPE
{
	JACOV1_ASSISTIVE = 0,
	MICO_6DOF_SERVICE = 1,
	MICO_4DOF_SERVICE = 2,
	JACOV2_6DOF_SERVICE = 3,
	JACOV2_4DOF_SERVICE = 4,
	MICO_6DOF_ASSISTIVE = 5,
	JACOV2_6DOF_ASSISTIVE = 6,

	ROBOT_ERROR = 255,
	GENERIC_ROBOT = 254
};

#endif
