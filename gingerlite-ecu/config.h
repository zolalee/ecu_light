#ifndef _CONFIG_H_
#define _CONFIG_H_

#define USE_INNFOS_DRIVING_SCA 1

#define DRIVING_WHEEL_1_ID 0x01
#define DRIVING_WHEEL_2_ID 0x02
#define DRIVING_WHEEL_3_ID 0x03
#define STEERING_WHEEL_1_ID 0x04
#define STEERING_WHEEL_2_ID 0x05
#define STEERING_WHEEL_3_ID 0x06

#define CAN_DEVICE_1_NAME "can0"
#define CAN_DEVICE_2_NAME "can1"

#define IQ24     16777216 //2^24

#define PI      3.1415926
#define DEBUG   1

#define IMU_PUBLISH_RATE 10 //hz
#define VEL_PUBLISH_RATE 50 //hz
#define BAT_PUBLISH_RATE 0.2 //hz
#define COMMAND_RATE 50 //hz
#define DEBUG_RATE 1

#define K_P    0.35 // P constant
#define K_I    0.35 // I constant
#define K_D    0.2 // D constant
/** motor param **/
#define MAX_RPM         300 //motor's maximum RPM
#if USE_INNFOS_DRIVING_SCA
#define DRIVING_GEAR_RATIO      36
#define STEERING_GEAR_RATIO      36
#else
#define DRIVING_GEAR_RATIO      8
#define STEERING_GEAR_RATIO      50
#endif
#define WHEEL_DIAMETER  0.12 //wheel's diameter in meters
#define BASE_WIDTH  0.37

#if USE_INNFOS_DRIVING_SCA
#define DRIVING_WHEEL_2_DIR 1
#else
#define DRIVING_WHEEL_2_DIR -1
#endif

#endif // _CONFIG_H_

