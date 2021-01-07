#include "can_interface.h"
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <iostream>
#include <stdlib.h>
#include <string>
#include <arpa/inet.h>
#include <sys/socket.h>

#include <ros.h>
#include <beetle_msgs/Velocities.h>
#include <beetle_msgs/Battery.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/JointState.h>
#include <ros/time.h>
#include "Kinematics.h"

can_device_t vcu_can;
can_obj_t send_obj;
can_device_t vcu_can2;
can_obj_t send_obj2;
double required_angular_vel = 0;
double required_linear_x_vel = 0;
double required_linear_y_vel = 0;
uint64_t previous_command_time = 0;
float driving_velocity1 = 0;
float driving_velocity2 = 0;
float driving_velocity3 = 0;
float steering_position1 = 0;
float steering_position2 = 0;
float steering_position3 = 0;
float steering_effort1 = 0;
float steering_effort2 = 0;
float steering_effort3 = 0;

#if USE_INNFOS_DRIVING_SCA
int rpm1, rpm2, rpm3;
#else
signed short rpm1, rpm2, rpm3;
#endif
int position1, position2, position3;

uint8_t modbus_enable[] = {0xff, 0x06, 0x00, 0x01, 0x01, 0x00};
uint8_t target_speed[] = {0xff, 0x06, 0x02, 0x01, 0x00, 0x00};
uint8_t current_speed[] = {0xff, 0x03, 0x10, 0x01};

uint8_t sca_active[] = {0x2a, 0x01};
uint8_t sca_position_mode[] = {0x07, 0x06};
uint8_t sca_target_position[] = {0x0a, 0x00, 0x00, 0x00, 0x00};
uint8_t sca_current_position[] = {0x06};
#if USE_INNFOS_DRIVING_SCA
uint8_t sca_speed_mode[] = {0x07, 0x07};
uint8_t sca_target_speed[] = {0x09, 0x00, 0x00, 0x00, 0x00};
uint8_t sca_current_speed[] = {0x05};
#endif

Kinematics kinematics(MAX_RPM, DRIVING_GEAR_RATIO, STEERING_GEAR_RATIO, WHEEL_DIAMETER, BASE_WIDTH);
ros::NodeHandle  nh;
beetle_msgs::Velocities raw_vel_msg;
sensor_msgs::JointState raw_joint_states_msg;
void command_callback( const geometry_msgs::Twist& cmd_msg);
ros::Subscriber<geometry_msgs::Twist> cmd_sub("cmd_vel", command_callback);
void joint_states_callback( const sensor_msgs::JointState& joint_states_msg);
ros::Subscriber<sensor_msgs::JointState> joint_states_sub("joint_states", joint_states_callback);
ros::Publisher raw_vel_pub("raw_vel", &raw_vel_msg);
ros::Publisher raw_joint_states_pub("raw_joint_states", &raw_joint_states_msg);

using namespace std;

unsigned long millis()
{
	unsigned long msec = 0;
    	struct timeval msectime;
    	gettimeofday(&msectime, NULL);
    	msec =msectime.tv_sec * 1000 + msectime.tv_usec / 1000;
	return msec;
}

void command_callback( const geometry_msgs::Twist& cmd_msg)
{
	Kinematics::output output;

	required_linear_x_vel = cmd_msg.linear.x;
	required_linear_y_vel = cmd_msg.linear.y;
	required_angular_vel = cmd_msg.angular.z;
	
#if 1
	char log_buffer[256];

	sprintf(log_buffer, "required_linear_x_vel: %f, required_linear_y_vel: %f, required_angular_vel: %f", required_linear_x_vel, required_linear_y_vel, required_angular_vel);
	nh.loginfo(log_buffer);
#endif
	output = kinematics.getOutput(required_linear_x_vel, required_linear_y_vel, required_angular_vel);
#if 1
#if USE_INNFOS_DRIVING_SCA
	sprintf(log_buffer, "rpm1: %f / 6000 * IQ24, rpm2: %f / 6000 * IQ24, rpm3: %f / 6000 * IQ24, position1: %f * IQ24, position2: %f * IQ24, position3: %f * IQ24", (float)output.rpm1 / IQ24 * 6000, (float)output.rpm2 / IQ24 * 6000, (float)output.rpm3 / IQ24 * 6000, (float)output.position1 / IQ24, (float)output.position2 / IQ24, (float)output.position3 / IQ24);
	nh.loginfo(log_buffer);
#else
	sprintf(log_buffer, "rpm1: %d, rpm2: %d, rpm3: %d, position1: %f * IQ24, position2: %f * IQ24, position3: %f * IQ24", output.rpm1, output.rpm2, output.rpm3, (float)output.position1 / IQ24, (float)output.position2 / IQ24, (float)output.position3 / IQ24);
	nh.loginfo(log_buffer);
#endif
#endif

#if 1
	rpm1 = output.rpm1;
	rpm2 = output.rpm2;
	rpm3 = output.rpm3;
	position1 = output.position1;
	position2 = output.position2;
	position3 = output.position3;
#if USE_INNFOS_DRIVING_SCA

#else
	if((position1 < -12.5 * IQ24) || (position1 > 12.5 * IQ24))
		position1 = 0;
	if((position2 < -12.5 * IQ24) || (position2 > 12.5 * IQ24))
		position2 = 0;
	if((position3 < -12.5 * IQ24) || (position3 > 12.5 * IQ24))
		position3 = 0;
#endif
#endif
	previous_command_time = millis();
}

void joint_states_callback( const sensor_msgs::JointState& joint_states_msg)
{
#if 1
	char log_buffer[512];

	sprintf(log_buffer, "joint_states_msg: name[0] = %s, name[1] = %s, name[2] = %s, position[0] = %f, position[1] = %f, position[2] = %f, velocity[0] = %f, velocity[1] = %f, velocity[2] = %f, effort[0] = %f, effort[1] = %f, effort[2] = %f", joint_states_msg.name[0], joint_states_msg.name[1], joint_states_msg.name[2], joint_states_msg.position[0], joint_states_msg.position[1], joint_states_msg.position[2], joint_states_msg.velocity[0], joint_states_msg.velocity[1], joint_states_msg.velocity[2], joint_states_msg.effort[0], joint_states_msg.effort[1], joint_states_msg.effort[2]);
	nh.loginfo(log_buffer);
#endif
#if USE_INNFOS_DRIVING_SCA
	rpm1 = ((joint_states_msg.velocity[0] * 60) / (PI * WHEEL_DIAMETER)) * DRIVING_GEAR_RATIO / 6000 * IQ24;
	rpm2 = ((joint_states_msg.velocity[1] * 60) / (PI * WHEEL_DIAMETER)) * DRIVING_GEAR_RATIO / 6000 * IQ24;
	rpm2 = DRIVING_WHEEL_2_DIR * rpm2;
	rpm3 = ((joint_states_msg.velocity[2] * 60) / (PI * WHEEL_DIAMETER)) * DRIVING_GEAR_RATIO / 6000 * IQ24;
#else
	rpm1 = ((joint_states_msg.velocity[0] * 60) / (PI * WHEEL_DIAMETER)) * DRIVING_GEAR_RATIO;
	rpm2 = ((joint_states_msg.velocity[1] * 60) / (PI * WHEEL_DIAMETER)) * DRIVING_GEAR_RATIO;
	rpm2 = DRIVING_WHEEL_2_DIR * rpm2;
	rpm3 = ((joint_states_msg.velocity[2] * 60) / (PI * WHEEL_DIAMETER)) * DRIVING_GEAR_RATIO;
#endif
	position1 = (joint_states_msg.position[0] / (2 * PI)) * STEERING_GEAR_RATIO * IQ24;
	position2 = (joint_states_msg.position[1] / (2 * PI)) * STEERING_GEAR_RATIO * IQ24;
	position3 = (joint_states_msg.position[2] / (2 * PI)) * STEERING_GEAR_RATIO * IQ24;

#if 1
	sprintf(log_buffer, "rpm1 = %d, rpm2 = %d, rpm3 = %d, position1 = %d, position2 = %d, position3 = %d", rpm1, rpm2, rpm3, position1, position2, position3);
	nh.loginfo(log_buffer);
#endif

	previous_command_time = millis();
}

void publish_linear_velocity()
{
	Kinematics::velocities vel;
#if 0
	char log_buffer[256];

	sprintf(log_buffer, "encoder_velocity1: %d, encoder_velocity2: %d", encoder_velocity1, encoder_velocity2);
	nh.loginfo(log_buffer);
#endif
	//vel = kinematics.getVelocities(encoder_velocity1, encoder_velocity2, encoder_velocity3);

	//fill in the object
	raw_vel_msg.linear_x = vel.linear_x;
	raw_vel_msg.linear_y = vel.linear_y;
	raw_vel_msg.angular_z = vel.angular_z;

	//publish raw_vel_msg object to ROS
	raw_vel_pub.publish(&raw_vel_msg);
}

void publish_raw_joint_states()
{
	char *name[] = {(char *)"front_left", (char *)"front_right", (char *)"back_center"};
	float vel[]={driving_velocity1, -driving_velocity2, driving_velocity3};
	float pos[]={steering_position1, steering_position2, steering_position3};
	float eff[]={steering_effort1, steering_effort2, steering_effort3};

	//fill in the object
	raw_joint_states_msg.header.frame_id = "";
	raw_joint_states_msg.name_length = 3;
	raw_joint_states_msg.velocity_length = 3;
	raw_joint_states_msg.position_length = 3;
	raw_joint_states_msg.effort_length = 3;
	raw_joint_states_msg.name = name;
	raw_joint_states_msg.velocity = vel;
	raw_joint_states_msg.position = pos;
	raw_joint_states_msg.effort = eff;

	//publish raw_joint_states_msg object to ROS
	raw_joint_states_pub.publish(&raw_joint_states_msg);
}


#if USE_INNFOS_DRIVING_SCA
void UpdateVelocity1(int32_t velocity)
{
	driving_velocity1 = (float)velocity / IQ24 * 6000 / DRIVING_GEAR_RATIO / 60 * (PI * WHEEL_DIAMETER);
}

void UpdateVelocity2(int32_t velocity)
{
	driving_velocity2 = (float)velocity / IQ24 * 6000 / DRIVING_GEAR_RATIO / 60 * (PI * WHEEL_DIAMETER);
}

void UpdateVelocity3(int32_t velocity)
{
	driving_velocity3 = (float)velocity / IQ24 * 6000 / DRIVING_GEAR_RATIO / 60 * (PI * WHEEL_DIAMETER);
}
#else
void UpdateVelocity1(int16_t velocity)
{
	driving_velocity1 = (float)velocity / 10 / DRIVING_GEAR_RATIO / 60 * (PI * WHEEL_DIAMETER);
}

void UpdateVelocity2(int16_t velocity)
{
	driving_velocity2 = (float)velocity / 10 / DRIVING_GEAR_RATIO / 60 * (PI * WHEEL_DIAMETER);
}

void UpdateVelocity3(int16_t velocity)
{
	driving_velocity3 = (float)velocity / 10 / DRIVING_GEAR_RATIO / 60 * (PI * WHEEL_DIAMETER);
}
#endif

void UpdatePosition1(int32_t position)
{
	steering_position1 = (float)position / IQ24 / STEERING_GEAR_RATIO * (2 * PI);
}

void UpdatePosition2(int32_t position)
{
	steering_position2 = (float)position / IQ24 / STEERING_GEAR_RATIO * (2 * PI);
}

void UpdatePosition3(int32_t position)
{
	steering_position3 = (float)position / IQ24 / STEERING_GEAR_RATIO * (2 * PI);
}

#if 1
void canreceive(int can_id, uint8_t *buf, int buf_len)
{
#if USE_INNFOS_DRIVING_SCA
	if(can_id == DRIVING_WHEEL_1_ID)
	{
		if(buf_len == 5 && buf[0] == 0x05)
			UpdateVelocity1((buf[1] << 24) | (buf[2] << 16) | (buf[3] << 8) | buf[4]);
	}
	if(can_id == DRIVING_WHEEL_2_ID)
	{
		if(buf_len == 5 && buf[0] == 0x05)
			UpdateVelocity2((buf[1] << 24) | (buf[2] << 16) | (buf[3] << 8) | buf[4]);
	}
	if(can_id == DRIVING_WHEEL_3_ID)
	{
		if(buf_len == 5 && buf[0] == 0x05)
			UpdateVelocity3((buf[1] << 24) | (buf[2] << 16) | (buf[3] << 8) | buf[4]);
	}
#else
	if(can_id == 0xFF)
	{
		if(buf_len == 6 && buf[0] == DRIVING_WHEEL_1_ID && buf[1] == 0x03 && buf[2] == 0x10 && buf[3] == 0x01)
			UpdateVelocity1((buf[5] << 8) | buf[4]);
		if(buf_len == 6 && buf[0] == DRIVING_WHEEL_2_ID && buf[1] == 0x03 && buf[2] == 0x10 && buf[3] == 0x01)
			UpdateVelocity2((buf[5] << 8) | buf[4]);
		if(buf_len == 6 && buf[0] == DRIVING_WHEEL_3_ID && buf[1] == 0x03 && buf[2] == 0x10 && buf[3] == 0x01)
			UpdateVelocity3((buf[5] << 8) | buf[4]);
	}
#endif
}
#else
void canreceive(int can_id, uint8_t *buf, int buf_len)
{
    int i;

    printf("ID: 0x%04X,LEN: %02d\t", can_id, buf_len);
    for (i = 0; i < buf_len; i++)
    {
        printf("%02X  ", *(buf + i));
    }
    printf("\r\n");
}
#endif

#if 1
void canreceive2(int can_id, uint8_t *buf, int buf_len)
{
	if(can_id == STEERING_WHEEL_1_ID)
	{
		if(buf_len == 5 && buf[0] == 0x06)
			UpdatePosition1((buf[1] << 24) | (buf[2] << 16) | (buf[3] << 8) | buf[4]);
	}
	if(can_id == STEERING_WHEEL_2_ID)
	{
		if(buf_len == 5 && buf[0] == 0x06)
			UpdatePosition2((buf[1] << 24) | (buf[2] << 16) | (buf[3] << 8) | buf[4]);
	}
	if(can_id == STEERING_WHEEL_3_ID)
	{
		if(buf_len == 5 && buf[0] == 0x06)
			UpdatePosition3((buf[1] << 24) | (buf[2] << 16) | (buf[3] << 8) | buf[4]);
	}
}
#else
void canreceive2(int can_id, uint8_t *buf, int buf_len)
{
    int i;

    printf("ID: 0x%04X,LEN: %02d\t", can_id, buf_len);
    for (i = 0; i < buf_len; i++)
    {
        printf("%02X  ", *(buf + i));
    }
    printf("\r\n");
}
#endif

void driving_motor_init(void)
{
#if USE_INNFOS_DRIVING_SCA
	send_obj.id = DRIVING_WHEEL_1_ID;
	memcpy(&send_obj.data_buf[0], &sca_active[0], sizeof(sca_active));
	send_obj.data_len = sizeof(sca_active);
	can_transimit(&vcu_can, &send_obj, 1);
	usleep(100);

	send_obj.id = DRIVING_WHEEL_2_ID;
	memcpy(&send_obj.data_buf[0], &sca_active[0], sizeof(sca_active));
	send_obj.data_len = sizeof(sca_active);
	can_transimit(&vcu_can, &send_obj, 1);
	usleep(100);

	send_obj.id = DRIVING_WHEEL_3_ID;
	memcpy(&send_obj.data_buf[0], &sca_active[0], sizeof(sca_active));
	send_obj.data_len = sizeof(sca_active);
	can_transimit(&vcu_can, &send_obj, 1);
	usleep(100);

	sleep(3);

	send_obj.id = DRIVING_WHEEL_1_ID;
	memcpy(&send_obj.data_buf[0], &sca_speed_mode[0], sizeof(sca_speed_mode));
	send_obj.data_len = sizeof(sca_speed_mode);
	can_transimit(&vcu_can, &send_obj, 1);
	usleep(100);

	send_obj.id = DRIVING_WHEEL_2_ID;
	memcpy(&send_obj.data_buf[0], &sca_speed_mode[0], sizeof(sca_speed_mode));
	send_obj.data_len = sizeof(sca_speed_mode);
	can_transimit(&vcu_can, &send_obj, 1);
	usleep(100);

	send_obj.id = DRIVING_WHEEL_3_ID;
	memcpy(&send_obj.data_buf[0], &sca_speed_mode[0], sizeof(sca_speed_mode));
	send_obj.data_len = sizeof(sca_speed_mode);
	can_transimit(&vcu_can, &send_obj, 1);
	usleep(100);
#else
	send_obj.id = DRIVING_WHEEL_1_ID;
	memcpy(&send_obj.data_buf[0], &modbus_enable[0], sizeof(modbus_enable));
	send_obj.data_len = sizeof(modbus_enable);
	can_transimit(&vcu_can, &send_obj, 1);
	usleep(100);

	send_obj.id = DRIVING_WHEEL_2_ID;
	memcpy(&send_obj.data_buf[0], &modbus_enable[0], sizeof(modbus_enable));
	send_obj.data_len = sizeof(modbus_enable);
	can_transimit(&vcu_can, &send_obj, 1);
	usleep(100);

	send_obj.id = DRIVING_WHEEL_3_ID;
	memcpy(&send_obj.data_buf[0], &modbus_enable[0], sizeof(modbus_enable));
	send_obj.data_len = sizeof(modbus_enable);
	can_transimit(&vcu_can, &send_obj, 1);
	usleep(100);
#endif
}

void steering_motor_init(void)
{
	send_obj2.id = STEERING_WHEEL_1_ID;
	memcpy(&send_obj2.data_buf[0], &sca_active[0], sizeof(sca_active));
	send_obj2.data_len = sizeof(sca_active);
	can_transimit(&vcu_can2, &send_obj2, 1);
	usleep(100);

	send_obj2.id = STEERING_WHEEL_2_ID;
	memcpy(&send_obj2.data_buf[0], &sca_active[0], sizeof(sca_active));
	send_obj2.data_len = sizeof(sca_active);
	can_transimit(&vcu_can2, &send_obj2, 1);
	usleep(100);

	send_obj2.id = STEERING_WHEEL_3_ID;
	memcpy(&send_obj2.data_buf[0], &sca_active[0], sizeof(sca_active));
	send_obj2.data_len = sizeof(sca_active);
	can_transimit(&vcu_can2, &send_obj2, 1);
	usleep(100);

	sleep(3);

	send_obj2.id = STEERING_WHEEL_1_ID;
	memcpy(&send_obj2.data_buf[0], &sca_position_mode[0], sizeof(sca_position_mode));
	send_obj2.data_len = sizeof(sca_position_mode);
	can_transimit(&vcu_can2, &send_obj2, 1);
	usleep(100);

	send_obj2.id = STEERING_WHEEL_2_ID;
	memcpy(&send_obj2.data_buf[0], &sca_position_mode[0], sizeof(sca_position_mode));
	send_obj2.data_len = sizeof(sca_position_mode);
	can_transimit(&vcu_can2, &send_obj2, 1);
	usleep(100);

	send_obj2.id = STEERING_WHEEL_3_ID;
	memcpy(&send_obj2.data_buf[0], &sca_position_mode[0], sizeof(sca_position_mode));
	send_obj2.data_len = sizeof(sca_position_mode);
	can_transimit(&vcu_can2, &send_obj2, 1);
	usleep(100);
}

void move_base()
{
#if 1
#if USE_INNFOS_DRIVING_SCA
	send_obj.id = DRIVING_WHEEL_1_ID;
	memcpy(&send_obj.data_buf[0], &sca_target_speed[0], sizeof(sca_target_speed));
	send_obj.data_buf[1] = (rpm1 >> 24) & 0xff;
	send_obj.data_buf[2] = (rpm1 >> 16) & 0xff;
	send_obj.data_buf[3] = (rpm1 >> 8) & 0xff;
	send_obj.data_buf[4] = rpm1 & 0xff;
	send_obj.data_len = sizeof(sca_target_speed);
	can_transimit(&vcu_can, &send_obj, 1);
	usleep(100);
	send_obj.id = DRIVING_WHEEL_2_ID;
	memcpy(&send_obj.data_buf[0], &sca_target_speed[0], sizeof(sca_target_speed));
	send_obj.data_buf[1] = (rpm2 >> 24) & 0xff;
	send_obj.data_buf[2] = (rpm2 >> 16) & 0xff;
	send_obj.data_buf[3] = (rpm2 >> 8) & 0xff;
	send_obj.data_buf[4] = rpm2 & 0xff;
	send_obj.data_len = sizeof(sca_target_speed);
	can_transimit(&vcu_can, &send_obj, 1);
	usleep(100);
	send_obj.id = DRIVING_WHEEL_3_ID;
	memcpy(&send_obj.data_buf[0], &sca_target_speed[0], sizeof(sca_target_speed));
	send_obj.data_buf[1] = (rpm3 >> 24) & 0xff;
	send_obj.data_buf[2] = (rpm3 >> 16) & 0xff;
	send_obj.data_buf[3] = (rpm3 >> 8) & 0xff;
	send_obj.data_buf[4] = rpm3 & 0xff;
	send_obj.data_len = sizeof(sca_target_speed);
	can_transimit(&vcu_can, &send_obj, 1);
	usleep(100);
#else
	send_obj.id = DRIVING_WHEEL_1_ID;
	memcpy(&send_obj.data_buf[0], &target_speed[0], sizeof(target_speed));
	send_obj.data_buf[4] = rpm1 & 0xff;
	send_obj.data_buf[5] = (rpm1 >> 8) & 0xff;
	send_obj.data_len = sizeof(target_speed);
	can_transimit(&vcu_can, &send_obj, 1);
	usleep(100);
	send_obj.id = DRIVING_WHEEL_2_ID;
	memcpy(&send_obj.data_buf[0], &target_speed[0], sizeof(target_speed));
	send_obj.data_buf[4] = rpm2 & 0xff;
	send_obj.data_buf[5] = (rpm2 >> 8) & 0xff;
	send_obj.data_len = sizeof(target_speed);
	can_transimit(&vcu_can, &send_obj, 1);
	usleep(100);
	send_obj.id = DRIVING_WHEEL_3_ID;
	memcpy(&send_obj.data_buf[0], &target_speed[0], sizeof(target_speed));
	send_obj.data_buf[4] = rpm3 & 0xff;
	send_obj.data_buf[5] = (rpm3 >> 8) & 0xff;
	send_obj.data_len = sizeof(target_speed);
	can_transimit(&vcu_can, &send_obj, 1);
	usleep(100);
#endif
#if 1
	send_obj2.id = STEERING_WHEEL_1_ID;
	memcpy(&send_obj2.data_buf[0], &sca_target_position[0], sizeof(sca_target_position));
	send_obj2.data_buf[1] = (position1 >> 24) & 0xff;
	send_obj2.data_buf[2] = (position1 >> 16) & 0xff;
	send_obj2.data_buf[3] = (position1 >> 8) & 0xff;
	send_obj2.data_buf[4] = position1 & 0xff;
	send_obj2.data_len = sizeof(sca_target_position);
	can_transimit(&vcu_can2, &send_obj2, 1);
	usleep(100);
	send_obj2.id = STEERING_WHEEL_2_ID;
	memcpy(&send_obj2.data_buf[0], &sca_target_position[0], sizeof(sca_target_position));
	send_obj2.data_buf[1] = (position2 >> 24) & 0xff;
	send_obj2.data_buf[2] = (position2 >> 16) & 0xff;
	send_obj2.data_buf[3] = (position2 >> 8) & 0xff;
	send_obj2.data_buf[4] = position2 & 0xff;
	send_obj2.data_len = sizeof(sca_target_position);
	can_transimit(&vcu_can2, &send_obj2, 1);
	usleep(100);
	send_obj2.id = STEERING_WHEEL_3_ID;
	memcpy(&send_obj2.data_buf[0], &sca_target_position[0], sizeof(sca_target_position));
	send_obj2.data_buf[1] = (position3 >> 24) & 0xff;
	send_obj2.data_buf[2] = (position3 >> 16) & 0xff;
	send_obj2.data_buf[3] = (position3 >> 8) & 0xff;
	send_obj2.data_buf[4] = position3 & 0xff;
	send_obj2.data_len = sizeof(sca_target_position);
	can_transimit(&vcu_can2, &send_obj2, 1);
	usleep(100);
#endif
#endif
}

void stop_base()
{
	Kinematics::output output;

	required_linear_x_vel = 0;
	required_linear_y_vel = 0;
	required_angular_vel = 0;
		
	output = kinematics.getOutput(required_linear_x_vel, required_linear_y_vel, required_angular_vel);
	rpm1 = output.rpm1;
	rpm2 = output.rpm2;
	rpm3 = output.rpm3;
	position1 = output.position1;
	position2 = output.position2;
	position3 = output.position3;
}

void update_velocity(void)
{
	send_obj.id = DRIVING_WHEEL_1_ID;
	memcpy(&send_obj.data_buf[0], &current_speed[0], sizeof(current_speed));
	send_obj.data_len = sizeof(current_speed);
	can_transimit(&vcu_can, &send_obj, 1);
	usleep(100);
	send_obj.id = DRIVING_WHEEL_2_ID;
	memcpy(&send_obj.data_buf[0], &current_speed[0], sizeof(current_speed));
	send_obj.data_len = sizeof(current_speed);
	can_transimit(&vcu_can, &send_obj, 1);
	usleep(100);
	send_obj.id = DRIVING_WHEEL_3_ID;
	memcpy(&send_obj.data_buf[0], &current_speed[0], sizeof(current_speed));
	send_obj.data_len = sizeof(current_speed);
	can_transimit(&vcu_can, &send_obj, 1);
	usleep(100);
}

void update_position(void)
{
	send_obj2.id = STEERING_WHEEL_1_ID;
	memcpy(&send_obj2.data_buf[0], &sca_current_position[0], sizeof(sca_current_position));
	send_obj2.data_len = sizeof(sca_current_position);
	can_transimit(&vcu_can2, &send_obj2, 1);
	usleep(100);
	send_obj2.id = STEERING_WHEEL_2_ID;
	memcpy(&send_obj2.data_buf[0], &sca_current_position[0], sizeof(sca_current_position));
	send_obj2.data_len = sizeof(sca_current_position);
	can_transimit(&vcu_can2, &send_obj2, 1);
	usleep(100);
	send_obj2.id = STEERING_WHEEL_3_ID;
	memcpy(&send_obj2.data_buf[0], &sca_current_position[0], sizeof(sca_current_position));
	send_obj2.data_len = sizeof(sca_current_position);
	can_transimit(&vcu_can2, &send_obj2, 1);
	usleep(100);
}

int main()
{
    int ret;
    uint64_t previous_control_time = 0;
    uint64_t publish_vel_time = 0;

    ret = can_init(&vcu_can, (char *)CAN_DEVICE_1_NAME, canreceive);
    if(ret < 0 )
    {
        printf(">> %s device init error!\r\n", CAN_DEVICE_1_NAME);
        return 0;
    }
    can_open(&vcu_can);

    ret = can_init(&vcu_can2, (char *)CAN_DEVICE_2_NAME, canreceive2);
    if(ret < 0 )
    {
        printf(">> %s device init error!\r\n", CAN_DEVICE_2_NAME);
        return 0;
    }
    can_open(&vcu_can2);

    driving_motor_init();
    steering_motor_init();
    nh.getHardware()->setConnection(192, 168, 1, 200, 11411);
    nh.initNode();
    nh.advertise(raw_vel_pub);
    nh.advertise(raw_joint_states_pub);
    nh.subscribe(cmd_sub);
    nh.subscribe(joint_states_sub);

    while (!nh.connected()){
	nh.spinOnce();
    }
    nh.loginfo("GingerLite_base Connected!");

    while(1){
	if ((millis() - previous_control_time) >= (1000 / COMMAND_RATE)){
		move_base();
		update_velocity();
		update_position();
		previous_control_time = millis();
	}
	if ((millis() - previous_command_time) >= 400){
		stop_base();
	}
	if ((millis() - publish_vel_time) >= (1000 / VEL_PUBLISH_RATE)){
		//publish_linear_velocity();
		publish_raw_joint_states();
		publish_vel_time = millis();
	}
	nh.spinOnce();
        usleep(10000);
    }

    can_close(&vcu_can);
    can_close(&vcu_can2);
    return 0;
}

