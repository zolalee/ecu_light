#ifndef KINEMATICS_H
#define KINEMATICS_H

#include "config.h"

class Kinematics
{
	public:
		struct output {
#if USE_INNFOS_DRIVING_SCA
			int rpm1;
			int rpm2;
			int rpm3;
#else
			signed short rpm1;
			signed short rpm2;
			signed short rpm3;
#endif
			int position1;
			int position2;
			int position3;
		};
		struct velocities {
			float linear_x;
			float linear_y;
			float angular_z;
		};
	Kinematics(int motor_max_rpm, float driving_gear_ratio , float steering_gear_ratio, float wheel_diameter, float base_width);
#if USE_INNFOS_DRIVING_SCA
	velocities getVelocities(int rpm1, int rpm2, int rpm3, int position1, int position2, int position3);
#else
	velocities getVelocities(signed short rpm1, signed short rpm2, signed short rpm3, int position1, int position2, int position3);
#endif
	output getOutput(float linear_x, float linear_y, float angular_z);

	private:
		float linear_vel_x_mins_;
		float linear_vel_y_mins_;
		float angular_vel_z_mins_;
		float circumference_;
		float tangential_vel_;
		float x_rpm_;
		float y_rpm_;
		float tan_rpm_;
		int max_rpm_;
		float driving_gear_ratio_;
		float steering_gear_ratio_;
		float wheel_diameter_;
		float base_width_;
		float V1;
		float V2;
		float V3;
};

#endif
