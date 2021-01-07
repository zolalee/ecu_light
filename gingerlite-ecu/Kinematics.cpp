#include <math.h>
#include "Kinematics.h"

Kinematics::Kinematics(int motor_max_rpm, float driving_gear_ratio, float steering_gear_ratio, float wheel_diameter, float base_width)
{
	wheel_diameter_ = wheel_diameter;
	circumference_ = PI * wheel_diameter_;
	max_rpm_ = motor_max_rpm;
	driving_gear_ratio_ = driving_gear_ratio;
	steering_gear_ratio_ = steering_gear_ratio;
	base_width_ = base_width;
}

Kinematics::output Kinematics::getOutput(float linear_x, float linear_y, float angular_z)
{
	Kinematics::output output;
	float V;
	float angular_linear;
	float R1, R2, R3, R_c, R;
	float A1, A2, A3, A;

	R_c = (base_width_ / 2) / sin(PI / 3);

	if(linear_x == 0.0 && linear_y == 0.0 && angular_z == 0.0)
	{
		V1 = V2 = V3 = V = 0.0;
		A1 = A2 = A3 = A = 0.0;
		R1 = R2 = R3 = R = 0.0;
	}
	else if(linear_y == 0.0 && angular_z == 0.0)
	{
		V1 = V2 = V3 = V = linear_x;
		A1 = A2 = A3 = A = 0.0;
		R1 = R2 = R3 = R = 0.0;
	}
	else if(linear_x == 0.0 && angular_z == 0.0)
	{
		V1 = V2 = V3 = V = linear_y;
		A1 = A2 = A3 = A = PI / 2;
		R1 = R2 = R3 = R = 0.0;
	}
	else if(angular_z == 0.0)
	{
		V = sqrt(pow(linear_x, 2) + pow(linear_y, 2));
		if(linear_x < 0.0)
			V = -V;
		A = atan(linear_y / linear_x);
		R1 = R2 = R3 = R = 0.0;
		V1 = V2 = V3 = V;
		A1 = A2 = A3 = A;
	}
	else if(linear_x == 0.0 && linear_y == 0.0)
	{
		V = 0.0;
		A = 0.0;
		R1 = R2 = R3 = R = 0.0;
		V1 = V2 = V3 = R_c * angular_z;
		A1 = (5 * PI) / 6;
		A2 = (1 * PI) / 6;
		A3 = (3 * PI) / 2;
	}
	else if(linear_x == 0.0)
	{
		V = sqrt(pow(linear_x, 2) + pow(linear_y, 2));
		A = PI / 2;
		R = V / angular_z;
		R1 = sqrt(pow(R, 2) + pow(R_c, 2) - 2 * R * R_c * cos(PI / 2 + PI / 3 + A));
		R2 = sqrt(pow(R, 2) + pow(R_c, 2) - 2 * R * R_c * cos(PI / 6 + A));
		R3 = sqrt(pow(R, 2) + pow(R_c, 2) - 2 * R * R_c * cos(PI / 2 - A));
		V1 = R1 * angular_z;
		V2 = R2 * angular_z;
		V3 = R3 * angular_z;
		A1 = PI / 2 - PI / 3 - acos((pow(R_c, 2) + pow(R1, 2) - pow(R, 2)) / (2 * R_c * R1));
		A2 = (PI * 5) / 6 - acos((pow(R_c, 2) + pow(R2, 2) - pow(R, 2)) / (2 * R_c * R2));
		A3 = PI / 2 - acos((pow(R_c, 2) + pow(R3, 2) - pow(R, 2)) / (2 * R_c * R3));
	}
	else
	{
		V = sqrt(pow(linear_x, 2) + pow(linear_y, 2));
		A = atan(linear_y / linear_x);
		if(linear_x < 0.0 && linear_y < 0.0)
			A = -A;
		R = V / angular_z;
		R1 = sqrt(pow(R, 2) + pow(R_c, 2) - 2 * R * R_c * cos(PI / 2 + PI / 3 + A));
		R2 = sqrt(pow(R, 2) + pow(R_c, 2) - 2 * R * R_c * cos(PI / 6 + A));
		R3 = sqrt(pow(R, 2) + pow(R_c, 2) - 2 * R * R_c * cos(PI / 2 - A));
		V1 = R1 * angular_z;
		V2 = R2 * angular_z;
		V3 = R3 * angular_z;
		A1 = PI / 2 - PI / 3 - acos((pow(R_c, 2) + pow(R1, 2) - pow(R, 2)) / (2 * R_c * R1));
		A2 = (PI * 5) / 6 - acos((pow(R_c, 2) + pow(R2, 2) - pow(R, 2)) / (2 * R_c * R2));
		A3 = PI / 2 - acos((pow(R_c, 2) + pow(R3, 2) - pow(R, 2)) / (2 * R_c * R3));
	}

#if USE_INNFOS_DRIVING_SCA
	if(A1 > (PI * 2 / 3) && A1 < (2 * PI * 5 / 6))
	{
		A1 = A1 - PI;
		V1 = -V1;
	}
	else if(A1 >= (2 * PI * 5 / 6) && A1 < (2 * PI))
	{
		A1 = A1 - (2 * PI);
	}

	if(A2 > (PI * 2 / 3) && A2 < (2 * PI * 5 / 6))
	{
		A2 = A2 - PI;
		V2 = -V2;
	}
	else if(A2 >= (2 * PI * 5 / 6) && A2 < (2 * PI))
	{
		A2 = A2 - (2 * PI);
	}

	if(A3 > (PI * 2 / 3) && A3 < (2 * PI * 5 / 6))
	{
		A3 = A3 - PI;
		V3 = -V3;
	}
	else if(A3 >= (2 * PI * 5 / 6) && A3 < (2 * PI))
	{
		A3 = A3 - (2 * PI);
	}

	A1 += (PI / 6);
	V1 = -V1;
	if(A1 > (PI / 2))
	{
		A1 -= PI;
		V1 = -V1;
	}
	A2 -= (PI / 6);
	A3 -= (PI / 2);
	V3 = -V3;
	if(A3 < -(PI / 2))
	{
		A3 += PI;
		V3 = -V3;
	}
#else
	if(A1 > (PI / 2) && A1 <= PI)
	{
		A1 = A1 - PI;
		V1 = -V1;
	}
	else if(A1 > PI && A1 <= (PI * 3 / 2))
	{
		A1 = (PI * 3 / 2) - A1;
		V1 = -V1;
	}
	else if(A1 > (PI * 3 / 2) && A1 <= (PI * 2))
	{
		A1 = A1 - (PI * 2);
	}

	if(A2 > (PI / 2) && A2 <= PI)
	{
		A2 = A2 - PI;
		V2 = -V2;
	}
	else if(A2 > PI && A2 <= (PI * 3 / 2))
	{
		A2 = (PI * 3 / 2) - A2;
		V2 = -V2;
	}
	else if(A2 > (PI * 3 / 2) && A2 <= (PI * 2))
	{
		A2 = A2 - (PI * 2);
	}

	if(A3 > (PI / 2) && A3 <= PI)
	{
		A3 = A3 - PI;
		V3 = -V3;
	}
	else if(A3 > PI && A3 <= (PI * 3 / 2))
	{
		A3 = (PI * 3 / 2) - A3;
		V3 = -V3;
	}
	else if(A3 > (PI * 3 / 2) && A3 <= (PI * 2))
	{
		A3 = A3 - (PI * 2);
	}
#endif

#if 0
	printf ("V is %f m/s\n", V);
	printf ("A is %f rad\n", A);
	printf ("R is %f m\n", R);
	printf ("R1 is %f m\n", R1);
	printf ("R2 is %f m\n", R2);
	printf ("R3 is %f m\n", R3);
	printf ("V1 is %f m/s\n", V1);
	printf ("V2 is %f m/s\n", V2);
	printf ("V3 is %f m/s\n", V3);
	printf ("A1 is %f rad\n", A1);
	printf ("A2 is %f rad\n", A2);
	printf ("A3 is %f rad\n", A3);
#endif

	//calculate for the target motor RPM and direction
#if USE_INNFOS_DRIVING_SCA
	//front left motor
	output.rpm1 = ((V1 * 60) / (PI * WHEEL_DIAMETER)) * DRIVING_GEAR_RATIO / 6000 * IQ24;
	//front right motor
	output.rpm2 = ((V2 * 60) / (PI * WHEEL_DIAMETER)) * DRIVING_GEAR_RATIO / 6000 * IQ24;
	output.rpm2 = DRIVING_WHEEL_2_DIR * output.rpm2;
	//back center motor
	output.rpm3 = ((V3 * 60) / (PI * WHEEL_DIAMETER)) * DRIVING_GEAR_RATIO / 6000 * IQ24;
#else
	//front left motor
	output.rpm1 = ((V1 * 60) / (PI * WHEEL_DIAMETER)) * DRIVING_GEAR_RATIO;
	//front right motor
	output.rpm2 = ((V2 * 60) / (PI * WHEEL_DIAMETER)) * DRIVING_GEAR_RATIO;
	output.rpm2 = DRIVING_WHEEL_2_DIR * output.rpm2;
	//back center motor
	output.rpm3 = ((V3 * 60) / (PI * WHEEL_DIAMETER)) * DRIVING_GEAR_RATIO;
#endif
	//front left steering
	output.position1 = (A1 / (2 * PI)) * STEERING_GEAR_RATIO * IQ24;
	//front right steering
	output.position2 = (A2 / (2 * PI)) * STEERING_GEAR_RATIO * IQ24;
	//back center steering
	output.position3 = (A3 / (2 * PI)) * STEERING_GEAR_RATIO * IQ24;

	return output;
}

#if USE_INNFOS_DRIVING_SCA
Kinematics::velocities Kinematics::getVelocities(int rpm1, int rpm2, int rpm3, int position1, int position2, int position3)
#else
Kinematics::velocities Kinematics::getVelocities(signed short rpm1, signed short rpm2, signed short rpm3, int position1, int position2, int position3)
#endif
{
	Kinematics::velocities vel;

	return vel;
}
