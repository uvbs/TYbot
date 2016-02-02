/*
 * DiffDrive.cpp
 *
 *  Created on: 2015年8月27日
 *      Author: gaujei
 */
#include <stdio.h> // printf
#include "../include/DiffDrive.h"
#include "../include/hw_config.h"

double DiffDrive_radius, DiffDrive_speed; // in [mm] ,[mm/s]

namespace asusbot {

	DiffDrive::DiffDrive() :
		radius(0.0), speed(0.0), // command velocities, in [mm] and [mm/s]
		bias(WHEELBASE_m), // wheelbase, wheel_to_wheel, in [m]
		wheel_radius(WHEEL_DIAMETER/1000/2), // radius of main wheel, in [m]
		tick_to_rad(0.000156766f)
		{
		}

	DiffDrive::~DiffDrive(){}

	void DiffDrive::velocityCommands(const double &vx, const double &wz) // vx: in m/s  // wz: in rad/s
	{
		velocity_mutex.lock();
		const double epsilon = 0.0001;

		// Special Case #1 : Straight Run
		if( abs(wz) < epsilon )
		{
			radius = 0.0f;
			speed  = 1000.0f * vx;  //linear_velocity in (mm/s)
			velocity_mutex.unlock();

			DiffDrive_radius = radius;
			DiffDrive_speed = speed;
			//printf("DiffDrive::velocityCommands -> Straight Run!! speed = %f.\n", speed);
			return;
		}

		radius = vx * 1000.0f / wz;  //radius = v / ω = linear_velocity(mm/s) / angular_velocity(rad/s)

		// Special Case #2 : Pure Rotation or Radius is less than or equal to 1.0 mm
		if((abs(vx*1000) < (epsilon*1000)) || (abs(radius) <= 1.0f))
		{
			speed  = 1000.0f * bias * wz / 2.0f;  //v = r x ω＝ (bias/2)*wz
			radius = 1.0f;
			velocity_mutex.unlock();

			DiffDrive_radius = radius;
			DiffDrive_speed = speed;
			//printf("DiffDrive::velocityCommands -> Pure Rotation!! speed = %f, radius = %f.\n",speed ,radius);
			return;
		}

		// General Case : point speed, radius
		if( radius > 0.0f )
		{
			speed  = (radius + 1000.0f * bias / 2.0f) * wz;
		}
		else
		{
			speed  = (radius - 1000.0f * bias / 2.0f) * wz;
		}

		velocity_mutex.unlock();

		DiffDrive_radius = radius;
		DiffDrive_speed = speed;

		printf("DiffDrive::velocityCommands - > speed = %f(mm/s), radius = %f(mm)\n", speed, radius);
		return;
	}
}
