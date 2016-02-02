/*
 * DiffDrive.h
 *
 *  Created on: 2015年8月27日
 *      Author: gaujei
 */

#ifndef DIFFDRIVE_H_
#define DIFFDRIVE_H_

#include <stdlib.h>
#include <vector>
#include "Mutex.h"

namespace asusbot {

	class DiffDrive {

		public:
		  DiffDrive();
		   virtual ~DiffDrive();

		  void velocityCommands(const double &vx, const double &wz);

		private:
		  Mutex velocity_mutex, state_mutex;

		  double radius; // in [mm]
		  double speed;  // in [mm/s]
		  double bias; //wheelbase, wheel_to_wheel, in [m]
		  double wheel_radius; // in [m]
		  const double tick_to_rad;
		  	};

}

#endif
