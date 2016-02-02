/*
 * Parameters.h
 *
 *  Created on: 2015年8月27日
 *      Author: mint
 */

#ifndef INCLUDE_PARAMETERS_H_
#define INCLUDE_PARAMETERS_H_

namespace asusbot
{
	class Parameters
	{
		public:

			Parameters():
				device_port("/dev/ttyACM0")
				{
				}

			const char* device_port;

			static const int asusbit_timeout = 3000; 	//[in ms]
			static const double Pi = 3.14159265357989;
			static const double bias = 260.0; 			//[in mm]
			static const double gear_ratio = 30.0; 		// motor to wheel = 17 : 1
			static const int rev_to_tick = 1336; 		// ticks per revolution
			static const double tick_to_rad = 0.000156766; //  = (2 * Pi) / (gear_ratio * rev_to_tick);
			static const double wheel_radius = 123.5; 	// [in mm]
			static const int checkTimer = 50;			// Check the status of command/motor after 50 msec, since

	};

}

#endif /* INCLUDE_PARAMETERS_H_ */
