//
// Created by gaujei on 11/30/15.
//

/**
  ******************************************************************************
  * @file    hw_config.h
  * @author
  * @version V1.1.1
  * @date    27-Sep-2015
  * @brief   Hardware Configuration & Setup
  ******************************************************************************
  * @attention
  *
  *
  *
  ******************************************************************************
  */
#ifndef __HW_CONFIG_H
#define __HW_CONFIG_H

/**------------------------------ Includes ------------------------------
*/
#include <stdlib.h>



/**------------------------------ define hardware ------------------------------
 */
#define F3DISCOVERY_BOARD           0           // 1 :use STM32F3DISCOVERY board
//												// 0 :use ASUS board

#define DEBUG_MIO_SETTING           0           // 1 :enable debug LED as io toggling usage
// 0 :enable alive test LED

#define BATTERY_CHARGER             1           // 1 :enable battery and charger function
//												// 0 :disable


#define MOTORNUM                    4           //set motro number
#define ABENCODERNUM                2           //set absolute encoder number
#define ULTRASONICNUM               3           //set ultransonic number

#define DEAD_ZONE                   98          //To compansate drive IC(L6206Q) dead zone, set 0 to disable this function.
/**------------------------------ test function ------------------------------
 */
#define TY_Test
//#define TY_Log
//#define Avoidance


#define CHECKPOSSTOP_NECKONLY		// check position to stop only work for neck motors
//#define IRJIG                       // for IR Jig to change IR slave address, on STM32F3DISCOVERY only.
#define IR_TEST

/**------------------------------ function team test------------------------------
 */
//#define Thermo_Test   // 2015.11.25 start with wheel PWM(gear will be fixed by ME), and use APP for neck continue moving.
//#define EMI_Test      // 2015.11.30 start with all motion.

/**------------------------------ define constant ------------------------------
 */
// API
#define WHEEL_NAVIGATION            0
#define WHEEL_REMOTE                1

//controller sampleing
#define CTRLLOOP_PERIOD             5           // 5ms
#define CTRLLOOP_FREQ               200         //200Hz

#define SendMsgTimer                99          //100*0.2ms=20ms
#define POWERMODULETIMER            24         //25*10ms = 250ms

#define KICKFORCE                   100         // kick for velocity controller
#define BrakeSpdLevel               7           //Avg speed =(8.62(100mm/s)+4.32(50mm/s)) / 2 = 6.47 => 7 (80mm /s )

#define MAX_PWM_VALUE               3600        // 20kHz
#define MIN_PWM_VALUE               0           //
#define ABN_PWM_VALUE               3600        // abnormal PWM value 80% of maximum
#define POS_MAX_PWM_VALUES    		3600

#define VEL_FINISHED                20          // velocity is acheived,
#define POS_FINISHED                5           // position is acheived, 5*360/4095=0.44degree

// Incremental Encoder
#define MAX_Encoder_Count           0xFFFF      // 16-bit
#define MinSpdResolution            1080/CTRLLOOP_FREQ  // 50mm/s=100*3.14*f,f=2.7Hz,2.7*400=1080 pulse,
#define AccTime                     100         //
#define StopTime                    500         //

#define VELOCITY_LIMIT_MAX          269         // 1000mm/s = 1000*VEL2ENCODER = 269/sample
#define VELOCITY_LIMIT_MIN          0           // 0mm/s = 0counts/sample

#define MOTORPOSVMAX_Neck_Yaw          MOTORVMAX_VELPOSITION_0
#define MOTORPOSVMAX_Neck_Pitch        MOTORVMAX_VELPOSITION_1
#define MOTORPOSVMAX_Wheel_L           MOTORVMAX_VELPOSITION_2
#define MOTORPOSVMAX_Wheel_R           MOTORVMAX_VELPOSITION_3
#define MOTORVMAX_VELPOSITION_0                 200          //30degree/s  scale : 0.1degree
#define MOTORVMAX_VELPOSITION_1                 200          //30degree/s  scale : 0.1degree
#define MOTORVMAX_VELPOSITION_2                 200          //30degree/s  scale : 0.1degree
#define MOTORVMAX_VELPOSITION_3                 200          //30degree/s  scale : 0.1degree

#define MOTORPOSAMAX_Neck_Yaw          MOTORAMAX_ACCPOSITION_0
#define MOTORPOSAMAX_Neck_Pitch        MOTORAMAX_ACCPOSITION_1
#define MOTORPOSAMAX_Wheel_L           MOTORAMAX_ACCPOSITION_2
#define MOTORPOSAMAX_Wheel_R           MOTORAMAX_ACCPOSITION_3
#define MOTORAMAX_ACCPOSITION_0                 50          //10degree/s/s  scale : 0.1degree
#define MOTORAMAX_ACCPOSITION_1                 50          //10degree/s/s  scale : 0.1degree
#define MOTORAMAX_ACCPOSITION_2                 50          //10degree/s/s  scale : 0.1degree
#define MOTORAMAX_ACCPOSITION_3                 50          //10degree/s/s  scale : 0.1degree



#define GEAR_RATIO                  30          // 30:1
#define WHEEL_DIAMETER              247         // 247mm
#define WHEEL_ENCODER               1336	    // 334 counts for one channel
#define ENCODER_SAMPLING            CTRLLOOP_FREQ
#define VEL2ENCODER                 0.26915317f     // =GEAR_RATIO*WHEEL_ENCODER/(WHEEL_DIAMETER*PI*ENCODER_SAMPLING)
#define ENCODER2VEL                 3.715356581f    // =1/VEL2ENCODER

#define WHEEL_DISTANCE              200	        // distance between 2 wheels is 200mm
#define WHEELBASE_m                 0.2f        // The subscipt "_m" indicates the unit is meter
//#define ratio_CountTomilliMeter   (float) WHEEL_DIAMETER*PI/(WHEEL_ENCODER*GEAR_RATIO)
//#define ratio_milliMeterToCount   (float) WHEEL_ENCODER*GEAR_RATIO/(WHEEL_DIAMETER*PI)
// #define CountTomilliMeter(X)     (float) X*ratio_CountTomilliMeter					// wheel_encoder*gear_ratio (counts/cycle) = 2*pi*radius (mm/cycle), where X is the reader from the encoder (counts)
#define CountTomilliMeter(X)        (float) X*WHEEL_DIAMETER*PI/WHEEL_ENCODER/GEAR_RATIO
//#define milliMeterToCount(X)      (float) X*ratio_milliMeterToCount

#define MOVE_BACKWARD_ANGLE		    PI/2.0f         // Heading angle is less than +/- PI/2
#define MOVE_BACKWARD_DIST		    0.1f            // 0.1 m
#define Vmax_m                      0.50f           // Unit: m/sec
#define Amax_m                      0.40f           // Unit: m/sec^2

// Parameters for Avoidance
#define distNear 										0.3f						//
#define distFar 										0.6f						// The near-field and far-field distances are defined as 0.3 and 0.6 m respectively.
#define SonarViewAngle							20.0f*PI/180.0f	// +/- 20 deg

// Absolute Encoder
#define DEGREE2ENC                  1.13777778f     //=4096/3600 resolution(12bits, 0.1degree)
#define ENC2DEGREE                  0.87890625f     //=3600/4096

#define ROTORY_ENCODER_RESOLUTION   4095
#define ROTORY_ENCODER_DUMMY        128
#define ROTORY_ENCODER_TOTAL        4351

#define NECK_YAW_ZERO_PWMIC		1779        //NECK_YAW Encoder Zero value           604(left end), 2851(right end)
#define NECK_YAW_MAX_POS		967		    //NECK_YAW Encoder Max FW limit 	    = 85 degree
#define NECK_YAW_MAXSTOP_POS	1024		//NECK_YAW Encoder mechenical stop  	= 90 degree
#define NECK_YAW_MIN_POS		-967		//NECK_YAW Encoder Min FW limit 	    = -85 degree
#define NECK_YAW_MINSTOP_POS	-1024		//NECK_YAW Encoder mechenical stop  	= -90 degree


#define NECK_PITCH_ZERO_PWMIC	1324		//NECK_PITCH Encoder Zero value         3530(up end), 2764(down end), 2764+181=2945
#define NECK_PITCH_MAX_POS		512		    //NECK_PITCH Encoder Max FW limit	    = 45 degree
#define NECK_PITCH_MAXSTOP_POS	534		    //NECK_PITCH Encoder mechenical stop	= 47 degree
#define NECK_PITCH_MIN_POS		-165		//NECK_PITCH Encoder Min FW limit       = -15 degree
#define NECK_PITCH_MINSTOP_POS	-181		//NECK_PITCH Encoder mechenical stop    = -17 degree

#define dPOS_MAX		20      // pos/time
#define ddPOS_MAX		20      // pos/time/time

#define dVEL_MAX		100      // vel/time
#define ddVEL_MAX		100      // vel/time/time

// CURRENT SENSE
#define CURRENT_SENSE               1                       //0:disable 1:enable

//DOCKING IR
#define DOCKING 					1                       //0:disable 1:enable

// IR RANGER FINDER
#define IR_RANGE_FINDER				1						//0:disable 1:enable

// DROP IR FINDER
#define DROP_IR				        1						//0:disable 1:enable

//I2C
#define I2C_TIMEOUT         	((uint32_t)0x1000)
#define I2C_LONG_TIMEOUT		((uint32_t)(4 * I2C_TIMEOUT))


//
#define HW_MONITOR                  1                       //0:disable 1:enable

//debug led
#if(F3DISCOVERY_BOARD == 1)

#define DEBUGLEDPORT            GPIOE
#define DEBUGLEDPIN0            GPIO_PIN_12
#define DEBUGLEDPIN1            GPIO_PIN_10


#else

#define DEBUGLEDPORT            GPIOC
#define DEBUGLEDPIN0            GPIO_PIN_0
#define DEBUGLEDPIN1            GPIO_PIN_1
#define DEBUGLEDPIN2            GPIO_PIN_2
#define DEBUGLEDPIN3            GPIO_PIN_3
#ifdef ASUSBOT_DBG
#if (DEBUG_MIO_SETTING != 1)
    #undef GPIO0
    #undef GPIO1
    #define GPIO0(value) (void)0
    #define GPIO1(value) (void)0
#endif
#endif
#endif
#endif  /*__HW_CONFIG_H*/

/* preSR1_base1
float KP_vc[MOTORNUM] = {0,0, 12,11.7 };
float KI_vc[MOTORNUM] = {0,0, 1500,1300 };
float KD_vc[MOTORNUM] = {0,0, 0.01,0.008 };

float KP_pc_forward[MOTORNUM] = {50,24,0,0};
float KI_pc_forward[MOTORNUM] = {40,20,0,0};
float KD_pc_forward[MOTORNUM] = {0.06,0.06,0,0};
*/

/***************** preSR1_neck1 *****************************/
//#define NECK_YAW_ZERO_PWMIC		2350	    //NECK_YAW Encoder Zero value           1318(left end), 3383(right end)

//Neck_Pitch AS5600 AGC GAIN = 74
//#define NECK_PITCH_ZERO_PWMIC	198		    //NECK_PITCH Encoder Zero value         802(up end), 17(down end), 17+181=198

/***************** preSR1_neck2 *****************************/
//AS5600 AGC GAIN = 103
//#define NECK_YAW_ZERO_PWMIC		1779         //NECK_YAW Encoder Zero value   769 2789
//AS5600 AGC GAIN = 87
//#define NECK_PITCH_ZERO_PWMIC	1324		//NECK_PITCH Encoder Zero value         // minus 128 first, 1833 1143

/* SR1 #1
##define NECK_YAW_ZERO_PWMIC	1728        //NECK_YAW Encoder Zero value           604(left end), 2851(right end)
#define NECK_PITCH_ZERO_PWMIC	2945		//NECK_PITCH Encoder Zero value         3530(up end), 2764(down end), 2764+181=2945

float KP_vc[MOTORNUM] = {0,0, 8,8 };
float KI_vc[MOTORNUM] = {0,0, 1100,1300 };
float KD_vc[MOTORNUM] = {0,0, 0.005,0.008 };

float KP_pc_forward[MOTORNUM] = {50,20,0,0};
float KI_pc_forward[MOTORNUM] = {40,20,0,0};
float KD_pc_forward[MOTORNUM] = {0.06,0.01,0,0};

*/


/* SR1 #2
##define NECK_YAW_ZERO_PWMIC	3535        //NECK_YAW Encoder Zero value           583(left end), 2390(right end)
#define NECK_PITCH_ZERO_PWMIC	3507		//NECK_PITCH Encoder Zero value         4124(up end), 3326(down end), 1264+181=1445
*/