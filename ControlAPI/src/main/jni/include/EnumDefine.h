
#ifndef _ENUMDEFINE_H_
#define _ENUMDEFINE_H_


typedef enum {
    e_WheelVelCtrl = 0x02,
    e_NeckJointCtrl = 0x04,
    e_BaseTaskSize = 0x06,
    e_BaseTaskData = 0x07,
    e_NeckJointCtrlTrk = 0x04,
    e_SpecificAction = 0x40
} e_cmdType;


// Enum for the
typedef enum {
    e_statusBase = 0x06,
} e_getStatus;


// Enum for
typedef enum {
    e_rtnNoRoute = -2,  // No path-finding route is found
    e_rtnError = -1,
    e_rtnUnknow = 0,
    e_rtnSuccess = 1
} e_returnStatus;

// define for the Mobile Status
#define e_statusBase_Standby 	0x01
#define e_statusBase_Error 		0x02
#define e_statusBase_Busy 		0x04
#define e_statusBase_Break 		0x08
#define e_statusBase_Unknow4 	0x10 //4
#define e_statusBase_Unkown5 	0x20	//5
#define e_statusBase_Unknow6 	0x40
#define e_statusBase_DriverOff  0x80


#endif