#ifndef SEEKER_HPP
#define SEEKER_HPP

#include <iostream>
#include <cstdio>
#include <stdint.h>

using namespace std;

typedef struct
{
    uint8_t   video_Resolution;          // 0:1280x720p, 2:1920x1080p,
    uint8_t   gimbal_Mode;               // 0:FrontView, 1:manual(joystick), 2:scan_AZ, 3:scan_EL, 4:gpsLock, 5:videoTrack, 6:manual(angleSet)
    int16_t   curr_angular_Motor_Pan;    // unit: scale 0.01 to deg/s, range:(0, 4500)
    int16_t   curr_angular_Motor_Tilt;   // unit: scale 0.01 to deg/s, range:(0, 4500)
    int16_t   curr_angle_Motor_Pan;      // unit: scale 0.01 to deg, range:(-11000{left}, 11000{right})
    int16_t   curr_angle_Motor_Tilt;     // unit: scale 0.01 to deg, range:(12000{down}, -3000{up})
    int16_t   set_angular_Motor_Pan;     // unit: scale 0.01 to deg/s, range:(0, 4500)
    int16_t   set_angular_Motor_Tilt;    // unit: scale 0.01 to deg/s, range:(0, 4500)
    int16_t   set_angle_Motor_Pan;       // unit: scale 0.01 to deg, range:(-11000{left}, 11000{right})
    int16_t   set_angle_Motor_Tilt;      // unit: scale 0.01 to deg, range:(12000{down}, -3000{up})
    uint16_t  eo_zoom;                   // unit: scale 0.1 to ratio, range:(10, 250)
    uint8_t   mavlink_connection;        // 0:disconnect, 1:connect, default:0:disconnect
    uint8_t   arm_state;                 // 0:disarm, 1:arm, default:0:disarm
    uint8_t   flight_mode;               // 0:hold, 1:offboard, 2:mission, 3:acro, 4.attitude 5.other
    uint8_t   navigate_point;            // default:1:first_point
    int32_t   nav_point_x;               // unit: scale 1E-04 to m, default:0, local
    int32_t   nav_point_y;               // unit: scale 1E-04 to m, default:0, local
    int32_t   nav_point_z;               // unit: scale 1E-04 to m, default:0, local
} __attribute__((packed)) uavData_t;

typedef struct
{
    uint8_t   set_video_Resolution;     // 0:1280x720p, 2:1920x1080p, default:0:1280x720p
    uint8_t   set_gimbal_mode;          // 0:FrontView, 1:manual(joystick), 2:scan_AZ, 3:scan_EL, 4:gpsLock, 5:videoTrack, 6:manual(angleSet), default:0:FrontView
    int16_t   set_joyStick_Xaxis;       // unit: scale 0.01 to %, range:(-10000, 10000), default:0
    int16_t   set_joyStick_Yaxis;       // unit: scale 0.01 to %, range:(-10000, 10000), default:0
    uint16_t  set_angular_Motor_Pan;    // unit: scale 0.01 to deg/s, range:(0, 4500), default:420
    uint16_t  set_angular_Motor_Tilt;   // unit: scale 0.01 to deg/s, range:(0, 4500), default:420
    int16_t   set_angle_Motor_Pan;      // unit: scale 0.01 to deg, range:(-11000{left}, 11000{right}), default:0
    int16_t   set_angle_Motor_Tilt;     // unit: scale 0.01 to deg, range:(12000{down}, -3000{up}),     default:  17.46(1746)
    uint16_t  set_eo_zoom;              // unit: scale 0.1 to ratio, range:(10, 250), default:1(10)
    uint8_t   command;                  // 0:standby, 1:waypoints, 2:cancel, 3:waypoint reset, default:0
} __attribute__((packed)) uav_CMD_t;

void uavParser(uint8_t Rxbyte);
void getUavData(void);

void sendUavCommand(void);   
uint8_t* UART_SendData();
void updateUavCmd(uav_CMD_t);
uav_CMD_t currentUavCmd(void);

void gcsParser(uint8_t Rxbyte);
void getGcsData(void);
uav_CMD_t currentGcsData(void);


#endif
