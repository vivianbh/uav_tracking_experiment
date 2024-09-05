/**************************************
 * stack and tested in Gazebo SITL
 **************************************/
#include <iostream>
#include <cmath>
#include <vector>
#include <Eigen/Dense>
#include <string>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/VFR_HUD.h>
#include <tracking_experiment/TelemGCS.h>
#include <tracking_experiment/TelemUAV.h>

#define PX4FlightMode "OFFBOARD"
#define APMFlightMode "GUIDED"

using namespace std;
using namespace Eigen;

mavros_msgs::State current_state;
sensor_msgs::NavSatFix current_globalPos;
nav_msgs::Odometry current_localPos;
mavros_msgs::VFR_HUD current_hud;
tracking_experiment::TelemGCS telem_cmd;
tracking_experiment::TelemUAV telem_nav;


void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void globalPos_cb(const sensor_msgs::NavSatFix::ConstPtr& msg){
    current_globalPos = *msg;
}

void localPos_cb(const nav_msgs::Odometry::ConstPtr& msg){
    current_localPos = *msg;
}

void hud_cb(const mavros_msgs::VFR_HUD::ConstPtr& msg){
    current_hud = *msg;
}

void telem_cb(const tracking_experiment::TelemGCS::ConstPtr& msg){
    telem_cmd = *msg;
}

vector<float> alignLocalFrame(vector<float> point, float theta)
{
    Matrix<float, 3, 3> Rz;
    Vector3f old_point;
    Vector3f new_point;
    vector<float> new_point_return = {0, 0, 0};

    old_point << point[0], point[1], point[2];
    theta = theta * (M_PI / 180);
    Rz <<  cos(theta), sin(theta), 0,
          -sin(theta), cos(theta), 0,
                    0,          0, 1;

    new_point = Rz*old_point;

    new_point_return[0] = new_point[0];
    new_point_return[1] = new_point[1];
    new_point_return[2] = new_point[2];

    return new_point_return;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "flightControl_waypoints");
    ros::NodeHandle nh;
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Subscriber global_pos_sub = nh.subscribe<sensor_msgs::NavSatFix>
            ("mavros/global_position/global", 10, globalPos_cb);
    ros::Subscriber local_pos_sub = nh.subscribe<nav_msgs::Odometry>
            ("mavros/global_position/local", 10, localPos_cb);
    ros::Subscriber hud_sub = nh.subscribe<mavros_msgs::VFR_HUD>
            ("mavros/vfr_hud", 10, hud_cb);
    ros::Subscriber telem_sub = nh.subscribe<tracking_experiment::TelemGCS>
            ("/exp/telem/gcs_cmd", 10, telem_cb);

    ros::Publisher local_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::Publisher telem_pub = nh.advertise<tracking_experiment::TelemUAV>
            ("/exp/offboard/waypoints", 10);

    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

   
    // wait for FCU connection
    while(ros::ok() && current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("FCU Connected");
    ROS_INFO("Loading navigation waypoints...");
    vector<float> _waypoint;
    vector<float> waypoint;
    vector<vector<float>> _waypoint_list;
    vector<vector<float>> waypoint_list;
    int nav_num;
    float pole_angle;
    float dist_threshold;
    string offb_mode;
    string FCUSYS;
    nh.getParam("waypoints/point_1", _waypoint);
    _waypoint_list.push_back(_waypoint);
    nh.getParam("waypoints/point_2", _waypoint);
    _waypoint_list.push_back(_waypoint);
    nh.getParam("waypoints/point_3", _waypoint);
    _waypoint_list.push_back(_waypoint);
    nh.getParam("waypoints/point_4", _waypoint);
    _waypoint_list.push_back(_waypoint);
    nh.getParam("waypoints/number", nav_num);
    _waypoint_list.push_back(_waypoint);
    nh.getParam("north_pole_diff_angle", pole_angle);
    nh.getParam("distance_threshold", dist_threshold);
    nh.getParam("fcu_sys", FCUSYS);

    telem_nav.north_pole_diff.data = pole_angle;
    telem_nav.nav_distance_threshold.data = dist_threshold;

    for(int i = 0; i < nav_num; i++){
        ROS_INFO("yes");
        _waypoint[0] = _waypoint_list[i][0];
        _waypoint[1] = _waypoint_list[i][1];
        _waypoint[2] = _waypoint_list[i][2];
        waypoint = alignLocalFrame(_waypoint, pole_angle);
        waypoint_list.push_back(waypoint);
    }
    
    //send a few setpoints before starting
    geometry_msgs::PoseStamped localPos;
    mavros_msgs::SetMode offb_set_mode;
    if(FCUSYS == "px4"){offb_mode = PX4FlightMode;}
    if(FCUSYS == "apm"){offb_mode = APMFlightMode;}
    offb_set_mode.request.custom_mode = offb_mode;
    
    ros::Time last_request = ros::Time::now();
    float flightHeight = 0;
    float alt_ini = current_hud.altitude;
    float dist[3], threshold = 0;
    int flag = 0;
    int nav_point = 1;
    int index = 0;
    int cnt = 0;
    telem_cmd.user_cmd.data = 0;
    telem_nav.nav_waypoint_num.data = 0;

    std::cout << "Vehicle armed: " << int(!current_state.armed) << std::endl;

    while(ros::ok()){
        cout << "flag: " << flag << endl;

        if(telem_cmd.user_cmd.data == 0){
            cnt++;
            flag = 0;
            nav_point = 0;
            threshold = 0;
            telem_nav.nav_waypoint_num.data = 0;
            cout << "Standby, wait for user command ... " << cnt << endl;
        }
        if(telem_cmd.user_cmd.data == 1 && flag == 0){
            flag = 1;
            nav_point = 0;
            telem_nav.nav_waypoint_num.data = 0;
        }
        if(flag == 1){
            if(current_state.mode != offb_mode &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( set_mode_client.call(offb_set_mode) &&
                    offb_set_mode.response.mode_sent ){
                    ROS_INFO("Offboard enabled");
                }
                last_request = ros::Time::now();
            }

            if(current_state.mode == offb_mode){
                flag = 2;
                nav_point = 1;
            }

            localPos.pose.position.x = waypoint_list[0][0];
            localPos.pose.position.y = waypoint_list[0][1];
            localPos.pose.position.z = waypoint_list[0][2];
            local_pub.publish(localPos);
            telem_nav.nav_waypoint_num.data = 0;
            threshold = 0;
        }
        if(flag == 2){
            index = nav_point-1;

            dist[0] = current_localPos.pose.pose.position.x - waypoint_list[index][0];
            dist[1] = current_localPos.pose.pose.position.y - waypoint_list[index][1];
            dist[2] = current_localPos.pose.pose.position.z - waypoint_list[index][2];
            threshold = sqrt(pow(dist[0], 2)+pow(dist[1], 2));
            
            if(threshold <= dist_threshold){
                nav_point++;
            }
            if(nav_point > nav_num){
                nav_point = 1;
            }

            localPos.pose.position.x = waypoint_list[index][0];
            localPos.pose.position.y = waypoint_list[index][1];
            localPos.pose.position.z = waypoint_list[index][2];
            local_pub.publish(localPos);

            telem_nav.nav_waypoint_num.data = nav_point;
            telem_nav.nav_distance_gap.data = threshold;
        }

        telem_pub.publish(telem_nav);
        cout << "Navigated Point: " << nav_point << endl;
        cout << "Horizontal Distance: " << threshold << endl;

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
