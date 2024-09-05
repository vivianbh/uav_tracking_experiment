#ifndef GIMBALCONTROL_HPP
#define GIMBALCONTROL_HPP

#include <iostream>
#include <ros/ros.h>
#include <vector>
#include <string>
#include <Eigen/Dense>

#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <mavros_msgs/MountControl.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/ObjectCount.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/CameraInfo.h>
#include <std_msgs/Float32.h>
#include <tracking_experiment/Dynamics.h>
#include <tracking_experiment/CameraFeatures.h>
#include <tracking_experiment/EstimateOutput.h>
#include <tracking_experiment/EoCommand.h>
#include <tracking_experiment/FunctionSwitch.h>

#define STATE 2

class GimbalControl
{
private:
    ros::NodeHandle nh;
    ros::Rate rate = 100;

    /*** publish & subscribe ***/
    ros::Subscriber car_odom_sub;
    ros::Subscriber fw_pose_sub;
    ros::Subscriber fw_odom_sub;
    ros::Subscriber fw_local_sub;
    ros::Subscriber img_detect_sub;
    ros::Subscriber detect_object_sub;
    ros::Subscriber gimbal_sub;
    ros::Subscriber gain_sub;
    ros::Subscriber cameraInfo_sub;
    ros::Subscriber cameraZoom_sub;
    ros::Subscriber camerahov_sub;
    ros::Publisher camerahov_pub;
    ros::Publisher yawVelCommand_pub;
    ros::Publisher pitchVelCommand_pub;
    ros::Publisher rollVelCommand_pub;
    ros::Publisher error_p3d_pub;
    ros::Publisher error_yolo_pub;
    ros::Publisher estimate_dyn_pub;
    ros::Publisher estimate_cam_pub;
    ros::Subscriber ukf_sub;
    ros::Subscriber functionSelect;
    /** INF EO **/
    ros::Publisher eo_pub;

    /*** agent info ***/
    Eigen::Vector3f carVel;
    geometry_msgs::Pose carPos;
    Eigen::Vector3f fwVel;
    Eigen::Vector3f fwAngVel;
    geometry_msgs::Pose fwPos;

    /*** camera sensor info***/
    Eigen::Vector3f param_camPlane_frd;
    Eigen::Vector3f param_panPlane_frd;
    Eigen::Vector3f param_camPan_pan;
    
    /*** agent info under camera frame ***/
    Eigen::Vector3f carVel_;
    Eigen::Vector3f fwVel_;
    Eigen::Vector3f fwAngVel_;
    Eigen::Vector3f camAngVel_;
    Eigen::Vector2f camAngVelr_{0, 0};
    //Eigen::Vector3f camAngVelr_{0, 0, 0};
    Eigen::Vector3f relPos_camPlane_;
    Eigen::Vector3f relPos_carCam_;
    Eigen::Vector3f pos_cam_;

    /*** agent info under earth frame ***/
    Eigen::Vector3f pos_cam_gnd;

    /*** ukf estimate result ***/
    Eigen::Vector3f carVel_ukf;
    geometry_msgs::Pose carPos_ukf;

    /*** rotation matrix ***/
    Eigen::Quaternionf baseLink_frd = Eigen::Quaternionf(0, 1, 0, 0);    // flu -> frd
    Eigen::Quaternionf quat_planeEarth_flu;
    Eigen::Quaternionf quat_planeEarth_frd;  // {p}^R_e
    Eigen::Quaternionf quat_panPlane;        // {a}^R_p
    Eigen::Quaternionf quat_camPan;          // {c}^R_a
    Eigen::Matrix3f R_cam_frd2rdf;
    Eigen::Matrix3f R_panPlane;
    Eigen::Matrix3f R_camPan;
    Eigen::Matrix3f R_camPlane;
    Eigen::Matrix3f R_frd;
    Eigen::Matrix3f R_planeEarth_frd;
    

    /*** system ***/
    std::vector<float> targetState{0, 0, 0.033333};
    std::vector<float> state{0, 0, 0};
#if (STATE == 2)
    Eigen::Vector2f error;
#elif (STATE == 3)
    Eigen::Vector3f error;
#endif

    /*** controller ***/
    float lambda, lambda_push = 0, gain_yolo = 0, gain_p3d = 0, maxrot = 0, minrot = 0;
    int gain_push = 0;
    float param1[3], param2[3];
    int vehicleType;
    std::vector<float> L2_elements{0, 0, 0, 0, 0, 0};
#if (STATE == 2)
    Eigen::Matrix<float, 2, 3> L1;
    Eigen::Matrix<float, 2, 2> L1r;
    Eigen::Matrix<float, 2, 6> L2;
    Eigen::Matrix<float, 3, 2> L1_inv;
    Eigen::Matrix<float, 2, 2> L1_invr;  
#elif (STATE == 3)
    Eigen::Matrix<float, 3, 3> L1;
    Eigen::Matrix<float, 3, 6> L2;
    Eigen::Matrix<float, 3, 3> L1_inv;
#endif
    Eigen::Matrix<float, 6, 1> knownTerm;

    /*** gimbal ***/
    std::vector<float> gimbalAng{0.0, 0.0, 0.0};           // rad (feedback)
    std::vector<float> angRec{0.0, 0.0, 0.0};
    std::vector<float> gimbalAngVel{0.0, 0.0, 0.0};        // rad/s (feedback)
    std::vector<float> commandAngVel{0.0, 0.0};            // rad/s
    //std::vector<float> commandAngVel{0.0, 0.0, 0.0};                 
    std_msgs::Float32 yawRate;
    std_msgs::Float32 pitchRate;
    std_msgs::Float32 rollRate;
    int yaw_direc = 1;
    int pitch_direc = 1;

    /*** info of image detection ***/
    sensor_msgs::CameraInfo camInfo;
    std::string trackingTarget;
    std::string detectObj;
    float legal_probability;
    bool isDetected = false;
    int objFind = 0;
    float fx, fy, cu, cv, u = NAN, v = NAN, depth, depth_cal;           // pixel
    int img_height, img_width;
    float hov, zoom, fl_1x;
    bool isSetZoom = 0;

    float angPitch = 0, angYaw = 0;

    /*** estimate ***/
    tracking_experiment::Dynamics dyn_value;
    tracking_experiment::CameraFeatures cam_value;
    float ukf_x1, ukf_x2;


    /*** check topic is exist ***/
    ros::master::V_TopicInfo topics;
    std::string topic_to_check = "/uav0/gimbal/joint_states";
    bool topic_exist = false;

    /*** INF EO ***/
    tracking_experiment::EoCommand eoCmd; 
    /*** user select functions ***/
    typedef struct
    {
        bool track = false;
        int mode = 0;
        bool depth = false;
    }userFunction;
    userFunction userFunc;

public: 
    GimbalControl();
    ~GimbalControl();
    enum Uav {typhoon=1, miniyy, techpod};

    /*** rostopic callback function ***/
    void getAgentOdom(const nav_msgs::Odometry::ConstPtr& odom);
    void getFwLocal(const nav_msgs::Odometry::ConstPtr& data);
    void getFwPose(const nav_msgs::Odometry::ConstPtr& pose);
    void getFwOdom(const nav_msgs::Odometry::ConstPtr& odom);
    void getDetectInfo(const darknet_ros_msgs::BoundingBoxes::ConstPtr& info);
    void checkIsDetect(const darknet_ros_msgs::ObjectCount::ConstPtr& data);
    void getGimbalState(const sensor_msgs::JointState::ConstPtr& state);
    void getNewGain(const std_msgs::Float32::ConstPtr& gain);
    void getCameraInfo(const sensor_msgs::CameraInfo::ConstPtr& msg);
    void calCamOrientation();
    void calRelPose();
    void Switch2CamFrame();
    void TrackingController();
    void getCameraHov(const std_msgs::Float64::ConstPtr& msg);
    void getCameraZoom(const std_msgs::Float64::ConstPtr& msg);

    void getUKFResults(const tracking_experiment::EstimateOutput::ConstPtr& data);
    void calRelPoseUKF();
    void Switch2CamFrameUKF();
    float calDepth();
    void DataPub();
    void selectFunctions(const tracking_experiment::FunctionSwitch::ConstPtr& data);
};

#endif