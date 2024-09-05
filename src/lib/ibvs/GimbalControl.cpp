#include "ibvs/GimbalControl.hpp"
#include "../util.cpp"

GimbalControl::GimbalControl()
{
    car_odom_sub = nh.subscribe<nav_msgs::Odometry>
            ("/prius/pose_ground_truth", 10, &GimbalControl::getAgentOdom, this);
    fw_odom_sub = nh.subscribe<nav_msgs::Odometry>
            ("mavros/odometry/in", 10, &GimbalControl::getFwOdom, this);
    fw_pose_sub = nh.subscribe<nav_msgs::Odometry>
            ("base_pose_ground_truth", 10, &GimbalControl::getFwPose, this);
    fw_local_sub = nh.subscribe<nav_msgs::Odometry>
            ("mavros/local_position/odom", 10, &GimbalControl::getFwLocal, this);
    img_detect_sub = nh.subscribe<darknet_ros_msgs::BoundingBoxes>
            ("darknet_ros/bounding_boxes", 10, &GimbalControl::getDetectInfo, this);
    detect_object_sub = nh.subscribe<darknet_ros_msgs::ObjectCount>
            ("darknet_ros/found_object", 10, &GimbalControl::checkIsDetect, this);
    cameraInfo_sub = nh.subscribe<sensor_msgs::CameraInfo>
            ("camera_ir/camera/color/camera_info", 10, &GimbalControl::getCameraInfo, this);
    camerahov_sub = nh.subscribe<std_msgs::Float64>
            ("camera_ir/set_hfov", 10, &GimbalControl::getCameraHov, this);
    camerahov_pub = nh.advertise<std_msgs::Float64>
            ("camera_ir/set_hfov", 10);
    cameraZoom_sub = nh.subscribe<std_msgs::Float64>
            ("camera_ir/set_zoom", 10, &GimbalControl::getCameraZoom, this);

    functionSelect = nh.subscribe<tracking_experiment::FunctionSwitch>
            ("tracking/function_select", 10, &GimbalControl::selectFunctions, this);
    
    rollVelCommand_pub = nh.advertise<std_msgs::Float32>
            ("gimbal/roll/cmd_joint_velocity", 10);
    pitchVelCommand_pub = nh.advertise<std_msgs::Float32>
            ("gimbal/pitch/cmd_joint_velocity", 10);
    yawVelCommand_pub = nh.advertise<std_msgs::Float32>
            ("gimbal/yaw/cmd_joint_velocity", 10);
    error_p3d_pub = nh.advertise<geometry_msgs::Point >
            ("tracking/image_error/p3d", 10);
    error_yolo_pub = nh.advertise<geometry_msgs::Point >
            ("tracking/image_error/yolo", 10);
    gain_sub = nh.subscribe<std_msgs::Float32>
            ("tracking/control_gain", 10, &GimbalControl::getNewGain, this);
    eo_pub = nh.advertise<tracking_experiment::EoCommand>
            ("/eo/cmd/angular_velocity", 10);


    ros::master::getTopics(topics);
    for (size_t it = 0; it < topics.size(); it++)
    {
        if (topic_to_check == topics[it].name)
        {
            topic_exist = true;
            break;
        }
    }
    if(topic_exist == true)
    {
        gimbal_sub = nh.subscribe<sensor_msgs::JointState>
            ("gimbal/joint_states", 10, &GimbalControl::getGimbalState, this);
    }
    else
    {
        gimbal_sub = nh.subscribe<sensor_msgs::JointState>
            ("joint_states", 10, &GimbalControl::getGimbalState, this);
    }



    /*** UKF ***/
    estimate_dyn_pub = nh.advertise<tracking_experiment::Dynamics>
            ("estimation/ukf/dynamics", 10);
    estimate_cam_pub = nh.advertise<tracking_experiment::CameraFeatures>
            ("estimation/ukf/camera_features", 10);
    ukf_sub = nh.subscribe<tracking_experiment::EstimateOutput>
            ("estimation/ukf/output_data", 10, &GimbalControl::getUKFResults, this);

    nh.getParam("gain_lambda/p3d", gain_p3d);
    nh.getParam("gain_lambda/yolo", gain_yolo);
    nh.getParam("vw_limit/max", maxrot);
    nh.getParam("vw_limit/min", minrot);
    nh.getParam("target", trackingTarget);
    nh.getParam("probability", legal_probability);

    R_frd << 1, 0, 0, 0, -1, 0, 0, 0, -1;
}

GimbalControl::~GimbalControl()
{
    
}

void GimbalControl::selectFunctions(const tracking_experiment::FunctionSwitch::ConstPtr& data)
{
    userFunc.track = data->doTrack.data;
    userFunc.mode = data->trackMode.data;
    userFunc.depth = data->depthType.data;
}

void GimbalControl::getUKFResults(const tracking_experiment::EstimateOutput::ConstPtr& data)
{
    carPos_ukf.position.x = data->target_pose.x;
    carPos_ukf.position.y = data->target_pose.y;
    carPos_ukf.position.z = data->target_pose.z;
    carVel_ukf << data->target_vel.x, data->target_vel.y, data->target_vel.z;
    ukf_x1 = data->feature_1.data;
    ukf_x2 = data->feature_2.data;
}

void GimbalControl::getNewGain(const std_msgs::Float32::ConstPtr& gain)
{
    lambda_push = gain->data;
}

// under inertial frame
void GimbalControl::getAgentOdom(const nav_msgs::Odometry::ConstPtr& odom)
{
    carVel << odom->twist.twist.linear.x, odom->twist.twist.linear.y, odom->twist.twist.linear.z;

    carPos.position.x = odom->pose.pose.position.x;
    carPos.position.y = odom->pose.pose.position.y;
    carPos.position.z = odom->pose.pose.position.z;
}

void GimbalControl::getFwLocal(const nav_msgs::Odometry::ConstPtr& data)
{
    fwPos.position.x = data->pose.pose.position.x; 
    fwPos.position.y = data->pose.pose.position.y;
    fwPos.position.z = data->pose.pose.position.z; 

    fwVel << data->twist.twist.linear.x, data->twist.twist.linear.y, data->twist.twist.linear.z;
    
    quat_planeEarth_flu = Eigen::Quaternionf( data->pose.pose.orientation.w, data->pose.pose.orientation.x, data->pose.pose.orientation.y, data->pose.pose.orientation.z );
    R_planeEarth_frd =  R_frd * Quat2RotaMatrix(quat_planeEarth_flu);

    fwAngVel << data->twist.twist.angular.x, data->twist.twist.angular.y, data->twist.twist.angular.z;
    fwAngVel = R_frd * fwAngVel;
}

void GimbalControl::getFwOdom(const nav_msgs::Odometry::ConstPtr& odom)
{
    // current frame of plane (flu)
    //fwAngVel << odom->twist.twist.angular.x, odom->twist.twist.angular.y, odom->twist.twist.angular.z;
    // frame: flu -> frd
    //fwAngVel = R_frd * fwAngVel;
}

void GimbalControl::getFwPose(const nav_msgs::Odometry::ConstPtr& pose)
{
    //fwPos.position.x = pose->pose.pose.position.x; 
    //fwPos.position.y = pose->pose.pose.position.y;
    //fwPos.position.z = pose->pose.pose.position.z; 

    //fwVel << pose->twist.twist.linear.x, pose->twist.twist.linear.y, pose->twist.twist.linear.z;

    //quat_planeEarth_flu = Eigen::Quaternionf( pose->pose.pose.orientation.w, pose->pose.pose.orientation.x, pose->pose.pose.orientation.y, pose->pose.pose.orientation.z );
    //R_planeEarth_frd =  R_frd * Quat2RotaMatrix(quat_planeEarth_flu);
}

void GimbalControl::getCameraInfo(const sensor_msgs::CameraInfo::ConstPtr& msg)
{
    camInfo.K = msg->K;
    
    if(isSetZoom == 0)
    {
        fx = camInfo.K[0];
        fy = camInfo.K[4];
    }
    cu = camInfo.K[2];
    cv = camInfo.K[5];
    img_height = msg->height;
    img_width = msg->width;
}

void GimbalControl::getCameraHov(const std_msgs::Float64::ConstPtr& msg)
{
    hov = msg->data;
    fx = img_width / ( 2*tan( hov /2 ) );
    fy = fx;

    isSetZoom = 1;
}

void GimbalControl::getCameraZoom(const std_msgs::Float64::ConstPtr& msg)
{
    zoom = msg->data;
    fx = zoom * fl_1x;
    fy = fx;
    hov = 2 * atan2( img_width, 2*fx );

    std_msgs::Float64 set_hov;
    set_hov.data = hov;
    camerahov_pub.publish(set_hov);

    isSetZoom = 1;
}

void GimbalControl::checkIsDetect(const darknet_ros_msgs::ObjectCount::ConstPtr& data)
{
    objFind = data->count;
    if(objFind == 0)
    {
        isDetected = false;
        u = NAN;
        v = NAN;
    }
    else
    {
        isDetected = true;
    }
}

void GimbalControl::getDetectInfo(const darknet_ros_msgs::BoundingBoxes::ConstPtr& info)
{
    if(objFind != 0 && objFind != NAN)
    {
        int objLen =  end(info->bounding_boxes) - begin(info->bounding_boxes);

        for(int i = 0; i < objLen; i++){

            //std::cout << "item: " << i+1 << std::endl;

            if(info->bounding_boxes[i].Class == trackingTarget && info->bounding_boxes[i].probability >= legal_probability)
            {
                u = (info->bounding_boxes[0].xmax + info->bounding_boxes[0].xmin)/2;
                v = (info->bounding_boxes[0].ymax + info->bounding_boxes[0].ymin)/2;
                isDetected = true;
                detectObj = info->bounding_boxes[0].Class;
                //std::cout << "The detect result is " << detectObj << std::endl;
                break;
            }
            if(info->bounding_boxes[i].Class == trackingTarget && info->bounding_boxes[i].probability < legal_probability)
            {
                u = (info->bounding_boxes[0].xmax + info->bounding_boxes[0].xmin)/2;
                v = (info->bounding_boxes[0].ymax + info->bounding_boxes[0].ymin)/2;
                //u = NAN;
                //v = NAN;
                isDetected = false;
                //std::cout << "The detect result is " << detectObj << std::endl;
            }
            if(info->bounding_boxes[i].Class != trackingTarget)
            {
                //u = (info->bounding_boxes[0].xmax + info->bounding_boxes[0].xmin)/2;
                //v = (info->bounding_boxes[0].ymax + info->bounding_boxes[0].ymin)/2;
                u = NAN;
                v = NAN;
                isDetected = false;
                detectObj = info->bounding_boxes[0].Class;
                //std::cout << "The detect result is " << detectObj << std::endl;
            }
        }
    }
    else
    {
        u = NAN;
        v = NAN;
        isDetected = false;
        //std::cout << "No object detected." << std::endl;
    }
}

void GimbalControl::getGimbalState(const sensor_msgs::JointState::ConstPtr& state)
{
    gimbalAng[0] = state->position[0];
    gimbalAng[1] = state->position[1];
    gimbalAng[2] = state->position[2];
    gimbalAngVel[0] = state->velocity[0];
    gimbalAngVel[1] = state->velocity[1];
    gimbalAngVel[2] = state->velocity[2];
}

void GimbalControl::calCamOrientation()
{
    R_panPlane = rotationMatrix('Z', gimbalAng[2]);
    R_camPan = rotationMatrix('X', 90*M_PI/180) * rotationMatrix('Z', 90*M_PI/180) * rotationMatrix('Y', gimbalAng[1]);
}

void GimbalControl::calRelPose()
{
    Eigen::Vector3f relPos_carPlane_gnd;
    Eigen::Vector3f relPos_carPlane;
    Eigen::Vector3f relPos_carPan;
    Eigen::Vector3f relPos_carCam;
    Eigen::Vector3f pos_plane_gnd;

    /****  car and plane  *****/
    // inertial frame
    pos_plane_gnd << fwPos.position.x, fwPos.position.y, fwPos.position.z;
    relPos_carPlane_gnd << (carPos.position.x - fwPos.position.x), (carPos.position.y - fwPos.position.y), (carPos.position.z - fwPos.position.z);
    //std::cout << "car & plane__earth frame: "  << std::endl << relPos_carPlane_gnd << std::endl;
    // plane flu
    relPos_carPlane = Quat2RotaMatrix(quat_planeEarth_flu) * relPos_carPlane_gnd;
    //std::cout << "car & plane__flu frame: "  << std::endl << relPos_carPlane << std::endl;
    relPos_carPan = relPos_carPlane - param_panPlane_frd;
    //std::cout << "car & pan__flu frame: "  << std::endl << relPos_carPan << std::endl;
    // plane frd frame
    relPos_carPan =  R_frd * relPos_carPan;
    //std::cout << "car & pan__frd frame: "  << std::endl << relPos_carPan << std::endl;
    // pan frame
    relPos_carPan = R_panPlane * relPos_carPan;
    //std::cout << "car & pan__pan frame: "  << std::endl << relPos_carPan << std::endl;
    relPos_carCam = relPos_carPan - param_camPan_pan;
    //std::cout << "car & cam__pan frame: "  << std::endl << relPos_carCam << std::endl;
    // cam frame
    relPos_carCam_ = R_camPan * relPos_carCam;
    //std::cout << "car & cam__cam frame: "  << std::endl << relPos_carCam_ << std::endl;


    /**** camera and plane ****/  
    relPos_camPlane_ = R_camPan * ( ( R_panPlane * R_frd * param_panPlane_frd   ) + param_camPan_pan );
    //std::cout << "cam & plane__cam frame: "  << std::endl << relPos_camPlane_ << std::endl;
    
    pos_cam_ = {0, 0, 0};

    Eigen::Matrix3f R_g2c;
    R_g2c = R_camPan * R_panPlane * R_planeEarth_frd;
    pos_cam_gnd = pos_plane_gnd + (Quat2RotaMatrix(quat_planeEarth_flu)).transpose() * param_panPlane_frd + (Quat2RotaMatrix(quat_planeEarth_flu)).transpose() * R_panPlane.transpose() * param_camPan_pan;
}

void GimbalControl::Switch2CamFrame()
{
    carVel_ = R_camPan * R_panPlane * R_planeEarth_frd * carVel;
    fwVel_ = R_camPan * R_panPlane * R_planeEarth_frd * fwVel;
    fwAngVel_ =  R_camPan * R_panPlane * fwAngVel;
}


void GimbalControl::calRelPoseUKF()
{
    Eigen::Vector3f relPos_carPlane_gnd;
    Eigen::Vector3f relPos_carPlane;
    Eigen::Vector3f relPos_carPan;
    Eigen::Vector3f relPos_carCam;
    Eigen::Vector3f pos_plane_gnd;

    /****  car and plane  *****/
    // inertial frame
    pos_plane_gnd << fwPos.position.x, fwPos.position.y, fwPos.position.z;
    relPos_carPlane_gnd << (carPos.position.x - fwPos.position.x), (carPos.position.y - fwPos.position.y), (carPos.position.z - fwPos.position.z);
    // plane flu
    relPos_carPlane = Quat2RotaMatrix(quat_planeEarth_flu) * relPos_carPlane_gnd;
    relPos_carPan = relPos_carPlane - param_panPlane_frd;
    // plane frd frame
    relPos_carPan =  R_frd * relPos_carPan;
    // pan frame
    relPos_carPan = R_panPlane * relPos_carPan;
    relPos_carCam = relPos_carPan - param_camPan_pan;
    // cam frame
    relPos_carCam_ = R_camPan * relPos_carCam;
    /**** camera and plane ****/  
    relPos_camPlane_ = R_camPan * ( ( R_panPlane * R_frd * param_panPlane_frd   ) + param_camPan_pan );
    
    pos_cam_ = {0, 0, 0};

    Eigen::Matrix3f R_g2c;
    R_g2c = R_camPan * R_panPlane * R_planeEarth_frd;
    pos_cam_gnd = pos_plane_gnd + (Quat2RotaMatrix(quat_planeEarth_flu)).transpose() * param_panPlane_frd + (Quat2RotaMatrix(quat_planeEarth_flu)).transpose() * R_panPlane.transpose() * param_camPan_pan;
}

void GimbalControl::Switch2CamFrameUKF()
{
    carVel_ = R_camPan * R_panPlane * R_planeEarth_frd * carVel_ukf;
    fwVel_ = R_camPan * R_panPlane * R_planeEarth_frd * fwVel;
    fwAngVel_ =  R_camPan * R_panPlane * fwAngVel;
}

float GimbalControl::calDepth()
{
    float alpha, d;
    Eigen::Vector3f x_gnd{0, 0, 1}, z_cam{0, 0, 1};
    Eigen::Vector3f x_cam;
    x_cam = R_camPan * R_panPlane * R_planeEarth_frd * x_gnd;
    alpha = acos( dotProduct(x_cam, z_cam) / ( vectorMagnitude(x_cam) *  vectorMagnitude(z_cam) ) ) - (M_PI / 2);
    d = (pos_cam_gnd[2] - 0.7016) / sin(alpha);

    return d;
}

/*** Command: angular velocity by ROS ***/
void GimbalControl::TrackingController()
{   
    int userType;
    nh.getParam("user", userType);
    nh.getParam("platform", vehicleType);
    std::cout << "type " << vehicleType << std::endl;

    if(vehicleType == GimbalControl::typhoon)
    {
        nh.getParam("typhoon/intrinsic_matrix/FL_1x", fl_1x);
        nh.getParam("typhoon/relPos_panPlane/x", param1[0]);
        nh.getParam("typhoon/relPos_panPlane/y", param1[1]);
        nh.getParam("typhoon/relPos_panPlane/z", param1[2]);
        nh.getParam("typhoon/relPos_camPan/x", param2[0]);
        nh.getParam("typhoon/relPos_camPan/y", param2[1]);
        nh.getParam("typhoon/relPos_camPan/z", param2[2]);
        param_panPlane_frd << param1[0], param1[1], param1[2];
        param_camPan_pan << param2[0], param2[1], param2[2];
    }
    if(vehicleType == GimbalControl::miniyy)
    {
        nh.getParam("miniyy/intrinsic_matrix/FL_1x", fl_1x);
        nh.getParam("miniyy/relPos_panPlane/x", param1[0]);
        nh.getParam("miniyy/relPos_panPlane/y", param1[1]);
        nh.getParam("miniyy/relPos_panPlane/z", param1[2]);
        nh.getParam("miniyy/relPos_camPan/x", param2[0]);
        nh.getParam("miniyy/relPos_camPan/y", param2[1]);
        nh.getParam("miniyy/relPos_camPan/z", param2[2]);
        param_panPlane_frd << param1[0], param1[1], param1[2];
        param_camPan_pan << param2[0], param2[1], param2[2];
    }
    if(vehicleType == GimbalControl::techpod)
    {
        nh.getParam("techpod/intrinsic_matrix/FL_1x", fl_1x);
        nh.getParam("techpod/relPos_panPlane/x", param1[0]);
        nh.getParam("techpod/relPos_panPlane/y", param1[1]);
        nh.getParam("techpod/relPos_panPlane/z", param1[2]);
        nh.getParam("techpod/relPos_camPan/x", param2[0]);
        nh.getParam("techpod/relPos_camPan/y", param2[1]);
        nh.getParam("techpod/relPos_camPan/z", param2[2]);
        param_panPlane_frd << param1[0], param1[1], param1[2];
        param_camPan_pan << param2[0], param2[1], param2[2];
    }

    int ckey = 0, old_ckey = 108;  // key (l)
    bool isUKFOn = 0;
    bool isYoloOn = 0;
    bool isDepthCalOn = 0;
    std::string controlMode = "lock"; 

    while(ros::ok())
    {
        ckey = getch();
        if(ckey != 0)
        {
            old_ckey = ckey;
        }
        std::cout << "---------- Looooooooooooop ---------" << std::endl;
        std::cout << "old ckey: " << char(old_ckey) << std::endl;

        GimbalControl::calCamOrientation();

        if( (userType == 0 && old_ckey != 0 && old_ckey == 104) || isDepthCalOn || (userType == 1 && userFunc.depth == true) ) // key (h)
        {
            depth_cal = GimbalControl::calDepth();
            depth = depth_cal;
            isDepthCalOn = 1;
            std::cout << "Depth Cal:  On" << std::endl; 
        }
        if( (userType == 0 && old_ckey != 0 && old_ckey == 110) || !isDepthCalOn || (userType == 1 && userFunc.depth == false) ) // key (n)
        {
            depth = relPos_carCam_[2];
            isDepthCalOn = 0;
            std::cout << "Depth Cal:  Off" << std::endl;
        }

        if( (userType == 0 && old_ckey != 0 && old_ckey == 121) || isYoloOn || (userType == 1 && userFunc.mode == 64) ) // key (y)
        {
            GimbalControl::calRelPose();
            GimbalControl::calRelPoseUKF();
            GimbalControl::Switch2CamFrame();
            GimbalControl::Switch2CamFrameUKF();
            isUKFOn = 1;
            std::cout << "UKF:  On" << std::endl;

            state[2] = 1 / depth;
            state[0] = (u - cu) / fx; 
            state[1] = (v - cv) / fy; 
            isYoloOn = 1;
            lambda = gain_yolo;
            std::cout << "Yolo:  On" << std::endl;
            
            if(lambda_push != NAN && lambda_push != 0)
            {
                lambda = lambda_push;
            }
            std::cout << "lambda: " << lambda << std::endl;
        }
        if( (userType == 0 && old_ckey != 0 && old_ckey == 111) || !isYoloOn || (userType == 1 && userFunc.mode == 0) ) // key (o)
        {
            GimbalControl::calRelPose();
            GimbalControl::Switch2CamFrame();
            isUKFOn = 0;
            std::cout << "UKF:  Off" << std::endl;

            state[2] = 1 / depth;
            state[0] = relPos_carCam_[0] / depth;
            state[1] = relPos_carCam_[1] / depth; 
            isYoloOn = 0;

            lambda = gain_p3d;
            std::cout << "Yolo:  Off" << std::endl;

            if(lambda_push != NAN && lambda_push != 0)
            {
                lambda = lambda_push;
            }
            std::cout << "lambda: " << lambda << std::endl;
        }
        if(isUKFOn && isYoloOn)
        {
            state[0] = ukf_x1; 
            state[1] = ukf_x2; 
        }
        

        std::cout << "isDetected: " << isDetected << std::endl;
        std::cout << "-------------- Error calculated by Yolo. ------------" << std::endl;
        std::cout << "error in u from yolo: " << u - cu << std::endl << "error in v from yolo: " << v - cv << std::endl << std::endl;        
        std::cout << "---------- Error calculated by 3D position. ----------" << std::endl;
        std::cout << "Error in the u-direction: " << relPos_carCam_[0]/depth*fx << " pixel" << std::endl;
        std::cout << "Error in the v-direction: " << relPos_carCam_[1]/depth*fy << " pixel" << std::endl;
        std::cout << "-------------------------------------------------------" << std::endl;
        std::cout << "FL:  " << fy << " pixel" << std::endl;

        error << (state[0] - targetState[0]), (state[1] - targetState[1]);
        std::cout << "state error: " << std::endl << error << std::endl;

        geometry_msgs::Point error_image_p3d;
        error_image_p3d.x = state[0]*fx;
        error_image_p3d.y = state[1]*fy;
        error_p3d_pub.publish(error_image_p3d);

        geometry_msgs::Point error_image_yolo;
        if(isDetected)
        {
            error_image_yolo.x = u - cu;
            error_image_yolo.y = v - cv;
        }
        if(!isDetected)
        {
            error_image_yolo.x = NAN;
            error_image_yolo.y = NAN;
        }
        error_yolo_pub.publish(error_image_yolo);

        
        L1r << state[0]*state[1],     -(1+state[0]*state[0]),
                (1+state[1]*state[1]), -state[0]*state[1];

        L2 << -state[2], 0,         state[0]*state[2], state[0]*(state[1]+relPos_camPlane_[1]*state[2]),                                -(1+state[0]*state[0]+state[2]*(relPos_camPlane_[2]+relPos_camPlane_[0]*state[0])), state[1]+relPos_camPlane_[1]*state[2],
                0,         -state[2], state[1]*state[2], 1+state[1]*state[1]+state[2]*(relPos_camPlane_[2]+relPos_camPlane_[1]*state[1]), -state[1]*(state[0]+relPos_camPlane_[0]*state[2]),                                  -state[0]-relPos_camPlane_[0]*state[2];
        
        L1_invr =  L1r.inverse(); 
        knownTerm << (fwVel_[0]-carVel_[0]), (fwVel_[1]-carVel_[1]), (fwVel_[2]-carVel_[2]), fwAngVel_[0], fwAngVel_[1], fwAngVel_[2];

        if( (userType == 0 && old_ckey != 0 && old_ckey == 116) || (userType == 1 && userFunc.track == true) ) // key (t)
        {
            controlMode = "normal";
        }
        if(userType == 0 && old_ckey !=0 && old_ckey == 114) // key (r)
        {
            controlMode = "minus";
        }
        if( (userType == 0 && old_ckey !=0 && old_ckey == 108) || (userType == 1 && userFunc.track == false) ) // key (l)
        {
            controlMode = "lock";
            std::cout << "[Angle locked]" << std::endl;
        }
        if(controlMode == "normal")
        {
            std::cout << "[Normal tracking]" << std::endl;
            camAngVelr_ = - L1_invr * L2 * knownTerm - lambda * L1_invr * error;
        }
        if(controlMode == "minus")
        {
            std::cout << "[Tracking w/o L1 term]" << std::endl;
            camAngVelr_ = - L1_invr * L2 * knownTerm;
        }

        commandAngVel[0] = camAngVelr_[0];
        commandAngVel[1] = camAngVelr_[1];  

        if(controlMode == "normal" || controlMode == "minus")
        {
            yawRate.data = commandAngVel[1] / cos( gimbalAng[1] );
            pitchRate.data = commandAngVel[0];
            //rollRate.data = 30.8*camAngVelr_[2];
            std::cout << "Output VW, pitch: " << pitchRate.data  << ", yaw: " << yawRate.data << std::endl;
        }
        if(controlMode == "lock")
        {
            yawRate.data = 0;
            pitchRate.data = 0;
            //rollRate.data = 0;
            std::cout << "Output VW, pitch: " << pitchRate.data  << ", yaw: " << yawRate.data << std::endl;
        }

        /* velocity limitation */
        if( std::abs(yawRate.data) > maxrot)
        {
            yawRate.data = std::copysign( maxrot, yawRate.data);
        }
        if( std::abs(pitchRate.data) > maxrot)
        {
            pitchRate.data = std::copysign( maxrot, pitchRate.data);
        }

        if( std::abs(yawRate.data) < minrot)
        {
            yawRate.data = 0;
        }
        if( std::abs(pitchRate.data) < minrot)
        {
            pitchRate.data = 0;
        }
        /*
        if( std::abs(rollRate.data) < minrot)
        {
            rollRate.data = 0;
        }
        */
        std::cout << "Final VW, pitch: " << pitchRate.data  << ", yaw: " << yawRate.data << std::endl;
        std::cout << "check cos(tilt): " << cos( gimbalAng[1] ) << std::endl;


        yawVelCommand_pub.publish(yawRate);
        pitchVelCommand_pub.publish(pitchRate);
        //rollVelCommand_pub.publish(rollRate);

        eoCmd.pan_wv.data = yawRate.data * rad2Deg;
        eoCmd.tilt_wv.data = pitchRate.data * rad2Deg;
        eo_pub.publish(eoCmd);

        std::cout << "----------------------------" << std::endl << "----------------------------" << std::endl << std::endl;

        GimbalControl::DataPub();
        
        ros::spinOnce();
        rate.sleep();
    }
    
}

void GimbalControl::DataPub()
{
    cam_value.fx.data = fx;
    cam_value.fy.data = fy;
    cam_value.cu.data = cu;
    cam_value.cv.data = cv;
    cam_value.u.data = u;
    cam_value.v.data = v;
    cam_value.isDetected.data = isDetected;

    dyn_value.target_vel.x = carVel_(0);
    dyn_value.target_vel.y = carVel_(1);
    dyn_value.target_vel.z = carVel_(2);
    dyn_value.uav_vel.x = fwVel_(0);
    dyn_value.uav_vel.y = fwVel_(1);
    dyn_value.uav_vel.z = fwVel_(2);
    dyn_value.uav_wvel.x = fwAngVel_(0);
    dyn_value.uav_wvel.y = fwAngVel_(1);
    dyn_value.uav_wvel.z = fwAngVel_(2);
    dyn_value.pt_wvel.x = 0;
    dyn_value.pt_wvel.y = gimbalAngVel[1];//camAngVelr_(0);
    dyn_value.pt_wvel.z = gimbalAngVel[2]; //camAngVelr_(1);
    //std::cout << "gimbal wvel: " << gimbalAngVel[1] << std::endl << std::endl;

    dyn_value.camuav_pose.x = relPos_camPlane_(0);
    dyn_value.camuav_pose.y = relPos_camPlane_(1);
    dyn_value.camuav_pose.z = relPos_camPlane_(2);
    dyn_value.targetcam_pose.x = relPos_carCam_(0);
    dyn_value.targetcam_pose.y = relPos_carCam_(1);
    dyn_value.targetcam_pose.z = relPos_carCam_(2);

    dyn_value.depth.data = depth;

    dyn_value.target_vel_inertial.x = carVel(0);
    dyn_value.target_vel_inertial.y = carVel(1);
    dyn_value.target_vel_inertial.z = carVel(2);
    dyn_value.target_pose_inertial.x = carPos.position.x;
    dyn_value.target_pose_inertial.y = carPos.position.y;
    dyn_value.target_pose_inertial.z = carPos.position.z;

    Eigen::Matrix3f R_g2c;
    Eigen::Matrix3f R_c2g;
    R_g2c = R_camPan * R_panPlane * R_planeEarth_frd;
    dyn_value.rot_g2c.data = {R_g2c.coeff(0, 0), R_g2c.coeff(0, 1), R_g2c.coeff(0, 2), 
                                R_g2c.coeff(1, 0), R_g2c.coeff(1, 1), R_g2c.coeff(1, 2),
                                R_g2c.coeff(2, 0), R_g2c.coeff(2, 1), R_g2c.coeff(2, 2)};
    R_c2g = R_planeEarth_frd.transpose() * R_panPlane.transpose() * R_camPan.transpose();
    dyn_value.rot_c2g.data = {R_c2g.coeff(0, 0), R_c2g.coeff(0, 1), R_c2g.coeff(0, 2), 
                                R_c2g.coeff(1, 0), R_c2g.coeff(1, 1), R_c2g.coeff(1, 2),
                                R_c2g.coeff(2, 0), R_c2g.coeff(2, 1), R_c2g.coeff(2, 2)};

    dyn_value.cam_pose_inertial.x = pos_cam_gnd[0]; 
    dyn_value.cam_pose_inertial.y = pos_cam_gnd[1];
    dyn_value.cam_pose_inertial.z = pos_cam_gnd[2];
    //std::cout << "fwPos.position: " << fwPos.position.x << ", " << fwPos.position.y << ", " << fwPos.position.z << std::endl;

    dyn_value.trans_g2c.x = - pos_cam_gnd[0];
    dyn_value.trans_g2c.y = - pos_cam_gnd[1];
    dyn_value.trans_g2c.z = - pos_cam_gnd[2];

    dyn_value.trans_c2g.x = - pos_cam_[0];
    dyn_value.trans_c2g.y = - pos_cam_[1];
    dyn_value.trans_c2g.z = - pos_cam_[2];

    estimate_dyn_pub.publish(dyn_value);
    estimate_cam_pub.publish(cam_value);
}
