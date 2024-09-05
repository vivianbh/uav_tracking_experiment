#include "ibvs/GimbalControl.hpp"

#define TRACK 1

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ibvs_gimbal");
    GimbalControl gb;  

#if TRACK
    std::cout << "start tracking ......" << std::endl;
    gb.TrackingController(); 
#endif

    return 0;
}