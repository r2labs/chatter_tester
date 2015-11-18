#include "math.h"
#include <string>
#include <ros/ros.h>
#include "chatter.hpp"
#include "chatter_tester/user_input.h"
#include "arm_model.hpp"

class chatter_driver {
public:
    ros::NodeHandle nh;
    ros::Subscriber sub;
    chatter chat;

    float x, y, z, ga, gripper_open;

    void spin();
    void get_coords(const chatter_tester::user_input::Request msg);
    int set_arm(float& bas_r, float& shl_r, float& elb_r,
                float& wri_r);
    float lerp(float x, float x_min, float x_max, float y_min, float y_max);

    void grip_close();
    void grip_open();

    chatter_driver(ros::NodeHandle &nh) {
        this->nh = nh;
        this->sub = nh.subscribe("/user_interface", 1, &chatter_driver::get_coords, this);
        chat = chatter();

        humerus_squared = HUMERUS*HUMERUS;
        ulna_squared = ULNA*ULNA;
    }

    inline double degrees(double radians) {
        return radians * (180.0 / M_PI);
    }

    inline double radians(double degrees) {
        return degrees * (M_PI / 180.0);
    }

    // Pre-calculations
    float humerus_squared;
    float ulna_squared;
};
