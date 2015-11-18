#include "math.h"
#include <string>
#include <ros/ros.h>
#include "chatter.hpp"
#include "chatter_tester/user_input.h"
#include "chatter_tester/set_position.h"
#include "chatter_tester/set_gripper.h"
#include "arm_model.hpp"

class chatter_driver {
public:
    ros::NodeHandle nh;
    ros::Subscriber ui_sub;
    ros::Subscriber set_grip_sub;
    ros::Subscriber set_pos_sub;
    chatter chat;

    float x, y, z, ga, gr;

    void spin();
    void get_coords(const chatter_tester::user_input::Request msg);
    int set_arm(float& bas_r, float& shl_r, float& elb_r,
                float& wri_r);
    float lerp(float x, float x_min, float x_max, float y_min, float y_max);

    void set_gripper(const chatter_tester::set_gripper msg);
    void set_position(const chatter_tester::set_position msg);

    chatter_driver(ros::NodeHandle &nh) {
        this->nh = nh;
        this->ui_sub = nh.subscribe("/user_interface", 1, &chatter_driver::get_coords, this);
        this->set_grip_sub = nh.subscribe("/set_gripper", 1, &chatter_driver::set_gripper, this);
        this->set_pos_sub = nh.subscribe("/set_position", 1, &chatter_driver::set_position, this);
        chat = chatter();

        x = 0;
        y = 150;
        z = 150;
        ga = -90;
        gr = 0.0;

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
