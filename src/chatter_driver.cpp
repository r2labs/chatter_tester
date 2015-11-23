#include "chatter_driver.hpp"
#include "std_msgs/String.h"
#include "math.h"
#include <sstream>
#include <trajectory_msgs/JointTrajectory.h>

/* void chatter_driver::setup() { */
/*     servo_park(); */
/* } */

void chatter_driver::spin() {
    ROS_INFO("HERSHAL: chatter spinning...");

    ros::Publisher chatter_pub =
        nh.advertise<trajectory_msgs::JointTrajectory>("tm4c/command", 10);

    ros::Rate loop_rate(5);

    trajectory_msgs::JointTrajectory traj;
    traj.joint_names.push_back("al5d_joint_0");
    traj.joint_names.push_back("al5d_joint_1");
    traj.joint_names.push_back("al5d_joint_2");
    traj.joint_names.push_back("al5d_joint_3");
    traj.joint_names.push_back("al5d_gripper");

    trajectory_msgs::JointTrajectoryPoint p;
    p.time_from_start = ros::Duration(0);
    for (int i=0; i<5; ++i) {
        p.positions.push_back(0.0);
        p.velocities.push_back(0.0);
        p.accelerations.push_back(0.0);
        p.effort.push_back(0.0);
    }
    traj.points.push_back(p);
    traj.header.stamp = ros::Time::now();

    float bas_r, shl_r, elb_r, wri_r;
    int ret;
    while (ros::ok) {

        /* get_coords(x, y, z, ga); */
        ret = set_arm(bas_r, shl_r, elb_r, wri_r);

        traj.points[0].positions[0] = bas_r;
        traj.points[0].positions[1] = shl_r;
        traj.points[0].positions[2] = elb_r;
        traj.points[0].positions[3] = wri_r;
        traj.points[0].positions[4] = gr;

        // ROS_INFO("LOOP: x: %f y: %f z: %f b: %f s: %f e: %f w: %f gr: %f", x, y, z, bas_r, shl_r, elb_r, wri_r, gr);

        chatter_pub.publish(traj);
        ros::spinOnce();
        loop_rate.sleep();
    }
}

/* TODO: get coordinates from python website */
void chatter_driver::get_coords(const chatter_tester::user_input::Request msg) {
    x = msg.pick_X;
    y = msg.pick_Y;
    z = msg.pick_Z;
    gr = msg.gripper_open ? 1.0 : 0.0;
    ga = -90.0; //angle of gripper with respect to horizontal (-90 to 90 degrees)
    ROS_INFO("user_input: x: %f y: %f z: %f gr: %f", x, y, z, gr);
}

void chatter_driver::set_position(const chatter_tester::set_position msg) {
    x = msg.x;
    y = msg.y;
    z = msg.z;
    ga = msg.ga_d;
    ROS_INFO("set_position: x: %f y: %f z: %f", x, y, z);
}

void chatter_driver::set_gripper(const chatter_tester::set_gripper msg) {
    gr = msg.gripper_percent;
    ROS_INFO("set_gripper: g: %f", gr);
}

int chatter_driver::set_arm(float& bas_r, float& shl_r, float& elb_r,
                            float& wri_r) {
    float gri_angle_r = radians(ga);
    float z_prime = z - BASE_HGT - (sin(gri_angle_r)*GRIPPER); //vertical distance from shoulder axis to wrist axis
    float r = sqrt(pow(x,2)+pow(y,2));         //distance from axis of rotation of base to position in x-y plane
    float s = r - (cos(gri_angle_r)*GRIPPER);  //distance from axis of rotation of base to wrist axis in x-y plane
    float q = sqrt(pow(s,2)+pow(z_prime,2));   //distance from shoulder axis to wrist axis
    float f = atan2(z_prime,s);                //angle between horizontal and q
    float g = acos((humerus_squared+pow(q,2)-ulna_squared)/(2*HUMERUS*q));   //angle between q and humerus (Law of Cosines)
    float a = f+g;                             //angle between horizontal and humerus
    float b = acos((ulna_squared + humerus_squared - pow(q,2))/(2*HUMERUS*ULNA)); //angle between humerus and ulna (Law of Cosines)
    float d = atan2(y,-x);                     //angle of base rotate;
    float wrist_angle_r = 2*M_PI - a - b + gri_angle_r;

    //set joint angles in radians
    bas_r = d;
    shl_r = a;
    elb_r = b;
    wri_r = wrist_angle_r;

    // ROS_INFO("setarm: x: %f y: %f z: %f b: %f s: %f e: %f w: %f", x, y, z, degrees(bas_r), degrees(shl_r), degrees(elb_r), degrees(wri_r));
    return IK_SUCCESS;
}

float chatter_driver::lerp(float x, float x_min, float x_max, float y_min, float y_max) {
    if (x > x_max) { return y_max; }
    else if (x < x_min) { return y_min; }
    return y_min + (y_min - y_max)*((x - x_min)/(x_max - x_min));
}

