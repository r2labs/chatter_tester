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
        nh.advertise<trajectory_msgs::JointTrajectory>("tm4c/command", 1000);

    ros::Rate loop_rate(10);

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

    float bas_us, shl_us, elb_us, wri_us;
    int ret;
    while (ros::ok) {

        /* get_coords(x, y, z, ga); */
        ret = set_arm(bas_us, shl_us, elb_us, wri_us);

        traj.points[0].positions[0] = bas_us;
        traj.points[0].positions[1] = shl_us;
        traj.points[0].positions[2] = elb_us;
        traj.points[0].positions[3] = wri_us;
        traj.points[0].positions[4] = 1.57;

        ROS_INFO("LOOP: x: %f y: %f z: %f b: %f s: %f e: %f w: %f", x, y, z, bas_us, shl_us, elb_us, wri_us);

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
    ga = 0.0;
    ROS_INFO("getcoords: x: %f y: %f z: %f", x, y, z);
    /*TODO: FIGURE OUT WHERE TO STORE PLACE YOURSELF :)*/
}

int chatter_driver::set_arm(float& bas_r, float& shl_r, float& elb_r,
                            float& wri_r) {
    /*TODO: ADD GRIPPER STUFF*/
    float r = sqrt(pow(x,2)+pow(y,2));      //distance from axis of rotation of base to x,y
    float s = r - GRIPPER;                  //distance from axis of rotation of base to wrist axis
    float q = sqrt(pow(s,2)+pow(z,2));      //distance from shoulder axis to wrist axis
    float f = atan2(z,s);                   //angle between horizontal and q
    float g = acos((humerus_squared+pow(q,2)-ulna_squared)/(2*HUMERUS*q));//angle between q and humerus (Law of Cosines)
    float a = f+g;                          //angle between horizontal and humerus
    float b = acos((ulna_squared + humerus_squared - pow(q,2))/(2*HUMERUS*ULNA));//angle between humerus and ulna (Law of Cosines)
    float c = -b - a + 2*M_PI;              //angle between ulna and wrist (keeping wrist parallel with horizontal)
    float d = atan2(x,y);                   //angle of base;
    
    /*begin old stuff

    float gri_angle_r = radians(ga);
    float bas_angle_r = atan2(y, -x);
    float r = sqrt(pow(x,2) + pow(y,2));
    float z_prime = z - BASE_HGT - (sin(gri_angle_r)*GRIPPER);
    float r_prime = r - (cos(gri_angle_r)*GRIPPER);
    float q = sqrt(pow(r_prime,2) + pow(z_prime,2));
    float shl_angle_r;
    if(y>=0) {
        shl_angle_r = atan2(z_prime, r_prime) + acos((pow(HUMERUS,2)+pow(q,2)-pow(ULNA,2))/(2*HUMERUS*q));
    }
    else {
        shl_angle_r = atan2(z_prime, -r_prime) + acos((pow(HUMERUS,2)+pow(q,2)-pow(ULNA,2))/(2*HUMERUS*q));
    }
    if (isnan(shl_angle_r) || isinf(shl_angle_r)) {
      ROS_INFO("shl angle nan/inf");
      return IK_ERROR;
    }

    float elb_angle_r = acos((pow(HUMERUS,2)+pow(ULNA,2)-pow(q,2))/(2*HUMERUS*ULNA));
    float wri_angle_r = (3*M_PI/2) - elb_angle_r - shl_angle_r + gri_angle_r;
     
    float shoulder_angle_degrees = degrees(a);
    float elbow_angle_degrees = degrees(b);
    float wrist_angle_degrees = degrees(c);
    float base_angle_degrees = degrees(d);

    // Calculate servo angles
    // Calc relative to servo midpoint to allow compensation for servo alignment
    float bas_pos = base_angle_degrees;
    float shl_pos = shoulder_angle_degrees;
    float elb_pos = elbow_angle_degrees;
    float wri_pos = wrist_angle_degrees;
    */ //end old stuff

    ROS_INFO("setarm: x: %f y: %f z: %f b: %f s: %f e: %f w: %f", x, y, z, d, a, b, c);
    return IK_SUCCESS;
}

float chatter_driver::lerp(float x, float x_min, float x_max, float y_min, float y_max) {
    if (x > x_max) { return y_max; }
    else if (x < x_min) { return y_min; }
    return y_min + (y_min - y_max)*((x - x_min)/(x_max - x_min));
}

void chatter_driver::grip_close()
{
    //TODO: add command to close grip
}

void chatter_driver::grip_open()
{
    //TODO: add command to open grip
}




// void chatter_driver::parse_joints() {

//     ros::NodeHandle priv_nh( "~" );

//     // Parse joints ros param
//     XmlRpc::XmlRpcValue joints_list;
//     if(priv_nh.getParam("joints", joints_list)) {
//         ROS_ASSERT(joints_list.getType() == XmlRpc::XmlRpcValue::TypeStruct);

//         XmlRpcValueAccess joints_struct_access(joints_list);
//         XmlRpc::XmlRpcValue::ValueStruct joints_struct = joints_struct_access.getValueStruct();

//         XmlRpc::XmlRpcValue::ValueStruct::iterator joints_it;

//         for(joints_it = joints_struct.begin(); joints_it != joints_struct.end(); joints_it++) {
//             Joint *joint = new Joint;
//             joint->name = static_cast<std::string>(joints_it->first);

//             std::string joint_graph_name = "joints/" + joint->name + "/";

//             priv_nh.param<int>(joint_graph_name + "channel", joint->properties.channel, 0);

//             // Channel must be between 0 and 31, inclusive
//             ROS_ASSERT(joint->properties.channel >= 0);
//             ROS_ASSERT(joint->properties.channel <= 31);

//             priv_nh.param<double>(joint_graph_name + "max_angle", joint->properties.max_angle, M_PI_2);
//             priv_nh.param<double>(joint_graph_name + "min_angle", joint->properties.min_angle, -M_PI_2);
//             priv_nh.param<double>(joint_graph_name + "offset_angle", joint->properties.offset_angle, 0);
//             priv_nh.param<double>(joint_graph_name + "default_angle", joint->properties.default_angle, joint->properties.offset_angle);
//             priv_nh.param<bool>(joint_graph_name + "initialize", joint->properties.initialize, true);
//             priv_nh.param<bool>(joint_graph_name + "invert", joint->properties.invert, false);

//             // Make sure no two joints have the same channel
//             ROS_ASSERT(channels[joint->properties.channel] == NULL);

//             // Make


 /* sure no two joints have the same name */
//             ROS_ASSERT(joints_map.find(joint->name) == joints_map.end());

//             channels[joint->properties.channel] = joint;
//             joints_map[joint->name] = joint;
//         }
//     } else {
//         ROS_FATAL("No joints were given");
//         ROS_BREAK();
//     }
// }
