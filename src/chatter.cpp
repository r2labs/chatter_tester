#include "chatter.hpp"
#include <ros/ros.h> 

chatter::chatter() {
     msg = "I'm chatting, fam!";
 }
std::string chatter::chat() {
    // ROS_INFO("HERSHAL: chatter chatting...");
    return msg;
}
