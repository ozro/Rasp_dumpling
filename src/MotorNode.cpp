#include "PololuSMC.hpp"
#include <vector>
#include <math.h>

#include "ros/ros.h"

#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/UInt16MultiArray.h"
#include "std_msgs/Int16MultiArray.h"

using namespace std;

PololuSMC controller;

ros::Publisher error_pub;
ros::Publisher limit_pub;
ros::Publisher target_pub;
ros::Publisher speed_pub;
ros::Publisher temp_pub;
ros::Publisher curr_pub;
ros::Publisher volt_pub;

float max_speed = 1;
void vel_callback(const std_msgs::Float32MultiArray::ConstPtr& msg){
    for(int i=0; i < 4; i++){
        int16_t speed = (int16_t) round(msg->data[i]*max_speed*1600);
        controller.set_target_speed(i, speed);
    }
}
void tray_callback(const std_msgs::Float32::ConstPtr& msg){
    int16_t speed = (int16_t) round(msg->data*max_speed*3200);
    controller.set_target_speed(4, speed);
}
void start_callback(const std_msgs::Bool::ConstPtr& msg){
    if(msg->data){
        ROS_INFO("Starting all motors");
        controller.safe_start_all();
    }
    else{
        ROS_INFO("Stopping all motors");
        controller.stop_all();
    }
}
void brake_callback(const std_msgs::Bool::ConstPtr& msg){
    if(msg->data){
        ROS_INFO("Braking all motors");
        controller.brake_all();
    }
}
void pub_errors(){
    vector<uint16_t> status(5);
    for(int i = 0; i < 5; i++){
        status[i] = controller.get_error_status(i);
    }
    std_msgs::UInt16MultiArray msg;
    msg.data=status;
    error_pub.publish(msg);
}
void pub_limits(){
    vector<uint16_t> status(5);
    for(int i = 0; i < 5; i++){
        status[i] = controller.get_limit_status(i);
    }
    std_msgs::UInt16MultiArray msg;
    msg.data=status;
    limit_pub.publish(msg);
}
void pub_targets(){
    vector<int16_t> status(5);
    for(int i = 0; i < 5; i++){
        status[i] = controller.get_target_speed(i);
    }
    std_msgs::Int16MultiArray msg;
    msg.data=status;
    target_pub.publish(msg);
}
void pub_speeds(){
    vector<int16_t> status(5);
    for(int i = 0; i < 5; i++){
        status[i] = controller.get_current_speed(i);
    }
    std_msgs::Int16MultiArray msg;
    msg.data=status;
    speed_pub.publish(msg);
}
void pub_temps(){
    vector<uint16_t> status(5);
    for(int i = 0; i < 5; i++){
        status[i] = controller.get_temperatures(i);
    }
    std_msgs::UInt16MultiArray msg;
    msg.data=status;
    temp_pub.publish(msg);
}
void pub_curr(){
    vector<uint16_t> status(5);
    for(int i = 0; i < 5; i++){
        status[i] = controller.get_current(i);
    }
    std_msgs::UInt16MultiArray msg;
    msg.data=status;
    temp_pub.publish(msg);
}
void pub_volt(){
    vector<uint16_t> status(5);
    for(int i = 0; i < 5; i++){
        status[i] = controller.get_input_voltage(i);
    }
    std_msgs::UInt16MultiArray msg;
    msg.data=status;
    volt_pub.publish(msg);
}

int main(int argc, char **argv){
    ros::init(argc, argv, "motornode");
    ros::NodeHandle nh;

    ROS_INFO("Motor Node: Starting serial controller...");
    controller.init_port();
    printf("Stopping all motors...");
    controller.stop_all();
    printf("Success!\n");
    
    printf("Initializing subscribers and publishers...");
    ros::Subscriber vel_sub = nh.subscribe("/motor_vel", 1, vel_callback);
    ros::Subscriber tray_sub = nh.subscribe("/tray_vel", 1, tray_callback);
    ros::Subscriber start_sub = nh.subscribe("/motor_cmd/start", 1, start_callback);
    ros::Subscriber brake_sub = nh.subscribe("/motor_cmd/brake", 1, brake_callback);

    error_pub = nh.advertise<std_msgs::UInt16MultiArray>("/motor_status/errors",1);
    limit_pub = nh.advertise<std_msgs::UInt16MultiArray>("/motor_status/limit_status",1);
    target_pub = nh.advertise<std_msgs::Int16MultiArray>("/motor_status/target_speeds",1);
    speed_pub = nh.advertise<std_msgs::Int16MultiArray>("/motor_status/current_speeds",1);
    temp_pub = nh.advertise<std_msgs::UInt16MultiArray>("/motor_status/temps",1);
    curr_pub = nh.advertise<std_msgs::UInt16MultiArray>("/motor_status/currents",1);
    volt_pub = nh.advertise<std_msgs::UInt16MultiArray>("/motor_status/voltages",1);
    printf("Success!\n");

    ros::Rate rate(1000);
    int cycle = 10;

    printf("Starting publishing loop...\n");
    while(ros::ok()){
        ros::spinOnce();
        if(cycle % 1 == 0){
            pub_speeds();
        }
        if((cycle-1) % 2 == 0){
            pub_errors();
        }
        if((cycle-2) % 100 == 0){
            pub_limits();
        }
        if((cycle-3) % 1 == 0){
            pub_targets();
        }
        if((cycle-4) % 50 == 0){
            pub_temps();
        }
        if((cycle-5) % 5 == 0){
            pub_curr();
        }
        if((cycle-6) % 5 == 0){
            pub_volt();
        }
        cycle += 1;
        if(cycle > 500){
            cycle = cycle % 500;
        }
        rate.sleep();
    }
    ROS_INFO("Motor Node exiting...");
    return 0;
}

