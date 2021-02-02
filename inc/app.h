#pragma once

#include <stm32f4xx_hal.h>
#include <ros/node_handle.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Int64MultiArray.h>
#include <std_msgs/String.h>
#include <hardware.h>
#include <steering.h>
#include <rosserial.h>

struct App {
  public:
    App();
    void inc_counters();
    void init();
    void init_rcc();
    void init_nvic();
    void init_swd();
    void init_heartbeat();
    void run();
    void run_heartbeat();
    void run_ros_spin();
    void run_pub_steering_pos();
    static void rcv_steering_cmd(const std_msgs::Int64 &new_steering_angle);
    static void rcv_zero_cmd(const std_msgs::Empty &zero);
    static void rcv_manual_cmd(const std_msgs::Bool &manual);
    std_msgs::Int64MultiArray steering_position_msg;
    ros::Publisher pub_steering;
    ros::Subscriber<std_msgs::Int64> sub_steering_cmd;
    ros::Subscriber<std_msgs::Empty> sub_zero_cmd;
    ros::Subscriber<std_msgs::Bool> sub_manual_cmd;
    NodeHandle node_handle;
    Steering steering;
    int64_t steering_position_msg_data[2];
    uint32_t ros_spin_counter;
    uint32_t heartbeat_counter;
    uint32_t steering_pos_pub_counter;
};