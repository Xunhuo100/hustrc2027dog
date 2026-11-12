/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/

#ifndef IOROS_H
#define IOROS_H

#include <rclcpp/rclcpp.hpp>
#include "interface/IOInterface.h"
#include "unitree_go/msg/low_cmd.hpp"
#include "unitree_go/msg/low_state.hpp"
#include <memory>
#include <mutex>
#include <thread>

/*
 * The control interface between the controller and the Mujoco simulator.
 * The simulator (unitree_mujoco) speaks the unitree_go DDS protocol, which is
 * accessible from ROS2 through rmw_cyclonedds_cpp: the ROS2 topic "/lowcmd"
 * is mapped to the DDS topic "rt/lowcmd" and "/lowstate" to "rt/lowstate",
 * just like the examples of unitree_ros2.
 */
class IOROS : public IOInterface{
public:
IOROS();
~IOROS();
void sendRecv(const LowlevelCmd *cmd, LowlevelState *state);

private:
void sendCmd(const LowlevelCmd *cmd);
void recvState(LowlevelState *state);

//init publisher and subscriber
void initSend();
void initRecv();

//Callback function for the low state of the simulator
void lowStateCallback(const unitree_go::msg::LowState::SharedPtr msg);

rclcpp::Node::SharedPtr _node;
rclcpp::Publisher<unitree_go::msg::LowCmd>::SharedPtr _lowCmd_pub;
rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr _lowState_sub;
rclcpp::executors::SingleThreadedExecutor::SharedPtr _executor;
std::thread _spinThread;

unitree_go::msg::LowCmd _lowCmd;
unitree_go::msg::LowState _lowState;
std::mutex _mutex;
};

#endif  // IOROS_H
