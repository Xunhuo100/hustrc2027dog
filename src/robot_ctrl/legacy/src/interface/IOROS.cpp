/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifdef COMPILE_WITH_ROS

#include "interface/IOROS.h"
#include "interface/KeyBoard.h"
#include "interface/motor_crc.h"
#include <iostream>
#include <unistd.h>

IOROS::IOROS():IOInterface(){
    std::cout << "The control interface for ROS2 Mujoco simulation" << std::endl;

    _node = std::make_shared<rclcpp::Node>("robot_ctrl");

    // start publisher
    initSend();
    // start subscriber
    initRecv();

    _executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    _executor->add_node(_node);
    _spinThread = std::thread([this](){ _executor->spin(); });

    usleep(300000);     //wait for publishers and subscribers start

    cmdPanel = new KeyBoard();
}

IOROS::~IOROS(){
    delete cmdPanel;

    if(_executor){
        _executor->cancel();
    }
    if(_spinThread.joinable()){
        _spinThread.join();
    }
    rclcpp::shutdown();
}

void IOROS::sendRecv(const LowlevelCmd *cmd, LowlevelState *state){
    sendCmd(cmd);
    recvState(state);

    state->userCmd = cmdPanel->getUserCmd();
    state->userValue = cmdPanel->getUserValue();
}

void IOROS::sendCmd(const LowlevelCmd *lowCmd){
    for(int i(0); i < 12; ++i){
        //unitree_go needs 0x01 (FOC) or 0x00 (passive) instead of the
        //MUJOCO motor mode used inside the controller
        _lowCmd.motor_cmd[i].mode = lowCmd->motorCmd[i].mode == 0 ? 0x00 : 0x01;
        _lowCmd.motor_cmd[i].q = lowCmd->motorCmd[i].q;
        _lowCmd.motor_cmd[i].dq = lowCmd->motorCmd[i].dq;
        _lowCmd.motor_cmd[i].tau = lowCmd->motorCmd[i].tau;
        _lowCmd.motor_cmd[i].kp = lowCmd->motorCmd[i].Kp;
        _lowCmd.motor_cmd[i].kd = lowCmd->motorCmd[i].Kd;
    }
    unitree_go_crc::get_crc(_lowCmd);   // check motor cmd crc
    _lowCmd_pub->publish(_lowCmd);
}

void IOROS::recvState(LowlevelState *state){
    std::lock_guard<std::mutex> lock(_mutex);
    for(int i(0); i < 12; ++i){
        state->motorState[i].mode = _lowState.motor_state[i].mode;
        state->motorState[i].q = _lowState.motor_state[i].q;
        state->motorState[i].dq = _lowState.motor_state[i].dq;
        state->motorState[i].ddq = _lowState.motor_state[i].ddq;
        state->motorState[i].tauEst = _lowState.motor_state[i].tau_est;
    }
    //unitree_go quaternion is (w, x, y, z)
    for(int i(0); i < 4; ++i){
        state->imu.quaternion[i] = _lowState.imu_state.quaternion[i];
    }
    for(int i(0); i < 3; ++i){
        state->imu.gyroscope[i] = _lowState.imu_state.gyroscope[i];
        state->imu.accelerometer[i] = _lowState.imu_state.accelerometer[i];
    }
}

void IOROS::initSend(){
    _lowCmd_pub = _node->create_publisher<unitree_go::msg::LowCmd>("/lowcmd", 10);

    //initialize the constant part of the low level command
    _lowCmd.head[0] = 0xFE;
    _lowCmd.head[1] = 0xEF;
    _lowCmd.level_flag = unitree_go_crc::LOWLEVEL;
    _lowCmd.frame_reserve = 0;
    _lowCmd.bms_cmd.off = 0xA5;
    for(int i(0); i < 20; ++i){
        _lowCmd.motor_cmd[i].mode = 0x01;
        _lowCmd.motor_cmd[i].q = unitree_go_crc::PosStopF;
        _lowCmd.motor_cmd[i].kp = 0;
        _lowCmd.motor_cmd[i].dq = unitree_go_crc::VelStopF;
        _lowCmd.motor_cmd[i].kd = 0;
        _lowCmd.motor_cmd[i].tau = 0;
    }
}

void IOROS::initRecv(){
    _lowState_sub = _node->create_subscription<unitree_go::msg::LowState>(
        "/lowstate", 10, std::bind(&IOROS::lowStateCallback, this, std::placeholders::_1));
}

void IOROS::lowStateCallback(const unitree_go::msg::LowState::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(_mutex);
    _lowState = *msg;
}

#endif  // COMPILE_WITH_ROS
