#include "FSM/State_Mytest.h"

#include <fstream>
#include <json/json.h>

#ifdef COMPILE_WITH_ROS
    #include <ament_index_cpp/get_package_share_directory.hpp>
#endif  // COMPILE_WITH_ROS

State_Mytest::State_Mytest(CtrlComponents *ctrlComp, std::string cmd)
             :FSMState(ctrlComp, FSMStateName::MYTEST, "mytest"),cmdName_(cmd),
             theta_(0.001),rate_(0.0)
             {}

State_Mytest::~State_Mytest(){}

void State_Mytest::enter(){
    if(!getCmdDegs()){
        std::cout << "[MYTEST] Failed to load cmd \"" << cmdName_ << "\"" << std::endl;
    }

    for(int i=0; i<4; i++){
        if(_ctrlComp->ctrlPlatform == CtrlPlatform::MUJOCO){
            _lowCmd->setSimStanceGain(i);
            _lowCmd->setZeroDq(i);
            _lowCmd->setZeroTau(i);
        }

        else if(_ctrlComp->ctrlPlatform == CtrlPlatform::REALROBOT){
        }
    }
    for (int i = 0;i<12;i++){
        startDegs_[i]=_lowState->motorState[i].q;
        _lowCmd->motorCmd[i].q=startDegs_[i];
    }
    _ctrlComp->setAllStance();
    
    getCmdDegs();
}

bool State_Mytest::getCmdDegs(){
    // The cmdPath_ points to the json file (or to the "mycmd" folder
    // which stores the json file). If it is empty, use the mycmd folder
    // of this package.
    std::string cmdFile = cmdPath_;
    if(cmdFile.empty()){
#ifdef COMPILE_WITH_ROS
        try{
            cmdFile = ament_index_cpp::get_package_share_directory("robot_ctrl") + "/mycmd";
        }
        catch(const std::exception &e){
            std::cout << "[MYTEST] Can not locate the package share directory: "
                      << e.what() << std::endl;
            cmdFile = "mycmd";
        }
#else
        cmdFile = "mycmd";
#endif  // COMPILE_WITH_ROS
    }
    if(cmdFile.size() < 5 || cmdFile.substr(cmdFile.size() - 5) != ".json"){
        if(cmdFile.back() != '/'){
            cmdFile += "/";
        }
        cmdFile += "mycmd.json";
    }

    std::ifstream ifs(cmdFile);
    if(!ifs.is_open()){
        std::cout << "[MYTEST] Can not open cmd file: " << cmdFile << std::endl;
        return false;
    }

    Json::Value root;
    Json::CharReaderBuilder builder;
    std::string errs;
    if(!Json::parseFromStream(builder, ifs, &root, &errs)){
        std::cout << "[MYTEST] Parse error in " << cmdFile << ": " << errs << std::endl;
        return false;
    }

    Json::Value cmd = root["command"][cmdName_];
    if(cmd.isNull()){
        std::cout << "[MYTEST] \"" << cmdName_ << "\" not found in " << cmdFile << std::endl;
        return false;
    }

    // Split the json into cmdDegs_: leg1 ~ leg4, 3 joints for each leg.
    for(int i=0; i<4; i++){
        Json::Value leg = cmd["leg" + std::to_string(i + 1)];
        if(!leg.isArray() || leg.size() != 3){
            std::cout << "[MYTEST] \"leg" << i + 1 << "\" should be an array of 3 numbers" << std::endl;
            return false;
        }
        for(int j=0; j<3; j++){
            if(leg[j].isNumeric()){
                cmdDegs_[i * 3 + j] = leg[j].asFloat();
            }
            else{
                try{
                    cmdDegs_[i * 3 + j] = std::stof(leg[j].asString());
                }
                catch(const std::exception &e){
                    std::cout << "[MYTEST] \"leg" << i + 1 << "\"[" << j
                              << "] is not a number" << std::endl;
                    return false;
                }
            }
        }
    }

    std::cout << "[MYTEST] Loaded \"" << cmdName_ << "\" from " << cmdFile << std::endl;
    return true;
}

void State_Mytest::run(){
    rate_+=theta_;
    rate_=(rate_>1)?1:rate_;
    for(int i = 0;i<12;i++){
        _lowCmd->motorCmd[i].q=startDegs_[i]+rate_*(cmdDegs_[i]-startDegs_[i]);
    }
}

void State_Mytest::exit(){
    rate_=0;
}

FSMStateName State_Mytest::checkChange(){
    if(_lowState->userCmd == UserCommand::L2_B){
        return FSMStateName::PASSIVE;
    }

    else{
        return FSMStateName::MYTEST;
    }
}
