#ifndef __STATE_MYTEST_H
#define __STATE_MYTSET_H

#include "FSMState.h"
class State_Mytest : public FSMState
{
public:
    State_Mytest(CtrlComponents *ctrlComp, std::string cmdName = "cmd1");
    ~State_Mytest();
    void enter();
    void run();
    void exit();
    FSMStateName checkChange();

    
private:
    std::string cmdName_ ;
    std::string cmdPath_ = "";

    float cmdDegs_[12]={0};

    float startDegs_[12]={0};

    bool getCmdDegs();

    float theta_;

    float rate_;

};
#endif
