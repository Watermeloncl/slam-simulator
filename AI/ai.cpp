#include "ai.h"
#include "..\config.h"

AIModule::AIModule() {

}

AIModule::~AIModule() {

}

RobotCommand AIModule::GetCommand(/*pose, map*/) {
    //prevents pre-rotation?
    if(!(this->started)) {
        this->started = true;
        return RobotCommand::STOP;
    }    

    return RobotCommand::RIGHT;
}