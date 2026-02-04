#include "ai.h"
#include "..\config.h"

AIModule::AIModule() {

}

AIModule::~AIModule() {

}

RobotCommand AIModule::GetCommand(/*pose, map*/) {
    //TODO

    // if(flipper) {
    //     flipper = !flipper;
    //     return RobotCommand::FORWARD;
    // } else {
    //     flipper = !flipper;
    //     return RobotCommand::STOP;
    // }

    return RobotCommand::FORWARD;
}