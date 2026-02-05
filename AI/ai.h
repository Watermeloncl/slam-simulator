#ifndef AI_H_
#define AI_H_

#include "..\config.h"

class AIModule {
public:
private:
    bool started = false;
    bool flipper = false;

public:
    AIModule();
    ~AIModule();

    RobotCommand GetCommand();
private:

};

#endif