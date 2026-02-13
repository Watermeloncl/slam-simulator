#ifndef AI_H_
#define AI_H_

#include <utility>
#include <vector>

#include "..\Listener\clickInput.h"
#include "..\Models\robotModel.h"
#include "..\config.h"

class AIModule {
public:
private:
    ClickInput* clickInput = nullptr;
    bool started = false;

    std::vector<std::pair<RobotCommand, int>> commandSequence;
    int currInstruction = 0;

    RobotModel* robotModel = nullptr;

public:
    AIModule();
    ~AIModule();

    void GiveClickInput(ClickInput* clickInput);
    void GiveRobotModel(RobotModel* robotModel);

    void UpdateAI();
    RobotCommand GetCommand();
private:

};

#endif