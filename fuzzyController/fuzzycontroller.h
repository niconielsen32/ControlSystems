#ifndef FUZZYCONTROLLER_H
#define FUZZYCONTROLLER_H

#include "fl/Headers.h"
#include "lidarsensor.h"

using namespace fl;


class fuzzyController
{

public:

    fuzzyController();

    void setupFuzzyController();
    void setupFuzzyControllerGoal();
    void runFuzzyController(float distFront, float distLeft, float distRight);
    void runFuzzyControllerGoal(double inputAngleY);

    double getOutputDirection(){ return outputDirection; }
    double getOutputVelocity(){ return outputVelocity; }
    double getOutputDirectionGoal(){ return outputDirectionGoal; }
    double getOutputVelocityGoal(){ return outputVelocityGoal; }

private:

    double outputDirectionGoal = 0.0;
    double outputVelocityGoal = 0.0;

    double outputDirection = 0.0;
    double outputVelocity = 0.0;

    Engine* engine = new Engine;
    Engine* engineGoal = new Engine;

    InputVariable* disF = new InputVariable;
    InputVariable* disL = new InputVariable;
    InputVariable* disR = new InputVariable;
    InputVariable* inputAngle = new InputVariable;

    OutputVariable* vel = new OutputVariable;
    OutputVariable* dir = new OutputVariable;
    OutputVariable* dirGoal = new OutputVariable;
    OutputVariable* velGoal = new OutputVariable;

    RuleBlock* mamdani = new RuleBlock;
    RuleBlock* mamdaniGoal = new RuleBlock;

};

#endif // FUZZYCONTROLLER_H
