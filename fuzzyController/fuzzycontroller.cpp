#include "fuzzycontroller.h"

fuzzyController::fuzzyController()
{

}

void fuzzyController::setupFuzzyController(){

    // Fuzzy Logic Controller Obstacle Avoidance
    engine->setName("ObstacleAvoidance");
    engine->setDescription("");

    // Inputs to the controller

    // Front distance input
    disF->setName("disF");
    disF->setDescription("");
    disF->setEnabled(true);
    disF->setRange(0.000, 10.000);
    disF->setLockValueInRange(false);
    disF->addTerm(new Ramp("close", 1.500, 1.000));
    disF->addTerm(new Triangle("medium", 1.000, 1.500, 2.000));
    disF->addTerm(new Ramp("okay", 1.500, 2.000));
    engine->addInputVariable(disF);

    // Left distance input
    disL->setName("disL");
    disL->setDescription("");
    disL->setEnabled(true);
    disL->setRange(0.000, 10.000);
    disL->setLockValueInRange(false);
    disL->addTerm(new Ramp("close", 1.000, 0.500));
    disL->addTerm(new Triangle("medium", 0.500, 1.000, 1.500));
    disL->addTerm(new Ramp("okay", 1.000, 1.500));
    engine->addInputVariable(disL);

    // Right distance input
    disR->setName("disR");
    disR->setDescription("");
    disR->setEnabled(true);
    disR->setRange(0.000, 10.000);
    disR->setLockValueInRange(false);
    disR->addTerm(new Ramp("close", 1.000, 0.500));
    disR->addTerm(new Triangle("medium", 0.500, 1.000, 1.500));
    disR->addTerm(new Ramp("okay", 1.000, 1.500));
    engine->addInputVariable(disR);


    // Outputs from the controller

    // Direction
    dir->setName("dir");
    dir->setDescription("");
    dir->setEnabled(true);
    dir->setRange(-0.600, 0.600);
    dir->setLockValueInRange(false);
    dir->setAggregation(new Maximum);
    dir->setDefuzzifier(new Centroid(100));
    dir->setDefaultValue(fl::nan);
    dir->setLockPreviousValue(false);
    dir->addTerm(new Ramp("leftBig", -0.200, -0.400));
    dir->addTerm(new Triangle("leftSmall", -0.400, -0.200, 0.000));
    dir->addTerm(new Triangle("none", -0.200, 0.000, 0.200));
    dir->addTerm(new Triangle("rightSmall", 0.000, 0.200, 0.400));
    dir->addTerm(new Ramp("rightBig", 0.200, 0.400));
    engine->addOutputVariable(dir);

    // Velocity
    vel->setName("vel");
    vel->setDescription("");
    vel->setEnabled(true);
    vel->setRange(0, 1.500);
    vel->setLockValueInRange(false);
    vel->setAggregation(new Maximum);
    vel->setDefuzzifier(new Centroid(100));
    vel->setDefaultValue(fl::nan);
    vel->setLockPreviousValue(false);
    vel->addTerm(new Ramp("slow", 0.300, 0.100)); // change values
    vel->addTerm(new Triangle("medium", 0.100, 0.300, 0.500));
    vel->addTerm(new Ramp("fast", 0.300, 0.500));
    engine->addOutputVariable(vel);


    // Rule Block/Base for the controller
    mamdani->setName("mamdani");
    mamdani->setDescription("");
    mamdani->setEnabled(true);
    mamdani->setConjunction(new AlgebraicProduct);
    mamdani->setDisjunction(new Maximum);
    mamdani->setImplication(new AlgebraicProduct);
    mamdani->setActivation(new General);

    // Front distance okay dir and vel
    mamdani->addRule(Rule::parse("if disF is okay and disL is okay and disR is okay then dir is none and vel is fast", engine));
    mamdani->addRule(Rule::parse("if disF is okay and disL is okay and disR is medium then dir is none and vel is fast", engine));
    mamdani->addRule(Rule::parse("if disF is okay and disL is okay and disR is close then dir is leftBig and vel is medium", engine));
    mamdani->addRule(Rule::parse("if disF is okay and disL is medium and disR is okay then dir is none and vel is fast", engine));
    mamdani->addRule(Rule::parse("if disF is okay and disL is medium and disR is medium then dir is none and vel is fast", engine));
    mamdani->addRule(Rule::parse("if disF is okay and disL is medium and disR is close then dir is leftSmall and vel is medium", engine));
    mamdani->addRule(Rule::parse("if disF is okay and disL is close and disR is okay then dir is rightBig and vel is medium", engine));
    mamdani->addRule(Rule::parse("if disF is okay and disL is close and disR is medium then dir is rightSmall and vel is medium", engine));
    mamdani->addRule(Rule::parse("if disF is okay and disL is close and disR is close then dir is none and vel is medium", engine));

    // Front distance medium dir and vel
    mamdani->addRule(Rule::parse("if disF is medium and disL is okay and disR is okay then dir is leftSmall and vel is medium", engine));
    mamdani->addRule(Rule::parse("if disF is medium and disL is okay and disR is medium then dir is leftSmall and vel is medium", engine));
    mamdani->addRule(Rule::parse("if disF is medium and disL is okay and disR is close then dir is leftBig and vel is medium", engine));
    mamdani->addRule(Rule::parse("if disF is medium and disL is medium and disR is okay then dir is rightSmall and vel is medium", engine));
    mamdani->addRule(Rule::parse("if disF is medium and disL is medium and disR is medium then dir is leftSmall and vel is medium", engine));
    mamdani->addRule(Rule::parse("if disF is medium and disL is medium and disR is close then dir is leftSmall and vel is medium", engine));
    mamdani->addRule(Rule::parse("if disF is medium and disL is close and disR is okay then dir is rightBig and vel is medium", engine));
    mamdani->addRule(Rule::parse("if disF is medium and disL is close and disR is medium then dir is rightSmall and vel is medium", engine));
    mamdani->addRule(Rule::parse("if disF is medium and disL is close and disR is close then dir is none and vel is slow", engine));

    // Front distance close dir and vel
    mamdani->addRule(Rule::parse("if disF is close and disL is okay and disR is okay then dir is leftBig and vel is slow", engine));
    mamdani->addRule(Rule::parse("if disF is close and disL is okay and disR is medium then dir is leftBig and vel is slow", engine));
    mamdani->addRule(Rule::parse("if disF is close and disL is okay and disR is close then dir is leftBig and vel is slow", engine));
    mamdani->addRule(Rule::parse("if disF is close and disL is medium and disR is okay then dir is rightBig and vel is slow", engine));
    mamdani->addRule(Rule::parse("if disF is close and disL is medium and disR is medium then dir is leftBig and vel is slow", engine));
    mamdani->addRule(Rule::parse("if disF is close and disL is medium and disR is close then dir is leftBig and vel is slow", engine));
    mamdani->addRule(Rule::parse("if disF is close and disL is close and disR is okay then dir is rightBig and vel is slow", engine));
    mamdani->addRule(Rule::parse("if disF is close and disL is close and disR is medium then dir is rightBig and vel is slow", engine));
    mamdani->addRule(Rule::parse("if disF is close and disL is close and disR is close then dir is leftSmall and vel is slow", engine));

    engine->addRuleBlock(mamdani);

    // Check if engine is ready
    std::string status;
    if (not engine->isReady(&status))
        throw fl::Exception("[engine error] engine is not ready:n" + status, FL_AT);

}

void fuzzyController::setupFuzzyControllerGoal() {

    // Fuzzy Logic Controller Rob to Goal
    engineGoal->setName("RobToGoal");
    engineGoal->setDescription("");

    // Inputs to the controller

    // Yaw angle correction from rob to goal point
    inputAngle->setName("inputAngle");
    inputAngle->setDescription("");
    inputAngle->setEnabled(true);
    inputAngle->setRange(-3.141, 3.141);
    inputAngle->setLockValueInRange(false);
    inputAngle->addTerm(new Ramp("negLarge", -0.250, -0.500));
    inputAngle->addTerm(new Triangle("negSmall", -0.500, -0.250, 0.000));
    inputAngle->addTerm(new Triangle("zero", -0.250, 0.000, 0.250));
    inputAngle->addTerm(new Triangle("posSmall", 0.000, 0.250, 0.500));
    inputAngle->addTerm(new Ramp("posLarge", 0.250, 0.500));
    engineGoal->addInputVariable(inputAngle);

    // Outputs from the controller

    // Direction
    dirGoal->setName("dirGoal");
    dirGoal->setDescription("");
    dirGoal->setEnabled(true);
    dirGoal->setRange(-0.600, 0.600);
    dirGoal->setLockValueInRange(false);
    dirGoal->setAggregation(new Maximum);
    dirGoal->setDefuzzifier(new Centroid(100));
    dirGoal->setDefaultValue(fl::nan);
    dirGoal->setLockPreviousValue(false);
    dirGoal->addTerm(new Ramp("leftBig", -0.200, -0.400));
    dirGoal->addTerm(new Triangle("leftSmall", -0.400, -0.200, 0.000));
    dirGoal->addTerm(new Triangle("none", -0.200, 0.000, 0.200));
    dirGoal->addTerm(new Triangle("rightSmall", 0.000, 0.200, 0.400));
    dirGoal->addTerm(new Ramp("rightBig", 0.200, 0.400));
    engineGoal->addOutputVariable(dirGoal);

    // Velocity
    velGoal->setName("velGoal");
    velGoal->setDescription("");
    velGoal->setEnabled(true);
    velGoal->setRange(0, 1.500);
    velGoal->setLockValueInRange(false);
    velGoal->setAggregation(new Maximum);
    velGoal->setDefuzzifier(new Centroid(100));
    velGoal->setDefaultValue(fl::nan);
    velGoal->setLockPreviousValue(false);
    velGoal->addTerm(new Ramp("slow", 0.300, 0.100)); // change values
    velGoal->addTerm(new Triangle("medium", 0.100, 0.300, 0.500));
    velGoal->addTerm(new Ramp("fast", 0.300, 0.500));
    engineGoal->addOutputVariable(velGoal);

    // Rule Block/Base for the controller
    mamdaniGoal->setName("mamdaniGoal");
    mamdaniGoal->setDescription("");
    mamdaniGoal->setEnabled(true);
    mamdaniGoal->setConjunction(new AlgebraicProduct);
    mamdaniGoal->setDisjunction(new Maximum);
    mamdaniGoal->setImplication(new AlgebraicProduct);
    mamdaniGoal->setActivation(new General);

    // Angle rules
    mamdaniGoal->addRule(Rule::parse("if inputAngle is negLarge then dirGoal is rightBig and velGoal is slow", engineGoal));
    mamdaniGoal->addRule(Rule::parse("if inputAngle is negSmall then dirGoal is rightSmall and velGoal is medium", engineGoal));
    mamdaniGoal->addRule(Rule::parse("if inputAngle is zero then dirGoal is none and velGoal is fast", engineGoal));
    mamdaniGoal->addRule(Rule::parse("if inputAngle is posSmall then dirGoal is leftSmall and velGoal is medium", engineGoal));
    mamdaniGoal->addRule(Rule::parse("if inputAngle is posLarge then dirGoal is leftBig and velGoal is slow", engineGoal));


    engineGoal->addRuleBlock(mamdaniGoal);

    // Check if engine is ready
    std::string status;
    if (not engineGoal->isReady(&status))
        throw fl::Exception("[engine error] engine is not ready:n" + status, FL_AT);

}

void fuzzyController::runFuzzyController(float distFront, float distLeft, float distRight){

    //std::cout << distFront << std::endl;
    disF->setValue(distFront);
    disL->setValue(distLeft);
    disR->setValue(distRight);

    engine->process();

    outputVelocity = double(vel->getValue());
    outputDirection = double(dir->getValue());
}

void fuzzyController::runFuzzyControllerGoal(double inputAngleY){

    inputAngle->setValue(inputAngleY);

    engineGoal->process();

    outputDirectionGoal = double(dirGoal->getValue());
    outputVelocityGoal = double(velGoal->getValue());
}

