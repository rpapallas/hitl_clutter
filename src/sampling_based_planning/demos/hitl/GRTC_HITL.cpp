//  Copyright (C) 2019 Rafael Papallas and The University of Leeds
//
//  This program is free software: you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation, either version 3 of the License, or
//  (at your option) any later version.
//
//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License
//  along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
//  Author: Rafael Papallas (rpapallas.com)

#include "../../../utils/MujocoGlobal.cpp"
#include "../../../utils/CommonFunctions.cpp"
#include "../../src/ProblemDefinition.cpp"
#include "../../src/ProjectionEvaluator.cpp"
#include "../../src/StateSpaceBase.cpp"
#include "../../src/ControlSpace.cpp"
#include "../../src/StateValidityCheckerBase.cpp"
#include "../../src/StatePropagatorBase.cpp"
#include "../../src/ProblemDefinition.cpp"
#include "../../src/SamplingBasedPlannerBase.cpp"
#include "../Execution.cpp"
#include "../../src/planners/GRTC/GRTC_Planner.cpp"
#include "../../src/planners/RTC/RTC_Planner.cpp"
#include <boost/filesystem.hpp>

const double timeLimit = 300.0;
double guidancePlanningTime = 0.0;
double guidanceTime = 0.0;
double overallPlanningTime = 0.0;

ProblemDefinition *problemDefinition;
string lowLevelPlannerName;
string goalObjectName;

void moveRobotBackwards() {
    // Move the robot back such that the object is not in the hand for next iteration.
    for (int i = 0; i < 1.5 / globalMujocoHelper->getTimeStep(); ++i) {
        globalMujocoHelper->setRobotVelocity(-0.1, 0.0, 0.0);
        globalMujocoHelper->step();
    }
}

void pushObject(string objectName, double x, double y) {
    printf("You selected object %s and you want to push it to location %f %f\n", objectName.c_str(), x, y);

    problemDefinition->setGoalObjectName(objectName);
    GRTC_Planner grtcHitl(globalMujocoHelper, problemDefinition);
    std::tuple<double, double> goalRegion = std::make_tuple(x, y);

    auto planningStart = std::chrono::high_resolution_clock::now();
    auto solution = grtcHitl.solve(lowLevelPlannerName, objectName, 0.1, goalRegion);
    auto planningFinish = std::chrono::high_resolution_clock::now();
    double subPlanningTime = formatDurationToSeconds(planningStart, planningFinish);

    if (solution) {
        oc::PathControl solutionPath1 = std::get<0>(solution.get());
        oc::PathControl solutionPath2 = std::get<1>(solution.get());
        std::vector<oc::PathControl> solutionPaths = {solutionPath1, solutionPath2};

        double planningTime = std::get<2>(solution.get());
        guidancePlanningTime += planningTime;
        overallPlanningTime += planningTime;

        double propagationStepSizeGRTC = grtcHitl.getPropagationStepSize();

        globalMujocoHelper->resetSimulation();

        finishedExecution = false;
        thread t1(executeSolutionsInRealTime, solutionPaths, propagationStepSizeGRTC);
        while (!finishedExecution) {
            render();
        }
        t1.join();

        // Move the robot back such that the object is not in the hand for next iteration.
        moveRobotBackwards();

        globalMujocoHelper->saveMujocoState();
        globalMujocoHelper->setResetLevelToLatest();
    } else {
        printf("Adding %f seconds to planning time\n", subPlanningTime);
        guidancePlanningTime += subPlanningTime;
        overallPlanningTime += subPlanningTime;
    }
}

void reachGoalObject() {
    printf("Planning to reach for the goal object.\n");
    globalMujocoHelper->resetSimulation();
    problemDefinition->setGoalObjectName(goalObjectName);

    RTC_Planner rtcPlanner(globalMujocoHelper, problemDefinition);
    double remainingRtcPlanningTimeLimit = timeLimit - overallPlanningTime;
    rtcPlanner.setMaxPlanningTime(remainingRtcPlanningTimeLimit);

    auto solution = rtcPlanner.solve(lowLevelPlannerName);

    if (solution) {
        globalMujocoHelper->resetSimulation();
        oc::PathControl solutionPath = std::get<0>(solution.get());
        double planningTime = std::get<1>(solution.get());
        overallPlanningTime += planningTime;

        double propagationStepSizeRTC = rtcPlanner.getPropagationStepSize();

        glfwShowWindow(window);
        std::this_thread::sleep_for(std::chrono::seconds(2));
        finishedExecution = false;
        thread t1(executeSolutionInRealTime, solutionPath, propagationStepSizeRTC);
        while (!finishedExecution) {
            render();
        }
        t1.join();
    }
}

tuple<string, double, double> getNextHighLevelAction() {
    printf("Click the goal object to stop giving guidance.\n");

    auto start = std::chrono::high_resolution_clock::now();
    string objectNameSelected = getObjectSelection(problemDefinition->getMovableObjectNames());
    auto finish = std::chrono::high_resolution_clock::now();
    double interactionTime = formatDurationToSeconds(start, finish);
    overallPlanningTime += interactionTime;
    guidanceTime += interactionTime;

    if (objectNameSelected != goalObjectName) {
        auto start = std::chrono::high_resolution_clock::now();
        auto clickPosition = getPushPosition();
        auto finish = std::chrono::high_resolution_clock::now();
        double interactionTime = formatDurationToSeconds(start, finish);
        overallPlanningTime += interactionTime;
        guidanceTime += interactionTime;

        double x = get<0>(clickPosition);
        double y = get<1>(clickPosition);

        return make_tuple(objectNameSelected, x, y);
    }

    return make_tuple(objectNameSelected, 0.0, 0.0);
}

void grtc() {
    string objectName;

    do {
        // If pushing accumulated planning time + guidance time is over the time limit, stop.
        if (overallPlanningTime >= timeLimit) {
            printf("Time over. Your interaction took more than %f seconds.\n", timeLimit);
            break;
        }

        auto highLevelAction = getNextHighLevelAction();
        objectName = get<0>(highLevelAction);

        if (objectName != goalObjectName) {
            double centroidX = get<1>(highLevelAction);
            double centroidY = get<2>(highLevelAction);
            pushObject(objectName, centroidX, centroidY);
        }
    } while (objectName != goalObjectName);

    if (overallPlanningTime < timeLimit) {
        reachGoalObject();
    }

    printf("Guidance Planning Time: %f\n", guidancePlanningTime);
    printf("Guidance Time: %f\n", guidanceTime);
    printf("Overall Planning Time: %f\n", overallPlanningTime);
}

#ifndef MAIN
#define MAIN

int main(int /* argc */, char **argv) {
    mj_activate(MJ_KEY_PATH.c_str());

    const std::string sceneFileName = SCENE_PATH + argv[1];
    const int numberOfMovableObjects = atoi(argv[2]);
    lowLevelPlannerName = argv[3];

    initMujocoFrom(sceneFileName);
    isZoomLocked = true;
    ProblemDefinition localProblemDefinition = createProblemDefinitionFromArgs(sceneFileName, numberOfMovableObjects);
    problemDefinition = &localProblemDefinition;
    goalObjectName = problemDefinition->getGoalObjectName();

    string problemName = globalMujocoHelper->getTextField("problemName");
    if (problemName.empty()) {
        throw std::invalid_argument(
                "The problemName attribute in the model XML file is not set. Please set a custom text attribute problemName in your XML file.");
    }

    glfwShowWindow(window);
    grtc();

    while (!glfwWindowShouldClose(window)) {
        render();
    }

    tideUp();
    return 0;
}

#endif
