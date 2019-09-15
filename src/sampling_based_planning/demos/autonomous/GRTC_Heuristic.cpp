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
#include "../../src/ProblemDefinition.cpp"
#include "../../src/SamplingBasedPlannerBase.cpp"
#include "../Execution.cpp"
#include "../../src/planners/RTC/RTC_Planner.cpp"
#include "../../src/planners/GRTC/GRTC_Planner.cpp"
#include <boost/filesystem.hpp>
#include <utility>

string GOAL_OBJECT_NAME;
double STRAIGHT_LINE_TRAJECTORY_DURATION = 3.0;
const int COLLISION_CHECKING_FREQUENCY = 200; // In terms of time steps.

void printCurrentTime() {
    auto end = std::chrono::system_clock::now();
    std::time_t end_time = std::chrono::system_clock::to_time_t(end);
    string timeNow = std::ctime(&end_time);
    printf("Planning will start now: %s", timeNow.c_str());
}

tuple<double, double, double>
getControlsToMoveTheRobotStraightTo() {
    double goalObjectX = globalMujocoHelper->getBodyXpos(GOAL_OBJECT_NAME);
    double goalObjectY = globalMujocoHelper->getBodyYpos(GOAL_OBJECT_NAME);

    auto endEffectorPosition = globalMujocoHelper->getSitePosition(
            "ee_point_1"); // This is the position of the end-effector.
    double endEffectorX = endEffectorPosition[0];
    double endEffectorY = endEffectorPosition[1];

    auto site2 = globalMujocoHelper->getSitePosition("ee_point_2");
    double site2_x = site2[0];
    double site2_y = site2[1];

    // Direction vector
    double eeVectorX = site2_x - endEffectorX;
    double eeVectorY = site2_y - endEffectorY;
    vector<double> directionVector = {eeVectorX, eeVectorY};
    vector<double> unitDirectionVector = globalMujocoHelper->unitVectorOf(directionVector);

    // Find the vector of the end-effector to the goal object.
    double eeToGoalX = goalObjectX - endEffectorX;
    double eeToGoalY = goalObjectY - endEffectorY;
    vector<double> eeToGoalVector = {eeToGoalX, eeToGoalY};
    vector<double> unitEeToGoalVector = globalMujocoHelper->unitVectorOf(eeToGoalVector);

    double x1 = eeVectorX;
    double y1 = eeVectorY;
    double x2 = eeToGoalX;
    double y2 = eeToGoalY;

    double dot = x1 * x2 + y1 * y2;
    double det = x1 * y2 - y1 * x2;
    double angle = atan2(det, dot) * 0.05;

    double linearX = eeToGoalX / STRAIGHT_LINE_TRAJECTORY_DURATION;
    double linearY = eeToGoalY / STRAIGHT_LINE_TRAJECTORY_DURATION;
    double angularZ = angle / STRAIGHT_LINE_TRAJECTORY_DURATION;

    return make_tuple(linearX, linearY, angularZ);
}

template<class T>
bool doesExist(T objectToCheck, vector<T> container) {
    return std::find(container.begin(), container.end(), objectToCheck) != container.end();
}

vector<string>
getObjectNamesInCollisionToStraightLine(const vector<string> &movableObstacleNames) {
    vector<string> objectsInCollision;

    // Ensure that collisions are on for all objects..
    for (const string &obstacleName : movableObstacleNames)
        globalMujocoHelper->enableCollisionsFor(obstacleName);

    // Move robot on a straight line.
    auto controls = getControlsToMoveTheRobotStraightTo();
    double linearX = get<0>(controls);
    double linearY = get<1>(controls);
    double angularZ = get<2>(controls);

    for (int step = 0; step < STRAIGHT_LINE_TRAJECTORY_DURATION / globalMujocoHelper->getTimeStep(); ++step) {
        globalMujocoHelper->setRobotVelocity(linearX, linearY, angularZ);
        globalMujocoHelper->forward();
        globalMujocoHelper->step();

        if (step % COLLISION_CHECKING_FREQUENCY == 0) {
            for (const string &obstacleName : movableObstacleNames) {
                if (obstacleName != GOAL_OBJECT_NAME) {
                    if (globalMujocoHelper->isRobotInContact(obstacleName)) {
                        objectsInCollision.push_back(obstacleName);
                        globalMujocoHelper->disableCollisionsFor(obstacleName);
                    }
                }
            }

        }
    }


    globalMujocoHelper->resetSimulation();

    // Enable collisions again.
    for (const string &obstacleName : movableObstacleNames)
        globalMujocoHelper->enableCollisionsFor(obstacleName);

    return objectsInCollision;
}

string getFirstObjectNameInCollisionToStraightLine(const vector<string> &movableObjectNames) {
    auto collisions = getObjectNamesInCollisionToStraightLine(movableObjectNames);
    return collisions.empty() ? string() : collisions[0];
}

bool isObjectInRobotsWayToStraightLine(const vector<string> &movableObjectNames, string objectName) {
    vector<string> newObjectsInCollision = getObjectNamesInCollisionToStraightLine(movableObjectNames);
    return doesExist(std::move(objectName), newObjectsInCollision);
}

bool isObjectWithinTheShelf(const string &objectName, ProblemDefinition *problemDefinition) {
    auto bounds = problemDefinition->getObjectStateSpaceBounds();

    double lowX = bounds.low[0];
    double highX = bounds.high[0];
    double lowY = bounds.low[1];
    double highY = bounds.high[1];

    double objectX = globalMujocoHelper->getBodyXpos(objectName);
    double objectY = globalMujocoHelper->getBodyYpos(objectName);

    if (objectX < lowX || objectX > highX) {
        return false;
    }

    if (objectY < lowY || objectY > highY) {
        return false;
    }

    return true;
}

bool isPlacementValid(const string &objectName, const vector<string> &movableObjectNames,
                      ProblemDefinition *problemDefinition) {
    bool objectIsCollisionFree = !globalMujocoHelper->isBodyInContact(objectName);

    if (!objectIsCollisionFree)
        return false;

    bool objectIsWithinTheShelf = isObjectWithinTheShelf(objectName, problemDefinition);
    if (!objectIsWithinTheShelf)
        return false;

    bool objectIsNotInSweptVolume = !isObjectInRobotsWayToStraightLine(movableObjectNames, objectName);

    return objectIsNotInSweptVolume;
}

tuple<double, double>
getHighLevelAction(ProblemDefinition *problemDefinition, const vector<string> &movableObjectNames,
                   const string &objectName) {
    std::random_device rd;
    std::array<int, std::mt19937::state_size> seed_data;
    std::generate_n(seed_data.data(), seed_data.size(), std::ref(rd));
    std::seed_seq seq(std::begin(seed_data), std::end(seed_data));
    std::mt19937 eng(seq);

    // Find places where we can put the object name that does not
    // collide when executing _solution.
    ob::RealVectorBounds objectSpaceBounds = problemDefinition->getObjectStateSpaceBounds();
    vector<double> low = objectSpaceBounds.low;
    vector<double> high = objectSpaceBounds.high;
    double lowX = low[0];
    double highX = high[0];
    double lowY = low[1];
    double highY = high[1];
    std::uniform_real_distribution<> stateSpaceDistrX(lowX, highX); // define the range
    std::uniform_real_distribution<> stateSpaceDistrY(lowY, highY); // define the range

    double x, y;

    double objectInitialX = globalMujocoHelper->getBodyXpos(objectName);
    double objectInitialY = globalMujocoHelper->getBodyYpos(objectName);
    double offset = 0.3;

    std::uniform_real_distribution<> distrX(objectInitialX - offset, objectInitialX + offset); // define the range
    std::uniform_real_distribution<> distrY(objectInitialY - offset, objectInitialY + offset); // define the range

    bool placementIsNotValid = true;

    int counter = 1;

    do {
        globalMujocoHelper->resetSimulation();

        double distance = 0.0;
        do {
            // If can't find a good placement using object distribution (distrX/distrY) that means that there is no collision-free
            // placement near the object, so instead find a position from the state space.
            if (counter <= 200) {
                x = distrX(eng);
                y = distrY(eng);
            } else {
                x = stateSpaceDistrX(eng);
                y = stateSpaceDistrY(eng);
            }

            double deltaX = x - objectInitialX;
            double sDeltaX = deltaX * deltaX;
            double deltaY = y - objectInitialY;
            double sDeltaY = deltaY * deltaY;
            distance = sqrt(sDeltaX + sDeltaY);
            counter++;
        } while (distance < 0.2);

        globalMujocoHelper->setBodyXYPosition(objectName, x, y);
        globalMujocoHelper->forward();
        globalMujocoHelper->step();

        placementIsNotValid = !isPlacementValid(objectName, movableObjectNames, problemDefinition);

        counter++;
    } while (placementIsNotValid);

    tuple<double, double> action = make_tuple(x, y);
    globalMujocoHelper->resetSimulation();

    return action;
}

void moveRobotBackwards() {
    // Move the robot back such that the object is not in the hand for next iteration.
    for (int i = 0; i < 1.5 / globalMujocoHelper->getTimeStep(); ++i) {
        globalMujocoHelper->setRobotVelocity(-0.1, 0.0, 0.0);
        globalMujocoHelper->step();
    }
}

void grtc(ProblemDefinition *problemDefinition, const std::string &plannerName) {
    const double overallPlanningTimeLimit = 300.0;
    auto movableObjectNames = problemDefinition->getMovableObjectNames();

    double overallGuidancePlanningTime = 0.0;
    double overallGuidanceTime = 0.0;
    double overallPlanningTime = 0.0;

    // Keep the initial robot position so we can always check which object is blocking the way from that position!
    double robotInitialX = globalMujocoHelper->getRobotXpos();
    double robotInitialY = globalMujocoHelper->getRobotYpos();
    double robotInitialYaw = globalMujocoHelper->getRobotYaw();

    // This will keep the robot last position (after a push) so we can go back to it.
    double robotLastX = globalMujocoHelper->getRobotXpos();
    double robotLastY = globalMujocoHelper->getRobotYpos();
    double robotLastYaw = globalMujocoHelper->getRobotYaw();

    do {
        // Go back to initial robot position so we can check for the next blocking obstacle.
        globalMujocoHelper->setRobotXYPosition(robotInitialX, robotInitialY);
        globalMujocoHelper->setRobotYaw(robotInitialYaw);
        globalMujocoHelper->step();
        globalMujocoHelper->saveMujocoState();
        globalMujocoHelper->setResetLevelToLatest();
        globalMujocoHelper->resetSimulation();

        auto start = std::chrono::high_resolution_clock::now();
        string objectInCollision = getFirstObjectNameInCollisionToStraightLine(movableObjectNames);
        auto finish = std::chrono::high_resolution_clock::now();
        double duration = formatDurationToSeconds(start, finish);
        overallGuidanceTime += duration;
        overallPlanningTime += duration;

        // Now that we found the next blocking obstacle, go back to where the robot was.
        globalMujocoHelper->setRobotXYPosition(robotLastX, robotLastY);
        globalMujocoHelper->setRobotYaw(robotLastYaw);
        globalMujocoHelper->step();
        globalMujocoHelper->saveMujocoState();
        globalMujocoHelper->setResetLevelToLatest();
        globalMujocoHelper->emptyResetQueueButLast();
        globalMujocoHelper->resetSimulation();

        // No collisions break out.
        if (objectInCollision.empty()) {
            break;
        }

        bool actionWasSuccessful;

        do {
            actionWasSuccessful = true;

            start = std::chrono::high_resolution_clock::now();
            auto proposedGoalRegion = getHighLevelAction(problemDefinition, movableObjectNames,
                                                         objectInCollision);
            finish = std::chrono::high_resolution_clock::now();
            double duration = formatDurationToSeconds(start, finish);
            overallGuidanceTime += duration;
            overallPlanningTime += duration;

            globalMujocoHelper->resetSimulation();

            problemDefinition->setGoalObjectName(objectInCollision);
            GRTC_Planner grtcHitl(globalMujocoHelper, problemDefinition);

            printf("Manipulating %s to (%f, %f)\n", objectInCollision.c_str(), get<0>(proposedGoalRegion),
                   get<1>(proposedGoalRegion));

            auto planningStart = std::chrono::high_resolution_clock::now();
            auto solution = grtcHitl.solve(plannerName, objectInCollision, 0.1, proposedGoalRegion);
            auto planningFinish = std::chrono::high_resolution_clock::now();
            double subPlanningTime = formatDurationToSeconds(planningStart, planningFinish);

            if (solution) {
                oc::PathControl solutionPath1 = std::get<0>(solution.get());
                oc::PathControl solutionPath2 = std::get<1>(solution.get());
                std::vector<oc::PathControl> solutionPaths = {solutionPath1, solutionPath2};

                double planningTime = std::get<2>(solution.get());
                overallGuidancePlanningTime += planningTime;
                overallPlanningTime += planningTime;

                double propagationStepSizeGRTC = grtcHitl.getPropagationStepSize();

                globalMujocoHelper->resetSimulation();

                executeSolutions(solutionPaths, propagationStepSizeGRTC);

                // Move the robot back such that the object is not in the hand for next iteration.
                moveRobotBackwards();

                globalMujocoHelper->saveMujocoState();
                globalMujocoHelper->setResetLevelToLatest();
                globalMujocoHelper->emptyResetQueueButLast();

                // Keep robot's last position so we can go back to it later.
                robotLastX = globalMujocoHelper->getRobotXpos();
                robotLastY = globalMujocoHelper->getRobotYpos();
                robotLastYaw = globalMujocoHelper->getRobotYaw();
            } else {
                printf("Failed guide numbered\n");
                overallGuidancePlanningTime += subPlanningTime;
                overallPlanningTime += subPlanningTime;
                actionWasSuccessful = false;
            }
        } while (!actionWasSuccessful && overallPlanningTime < overallPlanningTimeLimit);
    } while (overallPlanningTime < overallPlanningTimeLimit);

    if (overallPlanningTime < overallPlanningTimeLimit) {
        printf("Now reaching for goal\n");
        globalMujocoHelper->resetSimulation();
        problemDefinition->setGoalObjectName(GOAL_OBJECT_NAME);

        RTC_Planner rtcPlanner(globalMujocoHelper, problemDefinition);
        double remainingRtcPlanningTimeLimit = overallPlanningTimeLimit - overallPlanningTime;
        rtcPlanner.setMaxPlanningTime(remainingRtcPlanningTimeLimit);

        auto solution = rtcPlanner.solve(plannerName);

        if (solution) {
            oc::PathControl solutionPath = std::get<0>(solution.get());
            double planningTime = std::get<1>(solution.get());
            overallPlanningTime += planningTime;

            double propagationStepSizeRTC = rtcPlanner.getPropagationStepSize();

            globalMujocoHelper->resetSimulation();

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

    printf("Guidance Planning Time: %f\n", overallGuidancePlanningTime);
    printf("Guidance Time: %f\n", overallGuidanceTime);
    printf("Overall Planning Time: %f\n", overallPlanningTime);
}

void demo_main(ProblemDefinition *problemDefinition, const string &plannerName, const string &sceneFileName) {
    printf("Solving: %s using control-based %s\n", sceneFileName.c_str(), plannerName.c_str());
    printCurrentTime();

    grtc(problemDefinition, plannerName);

    while (!glfwWindowShouldClose(window)) {
        render();
    }
}

int main(int /*argc*/, char **argv) {
    mj_activate(MJ_KEY_PATH.c_str());

    const std::string sceneFileName = SCENE_PATH + argv[1];
    const int numberOfMovableObjects = atoi(argv[2]);
    const std::string plannerName = argv[3];

    initMujocoFrom(sceneFileName);
    ProblemDefinition problemDefinition = createProblemDefinitionFromArgs(sceneFileName, numberOfMovableObjects);
    GOAL_OBJECT_NAME = problemDefinition.getGoalObjectName();

    string problemName = globalMujocoHelper->getTextField("problemName");
    if (problemName.empty()) {
        throw std::invalid_argument(
                "The problemName attribute in the model XML file is not set. Please set a custom text attribute problemName in your XML file.");
    }

    isMouseLocked = true;
    isKeyboardLocked = true;

    demo_main(&problemDefinition, plannerName, sceneFileName);

    tideUp();
    return 0;
}
