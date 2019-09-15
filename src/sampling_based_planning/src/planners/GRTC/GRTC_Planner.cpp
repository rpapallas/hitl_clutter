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

#ifndef GRTC_PLANNER
#define GRTC_PLANNER

#include "../../SamplingBasedPlannerBase.cpp"
#include "approach/SimpleSetup.cpp"
#include "push/SimpleSetup.cpp"
#include "approach/Goal.cpp"
#include "push/Goal.cpp"

class GRTC_Planner : public SamplingBasedPlanner {
public:
    GRTC_Planner(MujocoHelper *mujocoHelper,
                 ProblemDefinition *problemDefinition) :
            SamplingBasedPlanner(mujocoHelper, problemDefinition) {
    }

    boost::optional<std::tuple<oc::PathControl, oc::PathControl, double, std::string>>
    solve(std::string plannerName,
          std::string goalObjectName,
          double distanceToGoalThreshold,
          std::tuple<double, double> goalRegion) {

        _solutionFound = false;
        _diststanceToGoalThreshold = distanceToGoalThreshold;

        const string sceneFileName = _problemDefinition->getSceneFileName();
        const vector<string> staticObjectNames = _problemDefinition->getStaticObjectNames();
        const vector<string> movableObjectNames = _problemDefinition->getMovableObjectNames();

        auto robotApproachingPositions = getApproachingPositions(goalObjectName, goalRegion);

        // Get multiple transforms to approach the object.
        std::vector<boost::thread *> threads;
        int threadId = 0;
        for (auto position : robotApproachingPositions) {
            _threadStartTime.push_back(chrono::steady_clock::now());
            threads.push_back(new boost::thread(&GRTC_Planner::push,
                                                this,
                                                position,
                                                plannerName,
                                                staticObjectNames,
                                                movableObjectNames,
                                                goalObjectName,
                                                goalRegion,
                                                threadId));
            threadId++;
        }

        // Wait for some thread to finish.
        for (unsigned int i = 0; i < threads.size(); ++i) {
            threads[i]->join();
            delete threads[i];
        }

        if (_solution) {
            boost::optional<shared_ptr<SimpleSetup2>> simpleSetup2 = std::get<0>(
                    _solution.get()); // Solution to approach the object.
            boost::optional<shared_ptr<SimpleSetup3>> simpleSetup3 = std::get<1>(
                    _solution.get()); // Solution to push the object.

            boost::optional<std::tuple<oc::PathControl, oc::PathControl, double, std::string>> result;

            oc::PathControl pathControl1 = simpleSetup2.get()->getSolutionPath();
            oc::PathControl pathControl2 = simpleSetup3.get()->getSolutionPath();
            double planningTime =
                    simpleSetup2.get()->getLastPlanComputationTime() + simpleSetup3.get()->getLastPlanComputationTime();
            std::string solutionType;

            if (simpleSetup2.get()->haveSolutionPath()) {
                if (simpleSetup2.get()->haveExactSolutionPath()) {
                    solutionType = "exact";
                } else {
                    solutionType = "approx";
                }
            } else {
                solutionType = "nosolution";
            }

            result = std::make_tuple(pathControl1, pathControl2, planningTime, solutionType);

            return result;
        } else {
            cout << "No solution" << endl;
            return boost::none;
        }
    }

private:
    void push(ApproachingRobotState robotGoalPosition,
              string plannerName,
              const vector<string> staticObjectNames,
              const vector<string> movableObjectNames,
              string goalObjectName,
              std::tuple<double, double> goalRegion,
              int threadId) {
        // We need to create local copy of the data so threads are not
        // changing or queueing for accessing global data.
        mjModel *model = _mujocoHelper->getModel();
        mjData *data = _mujocoHelper->getData();

        mjData *localData = mj_makeData(model);
        mj_copyData(localData, model, data);
        MujocoHelper *localMujocoHelper = new MujocoHelper(model, localData, "robot", "robot_lin_x", "robot_lin_y",
                                                           "robot_ang_z", 0.0, 0.0);

        auto stateSpace1 = std::make_shared<StateSpace2>(CELL_SIZE,
                                                         localMujocoHelper,
                                                         _problemDefinition);

        auto controlSpace1 = std::make_shared<ControlSpace>(stateSpace1, 3, -0.1, 0.1);

        // Initial position of goal object.
        double goalObjectGoalX = localMujocoHelper->getBodyXpos(goalObjectName);
        double goalObjectGoalY = localMujocoHelper->getBodyYpos(goalObjectName);
        double goalObjectGoalYaw = localMujocoHelper->getBodyYaw(goalObjectName);
        double robotGoalX = robotGoalPosition.getX();
        double robotGoalY = robotGoalPosition.getY();
        double robotGoalYaw = robotGoalPosition.getYaw();

        auto simpleSetup1 = std::make_shared<SimpleSetup2>(controlSpace1,
                                                           localMujocoHelper,
                                                           PROPAGATION_STEP_SIZE,
                                                           MIN_CONTROL_DURATION,
                                                           MAX_CONTROL_DURATION,
                                                           0.06,
                                                           staticObjectNames,
                                                           movableObjectNames,
                                                           goalObjectName,
                                                           "object_3",
                                                           plannerName,
                                                           goalRegion,
                                                           goalObjectGoalX,
                                                           goalObjectGoalY,
                                                           goalObjectGoalYaw,
                                                           robotGoalX,
                                                           robotGoalY,
                                                           robotGoalYaw);

        ob::PlannerTerminationConditionFn ptc1 = boost::bind(&SamplingBasedPlanner::terminationCondition, this,
                                                             threadId);
        ob::PlannerStatus solved1;
        setThreadStartTime(threadId);
        solved1 = simpleSetup1->solve(ptc1);

        if (!(solved1 && simpleSetup1->haveExactSolutionPath())) {
            return;
        }

        // Save this state of the system. This is needed such that when
        // the system propagates it can reset simulation to that state (
        // the state we found above to approach the object) instead of
        // resetting the simulation from scratch.
        localMujocoHelper->saveMujocoState();
        localMujocoHelper->setResetLevelToLatest();

        // Solve the problem of pushing the object to the goal region.
        std::tuple<double, double> goalObjectInitialPosition = std::make_tuple(
                localMujocoHelper->getBodyXpos(goalObjectName), localMujocoHelper->getBodyYpos(goalObjectName));
        auto stateSpace2 = std::make_shared<StateSpace3>(CELL_SIZE,
                                                         localMujocoHelper,
                                                         goalRegion,
                                                         goalObjectInitialPosition,
                                                         _problemDefinition);

        // Definition of the Control Space which consists of the velocities of
        // each joint of the robot.
        auto controlSpace2 = std::make_shared<ControlSpace>(stateSpace2, 3, -0.1, 0.1);

        // Configure and get a simple setup. The SimpleSetup is a custom class
        // defined in SimpleSetup.cpp. In this class we inherit from
        // ompl::control::SimpleSetup, we set some constants, the control space,
        // the StateValidityChecker, the StatePropagator, and the Goal.
        auto simpleSetup2 = std::make_shared<SimpleSetup3>(controlSpace2,
                                                           localMujocoHelper,
                                                           PROPAGATION_STEP_SIZE,
                                                           MIN_CONTROL_DURATION,
                                                           MAX_CONTROL_DURATION,
                                                           _diststanceToGoalThreshold,
                                                           staticObjectNames,
                                                           movableObjectNames,
                                                           goalObjectName,
                                                           "object_3",
                                                           plannerName,
                                                           goalRegion);

        ob::PlannerStatus solved2;
        ob::PlannerTerminationConditionFn ptc2 = boost::bind(&SamplingBasedPlanner::terminationCondition, this,
                                                             threadId);
        solved2 = simpleSetup2->solve(ptc2);

        if (!(solved2 && simpleSetup2->haveExactSolutionPath())) {
            return;
        }

        _solutionFound = true;
        _solution = std::make_tuple(simpleSetup1, simpleSetup2);
    }

    double _diststanceToGoalThreshold;
};

#endif
