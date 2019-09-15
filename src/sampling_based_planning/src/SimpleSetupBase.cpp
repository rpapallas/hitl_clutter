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

#include <ompl/control/SimpleSetup.h>
#include <ompl/control/planners/kpiece/KPIECE1.h>
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/control/SteeredControlSampler.h>
#include <ompl/control/SimpleDirectedControlSampler.h>
#include "../../utils/MujocoHelper.cpp"
#include "../src/StatePropagatorBase.cpp"
#include "../src/StateSpaceBase.cpp"
#include "ApproachingRobotState.cpp"
#include "StateValidityCheckerBase.cpp"

namespace oc = ompl::control;
namespace ob = ompl::base;

#ifndef SIMPLESETUP
#define SIMPLESETUP

class SimpleSetupBase : public oc::SimpleSetup {
public:
    SimpleSetupBase(const oc::ControlSpacePtr &controlSpace,
                    MujocoHelper *mujocoHelper,
                    const double propagationStepSize,
                    const double minControlDuration,
                    const double maxControlDuration,
                    const double distanceFromGoalThreshold,
                    std::vector<std::string> staticKinbodies,
                    std::vector<std::string> movableKinbodies,
                    std::string goalKinbody,
                    const string plannerName) :
            oc::SimpleSetup(controlSpace),
            _propagationStepSize(propagationStepSize),
            _minControlDuration(minControlDuration),
            _maxControlDuration(maxControlDuration),
            _distanceFromGoalThreshold(distanceFromGoalThreshold),
            _plannerName(plannerName),
            _goalKinbody(goalKinbody),
            _staticKinbodies(staticKinbodies),
            _movableKinbodies(movableKinbodies) {
        _mujocoHelper = mujocoHelper;
        _si = getSpaceInformation();
    }

    void setStartState() {
        auto stateSpace = getStateSpace();
        ob::ScopedState<ob::CompoundStateSpace> startState(stateSpace);
        ob::SE2StateSpace::StateType *robotStartState = startState->as<ob::SE2StateSpace::StateType>(0);

        double robotXposition = _mujocoHelper->getRobotXpos();
        double robotYposition = _mujocoHelper->getRobotYpos();
        double robotYaw = _mujocoHelper->getRobotYaw();

        robotStartState->setX(robotXposition);
        robotStartState->setY(robotYposition);
        robotStartState->setYaw(robotYaw);

        for (unsigned int i = 0; i < _movableKinbodies.size(); i++) {
            // i+1 because robot subspace is at index 0, and movable objects subspace at 0+1.
            std::string objectName = _movableKinbodies[i];

            double *objectState = startState->as<ob::RealVectorStateSpace::StateType>(i + 1)->values;
            objectState[0] = _mujocoHelper->getBodyXpos(objectName);
            objectState[1] = _mujocoHelper->getBodyYpos(objectName);
            objectState[2] = _mujocoHelper->getBodyYaw(objectName);

            std::tuple<double, double, double, double, double, double> velocities = _mujocoHelper->getBodyVelocity(
                    objectName);
            objectState[3] = std::get<0>(velocities);
            objectState[4] = std::get<1>(velocities);
            objectState[5] = std::get<2>(velocities);
            objectState[6] = std::get<3>(velocities);
            objectState[7] = std::get<4>(velocities);
            objectState[8] = std::get<5>(velocities);

            std::tuple<double, double, double, double, double, double> acceleration = _mujocoHelper->getBodyAccelerations(
                    objectName);
            objectState[9] = std::get<0>(acceleration);
            objectState[10] = std::get<1>(acceleration);
            objectState[11] = std::get<2>(acceleration);
            objectState[12] = std::get<3>(acceleration);
            objectState[13] = std::get<4>(acceleration);
            objectState[14] = std::get<5>(acceleration);
        }

        oc::SimpleSetup::setStartState(startState);
    }

    void setPlanner(double goalBias) {
        if (_plannerName == "KPIECE") {
            auto kpiece = new oc::KPIECE1(getSpaceInformation());
            kpiece->setGoalBias(goalBias);
            ob::PlannerPtr planner(kpiece);
            oc::SimpleSetup::setPlanner(planner);
        } else if (_plannerName == "RRT") {
            auto rrt = new oc::RRT(getSpaceInformation());
            rrt->setGoalBias(goalBias);
            ob::PlannerPtr planner(rrt);
            oc::SimpleSetup::setPlanner(planner);
        } else {
            exit(1);
        }
    }

    oc::DirectedControlSamplerPtr alloc() {
        return std::make_shared<oc::SimpleDirectedControlSampler>(_si.get(), 2);
    }

    void setupOtherPrameters() {
        _si->setPropagationStepSize(_propagationStepSize);
        _si->setMinMaxControlDuration(_minControlDuration,
                                      _maxControlDuration);
    }

protected:
    MujocoHelper *_mujocoHelper;
    oc::SpaceInformationPtr _si;
    const double _propagationStepSize;
    const double _minControlDuration;
    const double _maxControlDuration;
    const double _distanceFromGoalThreshold;
    const string _plannerName;
    const string _goalKinbody;
    const std::vector<std::string> _staticKinbodies;
    const std::vector<std::string> _movableKinbodies;
};

#endif
