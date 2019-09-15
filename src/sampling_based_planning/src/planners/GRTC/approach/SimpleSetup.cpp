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

#ifndef APPROACHSIMPLESETUP
#define APPROACHSIMPLESETUP

#include "../../../SimpleSetupBase.cpp"
#include "Goal.cpp"
#include "StatePropagator.cpp"
#include "StateValidityChecker.cpp"
#include "StateSpace.cpp"

class SimpleSetup2 : public SimpleSetupBase {
public:
    SimpleSetup2(const oc::ControlSpacePtr &controlSpace,
                 MujocoHelper *mujocoHelper,
                 const double propagationStepSize,
                 const double minControlDuration,
                 const double maxControlDuration,
                 const double distanceFromGoalThreshold,
                 std::vector<std::string> staticKinbodies,
                 std::vector<std::string> movableKinbodies,
                 std::string goalKinbody,
                 string actualGoalObjectName,
                 const string plannerName,
                 std::tuple<double, double> goalRegion,
                 double goalObjectGoalX,
                 double goalObjectGoalY,
                 double goalObjectGoalYaw,
                 double robotGoalX,
                 double robotGoalY,
                 double robotGoalYaw) :
            SimpleSetupBase(controlSpace,
                            mujocoHelper,
                            propagationStepSize,
                            minControlDuration,
                            maxControlDuration,
                            distanceFromGoalThreshold,
                            staticKinbodies,
                            movableKinbodies,
                            goalKinbody,
                            plannerName) {
        _goalRegion = goalRegion;
        _goalObjectGoalX = goalObjectGoalX;
        _goalObjectGoalY = goalObjectGoalY;
        _goalObjectGoalYaw = goalObjectGoalYaw;
        _robotGoalX = robotGoalX;
        _robotGoalY = robotGoalY;
        _robotGoalYaw = robotGoalYaw;
        _actualGoalObjectName = actualGoalObjectName;

        initialiseSimpleSetup();
    }

    void setStatePropagator() {
        oc::SimpleSetup::setStatePropagator(
                oc::StatePropagatorPtr(new StatePropagator2(_si, _mujocoHelper, _movableKinbodies)));
    }

    void setStateValidityChecker() {
        unsigned int goalObjectIndex;
        unsigned int actualGoalObjectIndex;
        for (unsigned int i = 0; i < _movableKinbodies.size(); ++i) {
            if (_movableKinbodies.at(i) == _goalKinbody) {
                goalObjectIndex = i;
            }

            if (_movableKinbodies.at(i) == _actualGoalObjectName) {
                actualGoalObjectIndex = i;
            }
        }
        _si->setStateValidityChecker(std::make_shared<StateValidityChecker2>(_si,
                                                                             actualGoalObjectIndex,
                                                                             _mujocoHelper,
                                                                             _staticKinbodies,
                                                                             _movableKinbodies,
                                                                             goalObjectIndex));
    }

    void setGoal() {
        oc::SimpleSetup::setGoal(std::make_shared<Goal2>(_si,
                                                         _mujocoHelper,
                                                         _goalKinbody,
                                                         _distanceFromGoalThreshold,
                                                         _goalObjectGoalX,
                                                         _goalObjectGoalY,
                                                         _goalObjectGoalYaw,
                                                         _robotGoalX,
                                                         _robotGoalY,
                                                         _robotGoalYaw,
                                                         _movableKinbodies));

        getStateSpace()->as<StateSpace2>()->setRobotGoalPosition(_robotGoalX, _robotGoalY, _robotGoalYaw);
    }

    void initialiseSimpleSetup() {
        setStateValidityChecker();
        setStatePropagator();

        setStartState();
        setGoal();
        setPlanner(0.6);
        setupOtherPrameters();
    }

private:
    std::tuple<double, double> _goalRegion;
    double _goalObjectGoalX;
    double _goalObjectGoalY;
    double _goalObjectGoalYaw;
    double _robotGoalX;
    double _robotGoalY;
    double _robotGoalYaw;
    string _actualGoalObjectName;
};

#endif
