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

#ifndef PUSHSIMPLESETUP
#define PUSHSIMPLESETUP

#include "../../../SimpleSetupBase.cpp"
#include "StatePropagator.cpp"
#include "Goal.cpp"
#include "StateValidityChecker.cpp"
#include "StateSpace.cpp"

class SimpleSetup3 : public SimpleSetupBase {
public:
    SimpleSetup3(const oc::ControlSpacePtr &controlSpace,
                 MujocoHelper *mujocoHelper,
                 const double propagationStepSize,
                 const double minControlDuration,
                 const double maxControlDuration,
                 const double distanceFromGoalThreshold,
                 std::vector<std::string> staticKinbodies,
                 std::vector<std::string> movableKinbodies,
                 std::string goalKinbody,
                 std::string actualGoalObjectName,
                 const string plannerName,
                 std::tuple<double, double> goalRegion) :
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
        _actualGoalObjectName = actualGoalObjectName;
        initialiseSimpleSetup();
    }

    void setStatePropagator() {
        oc::SimpleSetup::setStatePropagator(
                oc::StatePropagatorPtr(new StatePropagator3(_si, _mujocoHelper, _movableKinbodies)));
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

        _si->setStateValidityChecker(std::make_shared<StateValidityChecker3>(_si,
                                                                             actualGoalObjectIndex,
                                                                             _mujocoHelper,
                                                                             _staticKinbodies,
                                                                             _movableKinbodies,
                                                                             goalObjectIndex));
    }

    void setGoal() {
        oc::SimpleSetup::setGoal(std::make_shared<Goal3>(_si,
                                                         _mujocoHelper,
                                                         _goalKinbody,
                                                         _goalRegion,
                                                         _distanceFromGoalThreshold,
                                                         _movableKinbodies));
    }

    void initialiseSimpleSetup() {
        _mujocoHelper->setGoalRegion(std::get<0>(_goalRegion), std::get<1>(_goalRegion));

        setStateValidityChecker();
        setStatePropagator();

        setStartState();
        setGoal();
        setPlanner(0.5);
        setupOtherPrameters();
    }

private:
    std::tuple<double, double> _goalRegion;
    string _actualGoalObjectName;
};

#endif
