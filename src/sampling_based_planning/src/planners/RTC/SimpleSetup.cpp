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

#ifndef RTCSIMPLESETUP
#define RTCSIMPLESETUP

#include "../../SimpleSetupBase.cpp"
#include "Goal.cpp"
#include "StatePropagator.cpp"
#include "StateValidityChecker.cpp"
#include "StateSpace.cpp"

class SimpleSetup1 : public SimpleSetupBase {
public:
    SimpleSetup1(const oc::ControlSpacePtr &controlSpace,
                 MujocoHelper *mujocoHelper,
                 const double propagationStepSize,
                 const double minControlDuration,
                 const double maxControlDuration,
                 const double distanceFromGoalThreshold,
                 std::vector<std::string> staticKinbodies,
                 std::vector<std::string> movableKinbodies,
                 std::string goalKinbody,
                 const string plannerName) :
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
        initialiseSimpleSetup();
    }

    void setStateValidityChecker() {
        _si->setStateValidityChecker(std::make_shared<StateValidityChecker1>(_si,
                                                                             _mujocoHelper,
                                                                             _staticKinbodies,
                                                                             _movableKinbodies));
    }

    void setStatePropagator() {
        oc::SimpleSetup::setStatePropagator(
                oc::StatePropagatorPtr(new StatePropagator1(_si, _mujocoHelper, _movableKinbodies)));
    }

    void setGoal() {
        oc::SimpleSetup::setGoal(std::make_shared<Goal1>(_si,
                                                         _mujocoHelper,
                                                         _goalKinbody,
                                                         _distanceFromGoalThreshold,
                                                         _movableKinbodies));
    }

    void initialiseSimpleSetup() {
        setStateValidityChecker();
        setStatePropagator();

        setStartState();

        setGoal();
        setPlanner(0.1);
        setupOtherPrameters();
    }
};

#endif
