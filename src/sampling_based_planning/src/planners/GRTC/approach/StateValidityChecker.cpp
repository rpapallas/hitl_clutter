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

#ifndef APPROACHSTATEVALIDITY
#define APPROACHSTATEVALIDITY

#include "../../../StateValidityCheckerBase.cpp"

class StateValidityChecker2 : public StateValidityCheckerBase {
public:
    StateValidityChecker2(const ob::SpaceInformationPtr &si,
                          int actualGoalObjectIndex,
                          MujocoHelper *mujocoHelper,
                          std::vector<std::string> staticKinbodies,
                          std::vector<std::string> movableKinbodies,
                          unsigned int goalObjectIndex) :
            StateValidityCheckerBase(si, mujocoHelper, staticKinbodies, movableKinbodies) {

        _goalObjectIndex = goalObjectIndex;
        _actualGoalObjectIndex = actualGoalObjectIndex;

        for (std::string const &movableObjectName : _movableKinbodies) {
            double x = mujocoHelper->getBodyXpos(movableObjectName);
            double y = mujocoHelper->getBodyYpos(movableObjectName);
            double yaw = mujocoHelper->getBodyYaw(movableObjectName);
            std::vector<double> state = {x, y, yaw};
            _initialPositions.push_back(state);
        }
    }

    virtual bool isValid(const ob::State *state) const override {
        if (!si_->satisfiesBounds(state)) {
            return false;
        }

        if (isRobotInContactWithStaticObstacles(state)) {
            return false;
        }

        // Save this state.
        ob::State *newState = _si->allocState();
        _stateSpace->copyState(newState, state);
        _mujocoHelper->saveState(newState);

        return true;
    }

private:
    std::vector<std::vector<double>> _initialPositions;
    unsigned int _goalObjectIndex;
    unsigned int _actualGoalObjectIndex;
};

#endif
