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

#define _USE_MATH_DEFINES

#include <ompl/control/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include "../../utils/MujocoHelper.cpp"
#include <math.h>

using namespace std;
namespace ob = ompl::base;

#ifndef STATEVALIDITY
#define STATEVALIDITY

class StateValidityCheckerBase : public ob::StateValidityChecker {
public:
    StateValidityCheckerBase(const ob::SpaceInformationPtr &si,
                             MujocoHelper *mujocoHelper,
                             std::vector<std::string> staticKinbodies,
                             std::vector<std::string> movableKinbodies) :
            ob::StateValidityChecker(si) {
        _mujocoHelper = mujocoHelper;
        _staticKinbodies = staticKinbodies;
        _movableKinbodies = movableKinbodies;
        _si = si;
        _stateSpace = si->getStateSpace();
    }

    bool isEndEffectorWithinTheShelf(const ob::State *state) const {
        const ob::CompoundStateSpace::StateType *st = state->as<ob::CompoundStateSpace::StateType>();
        const ob::SE2StateSpace::StateType *robotState = st->as<ob::SE2StateSpace::StateType>(0);

        // Get End-Effector position and check if is within the bounds of the
        // objects (i.e., if the end-effector is basically within the shelf).
        double robotX = robotState->getX();
        double robotY = robotState->getY();
        double robotYaw = robotState->getYaw();
        Eigen::MatrixXf endEffectorInWorld = _mujocoHelper->calculateEndEffectorTransformFromRobotState(robotX, robotY,
                                                                                                        robotYaw);

        double endEffectorX = endEffectorInWorld(0, 3);
        double endEffectorY = endEffectorInWorld(1, 3);

        auto objectStateSpace = _stateSpace->as<ob::CompoundStateSpace>()->getSubspace(
                1)->as<ob::RealVectorStateSpace>();
        ob::RealVectorBounds bounds = objectStateSpace->getBounds();
        double lowX = bounds.low[0];
        double highX = bounds.high[0];
        double lowY = bounds.low[1];
        double highY = bounds.high[1];

        if (endEffectorX < lowX || endEffectorX > highX) {
            return false;
        }

        if (endEffectorY < lowY || endEffectorY > highY) {
            return false;
        }

        return true;
    }

    bool isRobotInContactWithStaticObstacles(const ob::State *state) const {
        const auto *st = state->as<ob::CompoundStateSpace::StateType>();
        const auto *robotState = st->as<ob::SE2StateSpace::StateType>(0);

        // Get End-Effector position and check if is within the bounds of the
        // objects (i.e., if the end-effector is basically within the shelf).
        double robotX = robotState->getX();
        double robotY = robotState->getY();
        double robotYaw = robotState->getYaw();

        _mujocoHelper->setRobotXYPosition(robotX, robotY);
        _mujocoHelper->setRobotYaw(robotYaw);
        _mujocoHelper->forward();
        _mujocoHelper->step();

        for (std::string const &staticObjectName : _staticKinbodies) {
            if (_mujocoHelper->isRobotInContact(staticObjectName)) {
                return true;
            }
        }

        return false;
    }

protected:
    ob::StateSpacePtr _stateSpace;
    MujocoHelper *_mujocoHelper;
    ob::SpaceInformationPtr _si;
    std::vector<std::string> _staticKinbodies;
    std::vector<std::string> _movableKinbodies;
};

#endif
