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

#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include "../../utils/MujocoHelper.cpp"
#include "../src/ProjectionEvaluator.cpp"
#include "ApproachingRobotState.cpp"
#include "ProblemDefinition.cpp"
#include <boost/math/constants/constants.hpp>
#include <cmath>

using namespace std;
namespace ob = ompl::base;

#ifndef STATESPACE
#define STATESPACE

class StateSpaceBase : public ob::CompoundStateSpace {
public:
    StateSpaceBase(const double cellSize,
                   ProblemDefinition *problemDefinition,
                   MujocoHelper *mujocoHelper) :
            ob::CompoundStateSpace(),
            _cellSize(cellSize),
            _movableObjects(problemDefinition->getMovableObjectNames()) {

        _problemDefinition = problemDefinition;
        _mujocoHelper = mujocoHelper;
    }

    void createRobotSpace() {
        auto robotStateSpace(std::make_shared<ob::SE2StateSpace>());
        robotStateSpace->setBounds(_problemDefinition->getRobotStateSpaceBounds());

        // For computing distances within the compound state space, the weight of
        // the component also needs to be specified (0.1).
        addSubspace(ob::StateSpacePtr(robotStateSpace), 1.0);
    }

    void createMovableObjectsSpace() {
        double share = 0.0 / _movableObjects.size();
        for (unsigned int i = 0; i < _movableObjects.size(); i++) {
            auto objectSpace(
                    std::make_shared<ob::RealVectorStateSpace>(_problemDefinition->getSizeOfObjectStateSpace()));
            objectSpace->setBounds(_problemDefinition->getObjectStateSpaceBounds());
            addSubspace(ob::StateSpacePtr(objectSpace), share);
        }
    }

    double getAngleBetweenYaws(double yaw1, double yaw2) const {
        double d = fabs(yaw1 - yaw2);
        return (d > boost::math::constants::pi<double>()) ? 2.0 * boost::math::constants::pi<double>() - d : d;
    }

    void registerProjections() override {
        registerDefaultProjection(std::make_shared<StateProjectionEvaluator>(this, _cellSize, _movableObjects));
    }

protected:
    const double _cellSize;
    const std::vector<std::string> _movableObjects;
    ProblemDefinition *_problemDefinition;
    MujocoHelper *_mujocoHelper;
};

#endif
