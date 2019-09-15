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
#include <math.h>
#include "../../utils/MujocoHelper.cpp"

using namespace std;
namespace ob = ompl::base;

#ifndef PROJECTIONEVALUATOR
#define PROJECTIONEVALUATOR

class StateProjectionEvaluator : public ob::ProjectionEvaluator {
public:
    StateProjectionEvaluator(const ob::StateSpace *space, const double cellSize,
                             const std::vector<std::string> movableObjects) :
            ob::ProjectionEvaluator(space), _cellSize(cellSize), _movableObjects(movableObjects) {
    }

    unsigned int getDimension() const override {
        return 3;
    }

    void defaultCellSizes() override {
        cellSizes_.resize(getDimension());

        for (unsigned int i = 0; i < getDimension(); i++) {
            cellSizes_[i] = _cellSize;
        }
    }

    void project(const ob::State *state, Eigen::Ref<Eigen::VectorXd> projection) const override {
        const ob::CompoundStateSpace::StateType *s = state->as<ob::CompoundStateSpace::StateType>();
        const ob::SE2StateSpace::StateType *robotState = s->as<ob::SE2StateSpace::StateType>(0);

        projection[0] = robotState->getX();
        projection[1] = robotState->getY();
        projection[2] = robotState->getYaw();
    }

private:
    const double _cellSize;
    const std::vector<std::string> _movableObjects;
};

#endif
