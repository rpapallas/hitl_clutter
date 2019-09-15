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
#include <chrono>
#include <thread>
#include <fstream>
#include "../../utils/MujocoHelper.cpp"
#include <math.h>

using namespace std;
namespace oc = ompl::control;
namespace ob = ompl::base;

#ifndef STATEPROPAGATOR
#define STATEPROPAGATOR

class StatePropagatorBase : public oc::StatePropagator {
public:
    StatePropagatorBase(const oc::SpaceInformationPtr &si, MujocoHelper *mujocoHelper,
                        const std::vector<std::string> movableObjects) :
            oc::StatePropagator(si), _movableObjects(movableObjects) {
        _mujocoHelper = mujocoHelper;
    }

    virtual void propagate(const ob::State *state, const oc::Control *control, const double duration,
                           ob::State *result) const override {
        _mujocoHelper->propagateThreadSafe(state, control, duration, result, &_movableObjects);
    }

    double getAngleBetweenYaws(double yaw1, double yaw2) const {
        double dyaw = atan2(tan(yaw1), M_PI) - atan2(tan(yaw2), M_PI);

        if (dyaw > M_PI) {
            dyaw = 2 * M_PI - dyaw;
        } else if (dyaw < -M_PI) {
            dyaw = -(2 * M_PI) - dyaw;
        }

        return dyaw;
    }

    virtual bool steer(const ob::State *state1, const ob::State *state2, oc::Control *control, double &duration) const {
        const ob::CompoundStateSpace::StateType *s1 = state1->as<ob::CompoundStateSpace::StateType>();
        const ob::CompoundStateSpace::StateType *s2 = state2->as<ob::CompoundStateSpace::StateType>();

        const ob::SE2StateSpace::StateType *robotState1 = s1->as<ob::SE2StateSpace::StateType>(0);
        const ob::SE2StateSpace::StateType *robotState2 = s2->as<ob::SE2StateSpace::StateType>(0);

        double dx = robotState2->getX() - robotState1->getX();
        double dy = robotState2->getY() - robotState1->getY();
        double dyaw = robotState2->getYaw() - robotState1->getYaw();

        double mag = sqrt(dx * dx + dy * dy + dyaw * dyaw);
        double unitx = dx / mag;
        double unity = dy / mag;
        double unityaw = dyaw / mag;

        double *controls = control->as<oc::RealVectorControlSpace::ControlType>()->values;
        double factor = 0.1;
        controls[0] = unitx * factor;
        controls[1] = unity * factor;
        controls[2] = unityaw * factor;

        duration = 1.0;

        return true;
    }

protected:
    MujocoHelper *_mujocoHelper;
    const std::vector<std::string> _movableObjects;
};

#endif
