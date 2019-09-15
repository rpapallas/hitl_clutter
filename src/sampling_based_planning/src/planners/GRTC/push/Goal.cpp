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

#ifndef PUSHGOAL
#define PUSHGOAL

#include <ompl/control/SimpleSetup.h>
#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <utility>
#include "../../../../../utils/MujocoHelper.cpp"

using namespace std;
namespace ob = ompl::base;

class Goal3 : public ob::GoalSampleableRegion {
public:
    Goal3(const ob::SpaceInformationPtr &si,
          MujocoHelper *mujocoHelper,
          std::string goalObjectName,
          std::tuple<double, double> goalRegion,
          const double threshold,
          std::vector<std::string> movableObjects) :
            ob::GoalSampleableRegion(si), _movableObjects(movableObjects) {
        threshold_ = threshold;
        _goalRegion = goalRegion;
        _goalObjectName = goalObjectName;
        _mujocoHelper = mujocoHelper;
        _si = si;

        for (unsigned int i = 0; i < _movableObjects.size(); ++i) {
            if (_movableObjects.at(i) == _goalObjectName)
                _goalObjectIndex = i;
        }

        _goalObjX = _mujocoHelper->getBodyXpos(_goalObjectName);
        _goalObjY = _mujocoHelper->getBodyYpos(_goalObjectName);
        double goalRegX = get<0>(_goalRegion);
        double goalRegY = get<1>(_goalRegion);
        double x = _goalObjX - goalRegX;
        double y = _goalObjY - goalRegY;

        _desiredDistance = sqrt(x * x + y * y);
    }

    double distanceGoal(const ob::State *st) const {
        const ob::CompoundStateSpace::StateType *state = st->as<ob::CompoundStateSpace::StateType>();
        double *goalObjectState = state->as<ob::RealVectorStateSpace::StateType>(_goalObjectIndex + 1)->values;

        const double go_x = goalObjectState[0];
        const double go_y = goalObjectState[1];

        const double region_x = std::get<0>(_goalRegion);
        const double region_y = std::get<1>(_goalRegion);

        double delta_x = go_x - region_x;
        double delta_y = go_y - region_y;

        return sqrt((delta_x * delta_x) + (delta_y * delta_y));
    }

    // Sample a state in the goal region.
    void sampleGoal(ob::State *st) const {
        ob::CompoundStateSpace::StateType *state = st->as<ob::CompoundStateSpace::StateType>();
        double *objectState = state->as<ob::RealVectorStateSpace::StateType>(_goalObjectIndex + 1)->values;

        objectState[0] = std::get<0>(_goalRegion); // X
        objectState[1] = std::get<1>(_goalRegion); // Y
    }

    // Return the maximum number of samples that can be asked for before
    // repeating.
    unsigned int maxSampleCount() const {
        return 1;
    }

private:
    int _goalObjectIndex;
    ob::SpaceInformationPtr _si;
    std::tuple<double, double> _goalRegion;
    double _goalObjX;
    double _goalObjY;
    double _desiredDistance;
    std::string _goalObjectName;
    MujocoHelper *_mujocoHelper;
    const std::vector<std::string> _movableObjects;
};

#endif
