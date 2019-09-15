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

#ifndef APPROACHGOAL
#define APPROACHGOAL

#include <ompl/control/SimpleSetup.h>
#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include "../../../../../utils/MujocoHelper.cpp"

using namespace std;
namespace ob = ompl::base;

class Goal2 : public ob::GoalSampleableRegion {
public:
    Goal2(const ob::SpaceInformationPtr &si,
          MujocoHelper *mujocoHelper,
          std::string goalObjectName,
          const double threshold,
          double goalObjectGoalX,
          double goalObjectGoalY,
          double goalObjectGoalYaw,
          double robotGoalX,
          double robotGoalY,
          double robotGoalYaw,
          std::vector<std::string> movableObjects) :
            ob::GoalSampleableRegion(si),
            _movableObjects(movableObjects) {
        threshold_ = threshold;
        _goalObjectName = goalObjectName;
        _mujocoHelper = mujocoHelper;
        _si = si;

        _goalObjectGoalX = goalObjectGoalX;
        _goalObjectGoalY = goalObjectGoalY;
        _goalObjectGoalYaw = goalObjectGoalYaw;

        _robotGoalX = robotGoalX;
        _robotGoalY = robotGoalY;
        _robotGoalYaw = robotGoalYaw;

        for (unsigned int i = 0; i < _movableObjects.size(); ++i) {
            if (_movableObjects.at(i) == _goalObjectName)
                _goalObjectIndex = i;
        }
    }

    double distanceGoal(const ob::State *st) const {
        const ob::CompoundStateSpace::StateType *state = st->as<ob::CompoundStateSpace::StateType>();
        const ob::SE2StateSpace::StateType *robotState = state->as<ob::SE2StateSpace::StateType>(0);

        double robotX = robotState->getX();
        double robotY = robotState->getY();
        double robotYaw = robotState->getYaw();

        double dx = robotX - _robotGoalX;
        double dy = robotY - _robotGoalY;
        double dyaw = robotYaw - _robotGoalYaw;

        return sqrt(dx * dx + dy * dy + dyaw * dyaw);
    }

    // Sample a state in the goal region.
    void sampleGoal(ob::State *st) const {
        ob::CompoundStateSpace::StateType *state = st->as<ob::CompoundStateSpace::StateType>();
        ob::SE2StateSpace::StateType *robotState = state->as<ob::SE2StateSpace::StateType>(0);

        robotState->setX(_robotGoalX);
        robotState->setY(_robotGoalY);
        robotState->setYaw(_robotGoalYaw);
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
    double _threshold;
    std::string _goalObjectName;
    MujocoHelper *_mujocoHelper;
    double _goalObjectGoalX;
    double _goalObjectGoalY;
    double _goalObjectGoalYaw;
    double _robotGoalX;
    double _robotGoalY;
    double _robotGoalYaw;
    const std::vector<std::string> _movableObjects;
};

#endif
