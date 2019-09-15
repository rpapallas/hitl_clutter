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

#ifndef RTCSTATESPACE
#define RTCSTATESPACE

#include "../../StateSpaceBase.cpp"

class StateSpace1 : public StateSpaceBase {
public:
    StateSpace1(const double cellSize,
                MujocoHelper *mujocoHelper,
                ProblemDefinition *problemDefinition) : StateSpaceBase(cellSize, problemDefinition, mujocoHelper) {
        createRobotSpace();
        createMovableObjectsSpace();

        for (unsigned int i = 0; i < _movableObjects.size(); ++i) {
            if (_movableObjects[i] == problemDefinition->getGoalObjectName()) {
                _goalObjectIndex = i;
                break;
            }
        }
    }

    double distance(const ob::State *state1, const ob::State *state2) const override {
        const ob::CompoundStateSpace::StateType *s1 = state1->as<ob::CompoundStateSpace::StateType>();
        const ob::CompoundStateSpace::StateType *s2 = state2->as<ob::CompoundStateSpace::StateType>();

        const ob::SE2StateSpace::StateType *robotState1 = s1->as<ob::SE2StateSpace::StateType>(0);
        const ob::SE2StateSpace::StateType *robotState2 = s2->as<ob::SE2StateSpace::StateType>(0);

        double rdx = robotState1->getX() - robotState2->getX();
        double rdy = robotState1->getY() - robotState2->getY();
        double rdyaw = getAngleBetweenYaws(robotState1->getYaw(), robotState2->getYaw());
        double distance = 0.2 * sqrt(rdx * rdx + rdy * rdy + rdyaw * rdyaw);

        double *goalObjectState1 = s1->as<ob::RealVectorStateSpace::StateType>(_goalObjectIndex + 1)->values;
        double *goalObjectState2 = s2->as<ob::RealVectorStateSpace::StateType>(_goalObjectIndex + 1)->values;
        double gdx = goalObjectState1[0] - goalObjectState2[0];
        double gdy = goalObjectState1[1] - goalObjectState2[1];
        distance += 0.4 * sqrt(gdx * gdx + gdy * gdy);

        double share = 0.05 / (_movableObjects.size() - 1);
        for (unsigned int i = 0; i < _movableObjects.size(); ++i) {
            if (_goalObjectIndex != i) {
                goalObjectState1 = s1->as<ob::RealVectorStateSpace::StateType>(i + 1)->values;
                goalObjectState2 = s2->as<ob::RealVectorStateSpace::StateType>(i + 1)->values;
                gdx = goalObjectState1[0] - goalObjectState2[0];
                gdy = goalObjectState1[1] - goalObjectState2[1];
                distance += share * sqrt(gdx * gdx + gdy * gdy);
            }
        }

        return distance;
    }

private:
    unsigned int _goalObjectIndex;
};

#endif
