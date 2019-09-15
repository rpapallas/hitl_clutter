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

#ifndef APPROACHSTATESPACE
#define APPROACHSTATESPACE

#include "../../../StateSpaceBase.cpp"

class StateSpace2 : public StateSpaceBase {
public:
    StateSpace2(const double cellSize,
                MujocoHelper *mujocoHelper,
                ProblemDefinition *problemDefinition) : StateSpaceBase(cellSize, problemDefinition, mujocoHelper) {

        for (unsigned int i = 0; i < _movableObjects.size(); ++i) {
            if (_movableObjects.at(i) == problemDefinition->getGoalObjectName())
                _goalObjectIndex = i;
        }

        createRobotSpace();
        createMovableObjectsSpace();
    }

    void setRobotGoalPosition(double x, double y, double yaw) {
        _goalRobotX = x;
        _goalRobotY = y;
        _goalRobotYaw = yaw;
    }

    // Distance function for the problem of approaching the object such that
    // can be pushed to another location.
    double distance(const ob::State *state1, const ob::State *state2) const {
        const ob::CompoundStateSpace::StateType *s1 = state1->as<ob::CompoundStateSpace::StateType>();
        const ob::CompoundStateSpace::StateType *s2 = state2->as<ob::CompoundStateSpace::StateType>();

        const ob::SE2StateSpace::StateType *robotState1 = s1->as<ob::SE2StateSpace::StateType>(0);
        const ob::SE2StateSpace::StateType *robotState2 = s2->as<ob::SE2StateSpace::StateType>(0);

        bool isState1GoalState = robotState1->getX() == _goalRobotX && robotState1->getY() == _goalRobotY &&
                                 robotState1->getYaw() == _goalRobotYaw;
        bool isState2GoalState = robotState2->getX() == _goalRobotX && robotState2->getY() == _goalRobotY &&
                                 robotState2->getYaw() == _goalRobotYaw;

        double distance;

        if (isState1GoalState || isState2GoalState) {
            double stateX = isState1GoalState ? robotState2->getX() : robotState1->getX();
            double stateY = isState1GoalState ? robotState2->getY() : robotState1->getY();
            double stateYaw = isState1GoalState ? robotState2->getYaw() : robotState1->getYaw();

            double dx = stateX - _goalRobotX;
            double dy = stateY - _goalRobotY;
            double dyaw = getAngleBetweenYaws(stateYaw, _goalRobotYaw);
            distance = 0.7 * sqrt(dx * dx + dy * dy + dyaw * dyaw);
        } else {
            double dx = robotState1->getX() - robotState2->getX();
            double dy = robotState1->getY() - robotState2->getY();
            double dyaw = getAngleBetweenYaws(robotState1->getYaw(), robotState2->getYaw());
            distance = 0.7 * sqrt(dx * dx + dy * dy + dyaw * dyaw);
        }

        double share = 0.1 / _movableObjects.size();
        for (unsigned int i = 0; i < _movableObjects.size(); ++i) {
            double *goalObjectState1 = s1->as<ob::RealVectorStateSpace::StateType>(i + 1)->values;
            double *goalObjectState2 = s2->as<ob::RealVectorStateSpace::StateType>(i + 1)->values;
            double gdx = goalObjectState1[0] - goalObjectState2[0];
            double gdy = goalObjectState1[1] - goalObjectState2[1];
            double gdyaw = getAngleBetweenYaws(goalObjectState1[2], goalObjectState2[2]);
            distance += share * sqrt(gdx * gdx + gdy * gdy + gdyaw * gdyaw);
        }

        return distance;
    }

private:
    int _goalObjectIndex;
    double _goalRobotX;
    double _goalRobotY;
    double _goalRobotYaw;
};

#endif
