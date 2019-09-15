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

#ifndef PUSHSTATESPACE
#define PUSHSTATESPACE

#include "../../../StateSpaceBase.cpp"

class StateSpace3 : public StateSpaceBase {
public:
    // This is the state space for the problem of pushing a goal object
    // from its initial position to the goal position.
    StateSpace3(const double cellSize,
                MujocoHelper *mujocoHelper,
                std::tuple<double, double> goalObjectGoalPosition,
                std::tuple<double, double> goalObjectInitialPosition,
                ProblemDefinition *problemDefinition) :
            StateSpaceBase(cellSize, problemDefinition, mujocoHelper) {

        _goalObjectGoalPosition = goalObjectGoalPosition;
        _goalObjectInitialPosition = goalObjectInitialPosition;

        for (unsigned int i = 0; i < _movableObjects.size(); ++i) {
            if (_movableObjects.at(i) == problemDefinition->getGoalObjectName())
                _goalObjectIndex = i;
        }

        createRobotSpace();
        createMovableObjectsSpace();
    }

    // Distance function for the problem of pushing an object from its
    // initial position to a goal position.
    double distance(const ob::State *state1, const ob::State *state2) const {
        const ob::CompoundStateSpace::StateType *s1 = state1->as<ob::CompoundStateSpace::StateType>();
        const ob::CompoundStateSpace::StateType *s2 = state2->as<ob::CompoundStateSpace::StateType>();

        const ob::SE2StateSpace::StateType *robotState1 = s1->as<ob::SE2StateSpace::StateType>(0);
        const ob::SE2StateSpace::StateType *robotState2 = s2->as<ob::SE2StateSpace::StateType>(0);

        double *goalObjectState1 = s1->as<ob::RealVectorStateSpace::StateType>(_goalObjectIndex + 1)->values;
        double *goalObjectState2 = s2->as<ob::RealVectorStateSpace::StateType>(_goalObjectIndex + 1)->values;

        bool isState1GoalState = goalObjectState1[0] == std::get<0>(_goalObjectGoalPosition) &&
                                 goalObjectState1[1] == std::get<1>(_goalObjectGoalPosition);
        bool isState2GoalState = goalObjectState2[0] == std::get<0>(_goalObjectGoalPosition) &&
                                 goalObjectState2[1] == std::get<1>(_goalObjectGoalPosition);

        double distance = 0.0;
        // Some of the state is the goal state.

        if (isState1GoalState || isState2GoalState) {
            double otherRobotStateX = isState1GoalState ? robotState2->getX() : robotState1->getX();
            double otherRobotStateY = isState1GoalState ? robotState2->getY() : robotState1->getY();
            double otherRobotStateYaw = isState1GoalState ? robotState2->getYaw() : robotState1->getYaw();

            double otherObjectStateX = isState1GoalState ? goalObjectState2[0] : goalObjectState1[0];
            double otherObjectStateY = isState1GoalState ? goalObjectState2[1] : goalObjectState1[1];
            double otherObjectStateYaw = isState1GoalState ? goalObjectState2[2] : goalObjectState1[2];

            double gdx = goalObjectState1[0] - goalObjectState2[0];
            double gdy = goalObjectState1[1] - goalObjectState2[1];
            double gdyaw = getAngleBetweenYaws(goalObjectState1[2], goalObjectState2[2]);
            double distanceTargetObjectToTargetRegion = 0.3 * sqrt(gdx * gdx + gdy * gdy + gdyaw *
                                                                                           gdyaw); // Although we compare target objects the one since is the goal state, should also means it's at the goal region.
            distance += distanceTargetObjectToTargetRegion;

            double rtotdx = otherObjectStateX - otherRobotStateX;
            double rtotdy = otherObjectStateY - otherRobotStateY;
            double rtotdyaw = getAngleBetweenYaws(otherObjectStateYaw, otherRobotStateYaw);
            double distanceOfRobotToTargetObject = 0.5 * sqrt(rtotdx * rtotdx + rtotdy * rtotdy + rtotdyaw * rtotdyaw);
            distance += distanceOfRobotToTargetObject;
        } else {
            double rdx = robotState1->getX() - robotState2->getX();
            double rdy = robotState1->getY() - robotState2->getY();
            double rdyaw = getAngleBetweenYaws(robotState1->getYaw(), robotState2->getYaw());
            distance += 0.5 * sqrt(rdx * rdx + rdy * rdy + rdyaw * rdyaw);

            double gdx = goalObjectState1[0] - goalObjectState2[0];
            double gdy = goalObjectState1[1] - goalObjectState2[1];
            double gdyaw = getAngleBetweenYaws(goalObjectState1[2], goalObjectState2[2]);

            distance += 0.3 * sqrt(gdx * gdx + gdy * gdy + gdyaw * gdyaw);
        }

        return distance;
    }

private:
    std::tuple<double, double> _goalObjectGoalPosition;
    std::tuple<double, double> _goalObjectInitialPosition;
    int _goalObjectIndex;
};

#endif
