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

#ifndef APPROACHINGROBOTSTATE
#define APPROACHINGROBOTSTATE

#include "../../utils/MujocoHelper.cpp"
#include <Eigen/Dense>

using namespace std;

class ApproachingRobotState {
public:
    ApproachingRobotState(MujocoHelper *mujocoHelper, const Eigen::MatrixXf &endEffectorInWorld, int type) {
        _mujocoHelper = mujocoHelper;
        _x = getXFromTransform(endEffectorInWorld);
        _y = getYFromTransform(endEffectorInWorld);
        _yaw = getYawFromTransform(endEffectorInWorld);
        _type = type;
        calculateNumberOfCollisions();
    }

    double getXFromTransform(const Eigen::MatrixXf &endEffectorInWorld) {
        tuple<double, double, double> robotState = _mujocoHelper->getRobotStateFromEndEffectorTransform(
                endEffectorInWorld);
        return get<0>(robotState);
    }

    double getYFromTransform(const Eigen::MatrixXf &endEffectorInWorld) {
        tuple<double, double, double> robotState = _mujocoHelper->getRobotStateFromEndEffectorTransform(
                endEffectorInWorld);
        return get<1>(robotState);
    }

    double getYawFromTransform(const Eigen::MatrixXf &endEffectorInWorld) {
        tuple<double, double, double> robotState = _mujocoHelper->getRobotStateFromEndEffectorTransform(
                endEffectorInWorld);
        return get<2>(robotState);
    }

    void calculateNumberOfCollisions() {
        int numberOfCollisions = _mujocoHelper->getNumberOfCollisionsForRobotsDesiredState(_x, _y, _yaw);
        _numberOfCollisions = numberOfCollisions;
    }

    double getX() {
        return _x;
    }

    double getY() {
        return _y;
    }

    double getYaw() {
        return _yaw;
    }

    int getNumberOfCollisions() {
        return _numberOfCollisions;
    }

    int getType() {
        return _type;
    }

private:
    MujocoHelper *_mujocoHelper;
    double _x;
    double _y;
    double _yaw;
    int _type;
    int _numberOfCollisions = 0;
};

#endif
