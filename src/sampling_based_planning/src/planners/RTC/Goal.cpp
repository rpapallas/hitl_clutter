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

#ifndef RTCGOAL
#define RTCGOAL

#include <ompl/control/SimpleSetup.h>
#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <utility>
#include "../../../../utils/MujocoHelper.cpp"

using namespace std;
namespace ob = ompl::base;

class Goal1 : public ob::GoalSampleableRegion {
public:
    Goal1(const ob::SpaceInformationPtr &si,
          MujocoHelper *mujocoHelper,
          std::string goalObjectName,
          const double threshold,
          std::vector<std::string> movableObjects) :
            ob::GoalSampleableRegion(si),
            _movableObjects(movableObjects) {
        threshold_ = threshold;
        _goalObjectName = goalObjectName;
        _mujocoHelper = mujocoHelper;
        _si = si;

        for (unsigned int i = 0; i < _movableObjects.size(); ++i) {
            if (_movableObjects[i] == _goalObjectName) {
                _goalObjectIndex = i;
                break;
            }
        }
    }

    double distanceGoal(const ob::State *st) const override {
        const ob::CompoundStateSpace::StateType *state = st->as<ob::CompoundStateSpace::StateType>();
        const ob::SE2StateSpace::StateType *robotState = state->as<ob::SE2StateSpace::StateType>(0);

        double robotX = robotState->getX();
        double robotY = robotState->getY();
        double robotYaw = robotState->getYaw();

        Eigen::MatrixXf transform = Eigen::MatrixXf::Identity(4, 4);

        transform(0, 0) = cos(robotYaw);
        transform(0, 1) = -sin(robotYaw);
        transform(1, 0) = sin(robotYaw);
        transform(1, 1) = cos(robotYaw);

        transform(0, 3) = robotX;
        transform(1, 3) = robotY;

        Eigen::MatrixXf endEffectorInRobot = _mujocoHelper->getEndEffectorInRobotTransform();
        Eigen::MatrixXf endEffectorInWorld = transform * endEffectorInRobot;

        double ee_x = endEffectorInWorld(0, 3);
        double ee_y = endEffectorInWorld(1, 3);

        double *goalObjectState = state->as<ob::RealVectorStateSpace::StateType>(_goalObjectIndex + 1)->values;

        const double go_x = goalObjectState[0];
        const double go_y = goalObjectState[1];

        double delta_x = go_x - ee_x;
        double delta_y = go_y - ee_y;

        return sqrt((delta_x * delta_x) + (delta_y * delta_y));
    }

    std::tuple<double, double, double>
    getRobotXYYawFromEndEffectorTransform(const Eigen::MatrixXf &endEffectorInWorld) const {
        // Convert endEffectorTransform to RobotTransform
        Eigen::MatrixXf endEffectorInRobot = _mujocoHelper->getEndEffectorInRobotTransform();
        Eigen::MatrixXf robotInEndEffector = endEffectorInRobot.inverse();
        Eigen::MatrixXf robotInWorld = endEffectorInWorld * robotInEndEffector;
        double x = robotInWorld(0, 3);
        double y = robotInWorld(1, 3);

        mjtNum target_transform[9];
        target_transform[0] = robotInWorld(0, 0);
        target_transform[1] = robotInWorld(0, 1);
        target_transform[2] = robotInWorld(0, 2);
        target_transform[3] = robotInWorld(1, 0);
        target_transform[4] = robotInWorld(1, 1);
        target_transform[5] = robotInWorld(1, 2);
        target_transform[6] = robotInWorld(2, 0);
        target_transform[7] = robotInWorld(2, 1);
        target_transform[8] = robotInWorld(2, 2);

        mjtNum quat[4];
        mju_mat2Quat(quat, target_transform);

        double yaw = _mujocoHelper->getYawFromQuat(quat[0], quat[1], quat[2], quat[3]);
        return std::make_tuple(x, y, yaw);
    }

    // Sample a state in the goal region.
    void sampleGoal(ob::State *st) const {
        ob::State *bestSt = _mujocoHelper->getBestState();

        ob::CompoundStateSpace::StateType *state = st->as<ob::CompoundStateSpace::StateType>();
        ob::CompoundStateSpace::StateType *bestState = bestSt->as<ob::CompoundStateSpace::StateType>();

        double *objectState = state->as<ob::RealVectorStateSpace::StateType>(_goalObjectIndex + 1)->values;
        double *objectBestState = bestState->as<ob::RealVectorStateSpace::StateType>(_goalObjectIndex + 1)->values;

        objectState[0] = objectBestState[0];
        objectState[1] = objectBestState[1];
        objectState[2] = objectBestState[2];
        objectState[3] = objectBestState[3];
        objectState[4] = objectBestState[4];
        objectState[5] = objectBestState[5];
        objectState[6] = objectBestState[6];
        objectState[7] = objectBestState[7];
        objectState[8] = objectBestState[8];
        objectState[9] = objectBestState[9];
        objectState[10] = objectBestState[10];
        objectState[11] = objectBestState[11];
        objectState[12] = objectBestState[12];
        objectState[13] = objectBestState[13];
        objectState[14] = objectBestState[14];

        ob::SE2StateSpace::StateType *robotState = state->as<ob::SE2StateSpace::StateType>(0);

        std::vector<Eigen::MatrixXf> transforms = getGoal(objectBestState[0], objectBestState[1]);
        for (auto currentTransform : transforms) {
            if (!_mujocoHelper->checkRobotTransformForCollisions(currentTransform, "shelf")) {
                auto robotDesiredState = _mujocoHelper->getRobotStateFromEndEffectorTransform(currentTransform);
                double x = get<0>(robotDesiredState);
                double y = get<1>(robotDesiredState);
                double yaw = get<2>(robotDesiredState);
                robotState->setX(x);
                robotState->setY(y);
                robotState->setYaw(yaw);

                break;
            }
        }
    }

    Eigen::MatrixXf
    getRotatedRelativeTransform(const Eigen::MatrixXf &originalTransform,
                                const Eigen::MatrixXf &objectsCurrentTransform,
                                double angle) const {
        Eigen::MatrixXf rotationMatrix = Eigen::MatrixXf::Identity(4, 4);
        rotationMatrix(0, 0) = cos(angle);
        rotationMatrix(0, 1) = -sin(angle);
        rotationMatrix(1, 0) = sin(angle);
        rotationMatrix(1, 1) = cos(angle);

        Eigen::MatrixXf endEffectorInObject = objectsCurrentTransform.inverse() * originalTransform;
        Eigen::MatrixXf endEffectorInEndEffector = endEffectorInObject.inverse() * rotationMatrix * endEffectorInObject;
        Eigen::MatrixXf endEffectorInWorld = objectsCurrentTransform * endEffectorInObject;
        Eigen::MatrixXf finalTransform = endEffectorInWorld * endEffectorInEndEffector;

        return finalTransform;
    }

    std::vector<Eigen::MatrixXf> getGoal(double objectX, double objectY) const {
        std::vector<Eigen::MatrixXf> transforms;

        // This is a fixed transform of the robot relative to the goal object.
        Eigen::MatrixXf robotInWorld = Eigen::MatrixXf::Identity(4, 4);
        robotInWorld(0, 3) = 0.318487;
        robotInWorld(1, 3) = 0.0591397;

        // This is a fixed transform of the goal object to find the relative
        // transform from.
        Eigen::MatrixXf objectInWorld = Eigen::MatrixXf::Identity(4, 4);
        objectInWorld(0, 3) = 1.62883;
        objectInWorld(1, 3) = -0.121746;

        Eigen::MatrixXf worldInObject = objectInWorld.inverse();

        // This a relative transform that has the goal object in the robot
        // hand that we can use to find new transforms like this one.
        Eigen::MatrixXf robotInObject = worldInObject * robotInWorld;

        Eigen::MatrixXf currentObjectInWorld = Eigen::MatrixXf::Identity(4, 4);
        currentObjectInWorld(0, 3) = objectX;
        currentObjectInWorld(1, 3) = objectY;

        Eigen::MatrixXf newRobotInWorld = currentObjectInWorld * robotInObject;
        Eigen::MatrixXf endEffectorInRobot = _mujocoHelper->getEndEffectorInRobotTransform();
        Eigen::MatrixXf endEffectorInWorld = newRobotInWorld * endEffectorInRobot;

        transforms.push_back(endEffectorInWorld);

        Eigen::MatrixXf transform_1 = endEffectorInWorld;
        Eigen::MatrixXf transform_2 = endEffectorInWorld;
        for (int i = 0; i < 4; i++) {
            double angle = 0.174533; // 10 degrees
            transform_1 = getRotatedRelativeTransform(transform_1, currentObjectInWorld, angle);
            transform_2 = getRotatedRelativeTransform(transform_2, currentObjectInWorld, -angle);

            transforms.push_back(transform_1);
            transforms.push_back(transform_2);
        }

        return transforms;
    }

    // Return the maximum number of samples that can be asked for before
    // repeating.
    unsigned int maxSampleCount() const {
        return 1;
    }

private:
    int _goalObjectIndex;
    ob::SpaceInformationPtr _si;
    std::string _goalObjectName;
    MujocoHelper *_mujocoHelper;
    const std::vector<std::string> _movableObjects;
};

#endif
