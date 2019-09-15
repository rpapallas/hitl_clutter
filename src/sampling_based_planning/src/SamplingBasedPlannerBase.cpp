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

#include <atomic>
#include <boost/optional.hpp>
#include "../../utils/MujocoHelper.cpp"
#include "../src/ApproachingRobotState.cpp"
#include "../src/ControlSpace.cpp"
#include "planners/GRTC/push/SimpleSetup.cpp"
#include "planners/GRTC/approach/SimpleSetup.cpp"
#include <boost/thread/thread.hpp>

#ifndef SAMPLINGBASEDPLANNERS
#define SAMPLINGBASEDPLANNERS

class SamplingBasedPlanner {
public:
    SamplingBasedPlanner(MujocoHelper *mujocoHelper, ProblemDefinition *problemDefinition) {
        _problemDefinition = problemDefinition;
        _mujocoHelper = mujocoHelper;
        _solutionFound = false;
        _solution = boost::none;
    }

    Eigen::MatrixXf getRotatedRelativeTransform(Eigen::MatrixXf originalTransform, string objectName, double angle) {
        Eigen::MatrixXf rotationMatrix = Eigen::MatrixXf::Identity(4, 4);
        rotationMatrix(0, 0) = cos(angle);
        rotationMatrix(0, 1) = -sin(angle);
        rotationMatrix(1, 0) = sin(angle);
        rotationMatrix(1, 1) = cos(angle);

        Eigen::MatrixXf endEffectorInObject = _mujocoHelper->getBodyTransform(objectName).inverse() * originalTransform;
        Eigen::MatrixXf endEffectorInEndEffector = endEffectorInObject.inverse() * rotationMatrix * endEffectorInObject;
        Eigen::MatrixXf robotInWorld = _mujocoHelper->getBodyTransform(objectName) * endEffectorInObject;
        Eigen::MatrixXf finalTransform = robotInWorld * endEffectorInEndEffector;

        return finalTransform;
    }

    Eigen::Vector3d getEndEffectorTransformToObject(double goal_x,
                                                    double goal_y,
                                                    double object_x,
                                                    double object_y,
                                                    double scale) {
        double distance_x = object_x - goal_x;
        double distance_y = object_y - goal_y;
        double distance_z = 0.0;

        double magnitude = sqrt(distance_x * distance_x + distance_y * distance_y + distance_z * distance_z);
        double unit_vector[3] = {distance_x / magnitude, distance_y / magnitude, distance_z / magnitude};

        unit_vector[0] *= scale;
        unit_vector[1] *= scale;

        double x = object_x + unit_vector[0];
        double y = object_y + unit_vector[1];
        double z = 0.0;

        Eigen::Vector3d endEffectorGoalPosition(x, y, z);
        return endEffectorGoalPosition;
    }

    Eigen::MatrixXf getEndEffectorTransformAlignedToGoal(double object_x,
                                                         double object_y,
                                                         double goal_x,
                                                         double goal_y,
                                                         double scale) {
        Eigen::Vector3d target_position = getEndEffectorTransformToObject(goal_x, goal_y, object_x, object_y, scale);
        double target_x = target_position[0];
        double target_y = target_position[1];

        double dx = object_x - target_x;
        double dy = object_y - target_y;
        double dz = 0.0;

        double magnitude = sqrt(dx * dx + dy * dy + dz * dz);
        double unit_vector[3] = {dx / magnitude, dy / magnitude, dz / magnitude};

        Eigen::Vector3d x_rotation(unit_vector[0], unit_vector[1], unit_vector[2]);
        Eigen::Vector3d z_rotation(0, 0, 1);
        Eigen::Vector3d y_rotation2 = z_rotation.cross(x_rotation);

        double mag = sqrt(
                y_rotation2[0] * y_rotation2[0] + y_rotation2[1] * y_rotation2[1] + y_rotation2[2] * y_rotation2[2]);
        Eigen::Vector3d y_rotation = Eigen::Vector3d(y_rotation2[0] / mag, y_rotation2[1] / mag,
                                                     y_rotation2[2] / mag); // unit vector.

        // Orientation X
        Eigen::MatrixXf endEffectorInWorld = Eigen::MatrixXf::Identity(4, 4);
        endEffectorInWorld(0, 0) = x_rotation[0];
        endEffectorInWorld(1, 0) = x_rotation[1];
        endEffectorInWorld(2, 0) = x_rotation[2];

        // Orientation Y
        endEffectorInWorld(0, 1) = y_rotation[0];
        endEffectorInWorld(1, 1) = y_rotation[1];
        endEffectorInWorld(2, 1) = y_rotation[2];

        // Orientation Z
        endEffectorInWorld(0, 2) = z_rotation[0];
        endEffectorInWorld(1, 2) = z_rotation[1];
        endEffectorInWorld(2, 2) = z_rotation[2];

        // Position
        endEffectorInWorld(0, 3) = target_x;
        endEffectorInWorld(1, 3) = target_y;

        return endEffectorInWorld;
    }

    void setThreadStartTime(int threadId) {
        std::lock_guard<std::mutex> lock(_startTimeMutex);
        _threadStartTime[threadId] = chrono::steady_clock::now();
    }

    chrono::steady_clock::time_point getThreadStartTime(int threadId) {
        std::lock_guard<std::mutex> lock(_startTimeMutex);
        return _threadStartTime[threadId];
    }

    vector<ApproachingRobotState>
    getApproachingPositions(string goalObjectName, tuple<double, double> goalRegion) {
        mjtGeom geomType = _mujocoHelper->getGeomTypeFromBodyName(goalObjectName);

        if (geomType == mjGEOM_BOX) {
            return getApproachingPositionsForBox(goalObjectName, goalRegion);
        } else if (geomType == mjGEOM_CYLINDER) {
            return getApproachingPositionsForCylinder(goalObjectName, goalRegion);
        } else {
            throw std::logic_error("Approaching states can only be calculated for boxes and cylinders.");
        }
    }

    vector<ApproachingRobotState>
    getApproachingPositionsForCylinder(string goalObjectName, tuple<double, double> goalRegion) {
        double object_x = _mujocoHelper->getBodyXpos(goalObjectName);
        double object_y = _mujocoHelper->getBodyYpos(goalObjectName);
        double goal_x = std::get<0>(goalRegion);
        double goal_y = std::get<1>(goalRegion);

        // This is a transform that has the object in the gripper and the gripper's
        // orientation is such that is facing against the pushing goal position.
        // Think of it that the object is in the gripper's hand and if the robot
        // moves forward the direction is towards the goal position.
        Eigen::MatrixXf original1 = getEndEffectorTransformAlignedToGoal(object_x, object_y, goal_x, goal_y, 0.055);

        //vector<Eigen::MatrixXf> transforms;
        //transforms.push_back(original1);

        // Now find what is the correction rotation direction (+ or - z).
        Eigen::MatrixXf robotOriginalTransform = _mujocoHelper->getEndEffectorTransform();
        Eigen::MatrixXf matrixOffset = original1.inverse() * robotOriginalTransform;

        Eigen::Matrix3d m = Eigen::Matrix3d::Identity(3, 3);
        m(0, 0) = matrixOffset(0, 0);
        m(0, 1) = matrixOffset(0, 1);
        m(0, 2) = matrixOffset(0, 2);
        m(1, 0) = matrixOffset(1, 0);
        m(1, 1) = matrixOffset(1, 1);
        m(1, 2) = matrixOffset(1, 2);
        m(2, 0) = matrixOffset(2, 0);
        m(2, 1) = matrixOffset(2, 1);
        m(2, 2) = matrixOffset(2, 2);

        Eigen::AngleAxisd newAngleAxis(m);
        double angle = newAngleAxis.angle();

        // If the returned angle is > PI then the shortest angle is negative.
        int sign = 0;
        if (angle > M_PI || angle > -M_PI) {
            sign = -1;
        }

        if (angle < -M_PI || angle < M_PI) {
            sign = 1;
        }
        sign = sign * newAngleAxis.axis()[2];

        // Rotate +/- 90 degrees from original1 (in case where the original1
        // is facing the goal object from one of the shelf's sides, if such
        // case occurs we should rotate +/- 90 degrees torwards the original
        // orientation of the robot (assuming that the robot's intiial orientation
        // is facing the table's from the correct side (from the front). This
        // ensures that if the original1 transform is actually from the
        // table side (where obviously is the shelf side) we rotate +/- 90
        // degrees torwards the original orientation of the robot.
        Eigen::MatrixXf original2 = getRotatedRelativeTransform(original1, goalObjectName, sign * M_PI / 2);

        // Now find transforms for pushing sideways
        Eigen::MatrixXf original3 = getEndEffectorTransformAlignedToGoal(object_x, object_y, goal_x, goal_y, 0.12);

        angle = sign * M_PI / 2;
        Eigen::MatrixXf rotationMatrix = Eigen::MatrixXf::Identity(4, 4);
        rotationMatrix(0, 0) = cos(angle);
        rotationMatrix(0, 1) = -sin(angle);
        rotationMatrix(1, 0) = sin(angle);
        rotationMatrix(1, 1) = cos(angle);
        Eigen::MatrixXf original4 = original3 * rotationMatrix;

        //transforms.push_back(original2);
        //transforms.push_back(original4);
        int type1 = 1; // This type has the object in hand.
        int type2 = 2; // This type has the object on the side of the hand.

        std::vector<std::tuple<double, double, double>> robotApproachingPositions;
        vector<ApproachingRobotState> robotApproachingStates;

        bool type1StateFound = false;
        //type1StateFound = true;

        if (!_mujocoHelper->checkRobotTransformForCollisions(original1, "shelf")) {
            ApproachingRobotState state(_mujocoHelper, original1, type1);
            robotApproachingStates.push_back(state);
            type1StateFound = true;
        }

        if (!type1StateFound) {
            // Get some more.
            Eigen::MatrixXf newTransform1 = original1;
            Eigen::MatrixXf newTransform2 = original1;

            // Check 60 degrees rotation +-
            for (int i = 0; i < 1; ++i) {
                double angleOfRotation1 = 0.174533; // 10 degrees
                double angleOfRotation2 = -0.174533; // 10 degrees
                newTransform1 = getRotatedRelativeTransform(newTransform1, goalObjectName, angleOfRotation1);
                newTransform2 = getRotatedRelativeTransform(newTransform2, goalObjectName, angleOfRotation2);

                if (!_mujocoHelper->checkRobotTransformForCollisions(newTransform1, "shelf")) {
                    ApproachingRobotState state(_mujocoHelper, newTransform1, type1);
                    robotApproachingStates.push_back(state);
                    type1StateFound = true;
                    break;
                }

                if (!_mujocoHelper->checkRobotTransformForCollisions(newTransform2, "shelf")) {
                    ApproachingRobotState state(_mujocoHelper, newTransform2, type1);
                    robotApproachingStates.push_back(state);
                    type1StateFound = true;
                    break;
                }
            }
        }

        bool ttt = false;
        if (!_mujocoHelper->checkRobotTransformForCollisions(original2, "shelf")) {
            ApproachingRobotState state(_mujocoHelper, original2, type1);
            robotApproachingStates.push_back(state);
            ttt = true;
        }

        if (!ttt) {
            Eigen::MatrixXf newTransform1 = original2;
            Eigen::MatrixXf newTransform2 = original2;

            // Check 60 degrees rotation +-
            for (int i = 0; i < 1; ++i) {
                double angleOfRotation1 = 0.174533; // 10 degrees
                double angleOfRotation2 = -0.174533; // 10 degrees
                newTransform1 = getRotatedRelativeTransform(newTransform1, goalObjectName, angleOfRotation1);
                newTransform2 = getRotatedRelativeTransform(newTransform2, goalObjectName, angleOfRotation2);

                if (!_mujocoHelper->checkRobotTransformForCollisions(newTransform1, "shelf")) {
                    ApproachingRobotState state(_mujocoHelper, newTransform1, type1);
                    robotApproachingStates.push_back(state);
                    ttt = true;
                    break;
                }

                if (!_mujocoHelper->checkRobotTransformForCollisions(newTransform2, "shelf")) {
                    ApproachingRobotState state(_mujocoHelper, newTransform2, type1);
                    robotApproachingStates.push_back(state);
                    ttt = true;
                    break;
                }
            }
        }


        bool type2StateFound = false;

        if (!_mujocoHelper->checkRobotTransformForCollisions(original4, "shelf")) {
            ApproachingRobotState state(_mujocoHelper, original4, type1);
            robotApproachingStates.push_back(state);
            type2StateFound = true;
        }

        if (!type2StateFound) {
            Eigen::MatrixXf newTransform1 = original4;
            Eigen::MatrixXf newTransform2 = original4;

            // Check 60 degrees rotation +-
            for (int i = 0; i < 1; ++i) {
                double angleOfRotation1 = 0.174533; // 10 degrees
                double angleOfRotation2 = -0.174533; // 10 degrees
                newTransform1 = getRotatedRelativeTransform(newTransform1, goalObjectName, angleOfRotation1);
                newTransform2 = getRotatedRelativeTransform(newTransform2, goalObjectName, angleOfRotation2);

                if (!_mujocoHelper->checkRobotTransformForCollisions(newTransform1, "shelf")) {
                    ApproachingRobotState state(_mujocoHelper, newTransform1, type2);
                    robotApproachingStates.push_back(state);
                    type2StateFound = true;
                    break;
                }

                if (!_mujocoHelper->checkRobotTransformForCollisions(newTransform2, "shelf")) {
                    ApproachingRobotState state(_mujocoHelper, newTransform2, type2);
                    robotApproachingStates.push_back(state);
                    type2StateFound = true;
                    break;
                }
            }
        }

        //auto filteredApproachingStates = filterOutSomeApproachingStates(robotApproachingStates);
        //return filteredApproachingStates;
        return robotApproachingStates;
    }

    int getDirectionOfShortestAngle(Eigen::MatrixXf matrixOffset) {
        Eigen::Matrix3d m = Eigen::Matrix3d::Identity(3, 3);
        m(0, 0) = matrixOffset(0, 0);
        m(0, 1) = matrixOffset(0, 1);
        m(0, 2) = matrixOffset(0, 2);
        m(1, 0) = matrixOffset(1, 0);
        m(1, 1) = matrixOffset(1, 1);
        m(1, 2) = matrixOffset(1, 2);
        m(2, 0) = matrixOffset(2, 0);
        m(2, 1) = matrixOffset(2, 1);
        m(2, 2) = matrixOffset(2, 2);

        Eigen::AngleAxisd newAngleAxis(m);
        double angle = newAngleAxis.angle();

        // If the returned angle is > PI then the shortest angle is negative.
        int direction;
        if (angle > M_PI) {
            direction = -1;
        } else {
            direction = 1;
        }

        direction = direction * newAngleAxis.axis()[2];

        cout << direction << endl;

        return direction;
    }

    double getAngle(Eigen::MatrixXf matrixOffset) {
        Eigen::Matrix3d m = Eigen::Matrix3d::Identity(3, 3);
        m(0, 0) = matrixOffset(0, 0);
        m(0, 1) = matrixOffset(0, 1);
        m(0, 2) = matrixOffset(0, 2);
        m(1, 0) = matrixOffset(1, 0);
        m(1, 1) = matrixOffset(1, 1);
        m(1, 2) = matrixOffset(1, 2);
        m(2, 0) = matrixOffset(2, 0);
        m(2, 1) = matrixOffset(2, 1);
        m(2, 2) = matrixOffset(2, 2);

        Eigen::AngleAxisd newAngleAxis(m);
        return newAngleAxis.angle();
    }

    vector<ApproachingRobotState>
    getApproachingPositionsForBox(string goalObjectName, tuple<double, double> goalRegion) {
        double object_x = _mujocoHelper->getBodyXpos(goalObjectName);
        double object_y = _mujocoHelper->getBodyYpos(goalObjectName);
        double goal_x = std::get<0>(goalRegion);
        double goal_y = std::get<1>(goalRegion);

        // This is a transform that has the object in the gripper and the gripper's
        // orientation is such that is facing against the pushing goal position.
        // Think of it that the object is in the gripper's hand and if the robot
        // moves forward the direction is towards the goal position.
        Eigen::MatrixXf original1 = getEndEffectorTransformAlignedToGoal(object_x, object_y, goal_x, goal_y, 0.15);

        // Now find what is the correction rotation direction (+ or - z).
        Eigen::MatrixXf robotOriginalTransform = _mujocoHelper->getEndEffectorTransform();
        Eigen::MatrixXf matrixOffset = original1.inverse() * robotOriginalTransform;

        int sign = getDirectionOfShortestAngle(matrixOffset);
        double angle = getAngle(matrixOffset);

        // Now find transforms for pushing sideways
        Eigen::MatrixXf original3 = getEndEffectorTransformAlignedToGoal(object_x, object_y, goal_x, goal_y, 0.165);

        angle = sign * M_PI / 2;
        Eigen::MatrixXf rotationMatrix = Eigen::MatrixXf::Identity(4, 4);
        rotationMatrix(0, 0) = cos(angle);
        rotationMatrix(0, 1) = -sin(angle);
        rotationMatrix(1, 0) = sin(angle);
        rotationMatrix(1, 1) = cos(angle);
        Eigen::MatrixXf original4 = original3 * rotationMatrix;

        int type1 = 1; // This type has the object in hand.

        std::vector<std::tuple<double, double, double>> robotApproachingPositions;
        vector<ApproachingRobotState> robotApproachingStates;

        if (!_mujocoHelper->checkRobotTransformForCollisions(original1, "shelf")) {
            ApproachingRobotState state(_mujocoHelper, original1, type1);
            robotApproachingStates.push_back(state);
        }

        if (!_mujocoHelper->checkRobotTransformForCollisions(original4, "shelf")) {
            ApproachingRobotState state(_mujocoHelper, original4, type1);
            robotApproachingStates.push_back(state);
        }

        return robotApproachingStates;
    }

    double getPropagationStepSize() {
        return PROPAGATION_STEP_SIZE;
    }

    void setMujocoHelper(MujocoHelper *mujocoHelper) {
        _mujocoHelper = mujocoHelper;
    }

    bool terminationCondition(int threadId) {
        auto begin = getThreadStartTime(threadId);
        auto end = chrono::steady_clock::now();
        double planningTime = chrono::duration_cast<chrono::seconds>(end - begin).count();
        return planningTime > MAX_PLANNING_TIME_IN_SECONDS_PER_THREAD_HITL || _solutionFound;
    }

    void setMaxPlanningTime(double seconds) {
        MAX_PLANNING_TIME_IN_SECONDS = seconds;
    }

protected:
    // OMPL PARAMETERS
    double MAX_PLANNING_TIME_IN_SECONDS = 300.0; // This is for autonomous RRT/KPIECE.
    const double MAX_PLANNING_TIME_IN_SECONDS_PER_THREAD_HITL = 10.0; // This is for HITL pushing.
    const double DISTANCE_FROM_GOAL_THRESHOLD = 0.07; // This is close to goal object's radius.
    const double CELL_SIZE = 0.05;
    const double PROPAGATION_STEP_SIZE = 1.0;
    const int MAX_CONTROL_DURATION = 1;
    const int MIN_CONTROL_DURATION = 1;

    // MuJoCo data structures
    ProblemDefinition *_problemDefinition;
    std::mutex _startTimeMutex;
    vector<chrono::steady_clock::time_point> _threadStartTime;
    MujocoHelper *_mujocoHelper;
    std::atomic<bool> _solutionFound;
    bool _isSingleThreadedFinished = false;
    boost::optional<std::tuple<shared_ptr<SimpleSetup2>, shared_ptr<SimpleSetup3>>> _solution;
};

#endif
