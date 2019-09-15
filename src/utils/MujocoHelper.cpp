//  Copyright (C) 2018 Rafael Papallas and The University of Leeds
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

#include "mujoco.h"
#include <ompl/control/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include "glfw3.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <mutex>
#include <Eigen/Dense>
#include <utility>

#ifndef MUJOCOHELPER
#define MUJOCOHELPER

namespace oc = ompl::control;
namespace ob = ompl::base;
using namespace std;

class MujocoHelper {
public:
    MujocoHelper(mjModel *model,
                 mjData *data,
                 std::string robotName,
                 std::string robotLinearXjointName,
                 std::string robotLinearYjointName,
                 std::string robotAngularZjointName,
                 const double robotInitialX,
                 const double robotInitialY) :
            _robotName(robotName),
            _robotLinearXjointName(robotLinearXjointName),
            _robotLinearYjointName(robotLinearYjointName),
            _robotAngularZjointName(robotAngularZjointName),
            _robotInitialXPosition(robotInitialX),
            _robotInitialYPosition(robotInitialY) {

        _model = model;
        _data = data;

        std::tuple<int, int> indicesLinearX = getJointQvelAddr(_robotLinearXjointName);
        std::tuple<int, int> indicesLinearY = getJointQvelAddr(_robotLinearYjointName);
        std::tuple<int, int> indicesAngularZ = getJointQvelAddr(_robotAngularZjointName);
        _robotLinearXjointQvelIndex = std::get<0>(indicesLinearX);
        _robotLinearYjointQvelIndex = std::get<0>(indicesLinearY);
        _robotAngularZjointQvelIndex = std::get<0>(indicesAngularZ);

        const std::string tableName = "table";
        _tableId = mj_name2id(_model, mjOBJ_BODY, tableName.c_str());
    }


    // ===================================================================
    //                         MuJoCo Functions
    // ===================================================================

    void step() {
        std::lock_guard<std::mutex> lockGuard(mtx);
        mj_step(_model, _data);
    }

    void forward() {
        std::lock_guard<std::mutex> lockGuard(mtx);
        mj_forward(_model, _data);
    }

    mjModel *getModel() {
        return _model;
    }

    mjData *getData() {
        return _data;
    }

    double *getNumericField(const std::string &fieldName) {
        for (int i = 0; i < _model->nnumeric; i++) {
            std::string f = _model->names + _model->name_numericadr[i];
            if (fieldName == f) {
                return _model->numeric_data + _model->numeric_adr[i];
            }
        }

        return nullptr;
    }

    string getTextField(const string &fieldName) {
        for (int i = 0; i < _model->ntext; i++) {
            std::string f = _model->names + _model->name_textadr[i];
            if (fieldName == f) {
                return _model->text_data + _model->text_adr[i];
            }
        }

        return string();

    }

    void setResetLevel(int level) {
        int resetLevelsAvailable = _mujocoStates.size();

        if (level > resetLevelsAvailable) {
            throw std::invalid_argument(
                    "The reset level is invalid. (there is/are " + std::to_string(resetLevelsAvailable) +
                    " only states you requested " + std::to_string(level) + ".)");
        } else if (level < 0) {
            throw std::invalid_argument("The reset level should be a positive number or 0.");
        }

        _resetLevel = level;
    }

    int getResetLevel() {
        return _resetLevel;
    }

    void setResetLevelToLatest() {
        setResetLevel(getMaxResetLevel());
    }

    int getMaxResetLevel() {
        return _mujocoStates.size();
    }

    void saveMujocoState() {
        mjData *newData = mj_makeData(_model);
        mj_copyData(newData, _model, _data);
        _mujocoStates.push_back(newData);
    }

    void deleteLastState() {
        int n = getMaxResetLevel();
        if (n >= 1) {
            _mujocoStates.pop_back();
            setResetLevel(n - 1);
        }
    }

    void emptyResetQueue() {
        _mujocoStates.clear();
        _mujocoStates.shrink_to_fit();
        setResetLevel(0);
    }

    void emptyResetQueueButLast() {
        mjData *latestData = mj_makeData(_model);
        mj_copyData(latestData, _model, _mujocoStates[_mujocoStates.size() - 1]);
        emptyResetQueue();
        _mujocoStates.push_back(latestData);
        setResetLevel(1);
    }

    void resetSimulation() {
        std::lock_guard<std::mutex> lockGuard(mtx);
        mj_resetData(_model, _data);

        if (_resetLevel > 0) {
            mjData *targetData = _mujocoStates.at(_resetLevel - 1);
            mj_copyData(_data, _model, targetData);
        }

        mj_forward(_model, _data);
        _isSystemValid = true;
    }

    void zeroOutVelocities() {
        for (int i = 0; i < _model->nv; ++i) {
            _data->qvel[i] = 0.0;
        }
    }

    double getTimeStep() {
        return _model->opt.timestep;
    }

    std::tuple<int, int> getJointQposAddr(const std::string &name) {
        int jointId = mj_name2id(_model, mjOBJ_JOINT, name.c_str());
        int jointType = _model->jnt_type[jointId];
        int jointAddr = _model->jnt_qposadr[jointId];

        int ndim;
        if (jointType == mjJNT_FREE)
            ndim = 7;
        else if (jointType == mjJNT_BALL)
            ndim = 4;
        else if (jointType == mjJNT_HINGE || jointType == mjJNT_SLIDE)
            ndim = 1;

        if (ndim == 1)
            return std::make_tuple(jointAddr, jointAddr);

        return std::make_tuple(jointAddr, jointAddr + ndim);
    }

    std::tuple<int, int> getJointQvelAddr(const std::string &jointName) {
        int jointId = mj_name2id(_model, mjOBJ_JOINT, jointName.c_str());
        int jointType = _model->jnt_type[jointId];
        int jointAddr = _model->jnt_dofadr[jointId];

        int ndim = 1;
        if (jointType == mjJNT_FREE)
            ndim = 6;
        else if (jointType == mjJNT_BALL)
            ndim = 3;
        else if (jointType == mjJNT_HINGE || jointType == mjJNT_SLIDE)
            ndim = 1;

        if (ndim == 1)
            return std::make_tuple(jointAddr, jointAddr);

        return std::make_tuple(jointAddr, jointAddr + ndim);
    }

    void setBodyVelocity(const std::string &name, double linearX, double linearY, double linearZ, double angularX,
                         double angularY, double angularZ) {
        int bodyId = mj_name2id(_model, mjOBJ_BODY, name.c_str());
        const int jointIndex = _model->body_jntadr[bodyId];
        const int start = _model->jnt_dofadr[jointIndex];
        //auto indices = getJointQvelAddr(name);
        //int start = get<0>(indices);

        std::lock_guard<std::mutex> lockGuard(mtx);
        _data->qvel[start + 0] = linearX;
        _data->qvel[start + 1] = linearY;
        _data->qvel[start + 2] = linearZ;
        _data->qvel[start + 3] = angularX;
        _data->qvel[start + 4] = angularY;
        _data->qvel[start + 5] = angularZ;
    }

    std::tuple<double, double, double, double, double, double> getBodyVelocity(const std::string &name) {
        int bodyId = mj_name2id(_model, mjOBJ_BODY, name.c_str());
        const int jointIndex = _model->body_jntadr[bodyId];
        const int start = _model->jnt_dofadr[jointIndex];

        std::lock_guard<std::mutex> lockGuard(mtx);
        double linearX = _data->qvel[start + 0];
        double linearY = _data->qvel[start + 1];
        double linearZ = _data->qvel[start + 2];
        double angularX = _data->qvel[start + 3];
        double angularY = _data->qvel[start + 4];
        double angularZ = _data->qvel[start + 5];

        return std::make_tuple(linearX, linearY, linearZ, angularX, angularY, angularZ);
    }

    // ===================================================================
    //                            Transformations
    // ===================================================================

    double getRollFromQuat(double w, double x, double y, double z) {
        mj_normalizeQuat(_model, _data->qpos);

        double aSinInput = -2 * (x * z - w * y);
        if (aSinInput > 1.0)
            aSinInput = 1;
        if (aSinInput < -1.0)
            aSinInput = -1;

        return atan2(2 * (y * z + w * x), w * w - x * x - y * y + z * z);
    }

    double getPitchFromQuat(double w, double x, double y, double z) {
        mj_normalizeQuat(_model, _data->qpos);

        double aSinInput = -2 * (x * z - w * y);
        if (aSinInput > 1.0)
            aSinInput = 1;
        if (aSinInput < -1.0)
            aSinInput = -1;

        return asin(aSinInput);
    }

    double getYawFromQuat(double w, double x, double y, double z) {
        mj_normalizeQuat(_model, _data->qpos);

        double aSinInput = -2 * (x * z - w * y);
        if (aSinInput > 1.0)
            aSinInput = 1;
        if (aSinInput < -1.0)
            aSinInput = -1;

        return atan2(2 * (x * y + w * z), w * w + x * x - y * y - z * z);
    }

    static boost::array<double, 4> getQuatFromAxisAngle(mjtNum *axis, double value) {
        mjtNum quaternion[4];
        mju_axisAngle2Quat(quaternion, axis, value);

        boost::array<double, 4> finalQuaternion = {{quaternion[0], quaternion[1], quaternion[2], quaternion[3]}};
        return finalQuaternion;
    }

    boost::array<double, 4> getQuatFromRoll(double roll) {
        mjtNum xaxis[3] = {1, 0, 0};
        return getQuatFromAxisAngle(xaxis, roll);
    }

    boost::array<double, 4> getQuatFromPitch(double pitch) {
        mjtNum yaxis[3] = {0, 1, 0};
        return getQuatFromAxisAngle(yaxis, pitch);
    }

    boost::array<double, 4> getQuatFromYaw(double yaw) {
        mjtNum zaxis[3] = {0, 0, 1};
        return getQuatFromAxisAngle(zaxis, yaw);
    }

    static boost::array<double, 4> getQuatFromTransform(Eigen::MatrixXf transform) {
        double m00 = transform(0, 0);
        double m01 = transform(0, 1);
        double m02 = transform(0, 2);
        double m10 = transform(1, 0);
        double m11 = transform(1, 1);
        double m12 = transform(1, 2);
        double m20 = transform(2, 0);
        double m21 = transform(2, 1);
        double m22 = transform(2, 2);

        double tr = m00 + m11 + m22;

        double qw, qx, qy, qz;
        if (tr > 0) {
            float S = sqrt(tr + 1.0) * 2; // S=4*qw
            qw = 0.25 * S;
            qx = (m21 - m12) / S;
            qy = (m02 - m20) / S;
            qz = (m10 - m01) / S;
        } else if ((m00 > m11) & (m00 > m22)) {
            float S = sqrt(1.0 + m00 - m11 - m22) * 2; // S=4*qx
            qw = (m21 - m12) / S;
            qx = 0.25 * S;
            qy = (m01 + m10) / S;
            qz = (m02 + m20) / S;
        } else if (m11 > m22) {
            float S = sqrt(1.0 + m11 - m00 - m22) * 2; // S=4*qy
            qw = (m02 - m20) / S;
            qx = (m01 + m10) / S;
            qy = 0.25 * S;
            qz = (m12 + m21) / S;
        } else {
            float S = sqrt(1.0 + m22 - m00 - m11) * 2; // S=4*qz
            qw = (m10 - m01) / S;
            qx = (m02 + m20) / S;
            qy = (m12 + m21) / S;
            qz = 0.25 * S;
        }

        boost::array<double, 4> quaternion = {{qw, qx, qy, qz}};
        return quaternion;
    }


    // ===================================================================
    //                            Bodies
    // ===================================================================

    mjtGeom getGeomTypeFromBodyName(const std::string &name) {
        int bodyId = mj_name2id(_model, mjOBJ_BODY, name.c_str());
        int bodyNumberOfGeoms = _model->body_geomnum[bodyId];

        // If the body has more geoms than 1 or no geoms then it's
        // ambigious and we throw an error.
        if (bodyNumberOfGeoms != 1) {
            throw std::invalid_argument(
                    "bodyGeomType: The given body name should have exactly 1 geom defined (" + name + ").");
        }

        int addrOfBodyGeom = _model->body_geomadr[bodyId];
        return (mjtGeom) _model->geom_type[addrOfBodyGeom];
    }

    double getBodyXpos(const std::string &name) {
        int bodyId = mj_name2id(_model, mjOBJ_BODY, name.c_str());
        const int jointIndex = _model->body_jntadr[bodyId];
        const int qposIndex = _model->jnt_qposadr[jointIndex];
        std::lock_guard<std::mutex> lockGuard(mtx);
        return _data->qpos[qposIndex + 0];
    }

    double getBodyYpos(const std::string &name) {
        int bodyId = mj_name2id(_model, mjOBJ_BODY, name.c_str());
        const int jointIndex = _model->body_jntadr[bodyId];
        const int qposIndex = _model->jnt_qposadr[jointIndex];
        std::lock_guard<std::mutex> lockGuard(mtx);
        return _data->qpos[qposIndex + 1];
    }

    double getBodyZpos(const std::string &name) {
        int bodyId = mj_name2id(_model, mjOBJ_BODY, name.c_str());
        const int jointIndex = _model->body_jntadr[bodyId];
        const int qposIndex = _model->jnt_qposadr[jointIndex];
        std::lock_guard<std::mutex> lockGuard(mtx);
        return _data->qpos[qposIndex + 2];
    }

    double getBodyRoll(std::string name) {
        return getBodyEulerAngle(name, 0);
    }

    double getBodyPitch(std::string name) {
        return getBodyEulerAngle(name, 1);
    }

    double getBodyYaw(std::string name) {
        return getBodyEulerAngle(name, 2);
    }

    double getBodyEulerAngle(const std::string &name, int axis) {
        int bodyId = mj_name2id(_model, mjOBJ_BODY, name.c_str());
        const int jointIndex = _model->body_jntadr[bodyId];
        const int qposIndex = _model->jnt_qposadr[jointIndex];

        std::lock_guard<std::mutex> lockGuard(mtx);
        double w = _data->qpos[qposIndex + 3];
        double x = _data->qpos[qposIndex + 4];
        double y = _data->qpos[qposIndex + 5];
        double z = _data->qpos[qposIndex + 6];

        switch (axis) {
            case 0 :
                return getRollFromQuat(w, x, y, z);
            case 1 :
                return getPitchFromQuat(w, x, y, z);
            case 2 :
                return getYawFromQuat(w, x, y, z);
            default:
                return 0.0;
        }
    }

    void setBodyRoll(const std::string &name, double roll) {
        int bodyId = mj_name2id(_model, mjOBJ_BODY, name.c_str());
        const int jointIndex = _model->body_jntadr[bodyId];
        const int qposIndex = _model->jnt_qposadr[jointIndex];

        boost::array<double, 4> quaternion = getQuatFromRoll(roll);
        double w = quaternion[0];
        double x = quaternion[1];
        double y = quaternion[2];
        double z = quaternion[3];

        std::lock_guard<std::mutex> lockGuard(mtx);
        _data->qpos[qposIndex + 3] = w;
        _data->qpos[qposIndex + 4] = x;
        _data->qpos[qposIndex + 5] = y;
        _data->qpos[qposIndex + 6] = z;
    }

    std::tuple<double, double, double, double> getBodyQuaternion(const std::string &name) {
        int bodyId = mj_name2id(_model, mjOBJ_BODY, name.c_str());
        const int jointIndex = _model->body_jntadr[bodyId];
        const int qposIndex = _model->jnt_qposadr[jointIndex];

        std::lock_guard<std::mutex> lockGuard(mtx);
        double w = _data->qpos[qposIndex + 3];
        double x = _data->qpos[qposIndex + 4];
        double y = _data->qpos[qposIndex + 5];
        double z = _data->qpos[qposIndex + 6];

        return std::make_tuple(w, x, y, z);
    }

    void setBodyPitch(const std::string &name, double pitch) {
        int bodyId = mj_name2id(_model, mjOBJ_BODY, name.c_str());
        const int jointIndex = _model->body_jntadr[bodyId];
        const int qposIndex = _model->jnt_qposadr[jointIndex];

        boost::array<double, 4> quaternion = getQuatFromPitch(pitch);
        double w = quaternion[0];
        double x = quaternion[1];
        double y = quaternion[2];
        double z = quaternion[3];

        std::lock_guard<std::mutex> lockGuard(mtx);
        _data->qpos[qposIndex + 3] = w;
        _data->qpos[qposIndex + 4] = x;
        _data->qpos[qposIndex + 5] = y;
        _data->qpos[qposIndex + 6] = z;
    }

    void setBodyYaw(const std::string &name, double yaw) {
        int bodyId = mj_name2id(_model, mjOBJ_BODY, name.c_str());
        const int jointIndex = _model->body_jntadr[bodyId];
        const int qposIndex = _model->jnt_qposadr[jointIndex];

        boost::array<double, 4> quaternion = getQuatFromYaw(yaw);
        double w = quaternion[0];
        double x = quaternion[1];
        double y = quaternion[2];
        double z = quaternion[3];

        std::lock_guard<std::mutex> lockGuard(mtx);
        _data->qpos[qposIndex + 3] = w;
        _data->qpos[qposIndex + 4] = x;
        _data->qpos[qposIndex + 5] = y;
        _data->qpos[qposIndex + 6] = z;
    }

    void setBodyQuat(const std::string &name, double w, double x, double y, double z) {
        int bodyId = mj_name2id(_model, mjOBJ_BODY, name.c_str());
        const int jointIndex = _model->body_jntadr[bodyId];
        const int qposIndex = _model->jnt_qposadr[jointIndex];

        std::lock_guard<std::mutex> lockGuard(mtx);
        _data->qpos[qposIndex + 3] = w;
        _data->qpos[qposIndex + 4] = x;
        _data->qpos[qposIndex + 5] = y;
        _data->qpos[qposIndex + 6] = z;
    }

    void setBodyXYPosition(const std::string &name, double x, double y) {
        int bodyId = mj_name2id(_model, mjOBJ_BODY, name.c_str());
        const int jointIndex = _model->body_jntadr[bodyId];
        const int qposIndex = _model->jnt_qposadr[jointIndex];
        std::lock_guard<std::mutex> lockGuard(mtx);

        _data->qpos[qposIndex + 0] = x;
        _data->qpos[qposIndex + 1] = y;
    }

    void setBodyZPosition(const std::string &name, double z) {
        int bodyId = mj_name2id(_model, mjOBJ_BODY, name.c_str());
        const int jointIndex = _model->body_jntadr[bodyId];
        const int qposIndex = _model->jnt_qposadr[jointIndex];
        std::lock_guard<std::mutex> lockGuard(mtx);
        _data->qpos[qposIndex + 2] = z;
    }

    double getBodyVelocity(std::string jointName, int desiredVelocityIndex) {
        std::tuple<int, int> indices = getJointQvelAddr(jointName);
        int startIndex = std::get<0>(indices);
        int endIndex = std::get<1>(indices);

        if (startIndex + desiredVelocityIndex > endIndex) {
            throw std::invalid_argument("getBodyVelocity: Index out of bounds.");
        }

        return _data->qvel[startIndex + desiredVelocityIndex];
    }

    void setBodyQuatToNull(const std::string &bodyName) {
        int bodyId = mj_name2id(_model, mjOBJ_BODY, bodyName.c_str());
        const int jointIndex = _model->body_jntadr[bodyId];
        const int qposIndex = _model->jnt_qposadr[jointIndex];

        double w = 1.0;
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;

        std::lock_guard<std::mutex> lockGuard(mtx);
        _data->qpos[qposIndex + 3] = w;
        _data->qpos[qposIndex + 4] = x;
        _data->qpos[qposIndex + 5] = y;
        _data->qpos[qposIndex + 6] = z;
    }

    void setBodyAccelerations(const std::string &bodyName, double a1, double a2, double a3, double a4, double a5,
                              double a6) {
        int bodyId = mj_name2id(_model, mjOBJ_BODY, bodyName.c_str());
        const int jointIndex = _model->body_jntadr[bodyId];
        const int qvelAdr = _model->jnt_dofadr[jointIndex];

        std::lock_guard<std::mutex> lockGuard(mtx);
        _data->qacc_warmstart[qvelAdr + 0] = a1;
        _data->qacc_warmstart[qvelAdr + 1] = a2;
        _data->qacc_warmstart[qvelAdr + 2] = a3;
        _data->qacc_warmstart[qvelAdr + 3] = a4;
        _data->qacc_warmstart[qvelAdr + 4] = a5;
        _data->qacc_warmstart[qvelAdr + 5] = a6;
    }

    std::tuple<double, double, double, double, double, double> getBodyAccelerations(const std::string &name) {
        int bodyId = mj_name2id(_model, mjOBJ_BODY, name.c_str());
        const int jointIndex = _model->body_jntadr[bodyId];
        const int qvelAdr = _model->jnt_dofadr[jointIndex];

        std::lock_guard<std::mutex> lockGuard(mtx);
        double a1 = _data->qacc[qvelAdr + 0];
        double a2 = _data->qacc[qvelAdr + 1];
        double a3 = _data->qacc[qvelAdr + 2];
        double a4 = _data->qacc[qvelAdr + 3];
        double a5 = _data->qacc[qvelAdr + 4];
        double a6 = _data->qacc[qvelAdr + 5];

        return std::make_tuple(a1, a2, a3, a4, a5, a6);
    }

    Eigen::MatrixXf getBodyTransform(const std::string &name) {
        // Create an identity matrix
        Eigen::MatrixXf transform = Eigen::MatrixXf::Identity(4, 4);

        // Get robot's theta value.
        double theta = getBodyYaw(name);
        double x = getBodyXpos(name);
        double y = getBodyYpos(name);
        double z = getBodyZpos(name);

        // Create the array such that it looks like this:
        //  cos(θ)    sin(θ)   0     x
        // -sin(θ)    cos(θ)   0     y
        //  0         0        1     z
        //  0         0        0     1

        transform(0, 0) = cos(theta);
        transform(0, 1) = -sin(theta);
        transform(1, 0) = sin(theta);
        transform(1, 1) = cos(theta);

        transform(0, 3) = x;
        transform(1, 3) = y;
        transform(2, 3) = z;

        return transform;
    }

    bool isHighForceAppliedToShelf() {
        return isHighForceAppliedToBody("shelf", 9);
    }

    bool isHighForceAppliedToBody(std::string bodyName, double threshold) {
        vector<mjtNum *> bodyForceTorques = getBodyForceTorques(bodyName);
        for (mjtNum *forceTorque : bodyForceTorques) {
            for (int i = 0; i < 6; ++i) { // 3 forces and 3 torques
                if (abs(forceTorque[i]) > threshold)
                    return true;
            }
        }

        return false;
    }

    vector<mjtNum *> getBodyForceTorques(const std::string &name) {
        int bodyId = mj_name2id(_model, mjOBJ_BODY, name.c_str());

        std::lock_guard<std::mutex> lockGuard(mtx);
        int numberOfContacts = _data->ncon;

        vector<int> contactIds;
        for (int i = 0; i < numberOfContacts; ++i) {
            auto contact = _data->contact[i];

            int bodyInContact1 = _model->body_rootid[_model->geom_bodyid[contact.geom1]];
            int bodyInContact2 = _model->body_rootid[_model->geom_bodyid[contact.geom2]];

            bool tableIsNotInContact = bodyInContact1 != _tableId && bodyInContact2 != _tableId;
            bool desiredBodyIsInContact = bodyInContact1 == bodyId || bodyInContact2 == bodyId;

            if (tableIsNotInContact && desiredBodyIsInContact) {
                contactIds.push_back(i);
            }
        }

        vector<mjtNum *> forceTorques = std::vector<mjtNum *>();
        if (!contactIds.empty()) {
            for (int contactId : contactIds) {
                mjtNum bodyForceTorque[6];
                mj_contactForce(_model, _data, contactId, bodyForceTorque);
                forceTorques.push_back(bodyForceTorque);
            }
        }

        return forceTorques;
    }

    std::tuple<double, double, double> getBodyTorque(const std::string &name) {
        int bodyId = mj_name2id(_model, mjOBJ_BODY, name.c_str());

        std::lock_guard<std::mutex> lockGuard(mtx);
        double t1 = _data->xfrc_applied[(6 * bodyId) + 3];
        double t2 = _data->xfrc_applied[(6 * bodyId) + 4];
        double t3 = _data->xfrc_applied[(6 * bodyId) + 5];
        return std::make_tuple(t1, t2, t3);
    }


    // ===================================================================
    //                                 Robot
    // ===================================================================

    double getRobotXpos() {
        int bodyId = mj_name2id(_model, mjOBJ_BODY, _robotName.c_str());

        const int jointIndex = _model->body_jntadr[bodyId];
        const int qposIndex = _model->jnt_qposadr[jointIndex];
        std::lock_guard<std::mutex> lockGuard(mtx);
        return _robotInitialXPosition + _data->qpos[qposIndex + 0];
    }

    static double normOf(const vector<double> &vector) {
        double sum = 0.0;
        for (auto value : vector) {
            sum += value * value;
        }

        return sqrt(sum);
    }

    vector<double> unitVectorOf(const vector<double> &vector) {
        double norm = normOf(vector);
        std::vector<double> unitVector;
        for (auto value : vector) {
            unitVector.push_back(value / norm);
        }

        return unitVector;
    }

    double getDistanceOfEndEffectorTo(const string &objectName) {
        // Current position of goal object.
        double goalObjectX = getBodyXpos(objectName);
        double goalObjectY = getBodyYpos(objectName);

        auto endEffectorPosition = getSitePosition("ee_point_1"); // This is the position of the end-effector.
        double endEffectorX = endEffectorPosition[0];
        double endEffectorY = endEffectorPosition[1];

        auto site2 = getSitePosition("ee_point_2");
        double site2_x = site2[0];
        double site2_y = site2[1];

        // Direction vector
        double eeVectorX = site2_x - endEffectorX;
        double eeVectorY = site2_y - endEffectorY;
        vector<double> directionVector = {eeVectorX, eeVectorY};
        vector<double> unitDirectionVector = unitVectorOf(directionVector);

        // Find the vector of the end-effector to the goal object.
        double eeToGoalX = goalObjectX - endEffectorX;
        double eeToGoalY = goalObjectY - endEffectorY;
        vector<double> eeToGoalVector = {eeToGoalX, eeToGoalY};
        vector<double> unitEeToGoalVector = unitVectorOf(eeToGoalVector);

        double x1 = eeVectorX;
        double y1 = eeVectorY;
        double x2 = eeToGoalX;
        double y2 = eeToGoalY;

        double dot = x1 * x2 + y1 * y2;
        double det = x1 * y2 - y1 * x2;
        double angle = atan2(det, dot) * 0.2;

        return sqrt(eeToGoalX * eeToGoalX + eeToGoalY * eeToGoalY + angle * angle);
    }

    vector<double> getSitePosition(const std::string &siteName) {
        int siteID = mj_name2id(_model, mjOBJ_SITE, siteName.c_str());

        double x = _data->site_xpos[3 * siteID + 0];
        double y = _data->site_xpos[3 * siteID + 1];
        double z = _data->site_xpos[3 * siteID + 2];

        vector<double> values = {x, y, z};
        return values;
    }

    Eigen::MatrixXf getSiteTransform(std::string siteName) {
        //int siteID = mj_name2id(_model, mjOBJ_SITE, siteName.c_str());
        vector<double> sitePosition = getSitePosition(siteName);

        // Site Position
        Eigen::MatrixXf transform = Eigen::MatrixXf::Identity(4, 4);
        transform(0, 3) = sitePosition[0];
        transform(1, 3) = sitePosition[1];
        transform(2, 3) = sitePosition[2];

        return transform;
    }

    double getRobotYpos() {
        int bodyId = mj_name2id(_model, mjOBJ_BODY, _robotName.c_str());

        const int jointIndex = _model->body_jntadr[bodyId];
        const int qposIndex = _model->jnt_qposadr[jointIndex];
        std::lock_guard<std::mutex> lockGuard(mtx);
        return _robotInitialYPosition + _data->qpos[qposIndex + 1];
    }

    // TODO: We assume that the robot is planar here, needs to be fixed when we moved away from the planar robot.
    static double getRobotZpos() {
        return 0.0;
    }

    Eigen::MatrixXf getRobotTransform() {
        // Create an identity matrix
        Eigen::MatrixXf transform = Eigen::MatrixXf::Identity(4, 4);

        // Get robot's theta value.
        double theta = getRobotYaw();
        double x = getRobotXpos();
        double y = getRobotYpos();
        double z = getRobotZpos();

        transform(0, 0) = cos(theta);
        transform(0, 1) = -sin(theta);
        transform(1, 0) = sin(theta);
        transform(1, 1) = cos(theta);

        transform(0, 3) = x;
        transform(1, 3) = y;
        transform(2, 3) = z;

        return transform;
    }

    Eigen::MatrixXf getEndEffectorInRobotTransform() {
        auto robotInWorldTransform = getRobotTransform();
        auto endEffectorInWorldTransform = getEndEffectorTransform();
        Eigen::MatrixXf test = robotInWorldTransform.inverse() * endEffectorInWorldTransform;

        // Create an identity matrix.
        Eigen::MatrixXf fixedTransform = Eigen::MatrixXf::Identity(4, 4);

        // Add in the required components that expresses a fixed transfrom
        // from robotTransform such that the new transfom is the end-effector's
        // transform. (i.e robot transform (at the back of the arm) vs hand
        // transform).
        fixedTransform(0, 3) = test(0, 3);
        fixedTransform(1, 3) = test(1, 3);
        fixedTransform(2, 3) = test(2, 3);

        return fixedTransform;
    }

    Eigen::MatrixXf getEndEffectorTransform() {
        return getSiteTransform("ee_point_1");
    }

    void setRobotXYPosition(double x, double y) {
        x = x - _robotInitialXPosition;
        y = y - _robotInitialYPosition;

        int bodyId = mj_name2id(_model, mjOBJ_BODY, _robotName.c_str());
        const int jointIndex = _model->body_jntadr[bodyId];
        const int qposIndex = _model->jnt_qposadr[jointIndex];
        std::lock_guard<std::mutex> lockGuard(mtx);

        _data->qpos[qposIndex + 0] = x;
        _data->qpos[qposIndex + 1] = y;
    }

    void setRobotYaw(double yaw) {
        int bodyId = mj_name2id(_model, mjOBJ_BODY, _robotName.c_str());
        const int jointIndex = _model->body_jntadr[bodyId];
        const int qposIndex = _model->jnt_qposadr[jointIndex];
        std::lock_guard<std::mutex> lockGuard(mtx);
        _data->qpos[qposIndex + 2] = yaw;
    }

    double getRobotYaw() {
        int bodyId = mj_name2id(_model, mjOBJ_BODY, _robotName.c_str());
        const int jointIndex = _model->body_jntadr[bodyId];
        const int qposIndex = _model->jnt_qposadr[jointIndex];
        std::lock_guard<std::mutex> lockGuard(mtx);
        return _data->qpos[qposIndex + 2];
    }


    // ===================================================================
    //                             Controllers
    // ===================================================================

    void setRobotVelocity(double linearX, double linearY, double angularZ) {
        std::lock_guard<std::mutex> lockGuard(mtx);
        _data->qvel[_robotLinearXjointQvelIndex] = linearX;
        _data->qvel[_robotLinearYjointQvelIndex] = linearY;
        _data->qvel[_robotAngularZjointQvelIndex] = angularZ;
    }

    double getRobotVelocity(int index) {
        std::lock_guard<std::mutex> lockGuard(mtx);
        switch (index) {
            case 0:
                return _data->qvel[_robotLinearXjointQvelIndex];
                break;
            case 1:
                return _data->qvel[_robotLinearYjointQvelIndex];
                break;
            case 2:
                return _data->qvel[_robotAngularZjointQvelIndex];
                break;
            default:
                throw std::invalid_argument("In getRobotVelocity: Index out of bounds.");
        }
    }

    void
    propagateThreadSafe(const ob::State *state, const oc::Control *control, const double duration, ob::State *result,
                        const vector<string> *_movableObjects) {
        std::lock_guard<std::mutex> lockGuard(propagateMtx);
        const vector<string> &movableObjects = *_movableObjects;

        resetSimulation();

        // Controls
        const double *controls = control->as<oc::RealVectorControlSpace::ControlType>()->values;

        // Robot state state prior propagation.
        const ob::CompoundStateSpace::StateType *s = state->as<ob::CompoundStateSpace::StateType>();
        const ob::SE2StateSpace::StateType *robotState = s->as<ob::SE2StateSpace::StateType>(0);

        // Set robot's start state.
        setRobotXYPosition(robotState->getX(), robotState->getY());
        setRobotYaw(robotState->getYaw());

        // Set objects' start state.
        for (unsigned int i = 0; i < movableObjects.size(); ++i) {
            std::string movableObjectName = movableObjects[i];

            // i + 1 because subspaces for movable objects are 1..n and 0
            // is the robot subspace.
            const double *q = s->as<ob::RealVectorStateSpace::StateType>(i + 1)->values;
            setBodyXYPosition(movableObjectName, q[0], q[1]);
            setBodyYaw(movableObjectName, q[2]);
            setBodyVelocity(movableObjectName, q[3], q[4], q[5], q[6], q[7], q[8]);
            setBodyAccelerations(movableObjectName, q[9], q[10], q[11], q[12], q[13], q[14]);
        }

        step();

        bool isSystemValid = true;

        // Propagate
        // The for loop will loop the required number of steps to complete
        // a propagation of a given duration. 1.0s / 0.001 gives 1000 steps.
        for (int i = 0; i < duration / getTimeStep(); ++i) {
            setRobotVelocity(controls[0], controls[1], controls[2]);
            step();

            if (i % 200 == 0 && isHighForceAppliedToShelf()) {
                isSystemValid = false;
                break;
            }
        }

        // Robot resulted state after propagation.
        ob::CompoundStateSpace::StateType *res = result->as<ob::CompoundStateSpace::StateType>();
        ob::SE2StateSpace::StateType *resultedRobotState = res->as<ob::SE2StateSpace::StateType>(0);

        // Get the result of the state space after propagation.
        resultedRobotState->setX(getRobotXpos());
        resultedRobotState->setY(getRobotYpos());
        resultedRobotState->setYaw(getRobotYaw());

        // and now for the objects.
        for (unsigned int i = 0; i < movableObjects.size(); ++i) {
            std::string movableObjectName = movableObjects[i];

            // i + 1 because subspaces for movable objects are 1..n and 0
            // is the robot subspace.
            double *resultedObjectState = res->as<ob::RealVectorStateSpace::StateType>(i + 1)->values;
            resultedObjectState[0] = getBodyXpos(movableObjectName);
            resultedObjectState[1] = getBodyYpos(movableObjectName);
            resultedObjectState[2] = getBodyYaw(movableObjectName);

            std::tuple<double, double, double, double, double, double> velocities = getBodyVelocity(movableObjectName);
            resultedObjectState[3] = std::get<0>(velocities);
            resultedObjectState[4] = std::get<1>(velocities);
            resultedObjectState[5] = std::get<2>(velocities);
            resultedObjectState[6] = std::get<3>(velocities);
            resultedObjectState[7] = std::get<4>(velocities);
            resultedObjectState[8] = std::get<5>(velocities);

            // INFO: Note that accelerations now are set to 0.0 regardless of these values (see MujocoHelper).
            std::tuple<double, double, double, double, double, double> acceleration = getBodyAccelerations(
                    movableObjectName);
            resultedObjectState[9] = std::get<0>(acceleration);
            resultedObjectState[10] = std::get<1>(acceleration);
            resultedObjectState[11] = std::get<2>(acceleration);
            resultedObjectState[12] = std::get<3>(acceleration);
            resultedObjectState[13] = std::get<4>(acceleration);
            resultedObjectState[14] = std::get<5>(acceleration);

            double roll = getBodyRoll(movableObjectName);
            double pitch = getBodyPitch(movableObjectName);

            if (roll > 0.1 || roll < -0.1)
                isSystemValid = false;
            if (pitch > 0.1 || pitch < -0.1)
                isSystemValid = false;
        }

        if (!isSystemValid) {
            // Change a state to be clearly out of bounds to "fake" that we don't have a valid system.
            resultedRobotState->setX(1000);
        }
    }


    // ===================================================================
    //                         Collision Checking
    // ===================================================================

    void setContactAttributesForAllGeomsInBody(const string &bodyName, int contype, int conaffinity) {
        const int bodyId = mj_name2id(_model, mjOBJ_BODY, bodyName.c_str());
        const int numberOfGeomsInBody = _model->body_geomnum[bodyId];

        const int bodyGeomStartingAddress = _model->body_geomadr[bodyId];

        for (int offset = 0; offset < numberOfGeomsInBody; ++offset) {
            _model->geom_contype[bodyGeomStartingAddress + offset] = contype;
            _model->geom_conaffinity[bodyGeomStartingAddress + offset] = conaffinity;
        }
    }

    void disableCollisionsFor(const string &bodyName) {
        setContactAttributesForAllGeomsInBody(bodyName, 4, 4);
        forward();
        step();
    }

    void enableCollisionsFor(const string &bodyName) {
        setContactAttributesForAllGeomsInBody(bodyName, 3, 3);
        forward();
        step();
    }

    Eigen::MatrixXf calculateEndEffectorTransformFromRobotState(double x, double y, double yaw) {
        Eigen::MatrixXf transform = Eigen::MatrixXf::Identity(4, 4);

        transform(0, 3) = x;
        transform(1, 3) = y;

        transform(0, 0) = cos(yaw);
        transform(0, 1) = -sin(yaw);
        transform(1, 0) = sin(yaw);
        transform(1, 1) = cos(yaw);

        Eigen::MatrixXf endEffectorInRobot = getEndEffectorInRobotTransform();
        Eigen::MatrixXf endEffectorInWorld = transform * endEffectorInRobot;

        return endEffectorInWorld;
    }

    int getBodyIdFromGeomId(int geomId) {
        return _model->geom_bodyid[geomId];
    }

    bool isBodyInContact(const std::string &bodyName) {
        std::lock_guard<std::mutex> lockGuard(mtx);
        const int bodyId = mj_name2id(_model, mjOBJ_BODY, bodyName.c_str());
        int numberOfContacts = _data->ncon;

        for (int i = 0; i < numberOfContacts; ++i) {
            auto contact = _data->contact[i];

            int bodyInContact1 = _model->body_rootid[_model->geom_bodyid[contact.geom1]];
            int bodyInContact2 = _model->body_rootid[_model->geom_bodyid[contact.geom2]];

            bool tableIsNotInContact = bodyInContact1 != _tableId && bodyInContact2 != _tableId;
            bool bodyIsInContact = bodyInContact1 == bodyId || bodyInContact2 == bodyId;

            if (tableIsNotInContact && bodyIsInContact)
                return true;
        }

        return false;
    }

    bool isBodyInContact(const std::string &bodyName1, const std::string &bodyName2) {
        std::lock_guard<std::mutex> lockGuard(mtx);
        const int bodyId1 = mj_name2id(_model, mjOBJ_BODY, bodyName1.c_str());
        const int bodyId2 = mj_name2id(_model, mjOBJ_BODY, bodyName2.c_str());

        int numberOfContacts = _data->ncon;

        for (int i = 0; i < numberOfContacts; ++i) {
            auto contact = _data->contact[i];

            int bodyInContact1 = _model->body_rootid[_model->geom_bodyid[contact.geom1]];
            int bodyInContact2 = _model->body_rootid[_model->geom_bodyid[contact.geom2]];

            bool isBody1InContact = bodyInContact1 == bodyId1 || bodyInContact2 == bodyId1;
            bool isBody2InContact = bodyInContact1 == bodyId2 || bodyInContact2 == bodyId2;

            if (isBody1InContact && isBody2InContact)
                return true;
        }

        return false;
    }

    bool isRobotInContact(const std::string &bodyName) {
        std::lock_guard<std::mutex> lockGuard(mtx);
        int bodyId1 = mj_name2id(_model, mjOBJ_BODY, _robotName.c_str());
        const int bodyId2 = mj_name2id(_model, mjOBJ_BODY, bodyName.c_str());

        int numberOfContacts = _data->ncon;

        for (int i = 0; i < numberOfContacts; ++i) {
            auto contact = _data->contact[i];

            int bodyInContact1 = _model->body_rootid[_model->geom_bodyid[contact.geom1]];
            int bodyInContact2 = _model->body_rootid[_model->geom_bodyid[contact.geom2]];

            bool isBody1InContact = bodyInContact1 == bodyId1 || bodyInContact2 == bodyId1;
            bool isBody2InContact = bodyInContact1 == bodyId2 || bodyInContact2 == bodyId2;

            if (isBody1InContact && isBody2InContact)
                return true;
        }

        return false;
    }

    bool isRobotInContact() {
        return isBodyInContact(_robotName);
    }

    std::tuple<double, double, double>
    getRobotStateFromEndEffectorTransform(const Eigen::MatrixXf &endEffectorInWorld) {
        // Convert endEffectorTransform to RobotTransform
        Eigen::MatrixXf endEffectorInRobot = getEndEffectorInRobotTransform();
        Eigen::MatrixXf robotInEndEffector = endEffectorInRobot.inverse();
        Eigen::MatrixXf robotInWorld = endEffectorInWorld * robotInEndEffector;
        double x = robotInWorld(0, 3);
        double y = robotInWorld(1, 3);

        // Convert Transformation Matrix into Quaternion.
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

        // Extract Yaw from Quaternion.
        double yaw = getYawFromQuat(quat[0], quat[1], quat[2], quat[3]);

        return std::make_tuple(x, y, yaw);
    }

    bool checkRobotTransformForCollisions(const Eigen::MatrixXf &transformToCheck,
                                          const vector<string> &objectNamesToCheckForCollisions) {

        for (const string &objectName : objectNamesToCheckForCollisions) {
            if (checkRobotTransformForCollisions(transformToCheck, objectName)) {
                return true;
            }
        }

        return false;
    }

    bool checkRobotTransformForCollisions(const Eigen::MatrixXf &transformToCheck,
                                          string objectNameToCheckForCollisions) {
        //std::lock_guard<std::mutex> lockGuard(mtx);
        tuple<double, double, double> robotState = getRobotStateFromEndEffectorTransform(transformToCheck);
        double x = get<0>(robotState);
        double y = get<1>(robotState);
        double yaw = get<2>(robotState);

        int originalResetLevel = _resetLevel;

        setRobotXYPosition(x, y);
        setRobotYaw(yaw);
        step();

        bool isInCollision = isRobotInContact(objectNameToCheckForCollisions);

        setResetLevel(originalResetLevel);

        // Reset the system to the previous state
        resetSimulation();

        return isInCollision;
    }

    int getBodyNumberOfCollisions(const string &bodyName) {
        std::lock_guard<std::mutex> lockGuard(mtx);
        int bodyId = mj_name2id(_model, mjOBJ_BODY, bodyName.c_str());

        int numberOfContacts = _data->ncon;
        int numberOfCollisions = 0;
        for (int i = 0; i < numberOfContacts; ++i) {
            auto contact = _data->contact[i];

            int bodyInContact1 = _model->body_rootid[_model->geom_bodyid[contact.geom1]];
            int bodyInContact2 = _model->body_rootid[_model->geom_bodyid[contact.geom2]];

            bool theDesiredBodyIsInContact = bodyInContact1 == bodyId || bodyInContact2 == bodyId;
            bool theOtherBodyIsNotTheTable = bodyInContact1 != _tableId && bodyInContact2 != _tableId;

            if (theDesiredBodyIsInContact && theOtherBodyIsNotTheTable)
                numberOfCollisions++;
        }

        return numberOfCollisions;
    }

    int getRobotNumberOfCollisions() {
        return getBodyNumberOfCollisions(_robotName);
    }

    int getNumberOfCollisionsForRobotsDesiredState(double x, double y, double yaw) {
        int originalResetLevel = _resetLevel;

        setRobotXYPosition(x, y);
        setRobotYaw(yaw);
        step();

        int numberOfCollisions = getRobotNumberOfCollisions();

        setResetLevel(originalResetLevel);

        // Reset the system to the previous state
        resetSimulation();

        return numberOfCollisions;
    }


    // ===================================================================
    //                         System Methods
    // ===================================================================

    void setGoalRegion(double x, double y) {
        _goalRegionX = x;
        _goalRegionY = y;
    }

    void saveState(ob::State *state) {
        //std::lock_guard<std::mutex> lockGuard(mtx);
        const ob::CompoundStateSpace::StateType *st = state->as<ob::CompoundStateSpace::StateType>();
        const ob::SE2StateSpace::StateType *robotState = st->as<ob::SE2StateSpace::StateType>(0);
        const double *goalObjectState = st->as<ob::RealVectorStateSpace::StateType>(1)->values;

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
        transform(2, 3) = 0.0;

        Eigen::MatrixXf endEffectorInRobot = getEndEffectorInRobotTransform();
        Eigen::MatrixXf endEffectorInWorld = transform * endEffectorInRobot;

        double ee_x = endEffectorInWorld(0, 3);
        double ee_y = endEffectorInWorld(1, 3);
        double goal_x = goalObjectState[0];
        double goal_y = goalObjectState[1];

        double dx = ee_x - goal_x;
        double dy = ee_y - goal_y;

        double distance = sqrt(dx * dx + dy * dy);

        if (distance < _bestStateDistance) {
            _bestStateDistance = distance;
            _bestState = state;
        }
    }

    ob::State *getBestState() {
        return _bestState;
    }

private:
    mjModel *_model;
    mjData *_data;
    std::mutex mtx;
    std::mutex propagateMtx;
    double _bestStateDistance = 100000;
    ob::State *_bestState;
    bool _isSystemValid = true; // This is used in StatePropagator to check if an object has rolled or pitched.
    double _goalRegionX;
    double _goalRegionY;
    int _resetLevel = 0;
    std::vector<mjData *> _mujocoStates;
    int _tableId;

    const string _robotName;
    const std::string _robotLinearXjointName;
    int _robotLinearXjointQvelIndex;

    const std::string _robotLinearYjointName;
    int _robotLinearYjointQvelIndex;

    const std::string _robotAngularZjointName;
    int _robotAngularZjointQvelIndex;

    double _robotInitialXPosition;
    double _robotInitialYPosition;
};

#endif
