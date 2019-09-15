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

#include <string>
#include <vector>
#include <ompl/base/spaces/RealVectorStateSpace.h>

#ifndef PROBLEMDEFINITION
#define PROBLEMDEFINITION

namespace ob = ompl::base;

class ProblemDefinition {
public:
    ProblemDefinition(std::string sceneFileName,
                      int numberOfMovableObjects,
                      std::string goalObjectName,
                      std::vector<std::string> staticObjectNames,
                      ob::RealVectorBounds robotStateSpaceBounds,
                      ob::RealVectorBounds objectStateSpaceBounds,
                      int objectStateSpaceSize) :
            _objectStateSpaceSize(objectStateSpaceSize),
            _robotBounds(robotStateSpaceBounds),
            _objectBounds(objectStateSpaceBounds),
            _sceneFileName(sceneFileName),
            _numberOfMovableObjects(numberOfMovableObjects),
            _staticObjectNames(staticObjectNames) {

        _goalObjectName = goalObjectName;
        for (int i = 0; i < _numberOfMovableObjects; ++i) {
            std::string objectName = "object_" + std::to_string(i + 1);
            _movableObjectNames.push_back(objectName);
        }
    }

    std::string getSceneFileName() {
        return _sceneFileName;
    }

    int getNumberOfMovableObjects() {
        return _numberOfMovableObjects;
    }

    std::string getGoalObjectName() {
        return _goalObjectName;
    }

    void setGoalObjectName(std::string name) {
        _goalObjectName = name;
    }

    std::vector<std::string> getMovableObjectNames() {
        return _movableObjectNames;
    }

    std::vector<std::string> getStaticObjectNames() {
        return _staticObjectNames;
    }

    ob::RealVectorBounds getRobotStateSpaceBounds() {
        return _robotBounds;
    }

    ob::RealVectorBounds getObjectStateSpaceBounds() {
        return _objectBounds;
    }

    int getSizeOfObjectStateSpace() {
        return _objectStateSpaceSize;
    }

private:
    const int _objectStateSpaceSize;
    const ob::RealVectorBounds _robotBounds;
    const ob::RealVectorBounds _objectBounds;
    const std::string _sceneFileName;
    const int _numberOfMovableObjects;
    std::string _goalObjectName;
    const std::vector<std::string> _staticObjectNames;
    std::vector<std::string> _movableObjectNames;
};

#endif
