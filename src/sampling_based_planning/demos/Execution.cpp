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

#ifndef SAMPLINGBASEDSOLUTIONEXECUTION
#define SAMPLINGBASEDSOLUTIONEXECUTION

#include <boost/filesystem.hpp>

void executeSolutionInRealTime(oc::PathControl path, double propagationStepSize) {
    for (size_t i = 0; i < path.getControlCount(); ++i) {
        const double *values = path.getControl(i)->as<oc::RealVectorControlSpace::ControlType>()->values;

        for (int step = 0; step < propagationStepSize / globalMujocoHelper->getTimeStep(); step++) {
            globalMujocoHelper->setRobotVelocity(values[0], values[1], values[2]);
            globalMujocoHelper->step();
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }

    finishedExecution = true;
}

void executeSolutionsInRealTime(const std::vector<oc::PathControl> &paths, double propagationStepSize) {
    for (oc::PathControl path : paths) {
        for (size_t i = 0; i < path.getControlCount(); ++i) {
            const double *values = path.getControl(i)->as<oc::RealVectorControlSpace::ControlType>()->values;

            for (int step = 0; step < propagationStepSize / globalMujocoHelper->getTimeStep(); step++) {
                globalMujocoHelper->setRobotVelocity(values[0], values[1], values[2]);
                globalMujocoHelper->step();
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        }
    }
    finishedExecution = true;
}

void executeSolutions(const vector<oc::PathControl> &paths, double propagationStepSize) {
    for (oc::PathControl path : paths) {
        for (size_t i = 0; i < path.getControlCount(); ++i) {
            const double *values = path.getControl(i)->as<oc::RealVectorControlSpace::ControlType>()->values;

            for (int step = 0; step < propagationStepSize / globalMujocoHelper->getTimeStep(); step++) {
                globalMujocoHelper->setRobotVelocity(values[0], values[1], values[2]);
                globalMujocoHelper->step();
            }
        }
    }
}

#endif
