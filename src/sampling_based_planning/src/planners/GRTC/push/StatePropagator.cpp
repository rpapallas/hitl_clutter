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

#ifndef PUSHSTATEPROPAGATOR
#define PUSHSTATEPROPAGATOR

class StatePropagator3 : public StatePropagatorBase {
public:
    StatePropagator3(const oc::SpaceInformationPtr &si, MujocoHelper *mujocoHelper,
                     const std::vector<std::string> movableObjects) :
            StatePropagatorBase(si, mujocoHelper, movableObjects) {
    }
};

#endif
