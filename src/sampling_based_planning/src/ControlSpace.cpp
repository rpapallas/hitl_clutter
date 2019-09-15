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

#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/SimpleDirectedControlSampler.h>

using namespace std;
namespace oc = ompl::control;
namespace ob = ompl::base;

#ifndef CONTROLSPACE
#define CONTROLSPACE

class ControlSpace : public oc::RealVectorControlSpace {
public:
    ControlSpace(const ob::StateSpacePtr &stateSpace,
                 const int ndof,
                 double lowBound,
                 double highBound) : oc::RealVectorControlSpace(stateSpace, ndof) {
        _ndof = ndof;

        ompl::base::RealVectorBounds controlBounds(_ndof);
        controlBounds.setLow(lowBound);
        controlBounds.setHigh(highBound);
        setBounds(controlBounds);
    }

private:
    int _ndof;
};

#endif
