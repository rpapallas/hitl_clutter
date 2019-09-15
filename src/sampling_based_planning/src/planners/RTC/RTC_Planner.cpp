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

#ifndef RTC_PLANNER
#define RTC_PLANNER

#include "../../SamplingBasedPlannerBase.cpp"
#include "SimpleSetup.cpp"
#include "Goal.cpp"

class RTC_Planner : public SamplingBasedPlanner {
public:
    RTC_Planner(MujocoHelper *mujocoHelper,
                ProblemDefinition *problemDefinition) :
            SamplingBasedPlanner(mujocoHelper, problemDefinition) {
    }

    boost::optional<std::tuple<oc::PathControl, double, std::string>>
    solve(std::string plannerName) {
        const vector<string> staticObjectNames = _problemDefinition->getStaticObjectNames();
        const vector<string> movableObjectNames = _problemDefinition->getMovableObjectNames();
        const string goalObjectName = _problemDefinition->getGoalObjectName();

        boost::optional<shared_ptr<SimpleSetup1>> setup = runPlanner(plannerName,
                                                                     staticObjectNames,
                                                                     movableObjectNames,
                                                                     goalObjectName);

        boost::optional<std::tuple<oc::PathControl, double, std::string>> result;

        if (setup) {
            oc::PathControl pathControl = setup.get()->getSolutionPath();
            double planningTime = setup.get()->getLastPlanComputationTime();
            std::string solutionType;

            if (setup.get()->haveSolutionPath()) {
                if (setup.get()->haveExactSolutionPath()) {
                    solutionType = "exact";
                } else {
                    solutionType = "approx";
                }
            } else {
                solutionType = "nosolution";
            }

            result = std::make_tuple(pathControl, planningTime, solutionType);
        }

        return result;
    }

private:
    boost::optional<shared_ptr<SimpleSetup1>> runPlanner(std::string plannerName,
                                                         std::vector<std::string> staticObjectNames,
                                                         std::vector<std::string> movableObjectNames,
                                                         std::string goalObjectName) {
        auto stateSpace = std::make_shared<StateSpace1>(CELL_SIZE, _mujocoHelper, _problemDefinition);
        auto controlSpace = std::make_shared<ControlSpace>(stateSpace, 3, -0.05, 0.05);
        auto simpleSetup = std::make_shared<SimpleSetup1>(controlSpace,
                                                          _mujocoHelper,
                                                          PROPAGATION_STEP_SIZE,
                                                          MIN_CONTROL_DURATION,
                                                          MAX_CONTROL_DURATION,
                                                          DISTANCE_FROM_GOAL_THRESHOLD,
                                                          staticObjectNames,
                                                          movableObjectNames,
                                                          goalObjectName,
                                                          plannerName);

        ob::PlannerStatus solved;
        solved = simpleSetup->solve(MAX_PLANNING_TIME_IN_SECONDS);

        if (solved) {
            if (simpleSetup->haveExactSolutionPath())
                std::printf("Exact solution found after %.2f seconds.\n", simpleSetup->getLastPlanComputationTime());
            else
                std::printf("Approximate solution found after %.2f seconds.\n",
                            simpleSetup->getLastPlanComputationTime());

            return simpleSetup;
        } else
            std::printf("No solution found after %.2f seconds.\n", simpleSetup->getLastPlanComputationTime());

        return boost::none;
    }
};

#endif
