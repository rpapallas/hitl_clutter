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

#include "../../../utils/MujocoGlobal.cpp"
#include "../../../utils/CommonFunctions.cpp"
#include "../../src/ProblemDefinition.cpp"
#include "../../src/ProjectionEvaluator.cpp"
#include "../../src/StateSpaceBase.cpp"
#include "../../src/ControlSpace.cpp"
#include "../../src/StateValidityCheckerBase.cpp"
#include "../../src/StatePropagatorBase.cpp"
#include "../../src/ProblemDefinition.cpp"
#include "../../src/SamplingBasedPlannerBase.cpp"
#include "../../src/planners/RTC/RTC_Planner.cpp"
#include "../Execution.cpp"

double solve(ProblemDefinition problemDefinition, std::string plannerName) {
    RTC_Planner rtcPlanner(globalMujocoHelper, &problemDefinition);

    auto solution = rtcPlanner.solve(plannerName);
    double planningTime = std::get<1>(solution.get());

    if (solution) {
        oc::PathControl solutionPath = std::get<0>(solution.get());
        planningTime = std::get<1>(solution.get());
        double propagationStepSize = rtcPlanner.getPropagationStepSize();

        globalMujocoHelper->resetSimulation();
        glfwShowWindow(window);

        finishedExecution = false;
        thread t1(executeSolutionInRealTime, solutionPath, propagationStepSize);

        while (!finishedExecution) {
            render();
        }

        t1.join();
    } else {
        cout << "No solution found." << endl;
    }

    printf("Planning Time: %f\n", planningTime);

    return planningTime;
}

int main(int argc, char **argv) {
    mj_activate(MJ_KEY_PATH.c_str());

    const std::string sceneFileName = SCENE_PATH + argv[1];
    const int numberOfMovableObjects = atoi(argv[2]);
    const std::string plannerName = argv[3];

    initMujocoFrom(sceneFileName);
    ProblemDefinition problemDefinition = createProblemDefinitionFromArgs(sceneFileName, numberOfMovableObjects);

    solve(problemDefinition, plannerName);

    while (!glfwWindowShouldClose(window)) {
        render();
    }

    tideUp();
}
