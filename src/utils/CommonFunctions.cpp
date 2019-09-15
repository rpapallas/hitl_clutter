#ifndef COMMONFUNCS
#define COMMONFUNCS

double formatDurationToSeconds(std::chrono::high_resolution_clock::time_point start,
                               std::chrono::high_resolution_clock::time_point end) {
    int milliseconds = chrono::duration_cast<chrono::milliseconds>(end - start).count();
    return milliseconds / 1000.0;
}

ProblemDefinition createProblemDefinitionFromArgs(const string sceneFileName, const int numberOfMovableObjects) {
    const vector<string> staticObstacles = {"shelf"};

    vector<double> robotSpaceBounds;
    vector<double> objectSpaceBounds;

    ob::RealVectorBounds robotStateSpaceBounds(2);
    const int vectorSize = 15; // 3 position values (x, y, theta), 6 velocity and 6 acceleration.
    ob::RealVectorBounds objectStateSpaceBounds(vectorSize);

    double *objectBoundariesFromXML = globalMujocoHelper->getNumericField("objectBounds");
    double *robotBoundariesFromXML = globalMujocoHelper->getNumericField("robotBounds");

    printf("Object Boundaries: hx: %f, lx: %f, hy: %f, ly: %f\n", objectBoundariesFromXML[0],
           objectBoundariesFromXML[1], objectBoundariesFromXML[2], objectBoundariesFromXML[3]);
    printf("Robot Boundaries: hx: %f, lx: %f, hy: %f, ly: %f\n", robotBoundariesFromXML[0], robotBoundariesFromXML[1],
           robotBoundariesFromXML[2], robotBoundariesFromXML[3]);

    // X
    robotStateSpaceBounds.setHigh(0, robotBoundariesFromXML[0]);
    robotStateSpaceBounds.setLow(0, robotBoundariesFromXML[1]);

    // Y
    robotStateSpaceBounds.setHigh(1, robotBoundariesFromXML[2]);
    robotStateSpaceBounds.setLow(1, robotBoundariesFromXML[3]);

    // Yaw
    robotStateSpaceBounds.setLow(2, -M_PI / 4);
    robotStateSpaceBounds.setHigh(2, M_PI / 4);

    // X
    objectStateSpaceBounds.setHigh(0, objectBoundariesFromXML[0]);
    objectStateSpaceBounds.setLow(0, objectBoundariesFromXML[1]);

    // Y
    objectStateSpaceBounds.setHigh(1, objectBoundariesFromXML[2]);
    objectStateSpaceBounds.setLow(1, objectBoundariesFromXML[3]);

    // Theta
    objectStateSpaceBounds.setHigh(2, 2 * M_PI);
    objectStateSpaceBounds.setLow(2, -2 * M_PI);

    // Velocity & Acceleration Bounds
    double linearXvelocity = 10.4;
    double linearYvelocity = 10.4;
    double linearZvelocity = 10.1;
    double angularXvelocity = 10.1;
    double angularYvelocity = 10.1;
    double angularZvelocity = 10.4;

    double linearXacceleration = 500.0;
    double linearYacceleration = 500.0;
    double linearZacceleration = 500.0;
    double angularXacceleration = 500.0;
    double angularYacceleration = 500.0;
    double angularZacceleration = 500.0;

    // Velocity bounds
    objectStateSpaceBounds.setHigh(3, linearXvelocity);
    objectStateSpaceBounds.setLow(3, -linearXvelocity);

    objectStateSpaceBounds.setHigh(4, linearYvelocity);
    objectStateSpaceBounds.setLow(4, -linearYvelocity);

    objectStateSpaceBounds.setHigh(5, linearZvelocity);
    objectStateSpaceBounds.setLow(5, -linearZvelocity);

    objectStateSpaceBounds.setHigh(6, angularXvelocity);
    objectStateSpaceBounds.setLow(6, -angularXvelocity);

    objectStateSpaceBounds.setHigh(7, angularYvelocity);
    objectStateSpaceBounds.setLow(7, -angularYvelocity);

    objectStateSpaceBounds.setHigh(8, angularZvelocity);
    objectStateSpaceBounds.setLow(8, -angularZvelocity);

    // Acceleration bounds
    objectStateSpaceBounds.setHigh(9, linearXacceleration);
    objectStateSpaceBounds.setLow(9, -linearXacceleration);

    objectStateSpaceBounds.setHigh(10, linearYacceleration);
    objectStateSpaceBounds.setLow(10, -linearYacceleration);

    objectStateSpaceBounds.setHigh(11, linearZacceleration);
    objectStateSpaceBounds.setLow(11, -linearZacceleration);

    objectStateSpaceBounds.setHigh(12, angularXacceleration);
    objectStateSpaceBounds.setLow(12, -angularXacceleration);

    objectStateSpaceBounds.setHigh(13, angularYacceleration);
    objectStateSpaceBounds.setLow(13, -angularYacceleration);

    objectStateSpaceBounds.setHigh(14, angularZacceleration);
    objectStateSpaceBounds.setLow(14, -angularZacceleration);

    ProblemDefinition problemDefinition = ProblemDefinition(sceneFileName,
                                                            numberOfMovableObjects,
                                                            "object_3",
                                                            staticObstacles,
                                                            robotStateSpaceBounds,
                                                            objectStateSpaceBounds,
                                                            vectorSize);

    return problemDefinition;
}

#endif
