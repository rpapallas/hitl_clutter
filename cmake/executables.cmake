add_executable(rtc src/sampling_based_planning/demos/autonomous/RTC.cpp)
target_link_libraries(rtc
        ${catkin_LIBRARIES}
        ${OMPL_LIBRARIES}
        ${LIB_MUJOCO}
        ${GLFW}
        libGL.so
        libglew.so
        ${CMAKE_THREAD_LIBS_INIT}  # To have -pthread
        GL
        GLU
        glut
        -lpthread
        -lboost_thread
        -lboost_system)

add_executable(grtc_heuristic src/sampling_based_planning/demos/autonomous/GRTC_Heuristic.cpp)
target_link_libraries(grtc_heuristic
        ${catkin_LIBRARIES}
        ${OMPL_LIBRARIES}
        ${LIB_MUJOCO}
        ${GLFW}
        libGL.so
        libglew.so
        ${CMAKE_THREAD_LIBS_INIT}  # To have -pthread
        GL
        GLU
        glut
        -lpthread
        -lboost_thread
        -lboost_system)

add_executable(grtc_hitl src/sampling_based_planning/demos/hitl/GRTC_HITL.cpp)
target_link_libraries(grtc_hitl
        ${catkin_LIBRARIES}
        ${OMPL_LIBRARIES}
        ${LIB_MUJOCO}
        ${GLFW}
        libGL.so
        libglew.so
        ${CMAKE_THREAD_LIBS_INIT}  # To have -pthread
        GL
        GLU
        glut
        -lpthread
        -lboost_thread
        -lboost_system)
