set(Mujoco_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include/mjpro200)
message(STATUS "MuJoCo include dir found at: " ${Mujoco_INCLUDE_DIRS})

find_library(GLFW libglfw.so.3 HINTS ${CMAKE_CURRENT_SOURCE_DIR}/lib/mjpro200/)
message(STATUS "GLFW lib found at: " ${GLFW})

if(${USE_GL})
    file(GLOB LIB_MUJOCO ${CMAKE_CURRENT_SOURCE_DIR}/lib/mjpro200/libmujoco[0-9][0-9][0-9].so)
else()
    file(GLOB LIB_MUJOCO ${CMAKE_CURRENT_SOURCE_DIR}/lib/mjpro200/libmujoco[0-9][0-9][0-9]nogl.so)
endif()

#Showing mujoco library found
message(STATUS "MuJoCo lib found at: " ${LIB_MUJOCO})

#Showing path to MuJoCo for checking
message(STATUS "MuJoCo path: " ${CMAKE_CURRENT_SOURCE_DIR}/lib/mjpro200/)
