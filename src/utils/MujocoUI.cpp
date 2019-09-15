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

#include "../sampling_based_planning/src/ProblemDefinition.cpp"

// MuJoCo data structures
extern mjModel *model;                  // MuJoCo model
extern mjData *data;                   // MuJoCo data
extern mjvCamera cam;                      // abstract camera
extern mjvOption opt;                      // visualization options
extern mjvScene scn;                       // abstract scene
extern mjrContext con;                     // custom GPU context
extern mjvPerturb pert;
extern GLFWwindow *window;
extern MujocoHelper *globalMujocoHelper;

// mouse interaction
extern bool button_left;
extern bool button_middle;
extern bool button_right;
extern double lastx;
extern double lasty;

extern const int WINDOW_WIDTH;
extern const int WINDOW_HEIGHT;

extern bool isMouseLocked;
extern bool isKeyboardLocked;
extern bool isZoomLocked;

extern bool objectSelected;
extern double clickedX, clickedY;

// keyboard callback
void keyboard(GLFWwindow * /* window */, int key, int /* scancode */, int act, int /* mods */) {
    if (isKeyboardLocked)
        return;

    // backspace: reset simulation
    if (act == GLFW_PRESS && key == GLFW_KEY_BACKSPACE) {
        mj_resetData(model, data);
        mj_forward(model, data);
    }
}

void tideUp() {
    //free visualization storage
    mjv_freeScene(&scn);
    mjr_freeContext(&con);

    // free MuJoCo model and data, deactivate
    mj_deleteData(data);
    mj_deleteModel(model);
    mj_deactivate();

    // terminate GLFW (crashes with Linux NVidia drivers)
#if defined(__APPLE__) || defined(_WIN32)
    glfwTerminate();
#endif
}

void windowCloseCallback(GLFWwindow * /*window*/) {
    // Use this flag if you wish not to terminate now.
    // glfwSetWindowShouldClose(window, GLFW_FALSE);
    tideUp();
}

std::tuple<GLdouble, GLdouble> ScreenToWorld(int x, int y) {
    GLint viewport[4];
    GLdouble modelview[16];
    GLdouble projection[16];
    GLfloat winX, winY, winZ;
    GLdouble posX, posY, posZ;

    glGetDoublev(GL_MODELVIEW_MATRIX, modelview);
    glGetDoublev(GL_PROJECTION_MATRIX, projection);
    glGetIntegerv(GL_VIEWPORT, viewport);

    winX = (float) x;
    winY = (float) viewport[3] - (float) y;
    glReadPixels(x, int(winY), 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &winZ);

    gluUnProject(winX, winY, winZ, modelview, projection, viewport, &posX, &posY, &posZ);

    return std::make_tuple(posX, posY);
}

// mouse button callback
void mouse_button(GLFWwindow *window, int /* button */, int /* act */, int /* mods */) {
    if (isMouseLocked)
        return;

    // update button state
    button_left = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
    button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
    button_right = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);

    // update mouse position
    glfwGetCursorPos(window, &lastx, &lasty);

    if (button_middle) {
        std::tuple<GLdouble, GLdouble> selectionCoordinate = ScreenToWorld(lastx, lasty);
        clickedX = std::get<0>(selectionCoordinate);
        clickedY = std::get<1>(selectionCoordinate);
        objectSelected = true;
    }
}

// mouse move callback
void mouse_move(GLFWwindow *window, double xpos, double ypos) {
    if (isMouseLocked)
        return;

    // no buttons down: nothing to do
    if (!button_left && !button_middle && !button_right)
        return;

    // compute mouse displacement, save
    double dx = xpos - lastx;
    double dy = ypos - lasty;
    lastx = xpos;
    lasty = ypos;

    // get current window size
    int width, height;
    glfwGetWindowSize(window, &width, &height);

    // get shift key state
    bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
                      glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS);

    // determine action based on mouse button
    mjtMouse action;
    if (button_right)
        action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    else if (button_left)
        action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    else
        action = mjMOUSE_ZOOM;

    // move camera
    mjv_moveCamera(model, action, dx / height, dy / height, &scn, &cam);
}

// scroll callback
void scroll(GLFWwindow * /* window */, double /* xoffset */, double yoffset) {
    if (isMouseLocked)
        return;

    if (!isZoomLocked) {
        // emulate vertical mouse motion = 5% of window height
        mjv_moveCamera(model, mjMOUSE_ZOOM, 0, -0.05 * yoffset, &scn, &cam);
    }
}

void mujocoConfiguration(const std::string &scenePath) {
    // load and compile model
    char error[1000] = "Could not load binary model";
    model = mj_loadXML(scenePath.c_str(), 0, error, 1000);

    // make data
    data = mj_makeData(model);

    // Simulation step size. Change this one if you want non-real-time simulation.
    model->opt.timestep = 0.001;

    // init GLFW
    if (!glfwInit())
        mju_error("Could not initialize GLFW");

    // create window, make OpenGL context current, request v-sync
    window = glfwCreateWindow(WINDOW_WIDTH,
                              WINDOW_HEIGHT,
                              "Manipulation in Clutter",
                              NULL,
                              NULL);

    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    // initialize visualization data structures
    mjv_defaultOption(&opt);
    mjv_defaultScene(&scn);
    mjr_defaultContext(&con);

    // create scene and context
    mjv_makeScene(model, &scn, 2000);
    mjr_makeContext(model, &con, mjFONTSCALE_150);

    // install GLFW mouse and keyboard callbacks
    glfwSetKeyCallback(window, keyboard);
    glfwSetCursorPosCallback(window, mouse_move);
    glfwSetScrollCallback(window, scroll);
    glfwSetMouseButtonCallback(window, mouse_button);
    glfwSetWindowCloseCallback(window, windowCloseCallback);

    // Hide the window for now.
    glfwHideWindow(window);
}

void render() {
    // get framebuffer viewport
    mjrRect viewport = {0, 0, 0, 0};
    glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

    // update scene and render
    mjv_updateScene(model, data, &opt, &pert, &cam, mjCAT_ALL, &scn);
    mjr_render(viewport, &scn, &con);

    // swap OpenGL buffers (blocking call due to v-sync)
    glfwSwapBuffers(window);

    // process pending GUI events, call GLFW callbacks
    glfwPollEvents();
}

string getObjectSelection(vector<string> movableObjectNames) {
    objectSelected = false;
    glfwShowWindow(window);
    std::string objectNameSelected;
    bool validObjectSelected = false;

    do {
        cout << "Select an object by clicking on it." << endl;
        while (!objectSelected) {
            render();
        }
        objectSelected = false;

        for (std::string objectName : movableObjectNames) {
            double objectX = globalMujocoHelper->getBodyXpos(objectName);
            double objectY = globalMujocoHelper->getBodyYpos(objectName);

            double dx = abs(clickedX - objectX);
            double dy = abs(clickedY - objectY);

            if (dx < 0.04 && dy < 0.04) {
                validObjectSelected = true;
                objectNameSelected = objectName;
            }
        }
    } while (!validObjectSelected);

    return objectNameSelected;
}

tuple<double, double> getPushPosition() {
    double x = clickedX, y = clickedY;
    objectSelected = false;
    cout << "Select a position to push the object to." << endl;
    while (!objectSelected || abs(clickedX - x) <= 0.0001 || abs(clickedY - y) <= 0.0001) {
        render();
    }

    objectSelected = false;

    double positionToPushObjectX = clickedX;
    double positionToPushObjectY = clickedY;

    return make_tuple(positionToPushObjectX, positionToPushObjectY);
}

void updateCameraSettingsFromModel() {
    // Camera settings
    double *cameraSettings = globalMujocoHelper->getNumericField("camSettings");

    if (cameraSettings == 0)
        throw std::invalid_argument("You need to set a custom attribute, camSettings, for the camera settings.");

    cam.lookat[0] = cameraSettings[0];
    cam.lookat[1] = cameraSettings[1];
    cam.lookat[2] = cameraSettings[2];
    cam.distance = cameraSettings[3];
    cam.azimuth = cameraSettings[4];
    cam.elevation = cameraSettings[5];
}

void initMujocoFrom(std::string sceneFileName) {
    mujocoConfiguration(sceneFileName);
    mj_step(model, data);
    globalMujocoHelper = new MujocoHelper(model, data, "robot", "robot_lin_x", "robot_lin_y", "robot_ang_z", 0.0, 0.0);
    updateCameraSettingsFromModel();
}
