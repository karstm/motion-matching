#include "crl-basic/gui/controller.h"

namespace crl {
namespace gui {

Controller::Controller(GLFWwindow *window){
    this->window = window;
    this->init();
}

void Controller::init(){
    //this->wasdVector = {false, false, false, false};
    Controller::setCallbacks();
}

void Controller::setCallbacks(){
    glfwSetKeyCallback(window, [](GLFWwindow *window, int key, int scancode, int action, int mods) {
        auto controller = static_cast<Controller *>(glfwGetWindowUserPointer(window));

        if (action == GLFW_PRESS)
            controller->keyPressed(key, mods);

        if (action == GLFW_RELEASE)
            controller->keyReleased(key, mods);
    });
}

bool Controller::keyPressed(int key, int mods) {
    if (key == GLFW_KEY_W) {
        wasdVector[0] = true;
    } else if(key == GLFW_KEY_A){
        wasdVector[1] = true;
    } else if (key == GLFW_KEY_S) {
        wasdVector[2] = true;
    } else if(key == GLFW_KEY_D){
        wasdVector[3] = true;
    }

    return false;
}

bool Controller::keyReleased(int key, int mods) {
    if (key == GLFW_KEY_W) {
        this->wasdVector[0] = false;
    } else if(key == GLFW_KEY_A){
        this->wasdVector[1] = false;
    } else if (key == GLFW_KEY_S) {
        this->wasdVector[2] = false;
    } else if(key == GLFW_KEY_D){
        this->wasdVector[3] = false;
    }
    return false;
}

V3D Controller::getInputDirection(){
    bool found_controller = false;
    V3D inputDir;
    for(int i = 0; i < GLFW_JOYSTICK_LAST; i++){
        if (glfwJoystickIsGamepad(i))
        {
            int count;
            const float *axis = glfwGetJoystickAxes(i, &count);
            crl::Logger::consolePrint("Gamepad axis: %d!\n", i);
            inputDir = V3D(axis[0], 0, axis[1]);        
            found_controller = true;
        }
    }

    if(!found_controller){
        float verticalDir = wasdVector[0]; 
        verticalDir -= wasdVector[2];
        float horizontalDir = wasdVector[1];
        horizontalDir -=  wasdVector[3]; 

        crl::Logger::consolePrint("Keyboard Used!\n");
        inputDir = V3D(verticalDir, 0, horizontalDir);
    }

    crl::Logger::consolePrint("Joystick value: %d, %d, %d\n", inputDir[0], inputDir[1], inputDir[2]);

    return inputDir;
}


}  // namespace gui
}  // namespace crl