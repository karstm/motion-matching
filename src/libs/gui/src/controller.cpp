#include "crl-basic/gui/controller.h"

namespace crl {
namespace gui {

Controller::Controller(){};
Controller::Controller(KeyboardState *keyboardState) {
    this->keyboardState = keyboardState;
    init();
}

void Controller::init(){
    position = V3D(0, 0, 0);
    direction = V3D(0, 0, 0);
}

void Controller::update(){
    setInputDirection();
    position += direction * speed;
    crl::Logger::consolePrint("Character position: %f, %f, %f\n", position[0], position[1], position[2]);

}

void Controller::setInputDirection(){
    bool found_controller = false;
    for(int i = 0; i < GLFW_JOYSTICK_LAST; i++) {
        if (glfwJoystickIsGamepad(i))
        {
            int count;
            const float *axis = glfwGetJoystickAxes(i, &count);
            V3D temp(axis[0], 0, axis[1]);
            if(temp.norm() > 0.1) {
                direction = V3D(axis[0], 0, axis[1]);        
                found_controller = true;
            }
        }
    }

    if(!found_controller){
        float verticalDir, horizontalDir;
        verticalDir = keyboardState->at(GLFW_KEY_W); 
        verticalDir -= keyboardState->at(GLFW_KEY_S);
        horizontalDir = keyboardState->at(GLFW_KEY_D);
        horizontalDir -= keyboardState->at(GLFW_KEY_A);

        if(verticalDir != 0 || horizontalDir != 0)
            direction = V3D(verticalDir, 0, horizontalDir).normalized();
        else
            direction = V3D(0, 0, 0);
    }
    crl::Logger::consolePrint("Character direction: %f, %f, %f\n", direction[0], direction[1], direction[2]);
}


}  // namespace gui
}  // namespace crl