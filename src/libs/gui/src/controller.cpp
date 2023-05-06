#include "crl-basic/gui/controller.h"

namespace crl {
namespace gui {

Controller::Controller(){};
Controller::Controller(KeyboardState *keyboardState) {
    this->keyboardState = keyboardState;
    init();
}

void Controller::init(){
    position = P3D(0, 0, 0);
    direction = V3D(0, 0, 0);
}

void Controller::update(TrackingCamera &camera){
    setInputDirection(camera);
    position = position + direction * speed;
    camera.target = V3Dtovec3(V3D(position));
    crl::Logger::consolePrint("Character position: %f, %f, %f\n", position[0], position[1], position[2]);
}

P3D Controller::getPos() {
    return position;
}

// returns direction vector with its length normalised
V3D Controller::getDir() {
    vec3 dirVec = glm::normalize(glm::vec3(direction.x(), direction.y(), direction.z()));
    return V3D(dirVec.x, dirVec.y, dirVec.z);
}

void Controller::setInputDirection(TrackingCamera &camera){
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

        if (verticalDir != 0 || horizontalDir != 0) {
            V3D cameraDir = vec3toV3D(camera.target - camera.position());
            cameraDir.y() = 0;
            cameraDir = cameraDir.unit();


            direction = (cameraDir * verticalDir + cameraDir.cross(V3D(0, 1, 0)) * horizontalDir).unit();
        } else {
            direction = V3D(0, 0, 0);
        }
    }
    crl::Logger::consolePrint("Character direction: %f, %f, %f\n", direction[0], direction[1], direction[2]);
}

// function to convert vec3 to V3D for convenience
V3D Controller::vec3toV3D(vec3 v) {
    V3D vec;
    vec.x() = v.x;
    vec.y() = v.y;
    vec.z() = v.z;
    return vec;
}

// function to convert V3D to vec3 for convenience
vec3 Controller::V3Dtovec3(V3D v) {
    vec3 vec;
    vec.x = v.x();
    vec.y = v.y();
    vec.z = v.z();
    return vec;
}

}  // namespace gui
}  // namespace crl