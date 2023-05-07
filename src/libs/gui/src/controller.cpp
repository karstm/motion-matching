#include "crl-basic/gui/controller.h"

namespace crl {
namespace gui {

Controller::Controller(){};
Controller::Controller(KeyboardState *keyboardState) {
    this->keyboardState = keyboardState;
    init();
}

void Controller::init() {
    for (int i = 0; i < 10; i++) {
        pos.push_back(P3D(0, 0, 0));
    }
    for (int i = 0; i < 2; i++) {
        vel.push_back(V3D(0, 0, 0));
    }
    acc = V3D(0, 0, 0);
    prevTime = std::chrono::steady_clock::now();
}

void Controller::update(TrackingCamera &camera) {
    setInputDirection(camera);
    P3D posPrev = pos[0];
    V3D velPrev = vel[vel.size() - 1];
    V3D accPrev = acc;
    
    currTime = std::chrono::steady_clock::now();
    float dt = (std::chrono::duration_cast<std::chrono::milliseconds> (currTime - prevTime)).count();
    prevTime = currTime;
    float T;
    
    for (int t = 0; t < pos.size(); t++) { // for each timestep
        if (t == 0) {
            T = dt / 1000.0; // take into account the speed at which the loop runs to calculate the actual position in the next frame
            vel.pop_front();
            vel.push_back(V3D(0, 0, 0));
        } else {
            T = 0.2 * t; // predict future positions at intervals of 0.2 s
        }
        
        float expLambdaT = exp(-lambda * T);
        
        for (int x = 0; x < 3; x += 1) { // for the x and z coordinates
            float j0 = velPrev[x] - velDesired[x];
            float j1 = accPrev[x] + j0 * lambda;
            pos[t][x] = expLambdaT * (((-j1) / (lambda * lambda)) + ((-j0 - j1 * T) / lambda)) + (j1 / (lambda * lambda)) + j0 / lambda + velDesired[x] * T +
                       posPrev[x];
            if (t == 0) {
                vel[vel.size() - 1][x] = expLambdaT * (j0 + j1 * T) + velDesired[x];
                acc[x] = expLambdaT * (accPrev[x] - j1 * lambda * T);
            }
        }
    }

    camera.target = V3Dtovec3(V3D(pos[0]));
    crl::Logger::consolePrint("Character position: %f, %f, %f\n", pos[0][0], pos[0][1], pos[0][2]);
}

std::vector<P3D> Controller::getPos() {
    return pos;
}

void Controller::setInputDirection(TrackingCamera &camera){
    bool found_controller = false;
    //for(int i = 0; i < GLFW_JOYSTICK_LAST; i++) {
    //    if (glfwJoystickIsGamepad(i))
    //    {
    //        int count;
    //        const float *axis = glfwGetJoystickAxes(i, &count);
    //        V3D temp(axis[0], 0, axis[1]);
    //        if(temp.norm() > 0.1) {   
    //            found_controller = true;
    //        }
    //    }
    //}

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


            velDesired = (cameraDir * verticalDir + cameraDir.cross(V3D(0, 1, 0)) * horizontalDir).unit();
            for (int i = 0; i < vel.size() - 1; i++) {
                velDesired += vel[i] * 0.5; // smooth our desired velocity vector with historical velocities
            }
            velDesired = (velDesired / (vel.size() + 1)).unit() * 1.5; // desired velocity magnitude of 1.5 m s^-1

        } else {
            velDesired = V3D(0, 0, 0);
        }
    }
    crl::Logger::consolePrint("Character velocity: %f\n", sqrt(pow(vel[0][0],2)+pow(vel[0][1],2)+pow(vel[0][2],2)));
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