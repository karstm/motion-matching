#include "crl-basic/gui/controller.h"
#include "crl-basic/gui/mxm_utils.h"

namespace crl {
namespace gui {

Controller::Controller(){};
Controller::Controller(KeyboardState *keyboardState) {
    this->keyboardState = keyboardState;
    init();
}

void Controller::init() {
    for (int i = 0; i < 6; i++) {
        pos.push_back(P3D(0, 0, 0));
    }
    vel = V3D(0, 0, 0);
    acc = V3D(0, 0, 0);

    prevTime = std::chrono::steady_clock::now();
}

void Controller::update(TrackingCamera &camera) {
    setInputDirection(camera);
    P3D posPrev = pos[0];
    V3D velPrev = vel;
    V3D accPrev = acc;
    
    if (posHist.size() >= 60) {
        posHist.pop_front();
    }
    posHist.push_back(posPrev);

    currTime = std::chrono::steady_clock::now();
    dt = (std::chrono::duration_cast<std::chrono::milliseconds> (currTime - prevTime)).count();
    prevTime = currTime;
    float T;
    
    for (int t = 0; t < pos.size(); t++) { // for each timestep
        if (t == 0) {
            T = dt / 1000.0; // take into account the speed at which the loop runs to calculate the actual position in the next frame
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
                vel[x] = expLambdaT * (j0 + j1 * T) + velDesired[x];
                acc[x] = expLambdaT * (accPrev[x] - j1 * lambda * T);
            }
        }
    }

    camera.target =  MxMUtils::V3Dtovec3(V3D(pos[0]));
    crl::Logger::consolePrint("Character position: %f, %f, %f\n", pos[0][0], pos[0][1], pos[0][2]);
}

std::vector<P3D> Controller::getPos() {
    return pos;
}

std::vector<P3D> Controller::getPosHist() {
    std::vector<P3D> posHistInterval;
    int n = 10;  // 0.2 / (dt / 1000.0); hard-coded this number because using the actual dt tends to be very jittery
    if (n != 0 && posHist.size() >= n) {
        for (int i = std::min(int(posHist.size() / n), 4); i > 0; i--) {
            posHistInterval.push_back(posHist[posHist.size() - n * i]);
        }
    }
    return posHistInterval;
}

void Controller::setInputDirection(TrackingCamera &camera){
    bool found_controller = false;
    float verticalDir = 0, horizontalDir = 0;

    for(int i = 0; i < GLFW_JOYSTICK_LAST; i++) {
       if (glfwJoystickIsGamepad(i))
       {
           int count;
           const float *axis = glfwGetJoystickAxes(i, &count);
           V3D temp(axis[0], 0, axis[1]);
           if(temp.norm() > 0.1) {   
               found_controller = true;
                horizontalDir = axis[0]; 
                verticalDir = -axis[1]; 
           }
       }
    }

    if(!found_controller){
        verticalDir = keyboardState->at(GLFW_KEY_W); 
        verticalDir -= keyboardState->at(GLFW_KEY_S);
        horizontalDir = keyboardState->at(GLFW_KEY_D);
        horizontalDir -= keyboardState->at(GLFW_KEY_A);
    }

    if (verticalDir != 0 || horizontalDir != 0) {
        V3D cameraDir = MxMUtils::vec3toV3D(camera.target - camera.position());
        cameraDir.y() = 0;
        cameraDir = cameraDir.unit();
        velDesired = (cameraDir * verticalDir + cameraDir.cross(V3D(0, 1, 0)) * horizontalDir).unit() * 1.5; // desired velocity magnitude of 1.5 m s^-1
    } else {
        velDesired = V3D(0, 0, 0);
    }
    //crl::Logger::consolePrint("Character velocity: %f\n", sqrt(pow(vel[0],2)+pow(vel[1],2)+pow(vel[2],2)));
}

}  // namespace gui
}  // namespace crl