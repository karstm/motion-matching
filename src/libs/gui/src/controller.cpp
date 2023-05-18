#include "crl-basic/gui/controller.h"

namespace crl {
namespace gui {

void Controller::init(KeyboardState *keyboardState) {
    this->keyboardState = keyboardState;
    for (int i = 0; i < 4; i++) {
        pos.push_back(P3D(0, 0, 0));
        actualPos.push_back(P3D(0, 0, 0));
        rot.push_back(M_PI);
        directions.push_back(V3D(0, 0, 0));
        actualDirections.push_back(V3D(0, 0, 0));
    }
    vel = V3D(0, 0, 0);
    acc = V3D(0, 0, 0);
    angVel = 0.0;

    prevTime = std::chrono::steady_clock::now();
}

void Controller::update(TrackingCamera &camera, Database &database) {
    if(frameCount >= targetFrameRate){
        std::vector<P3D> trajectoryPos = MxMUtils::worldToLocalPositions(pos, rot[0]);
        std::vector<float> trajectoryAngle = MxMUtils::worldToLocalDirectionsAngle(rot);
        std::vector<V3D> trajectoryDir;
        for (int i = 0; i < trajectoryAngle.size(); i++) {
            trajectoryDir.push_back(V3D(sin(trajectoryAngle[i]), 0, cos(trajectoryAngle[i])));
        }
        database.match(trajectoryPos, trajectoryDir, clipIdx, frameIdx);
        
        frameCount = -1;
    }
    frameIdx++;
    frameCount++;


    currTime = std::chrono::steady_clock::now();
    dt = (std::chrono::duration_cast<std::chrono::milliseconds> (currTime - prevTime)).count();
    
    camera.processRotation(dt);
    setInputDirection(camera);

    P3D posPrev = pos[0];
    V3D velPrev = vel;
    V3D accPrev = acc;
    float rotPrev = rot[0];
    float angVelPrev = angVel;
    
    if (posHist.size() >= 60) {
        posHist.pop_front();
        rotHist.pop_front();
    }
    posHist.push_back(posPrev);
    rotHist.push_back(rotPrev);
    prevTime = currTime;
    float T;
    
    for (int t = 0; t < pos.size(); t++) { // for each timestep
        if (t == 0) {
            T = dt / 1000.0; // take into account the speed at which the loop runs to calculate the actual position in the next frame
        } else {
            T = dt / 1000.0 + 1.0 / 3.0 * t; // predict future positions at intervals of 0.33 s
        }
        
        // translation
        float expLambdaT = exp(-lambda * T);
        for (int x = 0; x < 3; x += 2) { // for the x and z coordinates
            float j0 = velPrev[x] - velDesired[x];
            float j1 = accPrev[x] + j0 * lambda;
            pos[t][x] = expLambdaT * (((-j1) / (lambda * lambda)) + ((-j0 - j1 * T) / lambda)) + (j1 / (lambda * lambda)) + j0 / lambda + velDesired[x] * T +
                       posPrev[x];
            if (t == 0) {
                vel[x] = expLambdaT * (j0 + j1 * T) + velDesired[x];
                acc[x] = expLambdaT * (accPrev[x] - j1 * lambda * T);
            }
        }

        // rotation
        float expLambdaRotT = exp(-lambdaRot * T);
        float j0Rot = MxMUtils::minusPiToPi(rotPrev - rotDesired);
        float j1Rot = angVelPrev + j0Rot * lambdaRot;

        rot[t] = expLambdaRotT * (j0Rot + j1Rot * T) + rotDesired;
        if (t == 0) {
            angVel = expLambdaRotT * (angVelPrev - j1Rot * lambdaRot * T);
        }
    }

    camera.target =  MxMUtils::V3Dtovec3(V3D(pos[0]));
    //crl::Logger::consolePrint("Character position: %f, %f, %f\n", pos[0][0], pos[0][1], pos[0][2]);

    float dbEntry[27];
    database.getEntry(clipIdx, frameIdx, dbEntry);

    //trajectory position
    actualPos.clear();
    actualDirections.clear();

    P3D p0 = pos[0];
    V3D p1 = V3D(dbEntry[0], 0, dbEntry[1]);
    V3D p2 = V3D(dbEntry[2], 0, dbEntry[3]);
    V3D p3 = V3D(dbEntry[4], 0, dbEntry[5]);
    V3D d1 = V3D(dbEntry[6], 0, dbEntry[7]);
    V3D d2 = V3D(dbEntry[8], 0, dbEntry[9]);
    V3D d3 = V3D(dbEntry[10], 0, dbEntry[11]);

    Quaternion q0 = getRotationQuaternion(rot[0], V3D(0, 1, 0));
    actualPos.push_back(p0);
    actualPos.push_back(p0 + q0 * p1);
    actualPos.push_back(p0 + q0 * p2);
    actualPos.push_back(p0 + q0 * p3);

    actualDirections.push_back(q0 * d1);
    actualDirections.push_back(q0 * d2);
    actualDirections.push_back(q0 * d3);

    std::vector<float> trajectoryAngle = MxMUtils::worldToLocalDirectionsAngle(rot);
    for (int i = 0; i < trajectoryAngle.size(); i++) {
        directions[i] = q0 * V3D(sin(trajectoryAngle[i]), 0, cos(trajectoryAngle[i]));
    }

}

// returns a vector of future positions arranged in chronological order
std::vector<P3D> Controller::getPos() {
    return pos;
}

std::vector<P3D> Controller::getActualPos(){
    return actualPos;
}

// returns a vector of historical positions arranged in chronological order
std::vector<P3D> Controller::getPosHist() {
    std::vector<P3D> posHistInterval;
    int n = 20;  // 0.33 / (dt / 1000.0); hard-coded this number because using the actual dt tends to be very jittery
    if (n != 0 && posHist.size() >= n) {
        for (int i = std::min(int(posHist.size() / n), 3); i > 0; i--) {
            posHistInterval.push_back(posHist[posHist.size() - n * i]);
        }
    }
    return posHistInterval;
}

// returns a vector of future rotations arranged in chronological order
std::vector<float> Controller::getRot() {
    return rot;
}

// returns a vector of future positions arranged in chronological order
std::vector<V3D> Controller::getDirections() {
    return directions;
}

// returns a vector of future positions arranged in chronological order
std::vector<V3D> Controller::getActualDirections() {
    return actualDirections;
}

// returns a vector of historical rotations arranged in chronological order
std::vector<float> Controller::getRotHist() {
    std::vector<float> rotHistInterval;
    int n = 20;
    if (n != 0 && rotHist.size() >= n) {
        for (int i = std::min(int(rotHist.size() / n), 3); i > 0; i--) {
            rotHistInterval.push_back(rotHist[rotHist.size() - n * i]);
        }
    }
    return rotHistInterval;
}

// updates the desired velocity and rotation
void Controller::setInputDirection(TrackingCamera &camera){
    bool found_controller = false;
    float verticalDir = 0, horizontalDir = 0;

    for(int i = 0; i < GLFW_JOYSTICK_LAST; i++) {
       if (glfwJoystickIsGamepad(i))
       {
            found_controller = true;
            
            int buttonCount;
            const unsigned char *buttons = glfwGetJoystickButtons(i, &buttonCount);
            strave = buttons[GLFW_GAMEPAD_BUTTON_RIGHT_BUMPER];
            run = buttons[GLFW_GAMEPAD_BUTTON_SQUARE]; //for some reason this is the circle button
            Logger::consolePrint("strave: %d\n", strave);

            int axisCount;
            const float *axis = glfwGetJoystickAxes(i, &axisCount);
            V3D temp(axis[0], 0, axis[1]);
            if(temp.norm() > 0.15) {   
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
        strave = keyboardState->at(GLFW_KEY_RIGHT_SHIFT);
        run = keyboardState->at(GLFW_KEY_LEFT_SHIFT);
    }

    if (verticalDir != 0 || horizontalDir != 0) {
        V3D cameraDir = MxMUtils::vec3toV3D(camera.target - camera.position());
        cameraDir.y() = 0;
        cameraDir = cameraDir.unit();
        velDesired = cameraDir * verticalDir + cameraDir.cross(V3D(0, 1, 0)) * horizontalDir;
        if(!found_controller){
            velDesired = velDesired.unit();
        }

        velDesired *= run? runSpeed : walkSpeed;
        rotDesired = strave? rot[0] : MxMUtils::angleBetweenVectors(V3D(0, 0, 1), velDesired);
    } else {
        velDesired = V3D(0, 0, 0);
        rotDesired = rot[0];
    }
    //crl::Logger::consolePrint("Character velocity: %f\n", sqrt(pow(vel[0],2)+pow(vel[1],2)+pow(vel[2],2)));
    //crl::Logger::consolePrint("Angle desired: %f\n", rotDesired);
}

}  // namespace gui
}  // namespace crl