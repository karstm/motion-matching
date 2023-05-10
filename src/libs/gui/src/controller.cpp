#include "crl-basic/gui/controller.h"
#include "annoylib.h"
#include "kissrandom.h"

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
        rot.push_back(0.0);
    }
    vel = V3D(0, 0, 0);
    acc = V3D(0, 0, 0);
    angVel = 0.0;

    prevTime = std::chrono::steady_clock::now();
}

void test(){
    int numNodes = 3;
    int numFeatures = 3;
    int kNearest = 2;
    std::vector<int> closest;

#ifdef ANNOYLIB_MULTITHREADED_BUILD
    Annoy::AnnoyIndex<int, double, Annoy::Euclidean, Annoy::Kiss32Random, Annoy::AnnoyIndexMultiThreadedBuildPolicy> a 
        = Annoy::AnnoyIndex<int, double, Annoy::Euclidean, Annoy::Kiss32Random, Annoy::AnnoyIndexMultiThreadedBuildPolicy>(numFeatures);
    crl::Logger::consolePrint("MultiThreaded");
#else
    Annoy::AnnoyIndex<int, double, Annoy::Euclidean, Annoy::Kiss32Random, Annoy::AnnoyIndexSingleThreadedBuildPolicy> a 
        = Annoy::AnnoyIndex<int, double, Annoy::Euclidean, Annoy::Kiss32Random, Annoy::AnnoyIndexSingleThreadedBuildPolicy>(numFeatures);
    crl::Logger::consolePrint("SingleThreaded");
#endif

    double data[3][3] = {
        {1, 0, 0},
        {0, 1, 0},
        {0, 0, 1},
    };
    double v[3] = {1.0, 0.5, 0.4};

    char **error = (char**)malloc(sizeof(char*));
    a.add_item(0,  data[0], error);
    a.add_item(1, data[1], error);
    a.add_item(2, data[2], error);
    a.build(-1);
    a.get_nns_by_vector(v, kNearest, numNodes, &closest, nullptr);
    
    crl::Logger::consolePrint("Result: %d, %d\n", closest[0], closest[1]);
    // std::cout << "Error?: " << closest[2] << std::endl;
    // print(a.get_nns_by_item(0, 100))
    // print(a.get_nns_by_vector([1.0, 0.5, 0.5], 100))
}

void Controller::update(TrackingCamera &camera) {
    test();

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
            T = 0.2 * t; // predict future positions at intervals of 0.2 s
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
}

// returns a vector of future positions arranged in chronological order
std::vector<P3D> Controller::getPos() {
    return pos;
}

// returns a vector of historical positions arranged in chronological order
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

// returns a vector of future rotations arranged in chronological order
std::vector<float> Controller::getRot() {
    return rot;
}

// returns a vector of historical rotations arranged in chronological order
std::vector<float> Controller::getRotHist() {
    std::vector<float> rotHistInterval;
    int n = 10;
    if (n != 0 && rotHist.size() >= n) {
        for (int i = std::min(int(rotHist.size() / n), 4); i > 0; i--) {
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
        rotDesired = MxMUtils::angleBetweenVectors(V3D(0, 0, 1), velDesired);
    } else {
        velDesired = V3D(0, 0, 0);
        rotDesired = rot[0];
    }
    //crl::Logger::consolePrint("Character velocity: %f\n", sqrt(pow(vel[0],2)+pow(vel[1],2)+pow(vel[2],2)));
    //crl::Logger::consolePrint("Angle desired: %f\n", rotDesired);
}

}  // namespace gui
}  // namespace crl