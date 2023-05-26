#pragma once
#pragma warning(disable : 4312)

#include <glad/glad.h>

// do not put glfw before glad!
#include <GLFW/glfw3.h>
#include <crl-basic/gui/camera.h>
#include <crl-basic/gui/inputstate.h>
#include <crl-basic/gui/renderer.h>
#include <crl-basic/gui/shader.h>
#include <crl-basic/gui/shadow_casting_light.h>
#include <crl-basic/gui/shadow_map_fbo.h>
#include <crl-basic/gui/mxm_utils.h>
#include <crl-basic/gui/inertializationUtils.h>
#include <crl-basic/gui/footlockingUtils.h>
#include <backends/imgui_impl_glfw.h>
#include <backends/imgui_impl_opengl3.h>
#include <imgui_widgets/imGuIZMOquat.h>
#include <imgui_widgets/imgui_add.h>
#include <imgui_widgets/implot.h>
#include <mocap/Database.h>
#include <mocap/MocapClip.h>

#include <thread>
#include <deque>
#include <math.h>
#include <chrono>

namespace crl {
namespace gui {

// This class controls the behaviour of the player character,
// given an user-input via keyboard&mouse or gamepad (ps4).

class Controller {

// Methods
public:
    Controller() {};
    void init(KeyboardState *keyboardState, std::vector<std::unique_ptr<crl::mocap::BVHClip>> *clips, int targetFramerate);
    
    void update(TrackingCamera &camera, Database &database);
    void drawSkeleton(const Shader &shader);
    void drawTrajectory(const Shader &shader, Database &database, bool drawControllerTrajectory, bool drawAnimationTrajectory);

private:
    void updateControllerTrajectory();
    void getInput(TrackingCamera &camera);
    void updateFootLocking();

// Members
public:
    float walkSpeed = 1.1f;
    float runSpeed = 3.3f;
    float syncFactor = 0.2f;
    int motionMatchingRate = 6;
    float transitionTime = 0.4f;
    bool useInertialization = true;
    bool useFootLocking = true;
 
private:
    KeyboardState *keyboardState;
    std::vector<std::unique_ptr<crl::mocap::BVHClip>> *clips = nullptr;
    std::deque<mocap::MocapSkeletonState> motionStates;

    std::vector<P3D> controllerPos; // future positions arranged in chronological order (i.e. "future-r" positions at the back)
    std::vector<float> controllerRot; // future rotations about y-axis arranged in chronological order (0 degrees defined as z-axis)
    P3D simulationPos;
    Quaternion simulationRot;
    P3D lastMatchAnimationPos;
    Quaternion lastMatchAnimationRot;
    P3D lastMatchSimulationPos;
    Quaternion lastMatchSimulationRot;
    V3D vel;
    V3D acc;
    V3D velDesired;
    std::deque<float> oldVerticalDir;
    std::deque<float> oldHorizontalDir;
    std::deque<float> oldSpeed;

    bool strafe = false;
    bool run = false;

    float angVel;
    float rotDesired;

    int targetFramerate;
    int clipIdx = 0, frameIdx = 1;
    int motionMatchingFrameCount = 0;
    bool forceMatch = false;

    float lambda = 4.0f;
    float lambdaRot = 6.0f;
    float dt;
    std::chrono::steady_clock::time_point prevTime;
    std::chrono::steady_clock::time_point currTime;

    // Inertialization
    float t;
    int numMarkers;
    InertializationInfo rootPosInertializationInfo;
    InertializationInfo rootOrientInertializationInfo;
    std::vector<InertializationInfo> jointPositionInertializationInfos;
    std::vector<InertializationInfo> jointOrientInertializationInfos;

    // Foot locking
    std::vector<std::string> footMarkerNames = {"LeftToe", "RightToe"};
    std::vector<std::deque<bool>> contactHistories;
    std::deque<mocap::MocapSkeletonState> footLockedStates;

};

}  // namespace gui
}  // namespace crl