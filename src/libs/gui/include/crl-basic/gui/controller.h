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
#include <backends/imgui_impl_glfw.h>
#include <backends/imgui_impl_opengl3.h>
#include <imgui_widgets/imGuIZMOquat.h>
#include <imgui_widgets/imgui_add.h>
#include <imgui_widgets/implot.h>
#include <mocap/Database.h>

#include <thread>
#include <deque>
#include <math.h>
#include <chrono>

namespace crl {
namespace gui {

// This class controls the behaviour of the player character,
// given an user-input via keyboard&mouse or gamepad (ps4).

class Controller {

public:
    // Constructor
    Controller(){};
    void init(KeyboardState *keyboardState);
    
    // Methods
    void update(TrackingCamera &camera, Database &database);
    std::vector<P3D> getPos();
    std::vector<P3D> getPosHist();
    std::vector<float> getRot();
    std::vector<float> getRotHist();
    int getClipIdx() { return clipIdx; }
    int getFrameIdx() { return frameIdx; }

private:
    // Members
    std::vector<P3D> pos; // future positions arranged in chronological order (i.e. "future-r" positions at the back)
    std::deque<P3D> posHist; // historical positions arranged in chronological order (i.e. "past-er" positions at the front)
    V3D vel;
    V3D acc;
    V3D velDesired;
    
    std::vector<float> rot; // future rotations about y-axis arranged in chronological order (0 degrees defined as z-axis)
    std::deque<float> rotHist; // historical rotations arranged in chronological order
    float angVel;
    float rotDesired;
    int clipIdx = 0, frameIdx = 86;
    int frameCount = 0;
    const int targetFrameRate = 30;

    float lambda = 4.0f;
    float lambdaRot = 6.0f;
    float dt;
    KeyboardState *keyboardState;
    std::chrono::steady_clock::time_point prevTime;
    std::chrono::steady_clock::time_point currTime;

    // Methods
    void setInputDirection(TrackingCamera &camera);
};

}  // namespace gui
}  // namespace crl