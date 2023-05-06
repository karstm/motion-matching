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
#include <backends/imgui_impl_glfw.h>
#include <backends/imgui_impl_opengl3.h>
#include <imgui_widgets/imGuIZMOquat.h>
#include <imgui_widgets/imgui_add.h>
#include <imgui_widgets/implot.h>

#include <thread>

namespace crl {
namespace gui {

class Controller {

public:
    // Constructor
    Controller();
    Controller(KeyboardState *keyboardState);
    
    // Methods
    void update(TrackingCamera &camera);
    P3D getPos();
    V3D getDir();

private:
    // Members
    float speed = 0.1f;
    P3D position;
    V3D direction;
    KeyboardState *keyboardState;

    // Methods
    void init();
    void setInputDirection(TrackingCamera &camera);
    V3D vec3toV3D(vec3 v);
    vec3 V3Dtovec3(V3D v);
};

}  // namespace gui
}  // namespace crl