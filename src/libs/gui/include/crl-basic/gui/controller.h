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
    Controller::Controller(GLFWwindow *window);
    
    // Methods
    
    V3D getInputDirection();

private:
    // Members
    GLFWwindow *window;
    bool wasdVector[4];

    // Methods
    void Controller::init();
    void Controller::setCallbacks();

    //--- Interaction
    virtual bool keyPressed(int key, int mods);
    virtual bool keyReleased(int key, int mods);
};

}  // namespace gui
}  // namespace crl