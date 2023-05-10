#pragma once
#pragma warning(disable : 4312)

#include <crl-basic/gui/camera.h>
#include <imgui_widgets/imGuIZMOquat.h>
#define _USE_MATH_DEFINES
#include <math.h>

namespace crl {
namespace gui {

class MxMUtils {
// MotionMatching Utils is used to store and access useful function, which can be used by any class.
// Might want to write a .cpp file, if we want to add other and bigger functions

public:
    static V3D vec3toV3D(vec3 v) {
        V3D vec;
        vec.x() = v.x;
        vec.y() = v.y;
        vec.z() = v.z;
        return vec;
    }

    // function to convert V3D to vec3 for convenience
    static vec3 V3Dtovec3(V3D v) {
        vec3 vec;
        vec.x = v.x();
        vec.y = v.y();
        vec.z = v.z();
        return vec;
    }

    // returns the angle between 2 vectors on the x-z plane from -pi to pi
    static float angleBetweenVectors(V3D v1, V3D v2) {
        V3D v3 = v1.cross(v2);
        float dotProduct = v3.dot(V3D(0, 1, 0));
        int sign = (dotProduct > 0) - (dotProduct < 0);
        float c = v3.norm() * float(sign);
        return atan2f(c, v1.dot(v2));
    }

    // returns an angle between minus pi and pi for any angle
    static float minusPiToPi(float a) {
        int k = floor((M_PI - a) / (2 * M_PI));
        return a + 2.0 * (float)k * M_PI;
    }

    // returns a unit vector based on an angle off the z-axis
    static V3D angleToVector(float a) {
        return V3D(sin(a), 0, cos(a));
    }
};

}  // namespace gui
}  // namespace crl