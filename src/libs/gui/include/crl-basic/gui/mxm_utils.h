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

    static Quaternion getNegYrotation(Quaternion q){
        double alpha, beta, gamma;
        V3D side = V3D(1,0,0);
        V3D up = V3D(0,1,0);
        V3D front = V3D(0,0,1);
        computeEulerAnglesFromQuaternion(q, front, side, up, alpha, beta, gamma);

        Quaternion negYrotation = getRotationQuaternion(-gamma, up);
        return negYrotation;
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

    // returns n-1 positions in local coordinates with respect to the 0th postion of a trajectory vector of length n
    static std::vector<P3D> worldToLocalPositions(std::vector<P3D> traj) {
        std::vector<P3D> localTraj;
        for (int i = 1; i < traj.size(); i++) {
            localTraj.push_back(traj[i] - traj[0]);
        }
        return localTraj;
    }
};

}  // namespace gui
}  // namespace crl