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

    static Quaternion getYrotation(Quaternion q, bool inverse = true){
        double alpha, beta, gamma;
        V3D side = V3D(1,0,0);
        V3D up = V3D(0,1,0);
        V3D front = V3D(0,0,1);
        computeEulerAnglesFromQuaternion(q, front, side, up, alpha, beta, gamma);
        gamma = gamma+M_PI;
        Quaternion negYrotation = getRotationQuaternion(inverse? -gamma : gamma, up);
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
    static std::vector<P3D> worldToLocalPositions(std::vector<P3D> traj, Quaternion yRotation) {
        std::vector<P3D> localTraj;
        Quaternion qInverse = yRotation.inverse();
        for (int i = 1; i < traj.size(); i++) {
            V3D localDir = qInverse*V3D(traj[0], traj[i]);
            localTraj.push_back(P3D(localDir[0], localDir[1], localDir[2]));
        }
        return localTraj;
    }

    static std::vector<V3D> worldToLocalDirections(std::vector<float> rot) {
        std::vector<V3D> localTrajDir;
        V3D up = V3D(0,1,0);
        V3D forward = V3D(0,0,1);
        for (int i = 1; i < rot.size(); i++) {
            Quaternion orient = getRotationQuaternion(rot[i] - rot[0], up);
            localTrajDir.push_back(orient*forward);
        }
        return localTrajDir;
    }

    static std::vector<float> worldToLocalDirectionsAngle(std::vector<float> rot) {
        std::vector<float> localTrajDir;
        for (int i = 1; i < rot.size(); i++) {
            localTrajDir.push_back(minusPiToPi(rot[i] - rot[0]));
        }
        return localTrajDir;
    }

    static inline float lerpf(float x, float y, float a)
    {
        return (1.0f - a) * x + a * y;
    }

    static crl::P3D lerp(crl::P3D v, crl::P3D w, float alpha)
    {
        return v * (1.0f - alpha) + w * alpha;
    }

    static crl::Quaternion quaternionLerp(crl::Quaternion q, crl::Quaternion p, float alpha)
    {
        return crl::Quaternion(
                lerpf(q.w(), p.w(), alpha),
                lerpf(q.x(), p.x(), alpha),
                lerpf(q.y(), p.y(), alpha),
                lerpf(q.z(), p.z(), alpha)
            ).normalized();
    }

    static float quatDot(crl::Quaternion q, crl::Quaternion p)
    {
        return q.w()*p.w() + q.x()*p.x() + q.y()*p.y() + q.z()*p.z();
    }

    static crl::Quaternion quatNlerpShortest(crl::Quaternion q, crl::Quaternion p, float alpha)
    {
        if (quatDot(q, p) < 0.0f)
        {
            p = crl::Quaternion(-p.w(), -p.x(), -p.y(), -p.z());
        }
        
        return quaternionLerp(q, p, alpha);
    }
};

}  // namespace gui
}  // namespace crl