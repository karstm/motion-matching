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
    static std::vector<P3D> worldToLocalPositions(std::vector<P3D> traj, float yRotation) {
        std::vector<P3D> localTraj;
        V3D up = V3D(0,1,0);
        Quaternion currentOrient = getRotationQuaternion(yRotation, up);
        Quaternion qInverse = currentOrient.inverse();
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

    static void printGamma(float gamma, int i){
        if(gamma <= -2.5 || gamma >= 2.5){
            crl::Logger::consolePrint("Rot %d: %f\n", i, gamma);
        }
    }
    static std::vector<float> worldToLocalDirectionsAngle(std::vector<float> rot) {
        std::vector<float> localTrajDir;
        for (int i = 1; i < rot.size(); i++) {
            localTrajDir.push_back(minusPiToPi(rot[i] - rot[0]));
            printGamma(rot[i] - rot[0], i);
        }
        return localTrajDir;
    }

    // https://github.com/evbernardes/quaternion_to_euler/blob/main/euler_from_quat.py
    // converts a quarternion to desired Euler angles
    // i != j != k. i, j, k within {1, 2, 3} i.e. {x, y, z}
    // flip order of i, j, k depending on whether the rotations are extrinsic or intrinsic
    static std::vector<float> quarternionToAngles(crl::Quaternion quaternion, int i, int j, int k) {
        std::vector<float> q;
        q.push_back(quaternion.w());
        q.push_back(quaternion.x());
        q.push_back(quaternion.y());
        q.push_back(quaternion.z());
        float a;
        float b;
        float c;
        float d;
        std::vector<float> angles;
        for (int i = 0; i < 3; i++) {
            angles.push_back(0.0);
        }

        float sign = (float)((i - j) * (j - k) * (k - i) / 2);

        a = q[0] - q[j];
        b = q[i] + q[k] * sign;
        c = q[j] + q[0];
        d = q[k] * sign - q[i];

        angles[1] = acos(2 * (pow(a, 2) + pow(b, 2)) / (pow(a, 2) + pow(b, 2) + pow(c, 2) + pow(d, 2)) - 1);
        bool safe1 = abs(angles[1]) >= 1e-7;
        bool safe2 = abs(angles[1] - M_PI / 2) >= 1e-7;
        bool safe = safe1 && safe2;
        
        float aPlus = atan2(b, a);
        float aMinus = atan2(-d, c);

        if (safe) {
            angles[0] = aPlus + aMinus;
            angles[2] = aPlus - aMinus;
        } else {
            angles[2] = 0;
            if (!safe1) {
                angles[0] = 2 * aPlus;
            } else if (!safe2) {
                angles[0] = 2 * aMinus;
            }
        }

        angles[2] = sign * angles[2];
        angles[1] = angles[1] - M_PI_2;

        for (int i = 0; i < 3; i++) {
            angles[i] = minusPiToPi(angles[i]);
        }

        return angles;
    }

};

}  // namespace gui
}  // namespace crl