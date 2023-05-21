#pragma once

#include <algorithm>
#include <crl-basic/gui/camera.h>
#include <imgui_widgets/imGuIZMOquat.h>
#define _USE_MATH_DEFINES
#include <math.h>
#include <mocap/MocapClip.h>

namespace crl {
namespace gui {

struct InertializationInfo {
    V3D X0 = V3D(0, 0, 0);
    double x0 = 0, v0 = 0, a0 = 0;
    double t1 = 0;               // when inertialization finishes
    double A = 0, B = 0, C = 0;  // coefficients
};

class InertializationUtils {
// Tools for inertialization
public:
   /**
     * Compute inertialization infos (coefficients, x0, v0, a0, t1, etc.)
     * 
     * t1 is user's desired transition time (t1 = 0.5 is desirable...)
     * dt is one frame timestep
     */
    static void computeInertializationInfo(InertializationInfo &rootPositionInertializationInfo, 
                                           InertializationInfo &rootOrientationInertializationInfo, 
                                           std::vector<InertializationInfo> &jointOrientationInertializationInfos, 
                                           const int numMarkers,
                                           const mocap::MocapSkeletonState &stateMinus2, 
                                           const mocap::MocapSkeletonState &stateMinus1, 
                                           const mocap::MocapSkeletonState &state, 
                                           double t1, 
                                           double dt) {
        // root
        {
            // position
            V3D rootPositionMinus2 = V3D(stateMinus2.getRootPosition());
            V3D rootPositionMinus1 = V3D(stateMinus1.getRootPosition());
            V3D rootPosition = V3D(state.getRootPosition());

            rootPositionInertializationInfo = computeInertializationInfoVector(rootPositionMinus2, rootPositionMinus1, rootPosition, dt, t1);

            // orientation
            Quaternion rootOrientationMinus2 = stateMinus2.getRootOrientation();
            Quaternion rootOrientationMinus1 = stateMinus1.getRootOrientation();
            Quaternion rootOrinetation = state.getRootOrientation();

            rootOrientationInertializationInfo = computeInertializationInfoQuaternion(rootOrientationMinus2, rootOrientationMinus1, rootOrinetation, dt, t1);
        }

        // joints
        for (uint i = 0; i < numMarkers; i++) {
            Quaternion jointOrientationMinus2 = stateMinus2.getJointRelativeOrientation(i);
            Quaternion jointOrientationMinus1 = stateMinus1.getJointRelativeOrientation(i);
            Quaternion jointOrientation = state.getJointRelativeOrientation(i);

            jointOrientationInertializationInfos[i] = computeInertializationInfoQuaternion(jointOrientationMinus2, jointOrientationMinus1, jointOrientation, dt, t1);
        }
    }

    /**
     * initialize state at time t based on computed inertialization infos.
     * 
     * motion_t is a motion from new sequence at time t
     * t is a time since the last motion matching happened
     * dt is a frame timestep (for computing velocity by finite difference)
     */
    static mocap::MocapSkeletonState inertializeState(const InertializationInfo &rootPositionInertializationInfo, 
                                        const InertializationInfo &rootOrientationInertializationInfo, 
                                        const std::vector<InertializationInfo> &jointOrientationInertializationInfos, 
                                        const int numMarkers,
                                        const mocap::MocapSkeletonState &state, 
                                        const mocap::MocapSkeletonState &stateMinus1, 
                                        double t, double dt) {
        mocap::MocapSkeletonState inertializedState = state;
        // root
        {
            // position
            V3D rootPosition = V3D(state.getRootPosition());

            V3D inertializedRootPosition = inertializeVector(rootPositionInertializationInfo, rootPosition, t);
            inertializedState.setRootPosition(P3D() + inertializedRootPosition);
            inertializedState.setRootVelocity((inertializedRootPosition - V3D(stateMinus1.getRootPosition())) / dt);

            // orientation
            Quaternion rootOrientation = state.getRootOrientation();
            Quaternion rootOrientationMinus1 = stateMinus1.getRootOrientation();

            Quaternion inertializedRootOrientation = inertializeQuaternion(rootOrientationInertializationInfo, rootOrientation, t);
            inertializedState.setRootOrientation(inertializedRootOrientation);
            inertializedState.setRootAngularVelocity(mocap::estimateAngularVelocity(rootOrientationMinus1, inertializedRootOrientation, dt));
        }

        // joints
        for (uint i = 0; i < numMarkers; i++) {
            Quaternion jointOrientation = state.getJointRelativeOrientation(i);
            Quaternion jointOrientationMinus1 = stateMinus1.getJointRelativeOrientation(i);

            Quaternion inertializedJointOrientation = inertializeQuaternion(jointOrientationInertializationInfos[i], jointOrientation, t);
            inertializedState.setJointRelativeOrientation(inertializedJointOrientation, i);
            inertializedState.setJointRelativeAngVelocity(mocap::estimateAngularVelocity(jointOrientationMinus1, inertializedJointOrientation, dt), i);
        }

        return inertializedState;
    }


private:
    static InertializationInfo computeInertializationInfoQuaternion(Quaternion &orientationMinus2, Quaternion &orientationMinus1, Quaternion &orientation, double dt, double t1) {
        Quaternion q0 = orientationMinus1 * orientation.inverse();
        Quaternion qMinus1 = orientationMinus2 * orientation.inverse();

        V3D X0 = q0.vec().normalized();
        double x0 = getRotationAngle(q0, X0);

        double xMinus1 = 2 * atan2(qMinus1.vec().dot(X0), qMinus1.w());

        double v0 = (x0 - xMinus1) / dt;
        if(x0 * v0 > 0)
            v0 = 0;

        double a0 = (-8 * v0 * t1 - 20 * x0) / (t1 * t1); 
        if (a0 * v0 > 0)
            a0 = 0;

        if (v0 != 0)
            t1 = std::min(t1, -5 * x0 / v0);

        double A = -(a0 * t1 * t1 + 6 * v0 * t1 + 12 * x0) / (2 * t1 * t1 * t1 * t1 * t1);
        double B = (3 * a0 * t1 * t1 + 16 * v0 * t1 + 30 * x0) / (2 * t1 * t1 * t1 * t1);
        double C = -(3 * a0 * t1 * t1 + 12 * v0 * t1 + 20 * x0) / (2 * t1 * t1 * t1);

        InertializationInfo info;
        info.X0 = X0;
        info.A = A;
        info.B = B;
        info.C = C;
        info.x0 = x0;
        info.v0 = v0;
        info.a0 = a0;
        info.t1 = t1;
        return info;
    }

    static InertializationInfo computeInertializationInfoVector(V3D &positionMinus2, V3D &positionMinus1, V3D &position, double dt, double t1) {
        V3D X0 = positionMinus1 - position;
        V3D XMinus1 = positionMinus2 - position;

        double x0 = X0.norm();
        double xMinus1 = XMinus1.dot(X0/x0);
        double v0 = (x0 - xMinus1) / dt;
        v0 = std::max(v0, 0.0);
        double a0 = (-8 * v0 * t1 - 20 * x0) / (t1 * t1);
        a0 = std::min(a0, 0.0);

        if(v0 < 0.0)
            t1 = std::min(t1, -5 * x0 / v0);

        double A = -(a0 * t1 * t1 + 6 * v0 * t1 + 12 * x0) / (2 * t1 * t1 * t1 * t1 * t1);
        double B = (3 * a0 * t1 * t1 + 16 * v0 * t1 + 30 * x0) / (2 * t1 * t1 * t1 * t1);
        double C = -(3 * a0 * t1 * t1 + 12 * v0 * t1 + 20 * x0) / (2 * t1 * t1 * t1);

        InertializationInfo info;
        info.X0 = X0;
        info.A = A;
        info.B = B;
        info.C = C;
        info.x0 = x0;
        info.v0 = v0;
        info.a0 = a0;
        info.t1 = t1;
        return info;
    }

    /**
     * inertialize quaternion orientation at time t.
     */
    static Quaternion inertializeQuaternion(const InertializationInfo &info, Quaternion &orientation, double t) {
        if (t >= info.t1)
            // inertialization is done just return orientation
            return orientation;

        // xt = A * t^5 + B * t^4 + C * t^3 + a0/2 * t^2 + v0 * t + x0
        double xt = info.A * t * t * t * t * t    //
                  + info.B * t * t * t * t        //
                  + info.C * t * t * t            //
                  + info.a0 / 2.0 * t * t         //
                  + info.v0 * t                   //
                  + info.x0;                      //

        Quaternion qt = getRotationQuaternion(xt, info.X0) * orientation;

        return qt;
    }

    /**
     * inertialize vector position at time t.
     */
    static V3D inertializeVector(const InertializationInfo &info, V3D &position, double t) {
        if (t >= info.t1)
            // inertialization done just return newV
            return position;

        // xt = A * t^5 + B * t^4 + C * t^3 + a0/2 * t^2 + v0 * t + x0
        double xt = info.A * t * t * t * t * t    //
                  + info.B * t * t * t * t        //
                  + info.C * t * t * t            //
                  + info.a0 / 2.0 * t * t         //
                  + info.v0 * t                   //
                  + info.x0;                      //

        V3D Xt = xt * info.X0 / info.x0 + position;

        return Xt;
    }
};

}  // namespace gui
}  // namespace crl