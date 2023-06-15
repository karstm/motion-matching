#pragma once

#include <algorithm>
#include <crl-basic/gui/camera.h>
#include <imgui_widgets/imGuIZMOquat.h>
#define _USE_MATH_DEFINES
#include <math.h>
#include <mocap/MocapClip.h>

namespace crl {
namespace gui {

// Inertialization info for a single vector or quaternion
struct InertializationInfo {
    V3D X0 = V3D(0, 0, 0);
    double x0 = 0, v0 = 0, a0 = 0;
    double t1 = 0;               // when inertialization finishes
    double A = 0, B = 0, C = 0;  // coefficients
};

/*
 * Tools for inertialization
 * Adapted from the Gears of War 4 presentation at GDC 2018 by David Bollo
 * See the presentation slides for more details: 
 * https://cdn.gearsofwar.com/thecoalition/publications/GDC%202018%20-%20Inertialization%20-%20High%20Performance%20Animation%20Transitions%20in%20Gears%20of%20War.pdf
 */
class InertializationUtils {
public:
    /*
     * Compute inertialization infos
     * 
     * state:        new discontinuous state
     * stateMinus1:  previous state
     * stateMinus2:  state before previous state
     * t1:           desired maximal transition time
     */
    static void computeInertializationInfo(InertializationInfo &rootPositionInertializationInfo, 
                                           InertializationInfo &rootOrientationInertializationInfo, 
                                           std::vector<InertializationInfo>  &jointPositionInertializationInfos, 
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
            // position (important because we have non constant relative positions due to the simulation bone)
            V3D jointPositionMinus2 = V3D(stateMinus2.getJointTranslation(i));
            V3D jointPositionMinus1 = V3D(stateMinus1.getJointTranslation(i));
            V3D jointPosition = V3D(state.getJointTranslation(i));

            jointPositionInertializationInfos[i] = computeInertializationInfoVector(jointPositionMinus2, jointPositionMinus1, jointPosition, dt, t1);

            // orientation
            Quaternion jointOrientationMinus2 = stateMinus2.getJointRelativeOrientation(i);
            Quaternion jointOrientationMinus1 = stateMinus1.getJointRelativeOrientation(i);
            Quaternion jointOrientation = state.getJointRelativeOrientation(i);

            jointOrientationInertializationInfos[i] = computeInertializationInfoQuaternion(jointOrientationMinus2, jointOrientationMinus1, jointOrientation, dt, t1);
        }
    }

    /**
     * Initialize state at time t based on inertialization infos.
     * 
     * state:        state to be inertialized
     * stateMinus1:  previous state(used to set velocity)
     * t:            time progress in inertialization
     */
    static mocap::MocapSkeletonState inertializeState(const InertializationInfo &rootPositionInertializationInfo, 
                                        const InertializationInfo &rootOrientationInertializationInfo, 
                                        std::vector<InertializationInfo>  &jointPositionInertializationInfo, 
                                        const std::vector<InertializationInfo> &jointOrientationInertializationInfos, 
                                        const int numMarkers,
                                        const mocap::MocapSkeletonState &state, 
                                        const mocap::MocapSkeletonState &stateMinus1, 
                                        double t, double dt) {
        mocap::MocapSkeletonState inertializedState = state;
        // root
        if(false){
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
            // position
            V3D jointPosition = V3D(state.getJointTranslation(i));
            V3D jointPositionMinus1 = V3D(stateMinus1.getJointTranslation(i));

            V3D inertializedJointPosition = inertializeVector(jointPositionInertializationInfo[i], jointPosition, t);
            inertializedState.setJointTranslation(inertializedJointPosition, i);
            inertializedState.setJointRelativeVelocity((inertializedJointPosition - V3D(stateMinus1.getJointTranslation(i))) / dt, i);

            // orientation
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
        // compute q0, qMinus1, X0, x0, xMinus1, v0 and a0 as described in slide 49 of the GDC presentation 
        Quaternion q0 = orientationMinus1 * orientation.inverse();
        Quaternion qMinus1 = orientationMinus2 * orientation.inverse();
        V3D X0 = q0.vec().normalized();
        double x0 = getRotationAngle(q0, X0);
        double xMinus1 = 2 * atan2(qMinus1.vec().dot(X0), qMinus1.w());
        double v0 = (x0 - xMinus1) / dt;
        double a0 = (-8 * v0 * t1 - 20 * x0) / (t1 * t1); 

        // v0 must decrease x0 otherwise set it to 0
        if(x0 * v0 >= 0)
            v0 = 0;

        // a0 must decrease v0 otherwise set it to 0
        if (a0 * v0 >= 0)
            a0 = 0;

        // compute t1 to prevent overshoot as described in slide 29 of the GDC presentation
        if (v0 != 0)
            t1 = std::min(t1, -5 * x0 / v0);

        // compute coefficients as described in slide 29 of the GDC presentation
        double A = -(a0 * t1 * t1 + 6 * v0 * t1 + 12 * x0) / (2 * t1 * t1 * t1 * t1 * t1);
        double B = (3 * a0 * t1 * t1 + 16 * v0 * t1 + 30 * x0) / (2 * t1 * t1 * t1 * t1);
        double C = -(3 * a0 * t1 * t1 + 12 * v0 * t1 + 20 * x0) / (2 * t1 * t1 * t1);

        // generate InertializationInfo
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
        // compute X0, x0, XMinus1, xMinus1, v0 and a0 as described in slide 43 of the GDC presentation 
        V3D X0 = positionMinus1 - position;
        V3D XMinus1 = positionMinus2 - position;
        double x0 = X0.norm();
        // if x0 is too small we don't inertialize (do it here to prevent div by 0) 
        if (x0 < 1e-6)
            return InertializationInfo();
        double xMinus1 = XMinus1.dot(X0/x0);
        double v0 = (x0 - xMinus1) / dt;
        double a0 = (-8 * v0 * t1 - 20 * x0) / (t1 * t1);

        // v0 must decrease x0 otherwise set it to 0
        if(x0 * v0 >= 0)
            v0 = 0;

        // a0 must decrease v0 otherwise set it to 0
        if (a0 * v0 >= 0)
            a0 = 0;

        // compute t1 to prevent overshoot as described in slide 29 of the GDC presentation
        if(v0 != 0.0)
            t1 = std::min(t1, -5 * x0 / v0);

        // compute coefficients as described in slide 29 of the GDC presentation
        double A = -(a0 * t1 * t1 + 6 * v0 * t1 + 12 * x0) / (2 * t1 * t1 * t1 * t1 * t1);
        double B = (3 * a0 * t1 * t1 + 16 * v0 * t1 + 30 * x0) / (2 * t1 * t1 * t1 * t1);
        double C = -(3 * a0 * t1 * t1 + 12 * v0 * t1 + 20 * x0) / (2 * t1 * t1 * t1);

        // generate InertializationInfo
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
     * Inertialize quaternion at time t.
     */
    static Quaternion inertializeQuaternion(const InertializationInfo &info, Quaternion &orientation, double t) {
        //if inertialization is done just return orientation
        if (t >= info.t1)
            return orientation;

        // compute xt as described in slide 29 of the GDC presentation
        double xt = info.A * t * t * t * t * t    //
                  + info.B * t * t * t * t        //
                  + info.C * t * t * t            //
                  + info.a0 / 2.0 * t * t         //
                  + info.v0 * t                   //
                  + info.x0;                      //

        //compute inertialized quaternion qt as described in slide 49 of the GDC presentation
        Quaternion qt = getRotationQuaternion(xt, info.X0) * orientation;
        return qt;
    }

    /**
     * Inertialize vector at time t.
     */
    static V3D inertializeVector(const InertializationInfo &info, V3D &position, double t) {
        // if inertialization is done just return position
        if (t >= info.t1)
            return position;

        // compute xt as described in slide 29 of the GDC presentation
        double xt = info.A * t * t * t * t * t    //
                  + info.B * t * t * t * t        //
                  + info.C * t * t * t            //
                  + info.a0 / 2.0 * t * t         //
                  + info.v0 * t                   //
                  + info.x0;                      //

        // compute inertialized vector Xt as described in slide 43 of the GDC presentation
        V3D Xt = xt * info.X0 / info.x0 + position;
        return Xt;
    }
};

}  // namespace gui
}  // namespace crl