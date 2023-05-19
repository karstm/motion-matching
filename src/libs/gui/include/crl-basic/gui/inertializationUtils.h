#pragma once

#include <crl-basic/gui/camera.h>
#include <imgui_widgets/imGuIZMOquat.h>
#define _USE_MATH_DEFINES
#include <math.h>
#include <mocap/MocapClip.h>

namespace crl {
namespace gui {

struct InertializationInfo {
    V3D x0v = V3D(0, 0, 0);
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
    static void computeInertialization(InertializationInfo &rootPosInertializationInfo, 
                                       InertializationInfo &rootQInertializationInfo, 
                                        std::vector<InertializationInfo> &jointInertializationInfos, 
                                        const int numMarkers,
                                        const mocap::MocapSkeletonState &oldMinus1Motion, 
                                        const mocap::MocapSkeletonState &oldMotion, 
                                        const mocap::MocapSkeletonState &newMotion, 
                                        double t1, 
                                        double dt) {
        // root
        {
            // position
            V3D oldMinus1Pos = V3D(oldMinus1Motion.getRootPosition());
            V3D oldPos = V3D(oldMotion.getRootPosition());
            V3D newPos = V3D(newMotion.getRootPosition());

            rootPosInertializationInfo = computeInertialization(oldMinus1Pos, oldPos, newPos, dt, t1);

            // orientation
            Quaternion oldMinus1Q = oldMinus1Motion.getRootOrientation();
            Quaternion oldQ = oldMotion.getRootOrientation();
            Quaternion newQ = newMotion.getRootOrientation();

            rootQInertializationInfo = computeInertialization(oldMinus1Q, oldQ, newQ, dt, t1);
        }

        // joints
        for (uint i = 0; i < numMarkers; i++) {
            Quaternion oldMinus1Q = oldMinus1Motion.getJointRelativeOrientation(i);
            Quaternion oldQ = oldMotion.getJointRelativeOrientation(i);
            Quaternion newQ = newMotion.getJointRelativeOrientation(i);

            jointInertializationInfos[i] = computeInertialization(oldMinus1Q, oldQ, newQ, dt, t1);
        }
    }

    /**
     * initialize state at time t based on computed inertialization infos.
     * 
     * motion_t is a motion from new sequence at time t
     * t is a time since the last motion matching happened
     * dt is a frame timestep (for computing velocity by finite difference)
     */
    static mocap::MocapSkeletonState inertializeState(const InertializationInfo &rootPosInertializationInfo, 
                                        const InertializationInfo &rootQInertializationInfo, 
                                        const std::vector<InertializationInfo> &jointInertializationInfos, 
                                        const int numMarkers,
                                        const mocap::MocapSkeletonState &motion_t, 
                                        const mocap::MocapSkeletonState &motion_tMinus1, 
                                        double t, double dt) {
        // we will return inertializedMotion_t
        mocap::MocapSkeletonState inertializedMotion_t = motion_tMinus1;

        // root
        {
            // position
            V3D rootPosV = V3D(motion_t.getRootPosition());
            P3D pos_tminus1 = inertializedMotion_t.getRootPosition();

            V3D inertializedPosV = inertializeVector(rootPosInertializationInfo, rootPosV, t);
            inertializedMotion_t.setRootPosition(P3D() + inertializedPosV);
            inertializedMotion_t.setRootVelocity((inertializedPosV - V3D(pos_tminus1)) / dt);

            // orientation
            Quaternion rootQ = motion_t.getRootOrientation();
            Quaternion q_tminus1 = inertializedMotion_t.getRootOrientation();

            Quaternion inertializedQ = inertializeQuaternion(rootQInertializationInfo, rootQ, t);
            inertializedMotion_t.setRootOrientation(inertializedQ);
            inertializedMotion_t.setRootAngularVelocity(mocap::estimateAngularVelocity(q_tminus1, inertializedQ, dt));
        }

        // joints
        for (uint i = 0; i < numMarkers; i++) {
            Quaternion jointRelQ = motion_t.getJointRelativeOrientation(i);
            Quaternion q_tminus1 = inertializedMotion_t.getJointRelativeOrientation(i);
            auto &inertializeInfo = jointInertializationInfos[i];

            Quaternion inertializedQ = inertializeQuaternion(inertializeInfo, jointRelQ, t);
            inertializedMotion_t.setJointRelativeOrientation(inertializedQ, i);
            inertializedMotion_t.setJointRelativeAngVelocity(mocap::estimateAngularVelocity(q_tminus1, inertializedQ, dt), i);
        }

        return inertializedMotion_t;
    }


private:
    static InertializationInfo computeInertialization(Quaternion &oldPrevQ, Quaternion &oldQ, Quaternion &newQ, double dt, double t1) {
        Quaternion q0 = oldQ * newQ.inverse();
        Quaternion q_minus1 = oldPrevQ * newQ.inverse();

        // compute x0
        if (q0.vec().norm() < 1e-10) {
            // then we can just say q0 is identity...
            // just play coming sequences as it is
            InertializationInfo info;
            return info;
        }

        // if q0 is not identity
        V3D x0v = q0.vec().normalized();
        double x0 = getRotationAngle(q0, x0v);

        // compute x_minus1
        double x_minus1 = 2 * atan2(q_minus1.vec().dot(x0v), q_minus1.w());

        // compute v0 and a0
        // note.
        // if x0 >= 0 then v0 should be < 0 and a0 should be > 0
        // if x0 < 0 then v0 should be > 0 and a0 should be < 0
        // if this is not the case, just clamp v0 or a0

        // v0 = (x0 - x_minus1) / dt
        double v0 = (x0 - x_minus1) / dt;

        if ((x0 > 0 && v0 < 0) || (x0 < 0 && v0 > 0))
            // t1 = min(t1, -5 * x0/v0)
            t1 = std::min(t1, -5 * x0 / v0);

        // a0 = (-8 * v0 * t1 - 20 * x0) / (t1^2)
        double a0 = (-8 * v0 * t1 - 20 * x0) / (t1 * t1);

        if (x0 > 0) {
            if (v0 > 0)
                v0 = 0;
            if (a0 < 0)
                a0 = 0;
        } else {
            if (v0 < 0)
                v0 = 0;
            if (a0 > 0)
                a0 = 0;
        }

        // A = - (a0 * t1^2 + 6 * v0 * t1 + 12 * x0) / (2 * t1^5)
        double A = -(a0 * t1 * t1 + 6 * v0 * t1 + 12 * x0) / (2 * t1 * t1 * t1 * t1 * t1);
        // B = (3 * a0 * t1^2 + 16 * v0 * t1 + 30 * x0) / (2 * t1^4)
        double B = (3 * a0 * t1 * t1 + 16 * v0 * t1 + 30 * x0) / (2 * t1 * t1 * t1 * t1);
        // C = - (3 * a0 * t1^2 + 12 * v0 * t1 + 20 * x0) / (2 * t1^3)
        double C = -(3 * a0 * t1 * t1 + 12 * v0 * t1 + 20 * x0) / (2 * t1 * t1 * t1);

        InertializationInfo info;
        info.x0v = x0v;
        info.A = A;
        info.B = B;
        info.C = C;
        info.x0 = x0;
        info.v0 = v0;
        info.a0 = a0;
        info.t1 = t1;
        return info;
    }

    static InertializationInfo computeInertialization(V3D &oldPrevV3D, V3D &oldV3D, V3D &newV3D, double dt, double t1) {
        V3D x0v = oldV3D - newV3D;
        V3D x_minus1v = oldPrevV3D - newV3D;

        double x0 = x0v.norm();
        if (x0 < 1e-10) {
            // can be consider there's no change at all
            InertializationInfo info;
            return info;
        }

        // compute x_minus1
        double x_minus1 = x_minus1v.dot(x0v.normalized());

        // compute v0 and a0
        // note.
        // if x0 >= 0 then v0 should be < 0 and a0 should be > 0
        // if x0 < 0 then v0 should be > 0 and a0 should be < 0
        // if this is not the case, just clamp v0 or a0

        // v0 = (x0 - x_minus1) / dt
        double v0 = (x0 - x_minus1) / dt;

        if ((x0 > 0 && v0 < 0) || (x0 < 0 && v0 > 0))
            // t1 = min(t1, -5 * x0/v0)
            t1 = std::min(t1, -5 * x0 / v0);

        // a0 = (-8 * v0 * t1 - 20 * x0) / (t1^2)
        double a0 = (-8 * v0 * t1 - 20 * x0) / (t1 * t1);

        if (x0 > 0) {
            if (v0 > 0)
                v0 = 0;
            if (a0 < 0)
                a0 = 0;
        } else {
            if (v0 < 0)
                v0 = 0;
            if (a0 > 0)
                a0 = 0;
        }

        // A = - (a0 * t1^2 + 6 * v0 * t1 + 12 * x0) / (2 * t1^5)
        double A = -(a0 * t1 * t1 + 6 * v0 * t1 + 12 * x0) / (2 * t1 * t1 * t1 * t1 * t1);
        // B = (3 * a0 * t1^2 + 16 * v0 * t1 + 30 * x0) / (2 * t1^4)
        double B = (3 * a0 * t1 * t1 + 16 * v0 * t1 + 30 * x0) / (2 * t1 * t1 * t1 * t1);
        // C = - (3 * a0 * t1^2 + 12 * v0 * t1 + 20 * x0) / (2 * t1^3)
        double C = -(3 * a0 * t1 * t1 + 12 * v0 * t1 + 20 * x0) / (2 * t1 * t1 * t1);

        InertializationInfo info;
        info.x0v = x0v;
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
     * inertialize quaternion newQ_t at time t.
     */
    static Quaternion inertializeQuaternion(const InertializationInfo &info, Quaternion &newQ_t, double t) {
        if (t >= info.t1)
            // inertialization is done just return newQ
            return newQ_t;

        // xt = A * t^5 + B * t^4 + C * t^3 + a0/2 * t^2 + v0 * t + x0
        double x = info.A * t * t * t * t * t  //
                + info.B * t * t * t * t    //
                + info.C * (t * t * t)      //
                + 0.5 * info.a0 * t * t     //
                + info.v0 * t               //
                + info.x0;                  //

        Quaternion dq = Quaternion(cos(0.5 * x),                 //
                                sin(0.5 * x) * info.x0v.x(),  //
                                sin(0.5 * x) * info.x0v.y(),  //
                                sin(0.5 * x) * info.x0v.z())
                            .normalized();

        return dq * newQ_t;
    }

    /**
     * inertialize vector newV_t at time t.
     */
    static V3D inertializeVector(const InertializationInfo &info, V3D &newV_t, double t) {
        if (t >= info.t1)
            // inertialization done just return newV
            return newV_t;

        // xt = A * t^5 + B * t^4 + C * t^3 + a0/2 * t^2 + v0 * t + x0
        double x = info.A * t * t * t * t * t  //
                + info.B * t * t * t * t    //
                + info.C * (t * t * t)      //
                + 0.5 * info.a0 * t * t     //
                + info.v0 * t               //
                + info.x0;                  //

        V3D dv = V3D(info.x0v.normalized()) * x;

        return newV_t + dv;
    }
};

}  // namespace gui
}  // namespace crl