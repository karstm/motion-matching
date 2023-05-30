#pragma once

#include <algorithm>
#include <tuple>
#define _USE_MATH_DEFINES
#include <math.h>

#include <crl-basic/gui/camera.h>
#include <imgui_widgets/imGuIZMOquat.h>
#include <mocap/MocapClip.h>

namespace crl {
namespace gui {

/*
 * Tools for Foot-Locking
 * Adapted from Orangeduck's motion matching system.
 * https://theorangeduck.com/page/code-vs-data-driven-displacement#footlocking
 */

class Footlocking {
// Methods
public:
    Footlocking () {}

    void init(std::vector<std::unique_ptr<crl::mocap::BVHClip>> *clips) {
        lContactInfos.clear();
        rContactInfos.clear();

        for (int i = 0; i < clips->size(); i++) {
            std::vector<bool> lContactInfo, rContactInfo;

            crl::mocap::MocapSkeleton *sk = clips->at(i)->getModel();
            lContactInfo.push_back(true);
            rContactInfo.push_back(true);
            
            for (int j = 1; j < clips->at(i)->getFrameCount(); j++) {
                crl::mocap::MocapSkeletonState *stPrev = &clips->at(i)->getState(j - 1);
                crl::mocap::MocapSkeletonState *stCurr = &clips->at(i)->getState(j);
                lContactInfo.push_back(isInContact(sk, stPrev, stCurr, lFoot, 1 / 60.0));
                rContactInfo.push_back(isInContact(sk, stPrev, stCurr, rFoot, 1 / 60.0));
            }
            lContactInfo = filterContacts(lContactInfo, 5);
            rContactInfo = filterContacts(rContactInfo, 5);
            lContactInfos.push_back(lContactInfo);
            rContactInfos.push_back(rContactInfo);
        }
    }

    std::tuple<bool, bool> isFootInContact(int clipIdx, int frameIdx) {
        bool lContact = lContactInfos.at(clipIdx).at(frameIdx);
        bool rContact = rContactInfos.at(clipIdx).at(frameIdx);

        return { lContact, rContact };
    }

    void ikLookAt(Quaternion &boneRotation,
                const Quaternion wolrdParentRotation,
                const Quaternion worldRotation,
                const V3D worldPosition,
                const V3D wolrdChildPosition,
                const V3D wolrdTargetPosition,
                const float eps = 1e-5f) {
        V3D currDir = (wolrdChildPosition - worldPosition).normalized();
        V3D targetDir = (wolrdTargetPosition - worldPosition).normalized();

        if (fabs(1.0f - currDir.dot(targetDir) > eps)) {
            V3D c = currDir.cross(targetDir);
            Quaternion quatBetween =
                Quaternion(std::sqrtf(currDir.dot(currDir) * targetDir.dot(targetDir)) + currDir.dot(targetDir), c[0], c[1], c[2]).normalized();
            boneRotation = wolrdParentRotation.inverse() * (quatBetween * worldRotation);
        }
    }

    void ikTwoBone( 
        Quaternion &boneRootLocalRot,
        Quaternion &boneMidLocalRot,
        const V3D boneRoot,
        const V3D boneMid,
        const V3D boneEnd,
        const V3D target,
        const V3D fwd,
        const Quaternion boneRootWorldRot,
        const Quaternion boneMidWorldRot,
        const Quaternion boneParWorldRot,
        const float maxLengthBuffer) {
        V3D midToRoot = boneRoot - boneMid;
        V3D endToMid = boneMid - boneEnd;
        float maxExtension = midToRoot.norm() + endToMid.norm() - maxLengthBuffer;

        V3D targetClamp = target;
        if ((target - boneRoot).norm() > maxExtension) {
            targetClamp = boneRoot + maxExtension * (target - boneRoot).normalized();
        }

        V3D axisDown = (boneEnd - boneRoot).normalized();
        V3D axisRot = (axisDown.cross(fwd)).normalized();

        float lRootToMid = (boneMid - boneRoot).norm();
        float lEndToMid = (boneMid - boneEnd).norm();
        float lRootToTargetClamp = (targetClamp - boneRoot).norm();

        float rootToEnd_rootToMid_0 = std::acosf(std::clamp((boneEnd - boneRoot).normalized().dot((boneMid - boneRoot).normalized()), -1.0, 1.0));
        float midToRoot_midToEnd_0 = std::acosf(std::clamp((boneRoot - boneMid).normalized().dot((boneEnd - boneMid).normalized()), -1.0, 1.0));

        float rootToEnd_rootToMid_1 = std::acosf(std::clamp(
            (lRootToMid * lRootToMid + lRootToTargetClamp * lRootToTargetClamp - lEndToMid * lEndToMid) / (2.0 * lRootToMid * lRootToTargetClamp), -1.0, 1.0));
        float midToRoot_midToEnd_1 = std::acosf(std::clamp(
            (lRootToMid * lRootToMid + lEndToMid * lEndToMid - lRootToTargetClamp * lRootToTargetClamp) / (2.0 * lRootToMid * lEndToMid), -1.0, 1.0));

        Quaternion r0 = getRotationQuaternion(rootToEnd_rootToMid_1 - rootToEnd_rootToMid_0, axisRot);
        Quaternion r1 = getRotationQuaternion(midToRoot_midToEnd_1 - midToRoot_midToEnd_0, axisRot);

        V3D rootToEndNormalized = (boneEnd - boneRoot).normalized();
        V3D rootToTargetClampNormalized = (targetClamp - boneRoot).normalized();

        Quaternion r2 = getRotationQuaternion(std::acosf(std::clamp(rootToEndNormalized.dot(rootToTargetClampNormalized), -1.0, 1.0)),
                                              rootToEndNormalized.cross(rootToTargetClampNormalized).normalized());

        boneRootLocalRot = boneParWorldRot.inverse() * (r2 * (r0 * boneRootWorldRot));
        boneMidLocalRot = boneRootWorldRot.inverse() * (r1 * boneMidWorldRot);
    }

private:
    bool isInContact(crl::mocap::MocapSkeleton *sk,
                    crl::mocap::MocapSkeletonState *statePrev, crl::mocap::MocapSkeletonState *stateCurrent,
                    const std::string &jointName, float dt) {
        sk->setState(statePrev);
        const auto jointPrev = sk->getMarkerByName(jointName.c_str());
        P3D prevPos = jointPrev->state.getWorldCoordinates(jointPrev->endSites[0].endSiteOffset);

        sk->setState(stateCurrent);
        const auto jointCurr = sk->getMarkerByName(jointName.c_str());
        P3D currPos = jointCurr->state.getWorldCoordinates(jointCurr->endSites[0].endSiteOffset);

        V3D eevel = V3D(currPos, prevPos) / dt;
        bool isInContact = currPos.y < footHeightThreshold && eevel.norm() < footSpeedThreshold;

        return isInContact;
    }

    std::vector<bool> filterContacts(const std::vector<bool> &contacts, int windowSize) {
        std::vector<bool> filteredContacts(contacts.size());
        int halfWindow = windowSize / 2;

        for (int i = 0; i < contacts.size(); ++i) {
            std::vector<bool> window;

            for (int j = std::max(0, i - halfWindow); j <= std::min((int)contacts.size() - 1, i + halfWindow); ++j) {
                window.push_back(contacts[j]);
            }

            std::sort(window.begin(), window.end());
            filteredContacts[i] = window[window.size() / 2];
        }

        return filteredContacts;
    }


// Members
public:
    const std::string lFoot = "LeftToe";
    const std::string rFoot = "RightToe";
    std::vector<std::vector<bool>> lContactInfos, rContactInfos;
    // If I remove orgRContactInfos, which is never used, I get a whitescreen, if I use the controller and play an animation.
    // That is way please don't remove it! It is a very strange behaviour!
    std::vector<std::vector<bool>> orgRContactInfos;

    float footSpeedThreshold = 0.45f;
    float footHeightThreshold = 0.08f;

private:


};

}  // namespace gui
}  // namespace crl