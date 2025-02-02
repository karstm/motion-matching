//
// Created by Dongho Kang on 01.01.22.
//

#ifndef CRL_MOCAP_MOTIONPROCESSINGUTILS_H
#define CRL_MOCAP_MOTIONPROCESSINGUTILS_H

#include "crl-basic/utils/trajectory.h"
#include "mocap/MocapClip.h"

namespace crl::mocap {

std::vector<std::pair<double, bool>> extractFootContactStateFromBVHClip(crl::mocap::BVHClip *clip, const std::string &jointName, double footHeightThreshold,
                                                                        double footSpeedThreshold) {
    std::vector<std::pair<double, bool>> contactYN;
    contactYN.reserve(clip->getFrameCount());
    auto *sk = clip->getModel();

    for (int i = 0; i < clip->getFrameCount(); i++) {
        sk->setState(&clip->getState(i));
        if (const auto joint = sk->getMarkerByName(jointName.c_str())) {
            crl::P3D eepos = joint->state.getWorldCoordinates(joint->endSites[0].endSiteOffset);
            crl::V3D eevel = joint->state.getVelocityForPoint_local(joint->endSites[0].endSiteOffset);

            if (eepos.y < footHeightThreshold && eevel.norm() < footSpeedThreshold)
                contactYN.emplace_back(clip->getFrameTimeStep() * i, true);  // contact
            else
                contactYN.emplace_back(clip->getFrameTimeStep() * i, false);  // swing
        }
    }
    return contactYN;
}

std::vector<std::pair<double, double>> convertFootContactsSequenceFromFootContactStates(const std::vector<std::pair<double, bool>> &contactInfos) {
    std::vector<std::pair<double, double>> contacts;

    double tContactStart = 0;
    bool isCollectingContact = false;
    for (const auto isContactAtT : contactInfos) {
        if (isContactAtT.second) {
            // at T, it's contact
            if (!isCollectingContact) {
                isCollectingContact = true;
                tContactStart = isContactAtT.first;
            }
        } else {
            // at T, it's contact
            if (isCollectingContact) {
                contacts.emplace_back(tContactStart, isContactAtT.first);
                isCollectingContact = false;
            }
        }
    }

    return contacts;
}

crl::Trajectory3D extractMarkerPositionTrajectoryFromBVHClip(crl::mocap::BVHClip *clip, const std::string &jointName) {
    crl::Trajectory3D pos;
    auto *sk = clip->getModel();

    for (int i = 0; i < clip->getFrameCount(); i++) {
        sk->setState(&clip->getState(i));
        if (const auto joint = sk->getMarkerByName(jointName.c_str())) {
            pos.addKnot(clip->getFrameTimeStep() * i, V3D(joint->state.pos));
        }
    }
    return pos;
}

/**
 * return trajectory of forward, sideways, turning speed.
 */
crl::Trajectory3D extractRootSpeedProfileFromBVHClip(crl::mocap::BVHClip *clip) {
    crl::Trajectory3D vel;
    auto *sk = clip->getModel();
    for (int i = 0; i < clip->getFrameCount(); i++) {
        sk->setState(&clip->getState(i));

        double roll = 0, pitch = 0, yaw = 0;
        crl::computeEulerAnglesFromQuaternion(sk->root->state.orientation,                                     //
                                              sk->forwardAxis, sk->upAxis.cross(sk->forwardAxis), sk->upAxis,  //
                                              roll, pitch, yaw);
        crl::Quaternion heading = crl::getRotationQuaternion(yaw, sk->upAxis);
        double turning = sk->root->state.angularVelocity.dot(sk->upAxis);
        double forward = (heading.inverse() * sk->root->state.velocity).dot(sk->forwardAxis);
        double sideways = (heading.inverse() * sk->root->state.velocity).dot(sk->upAxis.cross(sk->forwardAxis));
        vel.addKnot(clip->getFrameTimeStep() * i, V3D(forward, sideways, turning));
    }
    return vel;
}

}  // namespace crl::mocap

#endif  //CRL_MOCAP_MOTIONPROCESSINGUTILS_H
