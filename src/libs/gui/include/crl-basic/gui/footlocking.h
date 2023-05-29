#pragma once

#include <algorithm>
#include <tuple>
#define _USE_MATH_DEFINES
#include <math.h>
#include <deque>

#include <crl-basic/gui/camera.h>
#include <imgui_widgets/imGuIZMOquat.h>
#include <mocap/MocapClip.h>

namespace crl {
namespace gui {

class Footlocking {
// Methods
public:
    Footlocking () {}
    void init(std::vector<std::unique_ptr<crl::mocap::BVHClip>> *clips){
        lContactInfos.clear();
        rContactInfos.clear();

        for(int i = 0; i < clips->size(); i++){
            std::vector<bool> lContactInfo, rContactInfo;
            
            crl::mocap::MocapSkeleton *sk = clips->at(i)->getModel();
            lContactInfo.push_back(true);
            rContactInfo.push_back(true);
            
            for(int j = 1; j < clips->at(i)->getFrameCount(); j++){
                crl::mocap::MocapSkeletonState *stPrev = &clips->at(i)->getState(j-1);
                crl::mocap::MocapSkeletonState *stCurr = &clips->at(i)->getState(j);
                lContactInfo.push_back(isInContact(sk, stPrev, stCurr, lFoot, 1 / 60.0));
                rContactInfo.push_back(isInContact(sk, stPrev, stCurr, rFoot, 1 / 60.0));
            }
            orgLContactInfos.push_back(lContactInfo);
            orgRContactInfos.push_back(rContactInfo);
            lContactInfo = filterContacts(lContactInfo, 5);
            rContactInfo = filterContacts(rContactInfo, 5);
            lContactInfos.push_back(lContactInfo);
            rContactInfos.push_back(rContactInfo);
        }
    }

    std::tuple<bool, bool> isFootInContact(int clipIdx, int frameIdx) {
        //bool b2 = orgLContactInfos.at(clipIdx).at(frameIdx);
        //bool b4 = orgRContactInfos.at(clipIdx).at(frameIdx);

        bool lContact = lContactInfos.at(clipIdx).at(frameIdx);
        bool rContact = rContactInfos.at(clipIdx).at(frameIdx);
        //Logger::consolePrint("left: %d; orgLeft, %d; right: %d; orgRight, %d;  \n", lContact, b2, rContact, b4);

        return { lContact , rContact };
    }

    static inline Quaternion quat_between(V3D p, V3D q)
    {
        V3D c = p.cross(q);
        return Quaternion(std::sqrtf(p.dot(p) * q.dot(q)) + p.dot(q), c[0], c[1], c[2]).normalized();
    }

    void ikLookAt(
        Quaternion& bone_rotation,
        const Quaternion global_parent_rotation,
        const Quaternion global_rotation,
        const V3D global_position,
        const V3D child_position,
        const V3D target_position,
        const float eps = 1e-5f
        ){
        V3D curr_dir = (child_position - global_position).normalized();
        V3D targ_dir = (target_position - global_position).normalized();

        if (fabs(1.0f - curr_dir.dot(targ_dir) > eps))
        {
            bone_rotation = global_parent_rotation.inverse() * (quat_between(curr_dir, targ_dir) * global_rotation);
        }
    }

    void ikTwoBone( 
        Quaternion &boneRootLr,
        Quaternion &boneMidLr,
        const V3D boneRoot,
        const V3D boneMid,
        const V3D boneEnd,
        const V3D target,
        const V3D fwd,
        const Quaternion boneRootGr,
        const Quaternion boneMidGr,
        const Quaternion boneParGr,
        const float maxLengthBuffer
        ){
        float max_extension = (boneRoot - boneMid).norm() + (boneMid - boneEnd).norm() - maxLengthBuffer;

        V3D target_clamp = target;
        if ((target - boneRoot).norm() > max_extension)
        {
            target_clamp = boneRoot + max_extension * (target - boneRoot).normalized();
        }
        
        V3D axis_dwn = (boneEnd - boneRoot).normalized();
        V3D axis_rot = (axis_dwn.cross(fwd)).normalized();

        V3D a = boneRoot;
        V3D b = boneMid;
        V3D c = boneEnd;
        V3D t = target_clamp;
        
        float lab = (b - a).norm();
        float lcb = (b - c).norm();
        float lat = (t - a).norm();

        float ac_ab_0 = std::acosf(std::clamp((c - a).normalized().dot((b - a).normalized()), -1.0, 1.0));
        float ba_bc_0 = std::acosf(std::clamp((a - b).normalized().dot((c - b).normalized()), -1.0, 1.0));

        float ac_ab_1 = std::acosf(std::clamp((lab * lab + lat * lat - lcb * lcb) / (2.0 * lab * lat), -1.0, 1.0));
        float ba_bc_1 = std::acosf(std::clamp((lab * lab + lcb * lcb - lat * lat) / (2.0 * lab * lcb), -1.0, 1.0));

        Quaternion r0 = getRotationQuaternion(ac_ab_1 - ac_ab_0, axis_rot);
        Quaternion r1 = getRotationQuaternion(ba_bc_1 - ba_bc_0, axis_rot);

        V3D c_a = (boneEnd - boneRoot).normalized();
        V3D t_a = (target_clamp - boneRoot).normalized();

        Quaternion r2 = getRotationQuaternion( std::acosf( std::clamp(c_a.dot(t_a), -1.0, 1.0)), c_a.cross(t_a).normalized() );
        
        boneRootLr = boneParGr.inverse() * (r2 * (r0 * boneRootGr));
        boneMidLr = boneRootGr.inverse() * (r1 * boneMidGr);
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

        V3D eevel =  V3D(currPos, prevPos) / dt;
        bool isInContact = currPos.y < footHeightThreshold && eevel.norm() < footSpeedThreshold;

        Logger::consolePrint("%s: %f, %f, %d; \n", jointName.c_str(), currPos.y, eevel.norm(), isInContact);
        return isInContact;
    }

    std::vector<bool> filterContacts(const std::vector<bool>& contacts, int windowSize) {
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
    std::vector<std::vector<bool>> orgLContactInfos, orgRContactInfos;

    float footSpeedThreshold = 0.45f;
    float footHeightThreshold = 0.08f;

private:


};

}  // namespace gui
}  // namespace crl