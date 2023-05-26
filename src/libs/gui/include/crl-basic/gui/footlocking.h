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
        V3D midToRoot = boneMid - boneRoot;
        V3D endToMid = boneEnd - boneMid;
        float maxExtension = midToRoot.norm() + endToMid.norm() - maxLengthBuffer;
        
        V3D targetClamp = target;
        if ((target - boneRoot).norm() > maxExtension)
        {
            targetClamp = boneRoot + maxExtension*(target - boneRoot).normalized();
        }
        
        V3D endToRoot = boneEnd - boneRoot;
        V3D axisDwn = endToRoot.normalized();
        V3D axisRot = (axisDwn.cross(fwd)).normalized();

        V3D targetToRoot = targetClamp - boneRoot;
        float lMidToRoot = midToRoot.norm();
        float lEndToMid = endToMid.norm();
        float lTargetToRoot = targetToRoot.norm();

        float temp = endToRoot.normalized().dot(midToRoot.normalized());
        float rootToEnd_rootToMid_0 = std::acosf(std::clamp(temp, -1.0f, 1.0f));

        V3D rootToMid = -midToRoot;
        temp = rootToMid.normalized().dot(endToMid.normalized());
        float midToRoot_midToEnd_0 = std::acosf(std::clamp(temp, -1.0f, 1.0f));

        temp = lMidToRoot * lMidToRoot + lTargetToRoot * lTargetToRoot - lEndToMid * lEndToMid;
        float rootToEnd_rootToMid_1 = std::acosf(std::clamp(temp / (2.0f * lMidToRoot * lTargetToRoot), -1.0f, 1.0f));
        temp = lMidToRoot * lMidToRoot - lTargetToRoot * lTargetToRoot + lEndToMid * lEndToMid;
        float midToRoot_midToEnd_1 = std::acosf(std::clamp(temp / (2.0f * lMidToRoot * lEndToMid), -1.0f, 1.0f));

        Quaternion r0 = getRotationQuaternion(rootToEnd_rootToMid_1 - rootToEnd_rootToMid_0, axisRot);
        Quaternion r1 = getRotationQuaternion(midToRoot_midToEnd_1 - midToRoot_midToEnd_0, axisRot);

        V3D endToRootNormalized = endToRoot.normalized();
        V3D targetToRootNormalized = targetToRoot.normalized();

        float rotationAngle2 = std::acosf(std::clamp((float)endToRootNormalized.dot(targetToRootNormalized), -1.0f, 1.0f)); 
        V3D rotationAxis2 = (endToRootNormalized.cross(targetToRootNormalized)).normalized();
        Quaternion r2 = getRotationQuaternion(rotationAngle2, rotationAxis2);
        
        boneRootLr = boneParGr.inverse() * (r2 * (r0 * boneRootGr));
        boneMidLr = boneRootGr.inverse() *(r1 * boneMidGr);
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