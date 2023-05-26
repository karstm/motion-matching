#pragma once

#include <algorithm>
#include <crl-basic/gui/camera.h>
#include <imgui_widgets/imGuIZMOquat.h>
#define _USE_MATH_DEFINES
#include <math.h>
#include <mocap/MocapClip.h>

namespace crl {
namespace gui {

class Footlocking {
// Methods
public:
static bool isInContact(crl::mocap::MocapSkeleton *sk,
                        crl::mocap::MocapSkeletonState *statePrev, crl::mocap::MocapSkeletonState *stateCurrent,
                        const std::string &jointName, float dt,
                        float footHeightThreshold, float footSpeedThreshold) {
    
    sk->setState(statePrev);
    const auto jointPrev = sk->getMarkerByName(jointName.c_str());
    P3D prevPos = jointPrev->state.getWorldCoordinates(jointPrev->endSites[0].endSiteOffset);

    sk->setState(stateCurrent);
    const auto jointCurr = sk->getMarkerByName(jointName.c_str());
    P3D currPos = jointCurr->state.getWorldCoordinates(jointCurr->endSites[0].endSiteOffset);

    V3D eevel =  V3D(currPos, prevPos) / dt;
    bool isInContact = currPos.y < footHeightThreshold && eevel.norm() < footSpeedThreshold;

    Logger::consolePrint("%s: %f, %f, %d; \n", jointName, currPos.y, eevel.norm(), isInContact);
    return isInContact;
}


private:

// Members
public:
    std::vector<std::vector<bool>> contactInfos;

private:


};

}  // namespace gui
}  // namespace crl