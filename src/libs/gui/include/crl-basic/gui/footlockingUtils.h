#pragma once

#include <algorithm>
#include <crl-basic/gui/camera.h>
#include <imgui_widgets/imGuIZMOquat.h>
#define _USE_MATH_DEFINES
#include <math.h>
#include <mocap/MocapClip.h>

namespace crl {
namespace gui {

class FootlockingUtils {
public:
static bool isInContact(crl::mocap::MocapSkeleton *sk, const std::string &jointName,
                        double footHeightThreshold = 0.055, double footSpeedThreshold = 10) {
                            
    if (const auto joint = sk->getMarkerByName(jointName.c_str())) {
        crl::P3D eepos = joint->state.getWorldCoordinates(joint->endSites[0].endSiteOffset);
        crl::V3D eevel = joint->state.getVelocityForPoint_local(joint->endSites[0].endSiteOffset);

        return eepos.y < footHeightThreshold && eevel.norm() < footSpeedThreshold;
    }
    return false;
}


private:

};

}  // namespace gui
}  // namespace crl