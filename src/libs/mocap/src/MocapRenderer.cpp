#include "mocap/MocapRenderer.h"

namespace crl::mocap {

V3D MocapRenderer::defaultLinkDrawColor_ = V3D(0.5, 0.5, 0.5);       // Gray
V3D MocapRenderer::selectedLinkDrawColor_ = V3D(0.9, 0.9, 0.0);      // Yellowish
V3D MocapRenderer::jointDrawColor_ = V3D(0.0, 0.0, 0.9);             // Blue
V3D MocapRenderer::defaultEndSiteLinkColor_ = V3D(0.5, 0.0, 0.0);    // Red
V3D MocapRenderer::selectedEndSiteLinkColor_ = V3D(0.9, 0.85, 0.0);  // Gold
V3D MocapRenderer::endSiteContactColor_ = V3D(0.0, 0.9, 0.0);        // Green

void MocapRenderer::drawMarker(MocapMarker *joint, const crl::gui::Shader &shader, bool drawSkeleton, bool drawModel, bool drawSimulationBone, V3D modelColor) {
    if (joint->model && drawModel)
    {
        joint->model->orientation = joint->state.orientation;
        joint->model->position = joint->state.pos;
        joint->model->draw(shader, modelColor);
    }

    V3D drawColor = joint->selected ? selectedLinkDrawColor_ : defaultLinkDrawColor_;

    if(drawSkeleton && joint->parent != NULL)
        drawSphere(joint->state.pos, linkCylinderRadius_ * 1.3, shader, jointDrawColor_);
    else if(drawSimulationBone && joint->parent == NULL)
        drawSphere(joint->state.pos, linkCylinderRadius_ * 3.3, shader, V3D(1,0,0));

    // bone ~ parent
    if (drawSkeleton && joint->parent != NULL && joint->parent->parent != NULL) {
        P3D startPos = joint->parent->state.pos;
        P3D endPos = joint->state.pos;
        drawCapsule(startPos, endPos, linkCylinderRadius_, shader, drawColor);
    }
    else if(drawSimulationBone && joint->parent == NULL){
        P3D startPos = joint->state.pos;
        V3D direction = joint->state.orientation * V3D(0.6, 0, 0);
        drawArrow3d(startPos, direction, linkCylinderRadius_, shader, V3D(1, 0, 0));
    }

    // bone ~ children
    for (uint i = 0; i < joint->children.size(); i++) {
        if(drawSkeleton && joint->parent != NULL) {
            P3D startPos = joint->state.pos;
            P3D endPos = joint->children[i]->state.pos;
            drawCapsule(startPos, endPos, linkCylinderRadius_, shader, drawColor);
        }
    }

    // bone ~ end effector
    if(drawSkeleton) {
        V3D endSiteLinkColor = joint->selected ? selectedEndSiteLinkColor_ : defaultEndSiteLinkColor_;

        for (uint i = 0; i < joint->endSites.size(); i++) {
            P3D startPos = joint->state.pos;
            P3D endPos = joint->state.getWorldCoordinates(joint->endSites[i].endSiteOffset);
            drawCapsule(startPos, endPos, linkCylinderRadius_, shader, endSiteLinkColor);
        }
    }
}

void MocapRenderer::drawCoordFrame(const MocapMarker *joint, const gui::Shader &shader) {
    drawArrow3d(joint->state.pos, V3D(joint->state.getWorldCoordinates(V3D(1, 0, 0)) * 0.1), 0.01, shader, V3D(1.0, 0.0, 0.0));
    drawArrow3d(joint->state.pos, V3D(joint->state.getWorldCoordinates(V3D(0, 1, 0)) * 0.1), 0.01, shader, V3D(0.0, 1.0, 0.0));
    drawArrow3d(joint->state.pos, V3D(joint->state.getWorldCoordinates(V3D(0, 0, 1)) * 0.1), 0.01, shader, V3D(0.0, 0.0, 1.0));
}

void MocapRenderer::drawJointVelocity(const MocapMarker *joint, const gui::Shader &shader) {
    drawArrow3d(joint->state.pos, joint->state.velocity * 0.1, 0.01, shader, V3D(1.0, 0.0, 0.0));
}

}  // namespace crl::mocap
