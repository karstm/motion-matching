#include <robot/GeneralizedCoordinatesRobotRepresentation.h>
#include <robot/RBJoint.h>
#include <robot/RBLoader.h>
#include <robot/Robot.h>
#include <robot/RobotState.h>
#include "crl-basic/gui/mxm_utils.h"
#include "crl-basic/utils/mathUtils.h"

namespace crl {

Robot::Robot(const char *filePath, const char *statePath, bool loadVisuals) {
    // load robot from rbLoader
    RBLoader rbLoader(filePath, loadVisuals);
    rbLoader.populateRobot(this);

    // set initial state
    // load robot state from rs file or
    // set default state based on joint default angle
    if (statePath && strlen(statePath) > 0) {
        loadReducedStateFromFile(statePath);
    } else {
        RobotState rs(this, true);
        rs.setPosition(P3D(0, 0.9, 0));
        rs.setOrientation(getRotationQuaternion(PI, V3D(0, 1, 0)));
        setState(&rs);
    }
}

Robot::~Robot() {
    for (uint i = 0; i < rbList.size(); i++) delete rbList[i];
    rbList.clear();
    for (uint i = 0; i < jointList.size(); i++) delete jointList[i];
    jointList.clear();
}

void Robot::populateState(RobotState *state, bool useDefaultAngles) {
    // we'll push the root's state information - ugly code....
    state->setPosition(root->state.pos);
    state->setOrientation(root->state.orientation);
    state->setHeadingAxis(RBGlobals::worldUp);

    state->setJointCount((int)jointList.size());

    // now each joint introduces one more rigid body, so we'll only record its
    // state relative to its parent. we are assuming here that each joint is
    // revolute!!!

    for (uint i = 0; i < jointList.size(); i++) {
        if (!useDefaultAngles) {
            state->setJointRelativeOrientation(
                getRelativeOrientationForJoint(jointList[i]), i);
        } else {
            state->setJointRelativeOrientation(
                getRotationQuaternion(jointList[i]->defaultJointAngle,
                                      jointList[i]->rotationAxis),
                i);
        }
    }
}

void Robot::setDefaultState() {
    RobotState rs;
    populateState(&rs, true);
    setState(&rs);
}

void Robot::setZeroState() {
    RobotState rs;
    populateState(&rs, true);
    for (uint i = 0; i < jointList.size(); i++) {
        rs.setJointRelativeOrientation(
            getRotationQuaternion(0, jointList[i]->rotationAxis), i);
    }
    setState(&rs);
}

void Robot::setState(RobotState *state) {
    // kinda ugly code....
    root->state.pos = state->getPosition()  ;
    root->state.orientation = state->getOrientation();
    root->state.orientation.normalize();

    // now each joint introduces one more rigid body, so we'll only record its
    // state relative to its parent. we are assuming here that each joint is
    // revolute!!!
    for (uint j = 0; j < jointList.size(); j++) {
        setRelativeOrientationForJoint(
            jointList[j],
            state->getJointRelativeOrientation((int)j).normalized());
        // and now set the linear position and velocity
        jointList[j]->fixJointConstraints(true, true);
    }
}

void Robot::setMocapState(crl::mocap::MocapSkeletonState *state) {
    if(state == nullptr) {
        return;
    }

    RobotState rs = RobotState(this, true);
    rs.setPosition(state->getRootPosition());
    rs.setOrientation(state->getRootOrientation() * getRotationQuaternion(-PI/2, V3D(0, 0, 1)) * getRotationQuaternion(-PI/2, V3D(0, 1, 0)));

    double alpha, beta, gamma;
    crl::V3D x = V3D(1, 0, 0);
    crl::V3D y = V3D(0, 1, 0);
    crl::V3D z = V3D(0, 0, 1);
    crl::V3D side = V3D(1, 0, 0);
    crl::V3D up = V3D(0, 1, 0);
    crl::V3D front = V3D(0, 0, 1);

    //left hip
    crl::Quaternion q = state->getJointRelativeOrientation(1);
    computeEulerAnglesFromQuaternion(q, y, x, z, alpha, beta, gamma);
    rs.setJointRelativeOrientation(crl::getRotationQuaternion(beta, jointList[7]->rotationAxis), 7); //y
    rs.setJointRelativeOrientation(crl::getRotationQuaternion(alpha + M_PI, jointList[4]->rotationAxis), 4);   //z
    rs.setJointRelativeOrientation(crl::getRotationQuaternion(gamma, jointList[1]->rotationAxis), 1); //x

    //right hip
    q = state->getJointRelativeOrientation(5);
    computeEulerAnglesFromQuaternion(q, y, x, z, alpha, beta, gamma);
    rs.setJointRelativeOrientation(crl::getRotationQuaternion(beta, jointList[8]->rotationAxis), 8); //y
    rs.setJointRelativeOrientation(crl::getRotationQuaternion(alpha + M_PI, jointList[5]->rotationAxis), 5);  //z
    rs.setJointRelativeOrientation(crl::getRotationQuaternion(gamma, jointList[2]->rotationAxis), 2); //x

    //left knee
    q = state->getJointRelativeOrientation(2);
    computeEulerAnglesFromQuaternion(q, y, x, z, alpha, beta, gamma);
    rs.setJointRelativeOrientation(crl::getRotationQuaternion(-gamma, jointList[10]->rotationAxis), 10); //x

    //right knee
    q = state->getJointRelativeOrientation(6);
    computeEulerAnglesFromQuaternion(q, y, x, z, alpha, beta, gamma);
    rs.setJointRelativeOrientation(crl::getRotationQuaternion(-gamma, jointList[11]->rotationAxis), 11); //x

    //left foot
    q = state->getJointRelativeOrientation(3);
    computeEulerAnglesFromQuaternion(q, y, x, z, alpha, beta, gamma);
    rs.setJointRelativeOrientation(crl::getRotationQuaternion(alpha, jointList[16]->rotationAxis), 16); //z
    rs.setJointRelativeOrientation(crl::getRotationQuaternion(-gamma + M_PI_2, jointList[13]->rotationAxis), 13); //x

    //right foot
    q = state->getJointRelativeOrientation(7);
    computeEulerAnglesFromQuaternion(q, y, x, z, alpha, beta, gamma);
    rs.setJointRelativeOrientation(crl::getRotationQuaternion(alpha, jointList[17]->rotationAxis), 17); //z
    rs.setJointRelativeOrientation(crl::getRotationQuaternion(-gamma + M_PI_2, jointList[14]->rotationAxis), 14); //x

    //left toe
    q = state->getJointRelativeOrientation(4);
    computeEulerAnglesFromQuaternion(q, y, x, z, alpha, beta, gamma);
    rs.setJointRelativeOrientation(crl::getRotationQuaternion(-gamma, jointList[21]->rotationAxis), 21); //x

    //right toe
    q = state->getJointRelativeOrientation(8);
    computeEulerAnglesFromQuaternion(q, y, x, z, alpha, beta, gamma);
    rs.setJointRelativeOrientation(crl::getRotationQuaternion(-gamma, jointList[22]->rotationAxis), 22); //x

    //lower back
    q = state->getJointRelativeOrientation(9);
    computeEulerAnglesFromQuaternion(q, y, z, x, alpha, beta, gamma);
    rs.setJointRelativeOrientation(crl::getRotationQuaternion(alpha, jointList[6]->rotationAxis), 6);  //z
    rs.setJointRelativeOrientation(crl::getRotationQuaternion(gamma, jointList[3]->rotationAxis), 3); //y
    rs.setJointRelativeOrientation(crl::getRotationQuaternion(beta, jointList[0]->rotationAxis), 0); //x

    //upper back
    q = state->getJointRelativeOrientation(10);
    computeEulerAnglesFromQuaternion(q, y, z, x, alpha, beta, gamma);
    q = state->getJointRelativeOrientation(11);
    double alpha2, beta2, gamma2;
    computeEulerAnglesFromQuaternion(q, y, z, x, alpha2, beta2, gamma2);
    rs.setJointRelativeOrientation(crl::getRotationQuaternion(alpha + alpha2, jointList[15]->rotationAxis), 15);  //z
    rs.setJointRelativeOrientation(crl::getRotationQuaternion(gamma + gamma2, jointList[12]->rotationAxis), 12);  //y
    rs.setJointRelativeOrientation(crl::getRotationQuaternion(beta + beta2, jointList[9]->rotationAxis), 9); //x

    //lower neck
    q = state->getJointRelativeOrientation(12);
    computeEulerAnglesFromQuaternion(q, y, z, x, alpha, beta, gamma);
    rs.setJointRelativeOrientation(crl::getRotationQuaternion(alpha, jointList[26]->rotationAxis), 26);  //z
    rs.setJointRelativeOrientation(crl::getRotationQuaternion(gamma, jointList[23]->rotationAxis), 23);  //y
    rs.setJointRelativeOrientation(crl::getRotationQuaternion(beta, jointList[18]->rotationAxis), 18);   //x

    //upper neck
    q = state->getJointRelativeOrientation(13);
    computeEulerAnglesFromQuaternion(q, y, z, x, alpha, beta, gamma);
    rs.setJointRelativeOrientation(crl::getRotationQuaternion(alpha, jointList[35]->rotationAxis), 35);  //z
    rs.setJointRelativeOrientation(crl::getRotationQuaternion(gamma, jointList[32]->rotationAxis), 32);  //y
    rs.setJointRelativeOrientation(crl::getRotationQuaternion(beta, jointList[29]->rotationAxis), 29);   //x

    //left scapula
    q = state->getJointRelativeOrientation(14);
    computeEulerAnglesFromQuaternion(q, y, x, z, alpha, beta, gamma);
    rs.setJointRelativeOrientation(crl::getRotationQuaternion(beta + M_PI, jointList[27]->rotationAxis), 27);   //y
    rs.setJointRelativeOrientation(crl::getRotationQuaternion(alpha - M_PI_2, jointList[24]->rotationAxis), 24); //z
    rs.setJointRelativeOrientation(crl::getRotationQuaternion(gamma, jointList[19]->rotationAxis), 19); //x

    //right scapula
    q = state->getJointRelativeOrientation(18);
    computeEulerAnglesFromQuaternion(q, y, x, z, alpha, beta, gamma);
    rs.setJointRelativeOrientation(crl::getRotationQuaternion(beta + M_PI, jointList[28]->rotationAxis), 28); //y
    rs.setJointRelativeOrientation(crl::getRotationQuaternion(alpha + M_PI/2, jointList[25]->rotationAxis), 25); //z
    rs.setJointRelativeOrientation(crl::getRotationQuaternion(gamma, jointList[20]->rotationAxis), 20);  //x

    //left shoulder
    q = state->getJointRelativeOrientation(15);
    computeEulerAnglesFromQuaternion(q, y, x, z, alpha, beta, gamma);
    rs.setJointRelativeOrientation(crl::getRotationQuaternion(-beta, jointList[36]->rotationAxis), 36); //y
    rs.setJointRelativeOrientation(crl::getRotationQuaternion(-alpha + M_PI_2, jointList[33]->rotationAxis), 33);  //z
    rs.setJointRelativeOrientation(crl::getRotationQuaternion(gamma + M_PI / 8.0, jointList[30]->rotationAxis), 30); //x

    //right shoulder
    q = state->getJointRelativeOrientation(19);
    computeEulerAnglesFromQuaternion(q, y, x, z, alpha, beta, gamma);
    Quaternion yRot = getRotationQuaternion(alphaSlider2, y);
    Quaternion xRot = getRotationQuaternion(betaSlider2, x);
    Quaternion zRot = getRotationQuaternion(gammaSlider2, z);
    Quaternion q2 = zRot * yRot * xRot;
    state->setJointRelativeOrientation(q2, 19);

    rs.setJointRelativeOrientation(crl::getRotationQuaternion(gammaSlider, jointList[37]->rotationAxis), 37); //y
    rs.setJointRelativeOrientation(crl::getRotationQuaternion(alphaSlider, jointList[34]->rotationAxis), 34); //z
    rs.setJointRelativeOrientation(crl::getRotationQuaternion(betaSlider, jointList[31]->rotationAxis), 31); //x

    //left elbow
    q = state->getJointRelativeOrientation(16);
    computeEulerAnglesFromQuaternion(q, y, x, z, alpha, beta, gamma);
    rs.setJointRelativeOrientation(crl::getRotationQuaternion(beta, jointList[40]->rotationAxis), 40); //y
    rs.setJointRelativeOrientation(crl::getRotationQuaternion(gamma, jointList[38]->rotationAxis), 38); //x

    //right elbow
    q = state->getJointRelativeOrientation(20);
    computeEulerAnglesFromQuaternion(q, y, x, z, alpha, beta, gamma);
    rs.setJointRelativeOrientation(crl::getRotationQuaternion(beta, jointList[41]->rotationAxis), 41); //y
    rs.setJointRelativeOrientation(crl::getRotationQuaternion(gamma, jointList[39]->rotationAxis), 39); //x

    //left wrist
    q = state->getJointRelativeOrientation(17);
    computeEulerAnglesFromQuaternion(q, y, x, z, alpha, beta, gamma);
    rs.setJointRelativeOrientation(crl::getRotationQuaternion(-alpha, jointList[44]->rotationAxis), 44); //z
    //rs.setJointRelativeOrientation(crl::getRotationQuaternion(0, jointList[42]->rotationAxis), 42); //x, looks weird regardless of alpha, beta, gamma

    //right wrist
    q = state->getJointRelativeOrientation(21);
    computeEulerAnglesFromQuaternion(q, y, x, z, alpha, beta, gamma);
    rs.setJointRelativeOrientation(crl::getRotationQuaternion(-alpha, jointList[45]->rotationAxis), 45); //z
    //rs.setJointRelativeOrientation(crl::getRotationQuaternion(0, jointList[43]->rotationAxis), 43); //x, looks weird regardless of alpha, beta, gamma

    setState(&rs);
}

void Robot::fixJointConstraints() {
    for (size_t j = 0; j < jointList.size(); j++)
        jointList[j]->fixJointConstraints(true, true);
}

P3D Robot::computeCOM() {
    P3D COM = root->state.pos * root->rbProps.mass;
    double totalMass = root->rbProps.mass;

    for (uint i = 0; i < jointList.size(); i++) {
        totalMass += jointList[i]->child->rbProps.mass;
        COM +=
            jointList[i]->child->state.pos * jointList[i]->child->rbProps.mass;
    }

    return COM / totalMass;
}

double Robot::getMass() {
    // compute the mass of the robot
    double mass = root->rbProps.mass;
    for (uint i = 0; i < jointList.size(); i++)
        mass += jointList[i]->child->rbProps.mass;
    return mass;
}

void Robot::setRootState(const P3D &position, const Quaternion &orientation) {
    RobotState state(this);
    populateState(&state);
    state.setPosition(position);
    state.setOrientation(orientation);
    setState(&state);
}

void Robot::loadReducedStateFromFile(const char *fName) {
    RobotState state(this);
    state.readFromFile(fName);
    setState(&state);
}

void Robot::saveReducedStateToFile(const char *fName) {
    RobotState state(this);
    state.writeToFile(fName, this);
}

RB *Robot::getRBByName(const char *jName) {
    for (uint i = 0; i < jointList.size(); i++) {
        if (strcmp(jointList[i]->parent->name.c_str(), jName) == 0)
            return jointList[i]->parent;
        if (strcmp(jointList[i]->child->name.c_str(), jName) == 0)
            return jointList[i]->child;
    }
    std::cout
        << "WARNING: Robot:getRBByName -> rigid body could not be found..."
        << std::endl;
    return nullptr;
}

}  // namespace crl
