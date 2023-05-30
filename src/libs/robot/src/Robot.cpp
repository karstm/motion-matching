#include <robot/GeneralizedCoordinatesRobotRepresentation.h>
#include <robot/RBJoint.h>
#include <robot/RBLoader.h>
#include <robot/Robot.h>
#include <robot/RobotState.h>
#include "crl-basic/gui/mxm_utils.h"
#include "crl-basic/utils/mathUtils.h"
#include "crl-basic/utils/mathDefs.h"

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
    root->state.pos = state->getPosition();
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
    rs.setPosition(state->getRootPosition() + state->getRootOrientation() * state->getJointTranslation(1));
    rs.setOrientation(state->getRootOrientation() * state->getJointRelativeOrientation(1) * getRotationQuaternion(-PI / 2, V3D(0, 0, 1)) *
                      getRotationQuaternion(-PI / 2, V3D(0, 1, 0)));

    double alpha, beta, gamma;
    crl::V3D x = V3D(1, 0, 0);
    crl::V3D y = V3D(0, 1, 0);
    crl::V3D z = V3D(0, 0, 1);
    Quaternion q;

    //Quaternion xRot = getRotationQuaternion(alphaSlider2, x);
    //Quaternion yRot = getRotationQuaternion(betaSlider2, y);
    //Quaternion zRot = getRotationQuaternion(gammaSlider2, z);
    //Quaternion q2 = zRot * yRot * xRot;
    //Quaternion q3 = xRot * yRot * zRot;
    //state->setJointRelativeOrientation(q2, 1);

    //left hip
    q = state->getJointRelativeOrientation(2);
    computeEulerAnglesFromQuaternion(q, y, x, z, alpha, beta, gamma);
    rs.setJointRelativeOrientation(crl::getRotationQuaternion(alpha + M_PI, jointList[7]->rotationAxis), 7); //z
    rs.setJointRelativeOrientation(crl::getRotationQuaternion(beta, jointList[4]->rotationAxis), 4); //y
    rs.setJointRelativeOrientation(crl::getRotationQuaternion(gamma, jointList[1]->rotationAxis), 1); //x

    //right hip
    q = state->getJointRelativeOrientation(6);
    computeEulerAnglesFromQuaternion(q, y, x, z, alpha, beta, gamma);
    rs.setJointRelativeOrientation(crl::getRotationQuaternion(alpha + M_PI, jointList[8]->rotationAxis), 8); //z
    rs.setJointRelativeOrientation(crl::getRotationQuaternion(beta, jointList[5]->rotationAxis), 5); //y
    rs.setJointRelativeOrientation(crl::getRotationQuaternion(gamma, jointList[2]->rotationAxis), 2); //x

    //left knee
    q = state->getJointRelativeOrientation(3);
    computeEulerAnglesFromQuaternion(q, -x, y, -z, alpha, beta, gamma);
    rs.setJointRelativeOrientation(crl::getRotationQuaternion(alpha, jointList[16]->rotationAxis), 16); //y
    rs.setJointRelativeOrientation(crl::getRotationQuaternion(beta, jointList[13]->rotationAxis), 13);  //z
    rs.setJointRelativeOrientation(crl::getRotationQuaternion(gamma, jointList[10]->rotationAxis), 10); //x

    //right knee
    q = state->getJointRelativeOrientation(7);
    computeEulerAnglesFromQuaternion(q, -x, y, -z, alpha, beta, gamma);
    rs.setJointRelativeOrientation(crl::getRotationQuaternion(alpha, jointList[17]->rotationAxis), 17); //y
    rs.setJointRelativeOrientation(crl::getRotationQuaternion(beta, jointList[14]->rotationAxis), 14);  //z
    rs.setJointRelativeOrientation(crl::getRotationQuaternion(gamma, jointList[11]->rotationAxis), 11); //x

    //left foot
    q = state->getJointRelativeOrientation(4);
    computeEulerAnglesFromQuaternion(q, -z, y, -x, alpha, beta, gamma);
    rs.setJointRelativeOrientation(crl::getRotationQuaternion(alpha + 0.47 * M_PI, jointList[31]->rotationAxis), 31); //x
    rs.setJointRelativeOrientation(crl::getRotationQuaternion(beta, jointList[26]->rotationAxis), 26); //z
    rs.setJointRelativeOrientation(crl::getRotationQuaternion(gamma, jointList[21]->rotationAxis), 21); //y

    //right foot
    q = state->getJointRelativeOrientation(8);
    computeEulerAnglesFromQuaternion(q, -z, y, -x, alpha, beta, gamma);
    rs.setJointRelativeOrientation(crl::getRotationQuaternion(alpha + 0.47 * M_PI, jointList[32]->rotationAxis), 32); //x
    rs.setJointRelativeOrientation(crl::getRotationQuaternion(beta, jointList[27]->rotationAxis), 27); //z
    rs.setJointRelativeOrientation(crl::getRotationQuaternion(gamma, jointList[22]->rotationAxis), 22); //y

    //left toe
    q = state->getJointRelativeOrientation(5);
    computeEulerAnglesFromQuaternion(q, x, y, -z, alpha, beta, gamma);
    rs.setJointRelativeOrientation(crl::getRotationQuaternion(gamma, jointList[36]->rotationAxis), 36); //x

    //right toe
    q = state->getJointRelativeOrientation(9);
    computeEulerAnglesFromQuaternion(q, x, y, -z, alpha, beta, gamma);
    rs.setJointRelativeOrientation(crl::getRotationQuaternion(gamma, jointList[37]->rotationAxis), 37); //x

    //lower back
    q = state->getJointRelativeOrientation(10);
    computeEulerAnglesFromQuaternion(q, y, x, z, alpha, beta, gamma);
    rs.setJointRelativeOrientation(crl::getRotationQuaternion(alpha, jointList[6]->rotationAxis), 6);  //z
    rs.setJointRelativeOrientation(crl::getRotationQuaternion(beta, jointList[3]->rotationAxis), 3); //y
    rs.setJointRelativeOrientation(crl::getRotationQuaternion(gamma, jointList[0]->rotationAxis), 0); //x

    //upper back
    q = state->getJointRelativeOrientation(11) * state->getJointRelativeOrientation(12);
    computeEulerAnglesFromQuaternion(q, y, x, z, alpha, beta, gamma);
    rs.setJointRelativeOrientation(crl::getRotationQuaternion(alpha, jointList[15]->rotationAxis), 15);  //x
    rs.setJointRelativeOrientation(crl::getRotationQuaternion(beta, jointList[12]->rotationAxis), 12);  //y
    rs.setJointRelativeOrientation(crl::getRotationQuaternion(gamma, jointList[9]->rotationAxis), 9); //z

    //lower neck
    q = state->getJointRelativeOrientation(13);
    computeEulerAnglesFromQuaternion(q, y, x, z, alpha, beta, gamma);
    rs.setJointRelativeOrientation(crl::getRotationQuaternion(alpha, jointList[28]->rotationAxis), 28);  //x
    rs.setJointRelativeOrientation(crl::getRotationQuaternion(beta, jointList[23]->rotationAxis), 23);  //y
    rs.setJointRelativeOrientation(crl::getRotationQuaternion(gamma, jointList[18]->rotationAxis), 18);   //z

    //upper neck
    q = state->getJointRelativeOrientation(14);
    computeEulerAnglesFromQuaternion(q, y, x, z, alpha, beta, gamma);
    rs.setJointRelativeOrientation(crl::getRotationQuaternion(alpha, jointList[41]->rotationAxis), 41);  //z
    rs.setJointRelativeOrientation(crl::getRotationQuaternion(beta, jointList[38]->rotationAxis), 38);  //y
    rs.setJointRelativeOrientation(crl::getRotationQuaternion(gamma, jointList[33]->rotationAxis), 33);   //x
    
    //left shoulder
    q = state->getJointRelativeOrientation(15) * state->getJointRelativeOrientation(16);

    //Quaternion qMocapToRb = getRotationQuaternion(M_PI, z);
    //Quaternion qRbToMocap = qMocapToRb.inverse();
    //Eigen::Vector3d axisRbToMocap = qRbToMocap.vec().normalized();
    //float angleRbToMocap = getRotationAngle(qRbToMocap, V3D(axisRbToMocap));
    //axisRbToMocap = q * axisRbToMocap;
    //qRbToMocap = getRotationQuaternion(angleRbToMocap, V3D(axisRbToMocap));
    //q = qRbToMocap * q;

    computeEulerAnglesFromQuaternion(q, z, x, y, alpha, beta, gamma);
    rs.setJointRelativeOrientation(crl::getRotationQuaternion(alpha + M_PI, jointList[42]->rotationAxis), 42); //z
    rs.setJointRelativeOrientation(crl::getRotationQuaternion(beta, jointList[39]->rotationAxis), 39);  //y
    rs.setJointRelativeOrientation(crl::getRotationQuaternion(gamma, jointList[34]->rotationAxis), 34); //x

    //right shoulder
    q = state->getJointRelativeOrientation(19) * state->getJointRelativeOrientation(20);

    //qMocapToRb = getRotationQuaternion(M_PI, z);
    //qRbToMocap = qMocapToRb.inverse();
    //axisRbToMocap = qRbToMocap.vec().normalized();
    //angleRbToMocap = getRotationAngle(qRbToMocap, V3D(axisRbToMocap));
    //axisRbToMocap = q * axisRbToMocap;
    //qRbToMocap = getRotationQuaternion(angleRbToMocap, V3D(axisRbToMocap));
    //q = qRbToMocap * q;

    computeEulerAnglesFromQuaternion(q, z, x, y, alpha, beta, gamma);
    rs.setJointRelativeOrientation(crl::getRotationQuaternion(alpha + M_PI, jointList[43]->rotationAxis), 43); //z
    rs.setJointRelativeOrientation(crl::getRotationQuaternion(beta, jointList[40]->rotationAxis), 40); //y
    rs.setJointRelativeOrientation(crl::getRotationQuaternion(gamma, jointList[35]->rotationAxis), 35); //x

    //left elbow
    q = state->getJointRelativeOrientation(17);
    computeEulerAnglesFromQuaternion(q, -y, -x, z, alpha, beta, gamma);
    rs.setJointRelativeOrientation(crl::getRotationQuaternion(alpha, jointList[48]->rotationAxis), 48); //z
    rs.setJointRelativeOrientation(crl::getRotationQuaternion(beta, jointList[46]->rotationAxis), 46); //y
    rs.setJointRelativeOrientation(crl::getRotationQuaternion(gamma, jointList[44]->rotationAxis), 44);  //x
    crl::Logger::consolePrint("%f %f %f\n", alpha, beta, gamma);

    //right elbow
    q = state->getJointRelativeOrientation(21);
    computeEulerAnglesFromQuaternion(q, -y, -x, z, alpha, beta, gamma);
    rs.setJointRelativeOrientation(crl::getRotationQuaternion(alpha, jointList[49]->rotationAxis), 49); //z
    rs.setJointRelativeOrientation(crl::getRotationQuaternion(beta, jointList[47]->rotationAxis), 47); //y
    rs.setJointRelativeOrientation(crl::getRotationQuaternion(gamma, jointList[45]->rotationAxis), 45);  //x

    //left wrist
    q = state->getJointRelativeOrientation(18);
    computeEulerAnglesFromQuaternion(q, -x, -y, z, alpha, beta, gamma);
    rs.setJointRelativeOrientation(crl::getRotationQuaternion(alpha, jointList[54]->rotationAxis), 54);  //y
    rs.setJointRelativeOrientation(crl::getRotationQuaternion(beta, jointList[52]->rotationAxis), 52); //z
    rs.setJointRelativeOrientation(crl::getRotationQuaternion(gamma, jointList[50]->rotationAxis), 50); //x

    //right wrist
    q = state->getJointRelativeOrientation(22);
    computeEulerAnglesFromQuaternion(q, -x, -y, z, alpha, beta, gamma);
    rs.setJointRelativeOrientation(crl::getRotationQuaternion(alpha, jointList[55]->rotationAxis), 55);  //y
    rs.setJointRelativeOrientation(crl::getRotationQuaternion(beta, jointList[53]->rotationAxis), 53); //z
    rs.setJointRelativeOrientation(crl::getRotationQuaternion(gamma, jointList[51]->rotationAxis), 51); //x

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
