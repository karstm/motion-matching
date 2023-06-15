#include <tuple>
#include "crl-basic/gui/controller.h"

namespace crl {
namespace gui {

void Controller::init(KeyboardState *keyboardState, std::vector<std::unique_ptr<crl::mocap::BVHClip>> *clips, Footlocking *footLocking, const char *dataPath, int targetFramerate) {
    this->keyboardState = keyboardState;
    this->clips = clips;
    this->footLocking = footLocking;
    this->targetFramerate = targetFramerate;
    motionMatchingRate = targetFramerate/5;

    // initialize controller
    for (int i = 0; i < 4; i++) {
        controllerPos.push_back(P3D(0, flatTerrain ? 0 : 0.624483, 0));
        controllerRot.push_back(M_PI);
    }
    simulationPos = controllerPos[0];
    simulationRot = getRotationQuaternion(M_PI_2, V3D(0, 1, 0));
    vel = V3D(0, 0, 0);
    acc = V3D(0, 0, 0);
    angVel = 0.0;
    for (int i=0; i<(targetFramerate/10)+1; i++)
    {
        oldVerticalDir.push_back(0.0);
        oldHorizontalDir.push_back(0.0);
        oldSpeed.push_back(0.0);
    }

    // initialize inertialization info
    numMarkers = clips->at(0)->getModel()->getMarkerCount();
    rootPosInertializationInfo = InertializationInfo();
    rootOrientInertializationInfo = InertializationInfo();
    for (uint i = 0; i < numMarkers; i++) {
        jointPositionInertializationInfos.push_back(InertializationInfo());
        jointOrientInertializationInfos.push_back(InertializationInfo());
    }
    rootPosInertializationInfo_footLocking = InertializationInfo();
    rootOrientInertializationInfo_footLocking = InertializationInfo();
    for (uint i = 0; i < numMarkers; i++) {
        jointPositionInertializationInfos_footLocking.push_back(InertializationInfo());
        jointOrientInertializationInfos_footLocking.push_back(InertializationInfo());
    }

    // initialize states
    for (int i = 0; i < 3 ; i++) {
        mocap::MocapSkeletonState state = clips->at(0)->getState(frameIdx++);
        state.setRootPosition(simulationPos);
        state.setRootOrientation(simulationRot);
        motionStates.push_front(state);
    }

    lastMatchAnimationPos = clips->at(0)->getState(frameIdx - 1).getRootPosition();
    lastMatchAnimationRot = clips->at(0)->getState(frameIdx - 1).getRootOrientation();

    lastMatchSimulationPos = simulationPos;
    lastMatchSimulationRot = simulationRot;

    // initialize foot locking
    for (int i = 0; i < 3 ; i++) {
        mocap::MocapSkeletonState state = clips->at(0)->getState(frameIdx-3+i);
        state.setRootPosition(simulationPos);
        state.setRootOrientation(simulationRot);
        footLockingStates.push_front(state);
    }
    for(int i = 0; i < footMarkerNames.size(); i++){
        std::deque<bool> cHistory;
        contactHistories.push_back(cHistory);
        for(int j = 0; j < 2; j++){
            contactHistories[i].push_front(true);
        }
    }

    if(playerSkeleton != nullptr)
        delete playerSkeleton;
    
    std::string mocapPath = fs::path(dataPath).append("lafan1_filtered_60fps/aiming1_subject1_walk2_filtered.bvh").string();
    playerSkeleton = new crl::mocap::MocapSkeleton(mocapPath.c_str());
    playerSkeleton->setState(&motionStates[0]);

    // initialize time
    prevTime = std::chrono::steady_clock::now();
}

Controller::~Controller() {
    delete playerSkeleton;
}

void Controller::update(TrackingCamera &camera, Database &database) {
    // Motion Matching
    bool transition = false; //a transition only occurs if the motion matching algorithm changes the clip or frame index
    if(motionMatchingFrameCount >= motionMatchingRate || forceMatch) {
        // prepare query
        std::vector<P3D> simulationPositions = {simulationPos, controllerPos[1], controllerPos[2], controllerPos[3]};
        std::vector<P3D> trajectoryPos = MxMUtils::worldToLocalPositions(simulationPositions, getRotationQuaternion(M_PI_2, V3D(0,1,0)) * simulationRot);

        float simulationAngle = MxMUtils::getYrotationRadians(simulationRot) + M_PI_2;
        std::vector<float> simulationAngles = {simulationAngle, controllerRot[1], controllerRot[2], controllerRot[3]};
        std::vector<float> trajectoryAngle = MxMUtils::worldToLocalDirectionsAngle(simulationAngles);
        std::vector<V3D> trajectoryDir;
        for (int i = 0; i < trajectoryAngle.size(); i++) {
            trajectoryDir.push_back(V3D(sin(trajectoryAngle[i]), 0, cos(trajectoryAngle[i])));
        }

        // save the last clip and frame index to check if a transition occured
        int lastclipIdx = clipIdx;
        int lastFrameIdx = frameIdx;

        // match the previous frame for better transition
        frameIdx--;
        database.match(trajectoryPos, trajectoryDir, clipIdx, frameIdx);
        frameIdx++;

        // check if a transition occured
        transition = (lastclipIdx != clipIdx) || (lastFrameIdx != frameIdx);
        
        // reset the frame count
        motionMatchingFrameCount = 0;
    }
    // input handling
    getInput(camera);
    updateControllerTrajectory();
    camera.processRotation(dt);
    camera.target = MxMUtils::V3Dtovec3(V3D(controllerPos[0]));
  
    // copy the state for the current clip and frame
    mocap::MocapSkeletonState prevState = clips->at(clipIdx)->getState(frameIdx-1);
    mocap::MocapSkeletonState state = clips->at(clipIdx)->getState(frameIdx);

    // compute simulation bone position and orientation
    simulationRot = simulationRot * prevState.getRootOrientation().inverse() * state.getRootOrientation();
    simulationPos = simulationPos + simulationRot * prevState.getRootOrientation().inverse() * (V3D(prevState.getRootPosition(), state.getRootPosition()));

    // set root position and orientation to the lerp between controller position and simulation position
    P3D finalPos = MxMUtils::lerp(simulationPos, controllerPos[0], syncFactor*syncFactor*syncFactor*syncFactor);
    Quaternion controllerOrientation = getRotationQuaternion(controllerRot[0] - M_PI_2, V3D(0, 1, 0));
    Quaternion finalRot = MxMUtils::quatNlerpShortest(simulationRot, controllerOrientation, syncFactor*syncFactor*syncFactor*syncFactor);
    
    simulationRot = finalRot;
    simulationPos = finalPos;

    // Setting the possibly synchronized root position and orientation
    state.setRootOrientation(finalRot);
    state.setRootPosition(finalPos);

    // update the state queue
    // 0 = current state, 1 = previous state, 2 = previous previous state
    motionStates.push_front(state);
    motionStates.pop_back();

    // if a transition happened, compute inertialization info
    if(transition)
    {   
        InertializationUtils::computeInertializationInfo(rootPosInertializationInfo, rootOrientInertializationInfo, jointPositionInertializationInfos, jointOrientInertializationInfos, numMarkers, motionStates[2], motionStates[1], motionStates[0], transitionTime, dt); //here we use the old dt
        t = 0;
    }

    // time keeping
    t += dt;
    currTime = std::chrono::steady_clock::now();
    dt = (std::chrono::duration_cast<std::chrono::milliseconds> (currTime - prevTime)).count()/1000.0f;
    prevTime = currTime;

    // inertialization
    if(useInertialization)
        motionStates[0] = InertializationUtils::inertializeState(rootPosInertializationInfo, rootOrientInertializationInfo, jointPositionInertializationInfos, jointOrientInertializationInfos, numMarkers, motionStates[0], motionStates[1], t, dt); // here we use the new dt
    
    // Update the sim bone y position based on the terrain
    P3D oldPos = motionStates[0].getRootPosition();
    oldPos.y = flatTerrain ? 0 : sBoneTerrainPos.y;
    motionStates[0].setRootPosition(oldPos);

    // Finally the state is set for the skeleton
    // If footlocking is not used then we end the state computation here
    // Else we set the inertialized state here to use it for footlocking and IK
    playerSkeleton->setState(&motionStates[0]);

    // Update Footlocking and use partial IK
    if(useFootLocking){
        updateFootLocking();
    }else{
        footLockingStates.push_front(motionStates[0]);
        footLockingStates.pop_back();
    }

    // update frame
    frameIdx++;
    motionMatchingFrameCount++;
}

void Controller::drawSkeleton(const Shader &shader)
{
    playerSkeleton->draw(shader);
    // clips->at(clipIdx)->drawState(shader, &footLockedStates[0]);
}

void Controller::drawTrajectory(const Shader &shader, Database &database, bool drawControllerTrajectory, bool drawAnimationTrajectory) {
    // compute trajectory
    float dbEntry[27];
    database.getEntry(clipIdx, frameIdx, dbEntry);
    P3D p0 = simulationPos;
    V3D p1 = V3D(dbEntry[0], 0, dbEntry[1]);
    V3D p2 = V3D(dbEntry[2], 0, dbEntry[3]);
    V3D p3 = V3D(dbEntry[4], 0, dbEntry[5]);
    V3D d1 = V3D(dbEntry[6], 0, dbEntry[7]);
    V3D d2 = V3D(dbEntry[8], 0, dbEntry[9]);
    V3D d3 = V3D(dbEntry[10], 0, dbEntry[11]);

    Quaternion q0 = getRotationQuaternion(controllerRot[0], V3D(0, 1, 0));
    std::vector<P3D> animationPos;
    animationPos.push_back(p0);
    animationPos.push_back(p0 + q0 * p1);
    animationPos.push_back(p0 + q0 * p2);
    animationPos.push_back(p0 + q0 * p3);

    std::vector<V3D> animationDirections;
    animationDirections.push_back(q0 * d1);
    animationDirections.push_back(q0 * d2);
    animationDirections.push_back(q0 * d3);
    std::vector<float> trajectoryAngle = MxMUtils::worldToLocalDirectionsAngle(controllerRot);
    std::vector<V3D> directions(trajectoryAngle.size()+1);
    directions[0] = q0 * V3D(0,0,1);
    for (int i = 0; i < trajectoryAngle.size(); i++) {
        directions[i + 1] = q0 * V3D(sin(trajectoryAngle[i]), 0, cos(trajectoryAngle[i]));
    }

    // draw trajectory
    crl::gui::drawSphere(controllerPos[0], 0.05, shader, V3D(1, 0.5, 0), 1.0);
    crl::gui::drawArrow3d(controllerPos[0], directions[0]*0.75, 0.005, shader, V3D(1, 0, 1), 0.5);
    for (int i = 0; i < controllerPos.size() - 1; i++) {
        if(drawControllerTrajectory)
        {
            crl::gui::drawSphere(controllerPos[i+1], 0.03, shader, V3D(1, 0.5, 0), 1.0);
            crl::gui::drawCapsule(controllerPos[i], controllerPos[i + 1], 0.01, shader, V3D(1, 0.5, 0), 1.0);
            crl::gui::drawArrow3d(controllerPos[i+1], directions[i+1] * 0.75, 0.005, shader, V3D(1, 0, 1), 0.5);
        }

        if(drawAnimationTrajectory)
        {
            crl::gui::drawSphere(animationPos[i+1], 0.03, shader, V3D(0, 0.5, 1), 1.0);
            crl::gui::drawCapsule(animationPos[i], animationPos[i + 1], 0.01, shader, V3D(0, 0.5, 1), 1.0);
            crl::gui::drawArrow3d(animationPos[i+1], animationDirections[i]*0.75, 0.005, shader, V3D(1, 1, 0), 0.5);
        }
    }
}

// updates the desired velocity and rotation from the player input
void Controller::getInput(TrackingCamera &camera){
    // Check if a joystick is connected
    int controllerId;
    bool found_controller = false;
    for(controllerId = 0; controllerId < GLFW_JOYSTICK_LAST; controllerId++) {
        if(glfwJoystickIsGamepad(controllerId)) {
            found_controller = true;
            break;
        }
    }

    // Read the input
    float verticalDir = 0, horizontalDir = 0;
    if(!found_controller) {
        verticalDir = keyboardState->at(GLFW_KEY_W); 
        verticalDir -= keyboardState->at(GLFW_KEY_S);
        horizontalDir = keyboardState->at(GLFW_KEY_D);
        horizontalDir -= keyboardState->at(GLFW_KEY_A);
        strafe = keyboardState->at(GLFW_KEY_LEFT_ALT);
        run = keyboardState->at(GLFW_KEY_LEFT_SHIFT);
    } else {
        int buttonCount, axisCount;
        const unsigned char *buttons = glfwGetJoystickButtons(controllerId, &buttonCount);
        const float *axis = glfwGetJoystickAxes(controllerId, &axisCount);
        strafe = buttons[GLFW_GAMEPAD_BUTTON_RIGHT_BUMPER];
        run = buttons[GLFW_GAMEPAD_BUTTON_SQUARE]; //for some reason this is the circle button

        if(V3D(axis[0], 0, axis[1]).norm() > 0.15) {   
            horizontalDir = axis[0]; 
            verticalDir = -axis[1]; 
        }
    }


    // compute desired velocity and rotation
    if (verticalDir != 0 || horizontalDir != 0) {
        V3D cameraDir = MxMUtils::vec3toV3D(camera.target - camera.position());
        cameraDir.y() = 0;
        cameraDir = cameraDir.unit();
        velDesired = cameraDir * verticalDir + cameraDir.cross(V3D(0, 1, 0)) * horizontalDir;
        if(!found_controller) {
            velDesired = velDesired.unit();
        }

        velDesired *= run? runSpeed : walkSpeed;
        rotDesired = strafe? controllerRot[0] : MxMUtils::angleBetweenVectors(V3D(0, 0, 1), velDesired);
    } else {
        velDesired = V3D(0, 0, 0);
        rotDesired = controllerRot[0];
    }

    oldHorizontalDir.push_front(horizontalDir);
    oldVerticalDir.push_front(verticalDir);
    oldHorizontalDir.pop_back();
    oldVerticalDir.pop_back();

    V3D oldDirection = V3D(oldHorizontalDir.back(), 0, oldVerticalDir.back()).normalized();
    V3D newDirection = V3D(horizontalDir, 0, verticalDir).normalized();
    
    oldSpeed.push_front(velDesired.norm());
    oldSpeed.pop_back();

    forceMatch = MxMUtils::angleBetweenVectors(oldDirection, newDirection) > M_PI_4/2.0; //Force Match angle is 22.5 degrees 
    forceMatch = forceMatch || abs(oldSpeed.front() - oldSpeed.back()) > runSpeed/16.0; 

    if(forceMatch) {
        for (int i = 0; i < oldDirection.size(); i++) {
            oldHorizontalDir[i] = horizontalDir;
            oldVerticalDir[i] = verticalDir;
            oldSpeed[i] = velDesired.norm();
        }
    }

}

// updates the controller position and rotation using the spring-damper model
void Controller::updateControllerTrajectory()
{
    P3D posPrev = controllerPos[0];
    V3D velPrev = vel;
    V3D accPrev = acc;
    float rotPrev = controllerRot[0];
    float angVelPrev = angVel;
    float T;
    
    for (int t = 0; t < controllerPos.size(); t++) { // for each timestep
        if (t == 0) {
            T = dt; // take into account the speed at which the loop runs to calculate the actual position in the next frame
        } else {
            T = dt + 1.0 / 3.0 * t; // predict future positions at intervals of 0.33 s
        }
        
        // translation
        float expLambdaT = exp(-lambda * T);
        for (int x = 0; x < 3; x += 2) { // for the x and z coordinates
            float j0 = velPrev[x] - velDesired[x];
            float j1 = accPrev[x] + j0 * lambda;
            controllerPos[t][x] = expLambdaT * (((-j1) / (lambda * lambda)) + ((-j0 - j1 * T) / lambda)) + (j1 / (lambda * lambda)) + j0 / lambda + velDesired[x] * T +
                       posPrev[x];
            if (t == 0) {
                vel[x] = expLambdaT * (j0 + j1 * T) + velDesired[x];
                acc[x] = expLambdaT * (accPrev[x] - j1 * lambda * T);
            }
        }

        // CHECK Updated
        controllerPos[t][1] = flatTerrain ? 0 : sBoneTerrainPos[1];

        // rotation
        float expLambdaRotT = exp(-lambdaRot * T);
        float j0Rot = MxMUtils::minusPiToPi(rotPrev - rotDesired);
        float j1Rot = angVelPrev + j0Rot * lambdaRot;

        controllerRot[t] = expLambdaRotT * (j0Rot + j1Rot * T) + rotDesired;
        if (t == 0) {
            angVel = expLambdaRotT * (angVelPrev - j1Rot * lambdaRot * T);
        }
    }

}

void Controller::updateFootLocking() {
    // Only call this function after playerSkeleton->setState(&motionStates[0]) is called!

    bool lFootInContactPrev, rFootInContactPrev;
    bool lFootInContact, rFootInContact;
    std::tie(lFootInContactPrev, rFootInContactPrev) = footLocking->isFootInContact(clipIdx, frameIdx-1);
    std::tie(lFootInContact, rFootInContact) = footLocking->isFootInContact(clipIdx, frameIdx);

    mocap::MocapSkeletonState footLockingState = motionStates[0];
    P3D orgLFootPos = playerSkeleton->getMarkerByName(footLocking->lFoot.c_str())->state.pos;
    P3D orgRFootPos = playerSkeleton->getMarkerByName(footLocking->rFoot.c_str())->state.pos;

    if (lFootInContact && !contactHistories[0].at(0)) {
        crl::mocap::MocapMarker *lFootMarker = playerSkeleton->getMarkerByName(footLocking->lFoot.c_str());
        lFootLockedPos = lFootMarker->state.pos;
        lFootLockedPos[1] = flatTerrain ? 0 : lFootTerrainPos.y;
    }

    float lDist = V3D(lFootLockedPos, orgLFootPos).norm();
    bool usinglFootLocking = lFootInContact  && lDist < unlockRadius;

    if (usinglFootLocking) {
        Quaternion lHipRot, lKneeRot;
        V3D lHeelPos = V3D(playerSkeleton->getMarkerByName("LeftFoot")->state.pos);
        Quaternion lKneeRotOrg = playerSkeleton->getMarkerByName("LeftLeg")->state.orientation;
        footLocking->ikTwoBone(
            lHipRot, lKneeRot,
            V3D(playerSkeleton->getMarkerByName("LeftUpLeg")->state.pos),
            V3D(playerSkeleton->getMarkerByName("LeftLeg")->state.pos),
            V3D(playerSkeleton->getMarkerByName("LeftFoot")->state.pos),
            V3D(lFootLockedPos) + lHeelPos - V3D(playerSkeleton->getMarkerByName(footLocking->lFoot.c_str())->state.pos),
            lKneeRotOrg * V3D(0, 1, 0),
            playerSkeleton->getMarkerByName("LeftUpLeg")->state.orientation,
            lKneeRotOrg,
            playerSkeleton->getMarkerByName("Hips")->state.orientation,
            0.015f
        );

        footLockingState.setJointRelativeOrientation(lHipRot, 2);
        footLockingState.setJointRelativeOrientation(lKneeRot, 3);

        // Setting the left foot orientation correctly
        playerSkeleton->setState(&footLockingState);

        Quaternion lHeelRot = footLockingState.getJointRelativeOrientation(4);
        footLocking->ikLookAt(
            lHeelRot, 
            playerSkeleton->getMarkerByName("LeftLeg")->state.orientation,
            playerSkeleton->getMarkerByName("LeftFoot")->state.orientation,
            V3D(playerSkeleton->getMarkerByName("LeftFoot")->state.pos),
            V3D(playerSkeleton->getMarkerByName("LeftToe")->state.pos),
            V3D(lFootLockedPos)
        );

        footLockingState.setJointRelativeOrientation(lHeelRot, 4);
        playerSkeleton->setState(&footLockingState);

        V3D toe_end_curr = playerSkeleton->getMarkerByName("LeftToe")->state.orientation * V3D(0.15f, 0.0f, 0.0f);
        toe_end_curr = toe_end_curr + V3D(playerSkeleton->getMarkerByName("LeftToe")->state.pos);
                    
        V3D toe_end_targ = toe_end_curr;
        toe_end_targ[1] = std::max(toe_end_targ[1], 0.02);

        Quaternion lToeRot = footLockingState.getJointRelativeOrientation(5);
        footLocking->ikLookAt(
            lToeRot,
            playerSkeleton->getMarkerByName("LeftFoot")->state.orientation,
            playerSkeleton->getMarkerByName("LeftToe")->state.orientation,
            V3D(playerSkeleton->getMarkerByName("LeftToe")->state.pos),
            toe_end_curr,
            toe_end_targ
        );

        footLockingState.setJointRelativeOrientation(lToeRot, 5);
        playerSkeleton->getMarkerByName(footLocking->lFoot.c_str())->state.pos = lFootLockedPos;
    }

    if (rFootInContact && !contactHistories[1].at(0)) {
        crl::mocap::MocapMarker *rFootMarker = playerSkeleton->getMarkerByName(footLocking->rFoot.c_str());     
        rFootLockedPos = rFootMarker->state.pos;
        rFootLockedPos[1] = flatTerrain ? 0 : rFootTerrainPos.y;
        //stCurr.get
    }

    float rDist = V3D(rFootLockedPos, orgRFootPos).norm();
    bool usingrFootLocking = rFootInContact && rDist < unlockRadius;
    
    if (usingrFootLocking) {
        Quaternion rHipRot, rKneeRot;
        V3D rHeelPos = V3D(playerSkeleton->getMarkerByName("RightFoot")->state.pos);
        Quaternion rKneeRotOrg = playerSkeleton->getMarkerByName("RightLeg")->state.orientation;

        footLocking->ikTwoBone(
            rHipRot, rKneeRot,
            V3D(playerSkeleton->getMarkerByName("RightUpLeg")->state.pos),
            V3D(playerSkeleton->getMarkerByName("RightLeg")->state.pos),
            V3D(playerSkeleton->getMarkerByName("RightFoot")->state.pos),
            V3D(rFootLockedPos) + rHeelPos - V3D(playerSkeleton->getMarkerByName(footLocking->rFoot.c_str())->state.pos),
            rKneeRotOrg * V3D(0, 1, 0),
            playerSkeleton->getMarkerByName("RightUpLeg")->state.orientation,
            rKneeRotOrg,
            playerSkeleton->getMarkerByName("Hips")->state.orientation,
            0.015f
        );
        
        footLockingState.setJointRelativeOrientation(rHipRot, 6);
        footLockingState.setJointRelativeOrientation(rKneeRot, 7);

        // Setting the right foot orientation correctly
        playerSkeleton->setState(&footLockingState);

        Quaternion rHeelRot = footLockingState.getJointRelativeOrientation(8);
        footLocking->ikLookAt(
            rHeelRot,
            playerSkeleton->getMarkerByName("RightLeg")->state.orientation,
            playerSkeleton->getMarkerByName("RightFoot")->state.orientation,
            V3D(playerSkeleton->getMarkerByName("RightFoot")->state.pos),
            V3D(playerSkeleton->getMarkerByName("RightToe")->state.pos),
            V3D(rFootLockedPos)
        );

        footLockingState.setJointRelativeOrientation(rHeelRot, 8);
        playerSkeleton->setState(&footLockingState);

        V3D toe_end_curr = playerSkeleton->getMarkerByName("RightToe")->state.orientation * V3D(0.15f, 0.0f, 0.0f);
        toe_end_curr = toe_end_curr + V3D(playerSkeleton->getMarkerByName("RightToe")->state.pos);

        V3D toe_end_targ = toe_end_curr;
        toe_end_targ[1] = std::max(toe_end_targ[1], 0.02);

        Quaternion rToeRot = footLockingState.getJointRelativeOrientation(9);
        footLocking->ikLookAt(
            rToeRot,
            playerSkeleton->getMarkerByName("RightFoot")->state.orientation,
            playerSkeleton->getMarkerByName("RightToe")->state.orientation,
            V3D(playerSkeleton->getMarkerByName("RightToe")->state.pos),
            toe_end_curr,
            toe_end_targ
        );

        footLockingState.setJointRelativeOrientation(rToeRot, 9);
        playerSkeleton->getMarkerByName(footLocking->rFoot.c_str())->state.pos = rFootLockedPos;
    }

    footLockingStates.push_front(footLockingState);
    footLockingStates.pop_back();

    // contact transition happens only only one frame after the contact is made
    bool transition = usinglFootLocking != contactHistories[0].at(0) || usingrFootLocking != contactHistories[1].at(0);
    
    if (transition) {
        // set inertialization info
        InertializationUtils::computeInertializationInfo(rootPosInertializationInfo_footLocking, 
                                                         rootOrientInertializationInfo_footLocking, 
                                                         jointPositionInertializationInfos_footLocking, 
                                                         jointOrientInertializationInfos_footLocking, 
                                                         numMarkers, 
                                                         footLockingStates[2], 
                                                         footLockingStates[1], 
                                                         footLockingStates[0], 
                                                         transitionTime/2.0, 
                                                         dt); //here we use the old dt
        t_footLocking = 0;
    }

    if(useInertialization)
        footLockingStates[0] = InertializationUtils::inertializeState(rootPosInertializationInfo_footLocking,
                                                                      rootOrientInertializationInfo_footLocking, 
                                                                      jointPositionInertializationInfos_footLocking, 
                                                                      jointOrientInertializationInfos_footLocking, 
                                                                      numMarkers, 
                                                                      footLockingStates[0], 
                                                                      footLockingStates[1], 
                                                                      t_footLocking, 
                                                                      dt); // here we use the new dt

    t_footLocking += dt;
     
    //Logger::consolePrint("left: %d; right, %d; \n", lFootInContact, rFootInContact);

    contactHistories[0].push_front(usinglFootLocking);
    contactHistories[0].pop_back();
    contactHistories[1].push_front(usingrFootLocking);
    contactHistories[1].pop_back();

    playerSkeleton->setState(&footLockingStates[0]);
}

crl::P3D Controller::getPosByName(const char *name){
    if (playerSkeleton != nullptr) 
        return playerSkeleton->getMarkerByName(name)->state.pos + P3D(0, -5, 0);
    else
        return P3D();
}

void Controller::posTerrainAdjust(P3D simBonePos, P3D lFootPos, P3D rFootPos){
    if (playerSkeleton != nullptr){
        lFootTerrainPos = lFootPos;
        rFootTerrainPos = rFootPos;
        sBoneTerrainPos = simBonePos;
    }
}

}  // namespace gui
}  // namespace crl