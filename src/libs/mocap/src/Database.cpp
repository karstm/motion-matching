#include "mocap/Database.h"
#include "crl-basic/gui/mxm_utils.h"

// Empty constructor to allow for member initialization
Database::Database() {
    data = nullptr;
}

// Destructor frees the data array
Database::~Database() {
    delete[] data;
    delete annoyIndex;
}

void Database::initializeAnnoy() 
{
    #ifdef ANNOYLIB_MULTITHREADED_BUILD
        annoyIndex = new Annoy::AnnoyIndex<int, float, Annoy::Euclidean, Annoy::Kiss32Random, Annoy::AnnoyIndexMultiThreadedBuildPolicy>(noFeatures);
        crl::Logger::consolePrint("MultiThreaded\n");
    #else
        annoyIndex = new Annoy::AnnoyIndex<int, float, Annoy::Euclidean, Annoy::Kiss32Random, Annoy::AnnoyIndexSingleThreadedBuildPolicy>(noFeatures);
        crl::Logger::consolePrint("SingleThreaded\n");
    #endif

    char **error = (char**)malloc(sizeof(char*));
    for (int frame = 0; frame < frameSums.back(); frame++)
    {   
        annoyIndex->add_item(frame, data + frame * noFeatures, error);
    }
    annoyIndex->build(-1);
}

// Builds the database from the given vector of BVHClips can be used to rebuild the database
// Combined with set weights because set weights should never be called without rebuilding the database
void Database::build(float trajectoryPositionWeight, float trajectoryFacingWeight,
                     float footPositionWeight, float footVelocityWeight,
                     float hipVelocityWeight,
                     std::vector<std::unique_ptr<crl::mocap::BVHClip>>* bvhClips)
{
    //start timer
    crl::Logger::consolePrint("\nBuilding database...\n");
    auto start = std::chrono::high_resolution_clock::now();

    //set weights
    this->trajectoryPositionWeight = trajectoryPositionWeight;
    this->trajectoryFacingWeight = trajectoryFacingWeight;
    this->footPositionWeight = footPositionWeight;
    this->footVelocityWeight = footVelocityWeight;
    this->hipVelocityWeight = hipVelocityWeight;
    
    //clear old data
    frameSums.clear();
    delete[] data;

    //build new data
    readFrameSums(bvhClips);
    data = new float[frameSums.back() * noFeatures]{0};
    readData(bvhClips);
    
    //end timer
    auto end = std::chrono::high_resolution_clock::now();

    //print info
    crl::Logger::consolePrint("Database built with %d clips and %d frames\n", frameSums.size() - 1, frameSums.back());
    crl::Logger::consolePrint("Weights used:                    \
                                \n\tTrajectory Position:\t %f   \
                                \n\tTrajectory Direction:\t %f  \
                                \n\tFoot Position:\t %f         \
                                \n\tFoot Velocity:\t %f         \
                                \n\tHip Velocity:\t %f\n", 
                                this->trajectoryPositionWeight, this->trajectoryFacingWeight,
                                this->footPositionWeight, this->footVelocityWeight, this->hipVelocityWeight);
    crl::Logger::consolePrint("Database build time: %f seconds\n", std::chrono::duration_cast<std::chrono::duration<double>>(end - start).count());
    crl::Logger::consolePrint("Database size: %f MB\n", (frameSums.back() * noFeatures * sizeof(float)) / 1000000.0);

    initializeAnnoy();
    crl::Logger::consolePrint("Annoy Initialized!\n");
}

// Matches the given query to the mocap database and returns the clip id and frame number
// TODO: implement this
void Database::match(std::vector<crl::P3D>& trajectoryPositions, std::vector<float>& trajectoryAngles,
                    int& clip_id, int& frame, std::vector<crl::P3D> &actualTrajectory) 
{
    int lineNumber;
    int kNearest = 5;

    //Steps: 
    // - (arange the query in array) DONE
    // - (normalize the query) DONE
    // - get the line number of the nearest neighbor in the database
    // - (get the clip id and frame number from the line number) DONE

    // arange the query in array
    // crl::P3D& leftFootPosition;
    // crl::P3D& rightFootPosition;
    // crl::V3D& leftFootVelocity,
    // crl::V3D& rightFootVelocity, 
    // crl::V3D& hipVelocity,
    int line = (frameSums[clip_id] + frame)*noFeatures;
    float* currentInfo = data + line; 
    
    float query[] = {(float)trajectoryPositions[0].x, (float)trajectoryPositions[0].z, (float)trajectoryPositions[1].x, (float)trajectoryPositions[1].z, (float)trajectoryPositions[2].x, (float)trajectoryPositions[2].z,
                     trajectoryAngles[0], trajectoryAngles[1], trajectoryAngles[2],
                     0, 0, 0,
                     0, 0, 0,
                     0, 0, 0,
                     0, 0, 0,
                     0, 0, 0};
    
    // normalize the query
    normalize(query);

    // Writing the already normalized data into the query.
    int trajOffset = 9;
    for (int i = trajOffset; i < noFeatures; i++) {
        query[i] = currentInfo[i];
    }

    //TODO: get the line number of the nearest neighbor in the database
    std::vector<int> closest;
    annoyIndex->get_nns_by_vector(query, kNearest, -1, &closest, NULL);
    lineNumber = closest[0];

    // get the line number from the nearest neighbor
    getClipAndFrame(lineNumber, clip_id, frame);

    float* matchedInfo = data + lineNumber;

    for(int i = 0; i < 3; i++){
        actualTrajectory.push_back(crl::P3D(matchedInfo[i*2 + 0], 0, matchedInfo[i*2 + 1]));
    } 
}

// Normalizes the given data array and applies the weights
// TODO: test this
void Database::normalize(float* data) 
{
    for (int i = 0; i < noFeatures; i++) 
    {   
        // ugly hardcoding of weights
        float weight;
        if (i < 6)
            weight = trajectoryPositionWeight;
        else if (i < 9)
            weight = trajectoryFacingWeight;
        else if (i < 15)
            weight = footPositionWeight;
        else if (i < 21)
            weight = footVelocityWeight;
        else if (i < 24)
            weight = hipVelocityWeight;

        data[i] = ((data[i] - means[i]) / standardDeviations[i]) * weight;
    }
}

// Converts a line number to a clip id and frame number using a binary search on the frameSums vector
//TODO: test this, this might be off by 1, haven't tested it yet
bool Database::getClipAndFrame(int lineNumber, int& clip_id, int& frame) {
    if (lineNumber < 0 || lineNumber >= frameSums.back())
        return false;

    // search on the frameSums to find the clip id
    // decided against binary search since array is only about 60 entries max
    clip_id = 0;
    while (lineNumber > frameSums[clip_id + 1])
        clip_id++;

    //find the frame number
    frame = lineNumber - frameSums[clip_id];

    return true;
}

// Finds the data in the BVH clips and stores it in the data array
//TODO: test this
void Database::readData(std::vector<std::unique_ptr<crl::mocap::BVHClip>>* bvhClips) 
{
    for (int clipId  = 0; clipId < bvhClips->size(); clipId++) 
    {
        auto *sk = bvhClips->at(clipId)->getModel();
        for (int frame = 0; frame < frameSums[clipId + 1] - frameSums[clipId]; frame++)
        {
            sk->setState(&bvhClips->at(clipId)->getState(frame));
            auto sk1 = &bvhClips->at(clipId)->getState(frame+10);
            auto sk2 = &bvhClips->at(clipId)->getState(frame+20);
            auto sk3 = &bvhClips->at(clipId)->getState(frame+30);
            int offset = (frameSums[clipId] + frame) * noFeatures;

            getTrajectoryPositions(sk, sk1, sk2, sk3,  offset);
            getTrajectoryDirections(sk, sk1, sk2, sk3, offset + 6);
            getFootPosition(sk, 0, offset + 9);
            getFootPosition(sk, 1, offset + 12);
            getFootVelocity(sk, 0, offset + 15);
            getFootVelocity(sk, 1, offset + 18);
            getHipVelocity(sk, offset + 21);
        }
    }

    computeMeans();
    computeStandardDeviations();
    for(int line = 0; line < frameSums.back(); line++) {
        normalize(data + line * noFeatures);
    }
}

// Finds the frame sums for each clip and stores them in the frameSums vector
void Database::readFrameSums(std::vector<std::unique_ptr<crl::mocap::BVHClip>>* bvhClips) 
{
    int runningTotal = 0;
    frameSums.push_back(runningTotal);
    for (int i = 0; i < bvhClips->size(); i++) {
        runningTotal += (*bvhClips)[i]->getFrameCount() - ignoredEndFrames;
        frameSums.push_back(runningTotal);
    }
}

// Compute the trajectory position data 
//TODO: implement this
void Database::getTrajectoryPositions(crl::mocap::MocapSkeleton *sk, const crl::mocap::MocapSkeletonState *sk1, const crl::mocap::MocapSkeletonState *sk2, const crl::mocap::MocapSkeletonState *sk3,  int offset) {
    crl::P3D p0 = sk->root->state.pos;
    crl::Quaternion q0Inverse = (sk->root->state.orientation).inverse();
    crl::V3D p1 = q0Inverse*crl::V3D(p0, sk1->getRootPosition());
    crl::V3D p2 = q0Inverse*crl::V3D(p0, sk2->getRootPosition());
    crl::V3D p3 = q0Inverse*crl::V3D(p0, sk3->getRootPosition());

    data[offset+0] = p1[0];
    data[offset+1] = p1[2];
    data[offset+2] = p2[0];
    data[offset+3] = p2[2];
    data[offset+4] = p3[0];
    data[offset+5] = p3[2];
}

// Compute the trajectory direction data
// TODO: implement this

void printGamma(float gamma, int i){
    if(gamma <= -M_PI || gamma >= M_PI){
        crl::Logger::consolePrint("Gamma %d: %f\n", i, gamma);
    }
}
void Database::getTrajectoryDirections(crl::mocap::MocapSkeleton *sk, const crl::mocap::MocapSkeletonState *sk1, const crl::mocap::MocapSkeletonState *sk2, const crl::mocap::MocapSkeletonState *sk3, int offset) {
    crl::Quaternion q0 = sk->root->state.orientation;
    crl::Quaternion q1 = sk1->getRootOrientation();
    crl::Quaternion q2 = sk2->getRootOrientation();
    crl::Quaternion q3 = sk3->getRootOrientation();

    float gamma0 = crl::gui::MxMUtils::getYAngle(q0);
    float gamma1 = crl::gui::MxMUtils::getYAngle(q1);
    float gamma2 = crl::gui::MxMUtils::getYAngle(q2);
    float gamma3 = crl::gui::MxMUtils::getYAngle(q3);
    printGamma(gamma0, 0);
    printGamma(gamma1, 1);
    printGamma(gamma2, 2);
    printGamma(gamma3, 3);
    // printGamma(gamma1 - gamma0, 4);
    // printGamma(gamma2 - gamma0, 5);
    // printGamma(gamma3 - gamma0, 6);
    data[offset + 0] = crl::gui::MxMUtils::minusPiToPi(gamma1 - gamma0);
    data[offset + 1] = crl::gui::MxMUtils::minusPiToPi(gamma2 - gamma0);
    data[offset + 2] = crl::gui::MxMUtils::minusPiToPi(gamma3 - gamma0);
}

// Compute a feature for the position of a bone relative to the simulation/root bone
void Database::getFootPosition(crl::mocap::MocapSkeleton *sk, int foot, int offset) 
{
        const auto& name = footMarkerNames[foot];
        const auto joint = sk->getMarkerByName(name.c_str());

        //TODO: eepos seem to be correct needs to be tested further
        crl::P3D eepos = joint->state.getLocalCoordinates(sk->root->state.getWorldCoordinates(crl::P3D(0,0,0)));

        data[offset + 0] = eepos.x;
        data[offset + 1] = eepos.y;
        data[offset + 2] = eepos.z;
}

// Compute a feature for the position of a bone relative to the simulation/root bone
void Database::getFootVelocity(crl::mocap::MocapSkeleton *sk, int foot, int offset) 
{
        const auto& name = footMarkerNames[foot];
        const auto joint = sk->getMarkerByName(name.c_str());

        //TODO: eevel seems to be correct needs to be tested further
        crl::V3D eevel = joint->state.getVelocityForPoint_local(joint->endSites[0].endSiteOffset);

        data[offset + 0] = eevel(0);
        data[offset + 1] = eevel(1);
        data[offset + 2] = eevel(2);
}

// Compute the hip velocity
// TODO: test this
void Database::getHipVelocity(crl::mocap::MocapSkeleton *sk, int offset) {
    double roll = 0, pitch = 0, yaw = 0;
    crl::computeEulerAnglesFromQuaternion(sk->root->state.orientation,                                     //
                                          sk->forwardAxis, sk->upAxis.cross(sk->forwardAxis), sk->upAxis,  //
                                          roll, pitch, yaw);
    crl::Quaternion heading = crl::getRotationQuaternion(yaw, sk->upAxis);
    double turning = sk->root->state.angularVelocity.dot(sk->upAxis);
    double forward = (heading.inverse() * sk->root->state.velocity).dot(sk->forwardAxis);
    double sideways = (heading.inverse() * sk->root->state.velocity).dot(sk->upAxis.cross(sk->forwardAxis));

    data[offset + 0] = turning;
    data[offset + 1] = forward;
    data[offset + 2] = sideways;
}

// Computes the means for each feature and stores them in the means vector
//TODO: test this
void Database::computeMeans() 
{
    means = std::vector<float>(noFeatures, 0);
    for (int line = 0; line < frameSums.back(); line++) {
        for (int i = 0; i < noFeatures; i++) {
            means[i] += data[line * noFeatures + i];
        }
    }

    for (int i = 0; i < noFeatures; i++) {
        means[i] /= frameSums.back();
    }
}

// Computes the standard deviations for each feature and stores them in the standardDeviations vector
// TODO: test this
void Database::computeStandardDeviations()
{
    standardDeviations = std::vector<float>(noFeatures, 0);
    for (int line = 0; line < frameSums.back(); line++) {
        for (int i = 0; i < noFeatures; i++) {
            standardDeviations[i] += pow(data[line * noFeatures + i] - means[i], 2);
        }
    }
    for (int i = 0; i < noFeatures; i++) {
        standardDeviations[i] = sqrt(standardDeviations[i]/ frameSums.back());
        assert(standardDeviations[i] != 0);
    }
}
