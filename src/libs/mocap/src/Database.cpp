#undef NDEBUG

#include "mocap/Database.h"
#include "crl-basic/gui/mxm_utils.h"
#include <algorithm>
#include <cassert>

// Empty constructor to allow for member initialization
Database::Database() {
    data = nullptr;
    annoyIndex = nullptr;
}

// Destructor frees the data array
Database::~Database() {
    if(data != nullptr)
        delete[] data;
    if (annoyIndex != nullptr)
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
    
    for (int clipId = 0; clipId < frameSums.size() - 1; clipId++) {
        for (int frame = frameSums[clipId] + 1; frame < frameSums[clipId + 1] - endFramesWhereIgnoreMatching; frame++) {
            annoyIndex->add_item(frame, data + frame * noFeatures, error);
        }
    }
    annoyIndex->build(-1);
}

// Builds the database from the given vector of BVHClips can be used to rebuild the database
// Combined with set weights because set weights should never be called without rebuilding the database
void Database::build(float trajectoryPositionWeight, float trajectoryFacingWeight,
                     float footPositionWeight, float footVelocityWeight,
                     float hipVelocityWeight,
                     std::vector<std::unique_ptr<crl::mocap::BVHClip>>* bvhClips, int targetFramerate)
{
    this->targetFramerate = targetFramerate;

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
void Database::match(std::vector<crl::P3D>& trajectoryPositions, std::vector<crl::V3D>& trajectoryDirections,
                    int& clip_id, int& frame) 
{
    int lineNumber;
    int kNearest = 1;

    // arange the query in array
    // x-z-coordinates for trajectory position for 3 frames
    // x-z-coordinates for trajectory direction for 3 frames
    // crl::P3D& leftFootPosition;
    // crl::P3D& rightFootPosition;
    // crl::V3D& leftFootVelocity,
    // crl::V3D& rightFootVelocity, 
    // crl::V3D& hipVelocity,
    int line = (frameSums[clip_id] + frame)*noFeatures;
    float* currentInfo = data + line; 
    
    float query[] = {(float)trajectoryPositions[0].x, (float)trajectoryPositions[0].z, (float)trajectoryPositions[1].x, (float)trajectoryPositions[1].z, (float)trajectoryPositions[2].x, (float)trajectoryPositions[2].z,
                     (float)trajectoryDirections[0][0], (float)trajectoryDirections[0][2], (float)trajectoryDirections[1][0], (float)trajectoryDirections[1][2], (float)trajectoryDirections[2][0], (float)trajectoryDirections[2][2],
                     0, 0, 0,
                     0, 0, 0,
                     0, 0, 0,
                     0, 0, 0,
                     0, 0, 0};
    
    // normalize the query
    normalize(query);

    // Writing the already normalized data into the query.
    int trajOffset = numTrajPos + numTrajOrient;
    for (int i = trajOffset; i < noFeatures; i++) {
        query[i] = currentInfo[i];
    }

    // Find most fitting clip with Spotifiy's Annoy seach-algorithm
    std::vector<int> closest;
    annoyIndex->get_nns_by_vector(query, kNearest, -1, &closest, NULL);

    // get the line number from the nearest neighbor
    lineNumber = closest[0];
    getClipAndFrame(lineNumber, clip_id, frame);
    assert(frame <= frameSums[clip_id + 1] - frameSums[clip_id] - endFramesWhereIgnoreMatching);
}

// This is used for trajectory generation in the controller
void Database::getEntry(int clip_id, int frame, float* entry)
{
    int line = (frameSums[clip_id] + frame)*noFeatures;
    float* currentInfo = data + line; 

    for (int i = 0; i < noFeatures; i++) {
        entry[i] = currentInfo[i];
    }

    denormalize(entry);
}

// Normalizes the given data array and applies the weights
void Database::normalize(float* data) 
{
    for (int i = 0; i < noFeatures; i++) 
    {   
        float weight;
        if (i < numTrajPos)
            weight = trajectoryPositionWeight;
        else if (i < numTrajPos + numTrajOrient)
            weight = trajectoryFacingWeight;
        else if (i < numTrajPos + numTrajOrient + numFootPos)
            weight = footPositionWeight;
        else if (i < numTrajPos + numTrajOrient + numFootPos + numFootVel)
            weight = footVelocityWeight;
        else if (i < numTrajPos + numTrajOrient + numFootPos + numFootVel + numHipVel)
            weight = hipVelocityWeight;

        data[i] = ((data[i] - means[i]) / standardDeviations[i]) * weight;
    }
}

// Denormalizes the given data array and applies the weights
void Database::denormalize(float* entry)
{
    for (int i = 0; i < noFeatures; i++) 
    {   
        float weight;
        if (i < numTrajPos)
            weight = trajectoryPositionWeight;
        else if (i < numTrajPos + numTrajOrient)
            weight = trajectoryFacingWeight;
        else if (i < numTrajPos + numTrajOrient + numFootPos)
            weight = footPositionWeight;
        else if (i < numTrajPos + numTrajOrient + numFootPos + numFootVel)
            weight = footVelocityWeight;
        else if (i < numTrajPos + numTrajOrient + numFootPos + numFootVel + numHipVel)
            weight = hipVelocityWeight;

        entry[i] = (entry[i] / weight) * standardDeviations[i] + means[i];
    }
}

// Converts a line number to a clip id and frame number of the frameSums vector
bool Database::getClipAndFrame(int lineNumber, int& clip_id, int& frame) {
    if (lineNumber < 0 || lineNumber >= frameSums.back())
        return false;

    // search on the frameSums to find the clip id
    // decided against binary search since array is only about 60 entries max
    clip_id = 0;
    while (lineNumber >= frameSums[clip_id + 1])
        clip_id++;

    //find the frame number
    frame = lineNumber - frameSums[clip_id];

    return true;
}

// Finds the data in the BVH clips and stores it in the data array
void Database::readData(std::vector<std::unique_ptr<crl::mocap::BVHClip>>* bvhClips) 
{
    for (int clipId  = 0; clipId < bvhClips->size(); clipId++) 
    {
        auto *sk = bvhClips->at(clipId)->getModel();
        int numFramesInClip = frameSums[clipId + 1] - frameSums[clipId];
        for (int frame = 0; frame < numFramesInClip; frame++)
        {
            sk->setState(&bvhClips->at(clipId)->getState(frame));
            int frame1_3 = std::min(frame + targetFramerate/3, numFramesInClip-1);
            int frame2_3 = std::min(frame + 2*targetFramerate/3, numFramesInClip-1);
            int frame3_3 = std::min(frame + targetFramerate, numFramesInClip-1);

            auto sk1 = &bvhClips->at(clipId)->getState(frame1_3);
            auto sk2 = &bvhClips->at(clipId)->getState(frame2_3);
            auto sk3 = &bvhClips->at(clipId)->getState(frame3_3);
            int offset = (frameSums[clipId] + frame) * noFeatures;

            getTrajectoryPositions(sk, sk1, sk2, sk3,  offset);
            getTrajectoryDirections(sk, sk1, sk2, sk3, offset + numTrajPos);
            getFootPosition(sk, 0, offset + numTrajPos + numTrajOrient);
            getFootPosition(sk, 1, offset + numTrajPos + numTrajOrient + 3);
            getFootVelocity(sk, 0, offset + numTrajPos + numTrajOrient + 6);
            getFootVelocity(sk, 1, offset + numTrajPos + numTrajOrient + 9);
            getHipVelocity(sk, offset + numTrajPos + numTrajOrient + 12);
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
        runningTotal += (*bvhClips)[i]->getFrameCount();
        frameSums.push_back(runningTotal);
    }
}

// Compute the trajectory position data 
void Database::getTrajectoryPositions(crl::mocap::MocapSkeleton *sk, const crl::mocap::MocapSkeletonState *sk1, const crl::mocap::MocapSkeletonState *sk2, const crl::mocap::MocapSkeletonState *sk3,  int offset) {
    crl::P3D p0 = sk->root->state.pos;
    crl::Quaternion q0Inverse = sk->root->state.orientation.inverse();
    crl::Quaternion nintyDegreeRotation = crl::getRotationQuaternion(-M_PI_2, crl::V3D(0, 1, 0));
    q0Inverse = q0Inverse * nintyDegreeRotation;
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
void Database::getTrajectoryDirections(crl::mocap::MocapSkeleton *sk, const crl::mocap::MocapSkeletonState *sk1, const crl::mocap::MocapSkeletonState *sk2, const crl::mocap::MocapSkeletonState *sk3, int offset) {
    crl::Quaternion q0Inverse = sk->root->state.orientation.inverse();
    crl::Quaternion q1 = sk1->getRootOrientation();
    crl::Quaternion q2 = sk2->getRootOrientation();
    crl::Quaternion q3 = sk3->getRootOrientation();

    crl::V3D trajectory_dir0 = q0Inverse*(q1* crl::V3D(0, 0, 1));
    crl::V3D trajectory_dir1 = q0Inverse*(q2* crl::V3D(0, 0, 1));
    crl::V3D trajectory_dir2 = q0Inverse*(q3* crl::V3D(0, 0, 1));

    data[offset+0] = trajectory_dir0[0];
    data[offset+1] = trajectory_dir0[2];
    data[offset+2] = trajectory_dir1[0];
    data[offset+3] = trajectory_dir1[2];
    data[offset+4] = trajectory_dir2[0];
    data[offset+5] = trajectory_dir2[2];
}

// Compute a feature for the position of a bone relative to the simulation/root bone
void Database::getFootPosition(crl::mocap::MocapSkeleton *sk, int foot, int offset) 
{
        const auto& name = footMarkerNames[foot];
        const auto joint = sk->getMarkerByName(name.c_str());

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

        // eevel seems to be not perfectly correct, but good enough
        crl::V3D eevel = joint->state.getVelocityForPoint_local(joint->endSites[0].endSiteOffset);

        data[offset + 0] = eevel(0);
        data[offset + 1] = eevel(1);
        data[offset + 2] = eevel(2);
}

// Compute the hip velocity
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
