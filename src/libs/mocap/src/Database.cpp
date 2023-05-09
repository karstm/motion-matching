#include "mocap/Database.h"

// Empty constructor to allow for member initialization
Database::Database() {}

// Constructor that takes a vector of BVHClips and initializes the database
Database::Database(std::vector<std::unique_ptr<crl::mocap::BVHClip>>* bvhClips) {
    trajectoryWeight = 1.0f;
    trajectoryFacingWeight = 1.0f;
    footWeight = 1.0f;
    footVelocityWeight = 1.0f;
    hipVelocityWeight = 1.0f;

    // 6 trajectory positions,
    // 6 trajectory directions,
    // 6 foot positions (3 left + 3 right)
    // 6 foot velocities (3 left + 3 right)
    // 3 hip velocities
    noFeatures = 6 + 6 + 3 + 3 + 3 + 3 + 3;

    this->bvhClips = bvhClips;
    readFrameSums();
    data = new float[frameSums.back() * noFeatures];
    readData();
}

// Destructor frees the data array
Database::~Database() {
    delete[] data;
}

// Normalizes the given data array and applies the weights
// Note: This method is public because it is also used to normalize the query
// TODO: test this
void Database::normalize(float* data) {
    for (int i = 0; i < noFeatures; i++) {
        data[i] = (data[i] - means[i]) / standardDeviations[i];
    }
}

// Matches the given query to the mocap database and returns the clip id and frame number
void Database::match(crl::Matrix& trajectoryPositions, crl::Matrix& trajectoryDirections, 
                     crl::V3D& leftFootPosition, crl::V3D& rightFootPosition,
                     crl::V3D& leftFootVelocity, crl::V3D& rightFootVelocity, 
                     crl::V3D& hipVelocity, 
                     int& clip_id, int& frame) {
    int lineNumber;

    //TODO: get the line number from the database
    //Steps: 
    // - arange the query in array
    // - normalize the query
    // - find the nearest neighbor, 
    // - get the line number from the nearest neighbor

    getClipAndFrame(lineNumber, clip_id, frame);
}

// Converts a line number to a clip id and frame number using a binary search on the frameSums vector
//TODO: test this
bool Database::getClipAndFrame(int lineNumber, int& clip_id, int& frame) {
    if (lineNumber < 0 || lineNumber >= frameSums.size())
        return false;

    //binary search to find the clip id
    int low = 0;
    int high = frameSums.size() - 1;
    while (low <= high) {
        int mid = (low + high) / 2;
        if (frameSums[mid] < lineNumber)
            low = mid + 1;
        else if (frameSums[mid] > lineNumber)
            high = mid - 1;
        else {
            clip_id = mid + 1;
            break;
        }
    }

    frame = lineNumber - frameSums[clip_id - 1];

    return true;
}

// Finds the data in the BVH clips and stores it in the data array
//TODO: test this
void Database::readData() {
    int offset = 0;
    getTrajectoryPositionsAndDirections(offset);
    getFootPositionsAndVelocities(offset);
    getHipVelocity(offset);
    computeMeans();
    computeStandardDeviations();
    for(int line = 0; line < frameSums.back(); line++) {
        normalize(data + line * noFeatures);
    }
}

// Finds the frame sums for each clip and stores them in the frameSums vector
void Database::readFrameSums() {
    int runningTotal = 0;
    frameSums.push_back(runningTotal);
    for (int i = 0; i < bvhClips->size(); i++) {
        runningTotal += (*bvhClips)[i]->getFrameCount();
        frameSums.push_back(runningTotal);
    }
}

// Compute a feature for the position of a bone relative to the simulation/root bone
void Database::getFootPositionsAndVelocities(int& offset) {
    for (int foot = 0; foot < footMarkerNames.size(); foot++) {
        for (int clipId  = 0; clipId < bvhClips->size(); clipId++) {
            auto *sk = bvhClips->at(clipId)->getModel();
            for (int frame = 0; frame < bvhClips->at(clipId)->getFrameCount(); frame++) {
                sk->setState(&bvhClips->at(clipId)->getState(frame));
                const auto& name = footMarkerNames[foot];
                if (const auto joint = sk->getMarkerByName(name.c_str())) {
                    //TODO: eepos and eevel don't seem to be correct
                    crl::P3D eepos = joint->state.getLocalCoordinates(joint->endSites[0].endSiteOffset);
                    crl::V3D eevel = joint->state.getVelocityForPoint_local(joint->endSites[0].endSiteOffset);
                    eevel.normalize();
                    data[frameSums[clipId] + frame * noFeatures + offset] = eepos.x;
                    data[frameSums[clipId] + frame * noFeatures + offset + 1] = eepos.y;
                    data[frameSums[clipId] + frame * noFeatures + offset + 2] = eepos.z;
                    data[frameSums[clipId] + frame * noFeatures + offset + 3] = eevel(0);
                    data[frameSums[clipId] + frame * noFeatures + offset + 4] = eevel(1);
                    data[frameSums[clipId] + frame * noFeatures + offset + 5] = eevel(2);
                }
            }
        }
        offset += 6;
    }
}

// Compute the trajectory data 
//TODO: implement this
void Database::getTrajectoryPositionsAndDirections(int& offset) {}

// Compute the hip velocity
// TODO: implement this
void Database::getHipVelocity(int& offset) {}

// Computes the means for each feature and stores them in the means vector
//TODO: test this
void Database::computeMeans() {
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
void Database::computeStandardDeviations() {
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
