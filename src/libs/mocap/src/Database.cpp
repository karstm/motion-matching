#include "mocap/Database.h"

// Empty constructor to allow for member initialization
Database::Database() {}

// Constructor that takes a vector of BVHClips and initializes the database
Database::Database(std::vector<std::unique_ptr<crl::mocap::BVHClip>>* bvhClips) {
    this->bvhClips = bvhClips;
    readFrameSums();
    data = new float[frameSums.back() * noFeatures];
    readData();
}

// Destructor frees the data array
Database::~Database() {
    // TODO: This seems to cause malloc errors
    delete[] data;
}

// Matches the given query to the mocap database and returns the clip id and frame number
// TODO: implement this
void Database::match(crl::Matrix& trajectoryPositions, crl::Matrix& trajectoryDirections, 
                     crl::P3D& leftFootPosition, crl::P3D& rightFootPosition,
                     crl::V3D& leftFootVelocity, crl::V3D& rightFootVelocity, 
                     crl::V3D& hipVelocity, 
                     int& clip_id, int& frame) 
{
    int lineNumber;

    //Steps: 
    // - (arange the query in array) DONE
    // - (normalize the query) DONE
    // - get the line number of the nearest neighbor in the database
    // - (get the clip id and frame number from the line number) DONE

    // arange the query in array
    float query[] = {(float)trajectoryPositions.coeff(0,0), (float)trajectoryPositions.coeff(0,1), (float)trajectoryPositions.coeff(1,0), (float)trajectoryPositions.coeff(1,1), (float)trajectoryPositions.coeff(2,0), (float)trajectoryPositions.coeff(2,1),
                     (float)trajectoryDirections.coeff(0,0), (float)trajectoryDirections.coeff(0,1), (float)trajectoryDirections.coeff(1,0), (float)trajectoryDirections.coeff(1,1), (float)trajectoryDirections.coeff(2,0), (float)trajectoryDirections.coeff(2,1),
                     (float)leftFootPosition.x, (float)leftFootPosition.y, (float)leftFootPosition.z,
                     (float)leftFootVelocity(0), (float)leftFootVelocity(1), (float)leftFootVelocity(2),
                     (float)rightFootVelocity(0), (float)rightFootVelocity(1), (float)rightFootVelocity(2),
                     (float)hipVelocity(0), (float)hipVelocity(1), (float)hipVelocity(2)};
    // normalize the query
    normalize(query);

    //TODO: get the line number of the nearest neighbor in the database

    // get the line number from the nearest neighbor
    getClipAndFrame(lineNumber, clip_id, frame);
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
        else if(i < 12)
            weight = trajectoryFacingWeight;
        else if(i < 18)
            weight = footPositionWeight;
        else if(i < 24)
            weight = footVelocityWeight;
        else if(i < 27)
            weight = hipVelocityWeight;

        data[i] = (data[i] - means[i]) / standardDeviations[i] * weight;
    }
}

// Converts a line number to a clip id and frame number using a binary search on the frameSums vector
//TODO: test this, this might be off by 1, haven't tested it yet
bool Database::getClipAndFrame(int lineNumber, int& clip_id, int& frame) {
    if (lineNumber < 0 || lineNumber >= frameSums.size())
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
void Database::readData() 
{
    for (int clipId  = 0; clipId < bvhClips->size(); clipId++) 
    {
        auto *sk = bvhClips->at(clipId)->getModel();
        for (int frame = 0; frame < frameSums[clipId + 1] - frameSums[clipId]; frame++)
        {
            sk->setState(&bvhClips->at(clipId)->getState(frame));

            int offset = frameSums[clipId] + frame * noFeatures;

            getTrajectoryPositions(sk, offset);
            getTrajectoryDirections(sk, offset + 6);
            getFootPosition(sk, 0, offset + 12);
            getFootPosition(sk, 1, offset + 15);
            getFootVelocity(sk, 0, offset + 18);
            getFootVelocity(sk, 1, offset + 21);
            getHipVelocity(sk, offset + 24);
        }
    }

    computeMeans();
    computeStandardDeviations();
    for(int line = 0; line < frameSums.back(); line++) {
        normalize(data + line * noFeatures);
    }
}

// Finds the frame sums for each clip and stores them in the frameSums vector
void Database::readFrameSums() 
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
void Database::getTrajectoryPositions(crl::mocap::MocapSkeleton *sk, int offset) {}

// Compute the trajectory direction data
// TODO: implement this
void Database::getTrajectoryDirections(crl::mocap::MocapSkeleton *sk, int offset) {}

// Compute a feature for the position of a bone relative to the simulation/root bone
void Database::getFootPosition(crl::mocap::MocapSkeleton *sk, int foot, int offset) 
{
        const auto& name = footMarkerNames[foot];
        const auto joint = sk->getMarkerByName(name.c_str());

        //TODO: eepos doesn't seem to be correct
        // Prob. calling argument should be root bone
        crl::P3D eepos = joint->state.getLocalCoordinates(joint->endSites[0].endSiteOffset);

        data[offset + 0] = eepos.x;
        data[offset + 1] = eepos.y;
        data[offset + 2] = eepos.z;
}

// Compute a feature for the position of a bone relative to the simulation/root bone
void Database::getFootVelocity(crl::mocap::MocapSkeleton *sk, int foot, int offset) 
{
        const auto& name = footMarkerNames[foot];
        const auto joint = sk->getMarkerByName(name.c_str());

        //TODO: eevel doesn't seem to be correct
        crl::V3D eevel = joint->state.getVelocityForPoint_local(joint->endSites[0].endSiteOffset);
        eevel.normalize();

        data[offset + 0] = eevel(0);
        data[offset + 1] = eevel(1);
        data[offset + 2] = eevel(2);
}

// Compute the hip velocity
// TODO: implement this
void Database::getHipVelocity(crl::mocap::MocapSkeleton *sk, int offset) {}

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
