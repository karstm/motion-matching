#pragma once

#include "mocap/MocapClip.h"
#include "mocap/MocapMarkers.h"
#include "mocap/MocapSkeletonState.h"

class Database {
    public:
        Database();
        Database(std::vector<std::unique_ptr<crl::mocap::BVHClip>>* bvhClips);
        ~Database();

        void init(std::vector<std::unique_ptr<crl::mocap::BVHClip>>* bvhClips);

        void match(crl::Matrix& trajectoryPositions, crl::Matrix& trajectoryDirections, 
                   crl::P3D& leftFootPosition, crl::P3D& rightFootPosition, 
                   crl::V3D& leftFootVelocity, crl::V3D& rightFootVelocity, 
                   crl::V3D& hipVelocity,
                   int& clip_id, int& frame);
        
    private:
        void normalize(float* data);
        bool getClipAndFrame(int lineNumber, int& clip_id, int& frame);
        void readData(); 
        void readFrameSums();
        void getTrajectoryPositions(crl::mocap::MocapSkeleton *sk, int offset);
        void getTrajectoryDirections(crl::mocap::MocapSkeleton *sk, int offset);
        void getFootPosition(crl::mocap::MocapSkeleton *sk, int foot, int offset);
        void getFootVelocity(crl::mocap::MocapSkeleton *sk, int foot, int offset);
        void getHipVelocity(crl::mocap::MocapSkeleton *sk, int offset);
        void computeMeans();
        void computeStandardDeviations();


    // Member Variables
    private:
        std::vector<std::unique_ptr<crl::mocap::BVHClip>> *bvhClips;

        // framesums[0] = 0
        // frameSums[i+1] = sum of frames in clips 0 to i
        std::vector<int> frameSums;

        // array of size frameSums.back() * noFeatures
        // stores the weighted normalized data
        float *data;
        
        // 27 features:
        //  6 trajectory positions,
        //  6 trajectory directions,
        //  6 foot positions (3 left + 3 right)
        //  6 foot velocities (3 left + 3 right)
        //  3 hip velocities
        int noFeatures = 27;
        std::vector<std::string> footMarkerNames = {"LeftToe", "RightToe"};
        int ignoredEndFrames = 60;

        // weights
        float trajectoryPositionWeight = 1.0f;
        float trajectoryFacingWeight = 1.5f;
        float footPositionWeight = 0.75f;
        float footVelocityWeight = 1.0f;
        float hipVelocityWeight = 1.0f;

        // means and standard deviations
        std::vector<float> means;
        std::vector<float> standardDeviations;
};