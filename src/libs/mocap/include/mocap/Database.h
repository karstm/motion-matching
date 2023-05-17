#pragma once

#include "mocap/MocapClip.h"
#include "mocap/MocapMarkers.h"
#include "mocap/MocapSkeletonState.h"
#include "annoylib.h"
#include "kissrandom.h"
#include <chrono>

class Database {
    public:
        Database();
        ~Database();

        void build(float trajectoryPositionWeight, float trajectoryFacingWeight,
                   float footPositionWeight, float footVelocityWeight,
                   float hipVelocityWeight,
                   std::vector<std::unique_ptr<crl::mocap::BVHClip>>* bvhClips);

        void match(std::vector<crl::P3D> &trajectoryPositions, std::vector<float> &trajectoryAngles,
                    int& clip_id, int& frame);

        void getEntry(int clip_id, int frame, float* entry);
        
    private:
        void initializeAnnoy();
        void normalize(float* data);
        bool getClipAndFrame(int lineNumber, int& clip_id, int& frame);
        void readData(std::vector<std::unique_ptr<crl::mocap::BVHClip>>* bvhClips); 
        void readFrameSums(std::vector<std::unique_ptr<crl::mocap::BVHClip>>* bvhClips);
        void getTrajectoryPositions(crl::mocap::MocapSkeleton *sk, const crl::mocap::MocapSkeletonState *sk1, const crl::mocap::MocapSkeletonState *sk2, const crl::mocap::MocapSkeletonState *sk3, int offset);
        void getTrajectoryDirections(crl::mocap::MocapSkeleton *sk, const crl::mocap::MocapSkeletonState *sk1, const crl::mocap::MocapSkeletonState *sk2, const crl::mocap::MocapSkeletonState *sk3, int offset);
        void getFootPosition(crl::mocap::MocapSkeleton *sk, int foot, int offset);
        void getFootVelocity(crl::mocap::MocapSkeleton *sk, int foot, int offset);
        void getHipVelocity(crl::mocap::MocapSkeleton *sk, int offset);
        void computeMeans();
        void computeStandardDeviations();
        void denormalize(float* data);


    // Member Variables
    private:
        // framesums[0] = 0
        // frameSums[i+1] = sum of frames in clips 0 to i
        std::vector<int> frameSums;

        // array of size frameSums.back() * noFeatures
        // stores the weighted normalized data
        float *data;
        
        // 27 features:
        //  6 trajectory positions,
        //  3 trajectory angles,
        //  6 foot positions (3 left + 3 right)
        //  6 foot velocities (3 left + 3 right)
        //  3 hip velocities
        int const numTrajPos = 6;
        int const numTrajOrient = 3;
        int const numFootPos = 6;
        int const numFootVel = 6;
        int const numHipVel = 3;
        int noFeatures = numTrajPos + numTrajOrient + numFootPos + numFootVel + numHipVel;
        
        std::vector<std::string> footMarkerNames = {"LeftToe", "RightToe"};
        int ignoredEndFrames = 60;

        // weights
        float trajectoryPositionWeight;
        float trajectoryFacingWeight;
        float footPositionWeight;
        float footVelocityWeight;
        float hipVelocityWeight;

        // means and standard deviations
        std::vector<float> means;
        std::vector<float> standardDeviations;

        // annoy
        #ifdef ANNOYLIB_MULTITHREADED_BUILD
        Annoy::AnnoyIndex<int, float, Annoy::Euclidean, Annoy::Kiss32Random, Annoy::AnnoyIndexMultiThreadedBuildPolicy> *annoyIndex;
        #else
        Annoy::AnnoyIndex<int, float, Annoy::Euclidean, Annoy::Kiss32Random, Annoy::AnnoyIndexSingleThreadedBuildPolicy> *annoyIndex;
        #endif
};