#pragma once

#include "mocap/MocapClip.h"
#include "mocap/MocapMarkers.h"
#include "mocap/MocapSkeletonState.h"

class Database {
    public:
        Database();
        Database(std::vector<std::unique_ptr<crl::mocap::BVHClip>>* bvhClips);
        ~Database();

        void normalize(float* data);
        void match(crl::Matrix& trajectoryPositions, crl::Matrix& trajectoryDirections, 
                   crl::V3D& leftFootPosition, crl::V3D& rightFootPosition, 
                   crl::V3D& leftFootVelocity, crl::V3D& rightFootVelocity, 
                   crl::V3D& hipVelocity,
                   int& clip_id, int& frame);
        
    private:
        bool getClipAndFrame(int lineNumber, int& clip_id, int& frame);
        void readData(); 
        void readFrameSums();
        void getFootPositionsAndVelocities(int& offset);
        void getTrajectoryPositionsAndDirections(int& offset);
        void getHipVelocity(int& offset);
        void computeMeans();
        void computeStandardDeviations();


    // Member Variables
    private:
        std::vector<std::unique_ptr<crl::mocap::BVHClip>> *bvhClips;

        std::vector<int> frameSums;
        float *data;
        
        int noFeatures;
        std::vector<std::string> footMarkerNames = {"LeftToe", "RightToe"};

        // weights
        float trajectoryWeight;
        float trajectoryFacingWeight;
        float footWeight;
        float footVelocityWeight;
        float hipVelocityWeight;

        // means and standard deviations
        std::vector<float> means;
        std::vector<float> standardDeviations;
};