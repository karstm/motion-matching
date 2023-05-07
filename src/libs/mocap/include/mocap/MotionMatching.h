#pragma once
#include <filesystem>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>

namespace fs = std::filesystem;

class MotionMatching {
    public:
        MotionMatching(const fs::path& motionDataPath, const fs::path& fileIndexPath);
        MotionMatching();
        ~MotionMatching();

        void match(Eigen::Matrix<double, 3, 2>& trajectoryPositions, Eigen::Matrix<double, 3, 2>& trajectoryFacingDirections, 
                    Eigen::Vector3d& leftFootPosition, Eigen::Vector3d& rightFootPosition, 
                    Eigen::Vector3d& leftFootVelocity, Eigen::Vector3d& rightFootVelocity, 
                    Eigen::Vector3d& hipVelocity,
                    int framesLeft,
                    int& clip_id, int& frame);
        
    private:
        bool getClipAndFrame(int lineNumber, int& clip_id, int& frame);
        bool readData(const fs::path& motionDataPath); 
        bool readFrameSums(const fs::path& fileIndexPath);

    // Members
    public:
        float trajectoryWeight;
        float trajectoryFacingWeight;
        float footWeight;
        float footVelocityWeight;
        float hipVelocityWeight;

    private:
        std::vector<int> frameSums;
        float *data;
};