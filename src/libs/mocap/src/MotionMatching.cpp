#include "mocap/MotionMatching.h"


MotionMatching::MotionMatching(const fs::path &motionDataPath, const fs::path &fileIndexPath) {
    trajectoryWeight = 1.0f;
    trajectoryFacingWeight = 1.0f;
    footWeight = 1.0f;
    footVelocityWeight = 1.0f;
    hipVelocityWeight = 1.0f;

    if(!readData(motionDataPath))
        std::cout << "Failed to read data from " << motionDataPath << std::endl;
    if(readFrameSums(fileIndexPath))
        std::cout << "Failed to read frame sums from " << fileIndexPath << std::endl;
}

MotionMatching::MotionMatching() {}

MotionMatching::~MotionMatching() {
    delete[] data;
}

void MotionMatching::match(Eigen::Matrix<double, 3, 2>& trajectoryPositions, Eigen::Matrix<double, 3, 2>& trajectoryFacingDirections,
                            Eigen::Vector3d& leftFootPosition, Eigen::Vector3d& rightFootPosition, 
                            Eigen::Vector3d& leftFootVelocity, Eigen::Vector3d& rightFootVelocity,
                            Eigen::Vector3d& hipVelocity, 
                            int framesLeft, 
                            int& clip_id, int& frame) {
    //TODO: Implement this

}

//TODO: test this
bool MotionMatching::getClipAndFrame(int lineNumber, int& clip_id, int& frame) {
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

//TODO: test this. if too slow, try using a binary file
bool MotionMatching::readData(const fs::path& motionDataPath) {
    std::ifstream file(motionDataPath);

    if (!file.is_open())
        return false;

    std::string line;
    if(!std::getline(file, line))
        return false;

    int numRows = std::stoi(line);
    data = new float[numRows * 24];

    int index = 0;
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string value;

        while (std::getline(ss, value, ',')) {
            float floatValue = std::stof(value);
            data[index++] = floatValue;
        }
    }

    file.close();

    return true;
}
//TODO: test this
bool MotionMatching::readFrameSums(const fs::path& fileIndexPath) {
    std::ifstream file(fileIndexPath);

    if (!file.is_open())
        return false;

    std::string line;
    if(!std::getline(file, line))
        return false;

    int numRows = std::stoi(line);
    frameSums.resize(numRows);

    int sum = 0;
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string value;
        sum += std::stoi(value);
        frameSums.push_back(sum);
    }

    file.close();

    return true;
}

