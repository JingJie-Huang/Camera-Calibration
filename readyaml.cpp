#include <iostream>
#include <opencv2/opencv.hpp>

using namespace std;


bool readYaml();


int main(int argc, const char *argv[])
{
    readYaml();
    
    return 0;
}

bool readYaml() {
    std::string filename = "camera_calibration_parameters.yaml";
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        cout << "failed to open file " << filename << endl;
        return false;
    }
    
    cv::Mat cameraMatrix, distCoeffs;
    fs["cameraMatrix"] >> cameraMatrix;
    fs["distCoeffs"] >> distCoeffs;
    cout << "cameraMatrix = \n" << cameraMatrix << endl;
    cout << "distCoeffs = \n" << distCoeffs << endl;
 
    // read string
    string timeRead;
    fs["calibrationDate"] >> timeRead;
    cout << "calibrationDate = " << timeRead << endl;
 
    fs.release();
    return true;
}


