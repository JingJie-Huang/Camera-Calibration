#include<opencv2/core/core.hpp>
#include<opencv2/opencv.hpp>
#include<opencv2/calib3d.hpp>
#include<opencv2/calib3d/calib3d.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<stdio.h>
#include<iostream>
#include<vector>
#include<string> 
#include<ctime>


using namespace std;

double computeReprojectionErrors( const vector<vector<cv::Point3f> >& objectPoints,
                                  const vector<vector<cv::Point2f> >& imagePoints,
                                  const vector<cv::Mat>& rvecs, const vector<cv::Mat>& tvecs,
                                  const cv::Mat& cameraMatrix , const cv::Mat& distCoeffs,
                                  vector<float>& perViewErrors);
                                  
bool writeYaml(const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs);                                 


int main(int argc, char* argv[])
{
	// Defining the dimensions of checkerboard
	//int CHECKERBOARD[2] = {6,9};
	int CHECKERBOARD[2] = {6,8}; 
	// Creating vector to store vectors of 3D points (world coordinate) for each checkerboard image
	vector< vector<cv::Point3f> > objpoints;
  	// Creating vector to store vectors of 2D points (pixel coordinate) for each checkerboard image
	vector< vector<cv::Point2f> > imgpoints;

  	// Defining the world coordinates for 3D points
  	vector<cv::Point3f> objp;
  	for(int i=0; i<CHECKERBOARD[1]; i++)
  	{
    	for(int j=0; j<CHECKERBOARD[0]; j++){
    		// assume z coordinate of the points on the checkerboard are zero
    	  	objp.push_back(cv::Point3f(j,i,0.0));
    	}
  	}

  	// Extracting path of individual image stored in a given directory
  	vector<cv::String> filenames;
  	// Path of the folder containing checkerboard images
  	// ** nend to ENTER "absolute path(絕對路徑)" **
  	//string path = "/home/entropy/Clearning/camera_calibration/images/*.jpg";
  	string path = "/home/entropy/Clearning/camera_calibration/fr1_rgb_calibration/*.png";
  	cv::glob(path, filenames);

  	cv::Mat frame, gray;
  	// vector to store the pixel coordinates of detected checker board corners 
  	vector<cv::Point2f> corner_pts;
  	bool success;

	cout << "There are total " << filenames.size() << " images in the folder." << endl << endl;
  	// Looping over all the images in the directory
  	for(int i=0; i<filenames.size(); i++)
  	{
    	cout << filenames[i] << endl;
    	frame = cv::imread(filenames[i], CV_LOAD_IMAGE_COLOR);
    	cv::cvtColor(frame,gray,cv::COLOR_BGR2GRAY);

    	// Finding checker board corners
    	// If desired number of corners are found in the image then success = true  
    	success = cv::findChessboardCorners(gray, cv::Size(CHECKERBOARD[0], CHECKERBOARD[1]), corner_pts, 
    	CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE);
    
    	if(success)
    	{
      		cv::TermCriteria criteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.001);
      		// refining pixel coordinates for given 2d points.
      		cv::cornerSubPix(gray,corner_pts,cv::Size(5, 5), cv::Size(-1, -1), criteria);      
      		// Displaying the detected corner points on the checker board
      		cv::drawChessboardCorners(frame, cv::Size(CHECKERBOARD[0], CHECKERBOARD[1]), corner_pts, success);
      
      		objpoints.push_back(objp);
      		imgpoints.push_back(corner_pts);
    	}

    	cv::imshow("Image",frame);
    	cv::waitKey(100); // wait ...ms per image
  	}
	// clear all displays
  	cv::destroyAllWindows();
  	// define intrinsic and extrinsic parameters
  	cv::Mat cameraMatrix, distCoeffs, newcameraMatrix;
  	vector<cv::Mat> R, T;
	// call opencv calibration function
	cout << "Camera calibrating ..." << endl << endl;
  	cv::calibrateCamera(objpoints, imgpoints, cv::Size(gray.rows,gray.cols), cameraMatrix, distCoeffs, R, T);
  	// with cv::CALIB_RATIONAL_MODEL
	// cv::calibrateCamera(objpoints, imgpoints, cv::Size(gray.rows,gray.cols), cameraMatrix, distCoeffs, R, T, cv::CALIB_RATIONAL_MODEL);
	
	// get new camera matrix
	newcameraMatrix = cv::getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, cv::Size(gray.rows,gray.cols), 1, cv::Size(gray.rows,gray.cols), 0);
	
	double reproj_error;
	vector<float> ViewError;
	reproj_error = computeReprojectionErrors(objpoints, imgpoints, R, T, cameraMatrix, distCoeffs, ViewError);
	

	cout << "Calibration Result: " << endl;
  	cout << "cameraMatrix: " << endl << cameraMatrix << endl << endl;
  	cout << "distCoeffs: " << endl << distCoeffs << endl << endl;
  	cout << "newcameraMatrix: " << endl << newcameraMatrix << endl;
  	cout << "reprojection error(unit: pixel): " << endl << reproj_error << endl << endl;
  	
  	cout << "Write camera paramters to .yaml file ..." << endl;
  	// save camera parameters to .yaml file
  	writeYaml(cameraMatrix, distCoeffs);
  	cout << "--- Finish ---" << endl;
  	

	return 0;
}



double computeReprojectionErrors( const vector<vector<cv::Point3f> >& objectPoints,
                                  const vector<vector<cv::Point2f> >& imagePoints,
                                  const vector<cv::Mat>& rvecs, const vector<cv::Mat>& tvecs,
                                  const cv::Mat& cameraMatrix , const cv::Mat& distCoeffs,
                                  vector<float>& perViewErrors)
{
    vector<cv::Point2f> imagePoints_pro;
    size_t totalPoints = 0;
    double totalErr = 0, err;
    perViewErrors.resize(objectPoints.size());
		//for(size_t i = 0; i < objectPoints.size(); ++i )
		for(size_t i=0; i<1; i++)
		{
		    cv::projectPoints(objectPoints[i], rvecs[i], tvecs[i], cameraMatrix, distCoeffs, imagePoints_pro);
		    err = cv::norm(imagePoints[i], imagePoints_pro, cv::NORM_L2);
		    size_t n = objectPoints[i].size();
		    perViewErrors[i] = (float) std::sqrt(err*err/n);
		    totalErr        += err*err;
		    totalPoints     += n;
		}
    return std::sqrt(totalErr/totalPoints);
}


bool writeYaml(const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs) 
{
    std::string filename = "camera_calibration_parameters.yaml";
    cv::FileStorage fs(filename, cv::FileStorage::WRITE);
    if (!fs.isOpened()) {
        cout << "failed to open file " << filename << endl;
        return false;
    }


    // write camera parameters 
    fs << "cameraMatrix" << cameraMatrix << "distCoeffs" << distCoeffs;
 
    // write local time  #include<ctime>
    time_t rawtime;
    time(&rawtime);
    fs << "calibrationDate" << asctime(localtime(&rawtime));

    fs.release();
    
    return true;
}










