#include "CircularPatternDetector.h"
#include <vector>
#include <string>


cv::SimpleBlobDetector::Params params()
{
	// Change thresholds
	cv::SimpleBlobDetector::Params DefaultParams;

	DefaultParams.minThreshold = 0;
	DefaultParams.maxThreshold = 50;

	// Filter by Area.
	DefaultParams.filterByArea = true;
	DefaultParams.minArea = 50;
	DefaultParams.maxArea = 9500;

	// Filter by Circularity
	DefaultParams.filterByCircularity = true;
	DefaultParams.minCircularity = 0.1;

	// Filter by Convexity
	DefaultParams.filterByConvexity = false;
	// params.minConvexity = 0.87;

	// Filter by Inertia
	DefaultParams.filterByInertia = true;
	DefaultParams.minInertiaRatio = 0.01;

	return DefaultParams;
}


std::shared_ptr<circularPatternDetector> factoryCircleDetector::createDetector()
{
	static std::map<std::string, cv::Point3f> DefaultCircleLocations;
	DefaultCircleLocations.insert(std::pair<std::string, cv::Point3f>("Red", cv::Point3f(1.35, 1.35, 0)));
	DefaultCircleLocations.insert(std::pair<std::string, cv::Point3f>("Green", cv::Point3f(-1.35, 1.35, 0)));
	DefaultCircleLocations.insert(std::pair<std::string, cv::Point3f>("Blue", cv::Point3f(-1.35, -1.35, 0)));
	DefaultCircleLocations.insert(std::pair<std::string, cv::Point3f>("Yellow", cv::Point3f(1.35, -1.35, 0)));

	// Change thresholds
	cv::SimpleBlobDetector::Params DefaultParams;

	DefaultParams.minThreshold = 0;
	DefaultParams.maxThreshold = 50;

	// Filter by Area.
	DefaultParams.filterByArea = true;
	DefaultParams.minArea = 50;
	DefaultParams.maxArea = 9500;

	// Filter by Circularity
	DefaultParams.filterByCircularity = true;
	DefaultParams.minCircularity = 0.1;

	// Filter by Convexity
	DefaultParams.filterByConvexity = false;
	// params.minConvexity = 0.87;

	// Filter by Inertia
	DefaultParams.filterByInertia = true;
	DefaultParams.minInertiaRatio = 0.01;
	std::shared_ptr<circularPatternDetector> createdDetector(new circularPatternDetector(DefaultCircleLocations, DefaultParams));
	return createdDetector;



}


bool circularPatternDetector::detect(const cv::Mat inputImage,const cv::Mat cameraMatrix ,const  cv::Mat distortionCoefficients)
{
	using namespace cv;
	using namespace std;
	bool returnVal = false;
	std::vector<std::string> color;
	color.push_back("Red");
	color.push_back("Blue");
	color.push_back("Green");
	color.push_back("Yellow");

	int scaleFactor = 5;

	cv::Mat image = cv::Mat(inputImage.rows / scaleFactor, inputImage.cols / scaleFactor, CV_8UC1);
	cv::resize(inputImage, image, image.size());
	cv::Mat imageGray;
	cv::Mat imageHSV;

	cv::cvtColor(image, imageGray, CV_BGR2GRAY);//converting to Gray
	cv::cvtColor(image, imageHSV, CV_BGR2HSV);//converting to HSV

	cv::Mat imageBlue;
	cv::Mat imageGreen;
	cv::inRange(imageHSV, cv::Scalar(80, 10, 10), cv::Scalar(130, 255, 255), imageBlue);//thresholding the blue portion
	cv::inRange(imageHSV, cv::Scalar(45, 20, 10), cv::Scalar(70, 255, 255), imageGreen);//thresholding for Green portion
	//imageBlue = cv::Scalar(255)- imageBlue;
	std::vector<cv::KeyPoint> keypoints;
	std::vector<cv::KeyPoint> keypoints2;
	/*keypoints.clear();
	keypoints2.clear();*/

#if CV_MAJOR_VERSION < 3   // If you are using OpenCV 2

	// Set up detector with params
	SimpleBlobDetector detector(params);

	// Detect blobs
	detector.detect(im, keypoints);
#else 

	// Set up detector with params
	cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(blobDectectorParams);
	detector->detect(imageBlue, keypoints);
	detector->detect(imageGreen, keypoints2);

#endif

	//drawKeypoints( resizedImage, keypoints, im_with_keypoints1, Scalar(0,0,255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS ); 
	//Blue circle
	Vec3b intensity;
	vector<Point2f> imagePoints;
	vector<Point3f> objectPoints;

	/*Detecting inner circles*/

	for (int i = 0; i<keypoints.size(); i++)
	{
		Point center = keypoints[i].pt;

		intensity = imageHSV.at<Vec3b>(center);
		//cout<<intensity;
		//Checking the Yellow color in the big blue Circle
		if ((intensity.val[0] >= 20 && intensity.val[0]<35))
		{
			if (intensity.val[1]>10 && intensity.val[1]<255)
			{
				if (intensity.val[2]>10 && intensity.val[2]<255)
				{
					imagePoints.push_back((Point2f)center * scaleFactor);
					objectPoints.push_back(refCircleLocations["Yellow"]);
					//circle( capturedImage, 4*center, 10, Scalar(150,0,150), 3, 8, 0 );
					//circle( imageBlue2, center, 5, Scalar(0,242,255), 2, 8, 0 );
				}
			}
		}
		//Checking the Green in the Blue circle
		if ((intensity.val[0] >= 45 && intensity.val[0]<70))
		{
			if (intensity.val[1]>10 && intensity.val[1]<255)
			{
				if (intensity.val[2]>10 && intensity.val[2]<255)
				{
					//circle( capturedImage, 4*center, 10, Scalar(255,0,0), 3, 8, 0 );
					//circle( imageBlue2, center, 5, Scalar(0,255,0), 2, 8, 0 );
					imagePoints.push_back((Point2f)center * scaleFactor);
					objectPoints.push_back(refCircleLocations["Green"]);
				}
			}
		}



	}
	//Big circle Green 
	for (int i = 0; i<keypoints2.size(); i++)
	{
		Point center = keypoints2[i].pt;

		intensity = imageHSV.at<Vec3b>(center);

		//Checking the Blue color in the big Green Circle
		if ((intensity.val[0] >= 80 && intensity.val[0]<120))
		{
			if (intensity.val[1]>10 && intensity.val[1]<255)
			{
				if (intensity.val[2]>10 && intensity.val[2]<255)
				{
					//circle( capturedImage, 4*center, 10, Scalar(0,150,0), 3, 8, 0 );
					//circle( imageGreen2, center, 5, Scalar(255,0,0), 2, 8, 0 );
					imagePoints.push_back((Point2f)center * scaleFactor);
					objectPoints.push_back(refCircleLocations["Blue"]);
				}
			}
		}


		//checking Red in Green channel   
		if ((intensity.val[0] >= 0 && intensity.val[0]<17) || (intensity.val[0]>160 && intensity.val[0]<180))
		{
			if (intensity.val[1]>10 && intensity.val[1]<255)
			{
				if (intensity.val[2]>10 && intensity.val[2]<255)
				{
					imagePoints.push_back((Point2f)center * scaleFactor);
					objectPoints.push_back(refCircleLocations["Red"]);
					//circle( capturedImage, 4*center, 10, Scalar(0,255,0), 3, 8, 0 );
					//circle( imageGreen2, center, 5, Scalar(0,0,255), 2, 8, 0 );
				}
			}
		}
			
	}
	


	keypoints.clear();
	keypoints2.clear();

	if (objectPoints.size() > 3)
	{
		
		//if (!cv::solvePnP(objectPoints, imagePoints, cameraMatrix, distortionCoefficients, rvec, tvec, true, CV_ITERATIVE)) //first use extrinsic guess
		cv::solvePnP(objectPoints, imagePoints, cameraMatrix, distortionCoefficients, rvec, tvec, false, CV_EPNP);
		cv::solvePnP(objectPoints, imagePoints, cameraMatrix, distortionCoefficients, rvec, tvec, true, CV_ITERATIVE);
		return true;
	}

	return returnVal;
}
