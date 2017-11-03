#include<opencv2\opencv.hpp>
#include"CircularPatternDetector.h"
#include<vector>

int main()
{
	factoryCircleDetector factory;
	std::shared_ptr<circularPatternDetector> detector = factory.createDetector();

	
	cv::Mat image  = cv::imread("../sources/sampleImage.jpg");
	if (image.empty())
	{
		std::cout << "could not open file"; return 0;
	}
	cv::Mat cameraMatrix = (cv::Mat_<float>(3,3) << 10, 0, 100, 0, 10, 100, 0, 0, 1);
	cv::Mat distCoeff = (cv::Mat_<float>(5, 1) << 0,0,0,0,0);
	if (detector->detect(image, cameraMatrix, distCoeff))
	{
	
		std::vector<cv::Point3f> PointsToProject;
		std::vector<cv::Point2f> ProjectedPoints;
		PointsToProject.push_back(cv::Point3f(0, 0, 0));
		PointsToProject.push_back(cv::Point3f(0, 0, 0.01));
		PointsToProject.push_back(cv::Point3f(0, 1, 0));
		PointsToProject.push_back(cv::Point3f(1, 0, 0));
		cv::projectPoints(PointsToProject, detector->getRvec(), detector->getTvec(), cameraMatrix, distCoeff, ProjectedPoints);
		arrowedLine(image, ProjectedPoints[0], ProjectedPoints[1], cv::Scalar(0, 0, 255), 2);
		arrowedLine(image, ProjectedPoints[0], ProjectedPoints[2], cv::Scalar(0, 255, 0), 2);
		arrowedLine(image, ProjectedPoints[0], ProjectedPoints[3], cv::Scalar(255, 0, 0), 2);
		cv::imshow("Detected Image", image);
		std::cout << ProjectedPoints[0] << "-" << ProjectedPoints[1];
		cv::waitKey(0);
	}
	
	return 1;

}