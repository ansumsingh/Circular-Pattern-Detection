#ifndef CIRCLULARPATTERNDETECTOR_H
#define CIRCLULARPATTERNDETECTOR_H
#include<map>
#include<opencv2/opencv.hpp>
#include<memory>


class circularPatternDetector;
//Factory pattern implementation



class factoryCircleDetector{
public:
	std::shared_ptr<circularPatternDetector> createDetector();


};




class circularPatternDetector{
public:
	circularPatternDetector() :
		refCircleLocations(),
		blobDectectorParams(),
		rvec(3,1,CV_64F), // copy this if it don't work: tvec(3,1,CV_64F) 
		tvec(3, 1, CV_64F){}

	circularPatternDetector(const std::map<std::string, cv::Point3f>& refCircleLocationsIP 
		, cv::SimpleBlobDetector::Params blobDectectorParamsIP )
		:refCircleLocations(refCircleLocationsIP),
		blobDectectorParams(blobDectectorParamsIP),
		rvec(3,1,CV_64F),// copy this if it don't work: tvec(3,1,CV_64F)
		tvec(3, 1, CV_64F)
	{
		//detector = cv::SimpleBlobDetector::create(blobDectectorParams);
	}
	bool detect(const cv::Mat,const cv::Mat, const cv::Mat);
	cv::Mat getRvec()const { return rvec; }
	cv::Mat getTvec()const { return tvec; }

private:
	std::map<std::string, cv::Point3f> refCircleLocations;
	cv::SimpleBlobDetector::Params blobDectectorParams;
	cv::Mat rvec;
	cv::Mat tvec;
	//std::vector<cv::KeyPoint> keyPoints;
	//std::vector<cv::KeyPoint> keypoints;
	//std::vector<cv::KeyPoint> keypoints2;
};
#endif