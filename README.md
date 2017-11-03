# Circular-Pattern-Detection
C++ based low latency circular pattern detection using circular patches. The pattern detector uses the concentric circular patches. For the details about 
the patter detection working principal visit : https://www.researchgate.net/publication/320291582_Low-latency_Vision-based_Fiducial_Detection_and_Localization_for_Object_Tracking

# USAGE
factoryCircleDetector factory;
std::shared_ptr<circularPatternDetector> detector = factory.createDetector();

detector->detect(image, cameraMatrix, distCoeff)


