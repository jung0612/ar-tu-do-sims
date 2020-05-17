////
//// Created by hms on 20. 5. 3..
////
//// Import the aruco module in OpenCV
//#include <opencv2/aruco.hpp>
//
//cv::Mat markerImage;
//// Load the predefined dictionary
//cv::Ptr<cv::aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
//
//// Generate the marker
//aruco::drawMarker(dictionary, 33, 200, markerImage, 1);
//// Load the dictionary that was used to generate the markers.
//cv::Ptr<Dictionary> dictionary = getPredefinedDictionary(DICT_6X6_250);
//
//// Initialize the detector parameters using default values
//cv::Ptr<DetectorParameters> parameters = DetectorParameters::create();
//
//// Declare the vectors that would contain the detected marker corners and the rejected marker candidates
//std::vector<vector<Point2f>> markerCorners, rejectedCandidates;
//
//// The ids of the detected markers are stored in a vector
//std::vector<int> markerIds;
//
//// Detect the markers in the image
//aruco::detectMarkers(frame, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);
