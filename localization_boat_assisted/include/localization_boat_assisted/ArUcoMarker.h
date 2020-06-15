//
// Created by alg on 15/06/20.
//

#ifndef SRC_ARUCOMARKER_H
#define SRC_ARUCOMARKER_H

#include <gtsam/geometry/Point2.h>
#include <vector>
#include <gtsam/geometry/Pose3.h>
#include <opencv2/opencv.hpp>
#include "aruco.h"


namespace localization_boat_assisted {

class ArUcoMarker {

public:
    //constants
    aruco::CameraParameters camera_parameters_;
    cv::Mat distortion_coefficient_;
    cv::Mat camera_matrix_;
    double marker_size_;
    aruco::MarkerDetector MDetector;
    int id_;
    std::vector<gtsam::Point2> corner_pixel_measurements_;
    gtsam::Pose3 boat_pose_;  //marker transformations obtained from the aruco marker

    /**
     * Constructor
     * */
    ArUcoMarker(){

    }



};

#endif //SRC_ARUCOMARKER_H

} //end namesapce

