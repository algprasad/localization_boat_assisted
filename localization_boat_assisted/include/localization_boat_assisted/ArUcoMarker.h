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
/**
 * ArUco marker class for handling the aruco marker in the current image frame
 * */
class ArUcoMarker {

public:
    //constants
    aruco::CameraParameters camera_parameters_;
    cv::Mat distortion_coefficient_;
    cv::Mat camera_matrix_;
    double marker_size_;
    aruco::MarkerDetector marker_detector;
    int id_;
    std::vector<aruco::Marker> all_marker_in_frame_; //list of all markers in frame // for the boat assisted localization, size =  1
    std::vector<gtsam::Point2> corner_pixel_measurements_;
    gtsam::Pose3 marker_pose_wrt_camera;  //marker transformations obtained from the aruco marker

    /**
     * Constructor
     * */
    ArUcoMarker(); //add values to the parameters from the yml fil


    /**
     * @param image is the current image
     * sets the value of all vector  all_markers_in_frame_
     * sets value of id of the marker
     * sets value of boat_pose wrt camera
     * */
    void detectMarkersInImage(cv::Mat image);

};

#endif //SRC_ARUCOMARKER_H

} //end namesapce

