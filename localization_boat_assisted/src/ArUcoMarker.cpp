//
// Created by alg on 15/06/20.
//

#include <yaml-cpp/yaml.h>
#include "localization_boat_assisted/ArUcoMarker.h"


localization_boat_assisted::ArUcoMarker::ArUcoMarker() { //TODO: Use the general path. Also, this might not work
    camera_parameters_.readFromXMLFile("localization_boat_assisted/localization_boat_assisted/localization_boat_assisted/config/aruco_calib_params.yml");
    distortion_coefficient_ = camera_parameters_.Distorsion;
    camera_matrix_ = camera_parameters_.CameraMatrix;

    //TODO use the config file for assigning the marker size
    //TODO: most probably wrong file name
    YAML::Node aruco_config = YAML::LoadFile("localization_boat_assisted/localization_boat_assisted/localization_boat_assisted/config/default.yaml");
    this->marker_size_ = aruco_config["aruco_marker_size"].as<double_t >();

}

void localization_boat_assisted::ArUcoMarker::detectMarkersInImage(cv::Mat image) {
    this->all_marker_in_frame_ = this->marker_detector.detect(image, this->camera_parameters_, this->marker_size_);
    this->id_ = all_marker_in_frame_[0].id;

    this->corner_pixel_measurements_.push_back(gtsam::Point2(this->all_marker_in_frame_[0][0].x, this->all_marker_in_frame_[0][0].y));
    this->corner_pixel_measurements_.push_back(gtsam::Point2(this->all_marker_in_frame_[0][1].x, this->all_marker_in_frame_[0][1].y));
    this->corner_pixel_measurements_.push_back(gtsam::Point2(this->all_marker_in_frame_[0][2].x, this->all_marker_in_frame_[0][2].y));
    this->corner_pixel_measurements_.push_back(gtsam::Point2(this->all_marker_in_frame_[0][3].x, this->all_marker_in_frame_[0][3].y));

    //set boat pose wrt camera
    cv::Mat cv_rot = this->all_marker_in_frame_[0].Rvec;
    cv::Mat cv_trans = this->all_marker_in_frame_[0].Tvec;
    gtsam::Rot3 gtsam_rot(  cv_rot.at<double_t>(0, 0), cv_rot.at<double_t>(0,1), cv_rot.at<double_t >(0,2),
                            cv_rot.at<double_t >(1,0), cv_rot.at<double_t >(1,1), cv_rot.at<double_t >(1,2),
                            cv_rot.at<double_t >(2,0), cv_rot.at<double_t>(2,1), cv_rot.at<double_t >(2,2));
    gtsam::Point3 gtsam_trans(cv_trans.at<double_t >(0), cv_trans.at<double_t >(1), cv_trans.at<double_t >(2));
    this->marker_pose_wrt_camera = gtsam::Pose3(gtsam_rot, gtsam_trans );

}
