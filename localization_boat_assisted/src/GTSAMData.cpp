//
// Created by alg on 12/06/20.
//
#include "localization_boat_assisted/GTSAMData.h"
#include "gtsam/base/Vector.h"

void localization_boat_assisted::GTSAMData::setBodyPSensor(){
    YAML::Node gtsam_config = YAML::LoadFile(this->gtsam_config_filename_);
    std::vector<double> double_body_p_sensor = gtsam_config["body_p_sensor"].as<std::vector<double> >();
    Eigen::Matrix4d temp_body_p_sensor;
    temp_body_p_sensor << double_body_p_sensor[0], double_body_p_sensor[1], double_body_p_sensor[2], double_body_p_sensor[3],
                            double_body_p_sensor[4], double_body_p_sensor[5], double_body_p_sensor[6], double_body_p_sensor[7],
                            double_body_p_sensor[8], double_body_p_sensor[9], double_body_p_sensor[10], double_body_p_sensor[11],
                            double_body_p_sensor[12], double_body_p_sensor[13], double_body_p_sensor[14], double_body_p_sensor[15];

    gtsam::Pose3 gtsam_body_p_sensor(temp_body_p_sensor);
    this->body_p_sensor_ = gtsam_body_p_sensor;
}

void localization_boat_assisted::GTSAMData::setK() {
    YAML::Node gtsam_config = YAML::LoadFile(this->gtsam_config_filename_);
    std::vector<double> vector_camera_matrix = gtsam_config["k"].as<std::vector<double> >();
    gtsam::Cal3_S2::shared_ptr K(new gtsam::Cal3_S2(vector_camera_matrix[0], vector_camera_matrix[1],
                                                    vector_camera_matrix[2],
                                                    vector_camera_matrix[3], vector_camera_matrix[4]));
    this->K_ = K;

}

void localization_boat_assisted::GTSAMData::getYamlFile() {

    //FIXME: Hackiest Hackjob for the filename. But would work in the general case.
    //TODO: Find a better way to pass the config.yaml file
    std::string current_file_path = __FILE__;
    unsigned int size_current_file_path = current_file_path.size();
    std::string initial_part = current_file_path.substr(0, size_current_file_path - 17);
    std::string gtsam_config_file = initial_part +  "config/gtsam_config.yaml";
    this->gtsam_config_filename_ = gtsam_config_file;
}

void localization_boat_assisted::GTSAMData::setLMParams() {
    YAML::Node gtsam_config = YAML::LoadFile(this->gtsam_config_filename_);
    this->lm_params_.setVerbosity("ERROR");
    this->lm_params_.setRelativeErrorTol(gtsam_config["relative_tolerance"].as<double_t >());


}

void localization_boat_assisted::GTSAMData::setNoiseValues() {
    YAML::Node gtsam_config = YAML::LoadFile(this->gtsam_config_filename_);
    double pixel_noise = gtsam_config["pixel_noise"].as<double_t >();
    std::vector<double> boat_pose_noise = gtsam_config["boat_pose_noise"].as<std::vector<double> >();
    std::vector<double> drone_odometry_noise = gtsam_config["drone_odometry_noise"].as<std::vector<double> >();
    std::vector<double> prior_pose_noise = gtsam_config["prior_pose_noise"].as<std::vector<double> >();

    this->marker_pixel_noise_ = gtsam::noiseModel::Isotropic::Sigma(8, pixel_noise);

    this->boat_pose_noise_ = gtsam::noiseModel::Diagonal::Sigmas
            ((gtsam::Vector(6) << boat_pose_noise[0], boat_pose_noise[1], boat_pose_noise[2],
                    boat_pose_noise[3], boat_pose_noise[4], boat_pose_noise[5]).finished());

    this->drone_odometry_noise_ = gtsam::noiseModel::Diagonal::Sigmas
            ((gtsam::Vector(6) << drone_odometry_noise[0], drone_odometry_noise[1], drone_odometry_noise[2],
             drone_odometry_noise[3], drone_odometry_noise[4], drone_odometry_noise[5]).finished());

    this->pose_prior_noise_ = gtsam::noiseModel::Diagonal::Sigmas(
            (gtsam::Vector(6) << prior_pose_noise[0], prior_pose_noise[1], prior_pose_noise[2],
                    prior_pose_noise[3], prior_pose_noise[4], prior_pose_noise[5]).finished());

}