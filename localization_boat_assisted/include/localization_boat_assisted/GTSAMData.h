//
// Created by alg on 11/06/20.
//

#ifndef SRC_GTSAMDATA_H
#define SRC_GTSAMDATA_H
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtParams.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <yaml-cpp/yaml.h>

//including this for getting current directory //FIXME: change this for a better way to get current directory
#include <stdio.h>
#include <unistd.h>
#define GetCurrentDir getcwd

namespace localization_boat_assisted{
struct GTSAMData {
public:
    /** Camera parameters from config file */
    gtsam::Cal3_S2::shared_ptr K_;

    /** Transformation from base_link to camera_link */
    gtsam::Pose3 body_p_sensor_;

    /** Optimization params */
    gtsam::LevenbergMarquardtParams lm_params_;

    /**  Main factor graph*/
    gtsam::NonlinearFactorGraph factor_graph_;

    /** Initial and final estimates obtained from factor graph*/
    gtsam::Values initial_estimate_;
    gtsam::Values result_;

    /** Prior Values */
    gtsam::Pose3 prior_pose_drone_ ;
    gtsam::Pose3 prior_pose_boat_;


    /** Noise values*/
    gtsam::noiseModel::Isotropic::shared_ptr pixel_noise_;
    gtsam::noiseModel::Diagonal::shared_ptr drone_odometry_noise_; //between_drone noise
    gtsam::noiseModel::Diagonal::shared_ptr boat_pose_noise_; //between_marker_noise

    /** Noise Pose Prior*/
    gtsam::noiseModel::Diagonal::shared_ptr pose_prior_noise_;


    /** Constructor */
    GTSAMData(){
        //getting the config filename
        char buff[FILENAME_MAX];
        getcwd(buff, FILENAME_MAX);
        std::string current_working_dir(buff);
        std::string gtsam_config_file = current_working_dir + "../../config/gtsam_config.yaml";
        YAML::Node gtsam_config = YAML::LoadFile(gtsam_config_file);
        double fx = gtsam_config["f_x"].as<double_t >();
        double fy = gtsam_config["f_y"].as<double_t >();
        double s = gtsam_config["s"].as<double_t >();
        double ux = gtsam_config["u_x"].as<double_t >();
        double uy = gtsam_config["u_y"].as<double_t >();

        std::cout<<"focalx: "<<fx<<std::endl;





    }






};

}


#endif //SRC_GTSAMDATA_H
