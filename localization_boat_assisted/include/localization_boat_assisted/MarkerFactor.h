/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/** Marker Factor based on ProjectionFactor.
 * Modified by ALG.
 * */

#ifndef ISAM2_MARKERFACTOR_H
#define ISAM2_MARKERFACTOR_H


#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/geometry/SimpleCamera.h>
#include <boost/optional.hpp>
//#include "InitialParameters.h"


#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/CalibratedCamera.h>


namespace gtsam{
    /*Non-linear factor for Factor between robot pose and ArUco landmark*/

    template<class POSE, class LANDMARK, class CALIBRATION = Cal3_S2>
    class MarkerFactor: public NoiseModelFactor2<POSE, LANDMARK> {
    protected:

        // Keep a copy of measurement and calibration for I/O
        Vector8 measured_;   ///< 2D measurement

        //individual measurements of points
        Point2 measured_tl_;
        Point2 measured_tr_;
        Point2 measured_br_;
        Point2 measured_bl_;

        boost::shared_ptr<CALIBRATION> K_;  ///< shared pointer to calibration object
        boost::optional<POSE> body_P_sensor_; ///< The pose of the sensor in the body frame

        // verbosity handling for Cheirality Exceptions
        bool throwCheirality_; ///< If true, rethrows Cheirality exceptions (default: false)
        bool verboseCheirality_; ///< If true, prints text for Cheirality exceptions (default: false)


    public:
        //Marker Size
        double marker_size_; ///Fiducial marker side length (Assuming that the fiducial marker is a square)

        /// shorthand for base class type
        typedef NoiseModelFactor2<POSE, LANDMARK> Base;

        /// shorthand for this class
        typedef MarkerFactor<POSE, LANDMARK, CALIBRATION> This;

        /// shorthand for a smart pointer to a factor
        typedef boost::shared_ptr<This> shared_ptr;

        /// Default constructor
        MarkerFactor() :
                 throwCheirality_(false), verboseCheirality_(false), measured_tl_(0,0), measured_tr_(0,0), measured_br_(0,0), measured_bl_(0,0){



            measured_ << 0, 0, 0, 0, 0, 0, 0, 0;

        }

            /**
         * Constructor
         * TODO: Mark argument order standard (keys, measurement, parameters)
         * @param measured is vector of 4 corner points
         * @param model is the standard deviation
         * @param poseKey is the index of the base_link
         * @param markerKey is the index of the ArUco marker
         * @param K shared pointer to the constant calibration
         * @param body_P_sensor is the transform from body to sensor frame (default identity)
         */

        MarkerFactor(const Point2& measured_tl, const Point2& measured_tr, const Point2& measured_br, const Point2& measured_bl,
                     const SharedNoiseModel& model,
                     Key poseKey, Key markerKey, const boost::shared_ptr<CALIBRATION>& K, double marker_size,
                     boost::optional<POSE> body_P_sensor = boost::none) :
                Base(model, poseKey, markerKey),
                measured_tl_(measured_tl), measured_tr_(measured_tr), measured_br_(measured_br), measured_bl_(measured_bl),
                K_(K), marker_size_(marker_size), body_P_sensor_(body_P_sensor),
                throwCheirality_(false), verboseCheirality_(false) {



            measured_ <<    measured_tl_.x(), measured_tl_.y(),
                            measured_tr_.x(), measured_tr_.y(),
                            measured_br_.x(), measured_br_.y(),
                            measured_bl_.x(), measured_bl_.y();

        }

            /**
        * Constructor with exception-handling flags
        * TODO: Mark argument order standard (keys, measurement, parameters)
        * @param measured is vector of 4 corner points
        * @param model is the standard deviation
        * @param poseKey is the index of the base_link
        * @param markerKey is the index of the ArUco marker
        * @param K shared pointer to the constant calibration
        * @param throwCheirality determines whether Cheirality exceptions are rethrown
        * @param verboseCheirality determines whether exceptions are printed for Cheirality
        * @param body_P_sensor is the transform from body to sensor frame  (default identity)

        */

        MarkerFactor(const Point2& measured_tl, const Point2& measured_tr, const Point2& measured_br, const Point2& measured_bl,
                     const SharedNoiseModel& model,
                     Key poseKey, Key markerKey, const boost::shared_ptr<CALIBRATION>& K, double marker_size,
                     bool throwCheirality, bool verboseCheirality,
                     boost::optional<POSE> body_P_sensor = boost::none) :
                    Base(model, poseKey, markerKey),
                    measured_tl_(measured_tl), measured_tr_(measured_tr), measured_br_(measured_br), measured_bl_(measured_bl), K_(K), marker_size_(marker_size),
                    body_P_sensor_(body_P_sensor),
                    throwCheirality_(throwCheirality), verboseCheirality_(verboseCheirality) {


                measured_ <<    measured_tl_.x(), measured_tl_.y(),
                                measured_tr_.x(), measured_tr_.y(),
                                measured_br_.x(), measured_br_.y(),
                                measured_bl_.x(), measured_bl_.y();

            }



        /** Virtual destructor */
        virtual ~MarkerFactor() {}

        /// @return a deep copy of this factor
        virtual gtsam::NonlinearFactor::shared_ptr clone() const {
            return boost::static_pointer_cast<gtsam::NonlinearFactor>(
                    gtsam::NonlinearFactor::shared_ptr(new This(*this))); }


        /**
         * print
         * @param s optional string naming the factor
         * @param keyFormatter optional formatter useful for printing Symbols
         */
        void print(const std::string& s = "", const KeyFormatter& keyFormatter = DefaultKeyFormatter) const {
            std::cout << s << "MarkerFactor, z = ";
            traits<Vector8>::Print(measured_);
            /*traits<Point2>::Print(measured_[1]);
            traits<Point2>::Print(measured_[2]);
            traits<Point2>::Print(measured_[3]);*/
            if(this->body_P_sensor_)
                this->body_P_sensor_->print("  sensor pose in body frame: ");
            Base::print("", keyFormatter);
        }


        /// equals
        virtual bool equals(const NonlinearFactor& p, double tol = 1e-9) const {
            const This *e = dynamic_cast<const This*>(&p);
            return e
                   && Base::equals(p, tol)
                   && traits<Vector8>::Equals(this->measured_, e->measured_, tol)
                   /*&& traits<Point2>::Equals(this->measured_[1], e->measured_[1], tol)
                   && traits<Point2>::Equals(this->measured_[2], e->measured_[2], tol)
                   && traits<Point2>::Equals(this->measured_[3], e->measured_[3], tol)*/
                   && this->K_->equals(*e->K_, tol)
                   && ((!body_P_sensor_ && !e->body_P_sensor_) || (body_P_sensor_ && e->body_P_sensor_ && body_P_sensor_->equals(*e->body_P_sensor_)));
        }



        /// Evaluate error h(x)-z and optionally derivatives
        Vector evaluateError(const Pose3& pose, const Pose3& landmark,
                             boost::optional<Matrix&> H1 = boost::none, boost::optional<Matrix&> H2 = boost::none) const {


            try {
                if (body_P_sensor_) {
                    if (H1) {
                        gtsam::Matrix H0;
                        PinholeCamera<CALIBRATION> camera(pose.compose(*body_P_sensor_, H0), *K_);

                        //h21_tl is the 3x6 Jacobian matrix that we get for transforming the corner in world coordinates from landmark coordinate frame
                        gtsam::Matrix36 h21_tl, h21_tr, h21_br, h21_bl;

                        //h22_tl is the 2x3 jacobian matrix that represents derivative of the pixel coordinates w.r.t to the corner point
                        gtsam::Matrix23 h22_tl, h22_tr, h22_br, h22_bl;

                        Point3 top_left(-marker_size_ / 2, marker_size_ / 2, 0),
                                top_right(marker_size_ / 2, marker_size_ / 2, 0),
                                bottom_right(marker_size_ / 2, -marker_size_ / 2, 0),
                                bottom_left(-marker_size_ / 2, -marker_size_ / 2, 0);

                        Point3 w_tl = landmark.transform_from(top_left, h21_tl);
                        Point3 w_tr = landmark.transform_from(top_right, h21_tr);
                        Point3 w_br = landmark.transform_from(bottom_right, h21_br);
                        Point3 w_bl = landmark.transform_from(bottom_left, h21_bl);

                        //h11_tl is the 2x6 jacobian matrix representing derivative of pixel coordinates wrt the pose of the robot
                        gtsam::Matrix26 h11_tl, h11_tr, h11_br, h11_bl;

                        Point2 reprojection_tl(camera.project(w_tl, h11_tl, h22_tl, boost::none) - measured_tl_);
                        Point2 reprojection_tr(camera.project(w_tr, h11_tr, h22_tr, boost::none) - measured_tr_);
                        Point2 reprojection_br(camera.project(w_br, h11_br, h22_br, boost::none) - measured_br_);
                        Point2 reprojection_bl(camera.project(w_bl, h11_bl, h22_bl, boost::none) - measured_bl_);

                        h11_tl = h11_tl * H0;
                        h11_tr = h11_tr * H0;
                        h11_br = h11_br * H0;
                        h11_bl = h11_bl * H0;



                        if(H1){ 
                            
                           // setH(H1, h11_tl, h11_tr, h11_br, h11_bl);
                        
                            *H1 = (gtsam::Matrix86() <<  h11_tl(0,0), h11_tl(0,1), h11_tl(0,2), h11_tl(0,3), h11_tl(0,4), h11_tl(0,5),
                                    h11_tl(1,0), h11_tl(1,1), h11_tl(1,2), h11_tl(1,3), h11_tl(1,4), h11_tl(1,5),
                                    h11_tr(0,0), h11_tr(0,1), h11_tr(0,2), h11_tr(0,3), h11_tr(0,4), h11_tr(0,5),
                                    h11_tr(1,0), h11_tr(1,1), h11_tr(1,2), h11_tr(1,3), h11_tr(1,4), h11_tr(1,5),
                                    h11_br(0,0), h11_br(0,1), h11_br(0,2), h11_br(0,3), h11_br(0,4), h11_br(0,5),
                                    h11_br(1,0), h11_br(1,1), h11_br(1,2), h11_br(1,3), h11_br(1,4), h11_br(1,5),
                                    h11_bl(0,0), h11_bl(0,1), h11_bl(0,2), h11_bl(0,3), h11_bl(0,4), h11_bl(0,5),
                                    h11_bl(1,0), h11_bl(1,1), h11_bl(1,2), h11_bl(1,3), h11_bl(1,4), h11_bl(1,5)).finished();

                            /*(*H1) << 1, 0, 0, 0, 0, 0,
                                    0, 1, 0, 0, 0, 0,
                                    0, 0, 1, 0, 0, 0,
                                    0, 0, 0, 1, 0, 0,
                                    0, 0, 0, 0, 1, 0,
                                    0, 0, 0, 0, 0, 1,
                                    0, 0, 0, 0, 0 ,1,
                                    0, 0, 0, 0, 0, 1;*/

                        }


                        if(H2){

                            gtsam::Matrix26 h2_tl, h2_tr, h2_br, h2_bl;
                            h2_tl = h22_tl * h21_tl;
                            h2_tr = h22_tr * h21_tr,
                            h2_br = h22_br * h21_br,
                            h2_bl = h22_bl * h21_bl;

                            //setH(H2, h2_tl, h2_tr, h2_br, h2_bl);

                            *H2 = (gtsam::Matrix86() << h2_tl(0,0), h2_tl(0,1), h2_tl(0,2), h2_tl(0,3), h2_tl(0,4), h2_tl(0,5),
                                    h2_tl(1,0), h2_tl(1,1), h2_tl(1,2), h2_tl(1,3), h2_tl(1,4), h2_tl(1,5),
                                    h2_tr(0,0), h2_tr(0,1), h2_tr(0,2), h2_tr(0,3), h2_tr(0,4), h2_tr(0,5),
                                    h2_tr(1,0), h2_tr(1,1), h2_tr(1,2), h2_tr(1,3), h2_tr(1,4), h2_tr(1,5),
                                    h2_br(0,0), h2_br(0,1), h2_br(0,2), h2_br(0,3), h2_br(0,4), h2_br(0,5),
                                    h2_br(1,0), h2_br(1,1), h2_br(1,2), h2_br(1,3), h2_br(1,4), h2_br(1,5),
                                    h2_bl(0,0), h2_bl(0,1), h2_bl(0,2), h2_bl(0,3), h2_bl(0,4), h2_bl(0,5),
                                    h2_bl(1,0), h2_bl(1,1), h2_bl(1,2), h2_bl(1,3), h2_bl(1,4), h2_bl(1,5)).finished();


                            /*(*H2) << 1, 0, 0, 0, 0, 0,
                                    0, 1, 0, 0, 0, 0,
                                    0, 0, 1, 0, 0, 0,
                                    0, 0, 0, 1, 0, 0,
                                    0, 0, 0, 0, 1, 0,
                                    0, 0, 0, 0, 0, 1,
                                    0, 0, 0, 0, 0 ,1,
                                    0, 0, 0, 0, 0, 1;*/

                        }



                        Vector8 measurement_error;
                        measurement_error <<    reprojection_tl.x(), reprojection_tl.y(),
                                reprojection_tr.x(), reprojection_tr.y(),
                                reprojection_br.x(), reprojection_br.y(),
                                reprojection_bl.x(), reprojection_bl.y();
                        return measurement_error;

                    } else { //i.e. if(!H1)

                        PinholeCamera<CALIBRATION> camera(pose.compose(*body_P_sensor_), *K_);

                        //h21_tl is the 3x6 Jacobian matrix that we get for transforming the corner in world coordinates from landmark coordinate frame
                        gtsam::Matrix36 h21_tl, h21_tr, h21_br, h21_bl;

                        //h22_tl is the 2x3 jacobian matrix that represents derivative of the pixel coordinates w.r.t to the corner point
                        gtsam::Matrix23 h22_tl, h22_tr, h22_br, h22_bl;

                        Point3 top_left(-marker_size_ / 2, marker_size_ / 2, 0), top_right(marker_size_ / 2,
                                                                                           marker_size_ / 2, 0),
                                bottom_right(marker_size_ / 2, -marker_size_ / 2, 0), bottom_left(-marker_size_ / 2,
                                                                                                  marker_size_ / 2, 0);

                        Point3 w_tl = landmark.transform_from(top_left, h21_tl);
                        Point3 w_tr = landmark.transform_from(top_right, h21_tr);
                        Point3 w_br = landmark.transform_from(bottom_right, h21_br);
                        Point3 w_bl = landmark.transform_from(bottom_left, h21_bl);

                        //h11_tl is the 2x6 jacobian matrix representing derivative of pixel coordinates wrt the pose of the robot
                        gtsam::Matrix26 h11_tl, h11_tr, h11_br, h11_bl;

                        Point2 reprojection_tl(camera.project(w_tl, h11_tl, h22_tl, boost::none) - measured_tl_);
                        Point2 reprojection_tr(camera.project(w_tr, h11_tr, h22_tr, boost::none) - measured_tr_);
                        Point2 reprojection_br(camera.project(w_br, h11_br, h22_br, boost::none) - measured_br_);
                        Point2 reprojection_bl(camera.project(w_bl, h11_bl, h22_bl, boost::none) - measured_bl_);


                        if(H1){
                            //setH(H1, h11_tl, h11_tr, h11_br, h11_bl);

                            *H1  = (gtsam::Matrix86() <<  h11_tl(0,0), h11_tl(0,1), h11_tl(0,2), h11_tl(0,3), h11_tl(0,4), h11_tl(0,5),
                                    h11_tl(1,0), h11_tl(1,1), h11_tl(1,2), h11_tl(1,3), h11_tl(1,4), h11_tl(1,5),
                                    h11_tr(0,0), h11_tr(0,1), h11_tr(0,2), h11_tr(0,3), h11_tr(0,4), h11_tr(0,5),
                                    h11_tr(1,0), h11_tr(1,1), h11_tr(1,2), h11_tr(1,3), h11_tr(1,4), h11_tr(1,5),
                                    h11_br(0,0), h11_br(0,1), h11_br(0,2), h11_br(0,3), h11_br(0,4), h11_br(0,5),
                                    h11_br(1,0), h11_br(1,1), h11_br(1,2), h11_br(1,3), h11_br(1,4), h11_br(1,5),
                                    h11_bl(0,0), h11_bl(0,1), h11_bl(0,2), h11_bl(0,3), h11_bl(0,4), h11_bl(0,5),
                                    h11_bl(1,0), h11_bl(1,1), h11_bl(1,2), h11_bl(1,3), h11_bl(1,4), h11_bl(1,5)).finished();

                            /*(*H1) << 1, 0, 0, 0, 0, 0,
                                    0, 1, 0, 0, 0, 0,
                                    0, 0, 1, 0, 0, 0,
                                    0, 0, 0, 1, 0, 0,
                                    0, 0, 0, 0, 1, 0,
                                    0, 0, 0, 0, 0, 1,
                                    0, 0, 0, 0, 0 ,1,
                                    0, 0, 0, 0, 0, 1;*/

                        }


                        if(H2){
                            

                            gtsam::Matrix26 h2_tl, h2_tr, h2_br, h2_bl;
                            h2_tl = h22_tl * h21_tl;
                            h2_tr = h22_tr * h21_tr,
                            h2_br = h22_br * h21_br,
                            h2_bl = h22_bl * h21_bl;

                            //setH(H2, h2_tl, h2_tr, h2_br, h2_bl);


                            *H2 = (gtsam::Matrix86() << h2_tl(0,0), h2_tl(0,1), h2_tl(0,2), h2_tl(0,3), h2_tl(0,4), h2_tl(0,5),
                                    h2_tl(1,0), h2_tl(1,1), h2_tl(1,2), h2_tl(1,3), h2_tl(1,4), h2_tl(1,5),
                                    h2_tr(0,0), h2_tr(0,1), h2_tr(0,2), h2_tr(0,3), h2_tr(0,4), h2_tr(0,5),
                                    h2_tr(1,0), h2_tr(1,1), h2_tr(1,2), h2_tr(1,3), h2_tr(1,4), h2_tr(1,5),
                                    h2_br(0,0), h2_br(0,1), h2_br(0,2), h2_br(0,3), h2_br(0,4), h2_br(0,5),
                                    h2_br(1,0), h2_br(1,1), h2_br(1,2), h2_br(1,3), h2_br(1,4), h2_br(1,5),
                                    h2_bl(0,0), h2_bl(0,1), h2_bl(0,2), h2_bl(0,3), h2_bl(0,4), h2_bl(0,5),
                                    h2_bl(1,0), h2_bl(1,1), h2_bl(1,2), h2_bl(1,3), h2_bl(1,4), h2_bl(1,5)).finished();


                            /*(*H2) << 1, 0, 0, 0, 0, 0,
                                    0, 1, 0, 0, 0, 0,
                                    0, 0, 1, 0, 0, 0,
                                    0, 0, 0, 1, 0, 0,
                                    0, 0, 0, 0, 1, 0,
                                    0, 0, 0, 0, 0, 1,
                                    0, 0, 0, 0, 0 ,1,
                                    0, 0, 0, 0, 0, 1;*/

                        }

                        Vector8 measurement_error;
                        measurement_error <<    reprojection_tl.x(), reprojection_tl.y(),
                                reprojection_tr.x(), reprojection_tr.y(),
                                reprojection_br.x(), reprojection_br.y(),
                                reprojection_bl.x(), reprojection_bl.y();
                        return measurement_error;


                    }
                }

                    else{ //i.e. if(!body_p_sensor)

                        PinholeCamera<CALIBRATION> camera(pose, *K_);

                        //h21_tl is the 3x6 Jacobian matrix that we get for transforming the corner in world coordinates from landmark coordinate frame
                        gtsam::Matrix36 h21_tl, h21_tr, h21_br, h21_bl;

                        //h22_tl is the 2x3 jacobian matrix that represents derivative of the pixel coordinates w.r.t to the corner point
                        gtsam::Matrix23 h22_tl, h22_tr, h22_br, h22_bl;

                        Point3 top_left(-marker_size_ / 2, marker_size_ / 2, 0), top_right(marker_size_ / 2,
                                                                                           marker_size_ / 2, 0),
                                bottom_right(marker_size_ / 2, -marker_size_ / 2, 0), bottom_left(-marker_size_ / 2,
                                                                                                  marker_size_ / 2, 0);

                        Point3 w_tl = landmark.transform_from(top_left, h21_tl);
                        Point3 w_tr = landmark.transform_from(top_right, h21_tr);
                        Point3 w_br = landmark.transform_from(bottom_right, h21_br);
                        Point3 w_bl = landmark.transform_from(bottom_left, h21_bl);

                        //h11_tl is the 2x6 jacobian matrix representing derivative of pixel coordinates wrt the pose of the robot
                        gtsam::Matrix26 h11_tl, h11_tr, h11_br, h11_bl;

                        Point2 reprojection_tl(camera.project(w_tl, h11_tl, h22_tl, boost::none) - measured_tl_);
                        Point2 reprojection_tr(camera.project(w_tr, h11_tr, h22_tr, boost::none) - measured_tr_);
                        Point2 reprojection_br(camera.project(w_br, h11_br, h22_br, boost::none) - measured_br_);
                        Point2 reprojection_bl(camera.project(w_bl, h11_bl, h22_bl, boost::none) - measured_bl_);

                        //TODO(ALG): Need to initialize H1 and H2 before dereferencing --Maybe NOT


                        if(H1){

                            //setH(H1, h11_tl, h11_tr, h11_br, h11_bl);


                            *H1 = (gtsam::Matrix86() <<  h11_tl(0,0), h11_tl(0,1), h11_tl(0,2), h11_tl(0,3), h11_tl(0,4), h11_tl(0,5),
                                    h11_tl(1,0), h11_tl(1,1), h11_tl(1,2), h11_tl(1,3), h11_tl(1,4), h11_tl(1,5),
                                    h11_tr(0,0), h11_tr(0,1), h11_tr(0,2), h11_tr(0,3), h11_tr(0,4), h11_tr(0,5),
                                    h11_tr(1,0), h11_tr(1,1), h11_tr(1,2), h11_tr(1,3), h11_tr(1,4), h11_tr(1,5),
                                    h11_br(0,0), h11_br(0,1), h11_br(0,2), h11_br(0,3), h11_br(0,4), h11_br(0,5),
                                    h11_br(1,0), h11_br(1,1), h11_br(1,2), h11_br(1,3), h11_br(1,4), h11_br(1,5),
                                    h11_bl(0,0), h11_bl(0,1), h11_bl(0,2), h11_bl(0,3), h11_bl(0,4), h11_bl(0,5),
                                    h11_bl(1,0), h11_bl(1,1), h11_bl(1,2), h11_bl(1,3), h11_bl(1,4), h11_bl(1,5)).finished();

                            /*(*H1) << 1, 0, 0, 0, 0, 0,
                                     0, 1, 0, 0, 0, 0,
                                     0, 0, 1, 0, 0, 0,
                                     0, 0, 0, 1, 0, 0,
                                     0, 0, 0, 0, 1, 0,
                                     0, 0, 0, 0, 0, 1,
                                     0, 0, 0, 0, 0 ,1,
                                     0, 0, 0, 0, 0, 1;*/

                        }


                        if(H2){

                            gtsam::Matrix26 h2_tl, h2_tr, h2_br, h2_bl;
                            h2_tl = h22_tl * h21_tl;
                            h2_tr = h22_tr * h21_tr,
                            h2_br = h22_br * h21_br,
                            h2_bl = h22_bl * h21_bl;

                            //setH(H2, h2_tl, h2_tr, h2_br, h2_bl);
                            
                            

                            *H2 = (gtsam::Matrix86() << h2_tl(0,0), h2_tl(0,1), h2_tl(0,2), h2_tl(0,3), h2_tl(0,4), h2_tl(0,5),
                                    h2_tl(1,0), h2_tl(1,1), h2_tl(1,2), h2_tl(1,3), h2_tl(1,4), h2_tl(1,5),
                                    h2_tr(0,0), h2_tr(0,1), h2_tr(0,2), h2_tr(0,3), h2_tr(0,4), h2_tr(0,5),
                                    h2_tr(1,0), h2_tr(1,1), h2_tr(1,2), h2_tr(1,3), h2_tr(1,4), h2_tr(1,5),
                                    h2_br(0,0), h2_br(0,1), h2_br(0,2), h2_br(0,3), h2_br(0,4), h2_br(0,5),
                                    h2_br(1,0), h2_br(1,1), h2_br(1,2), h2_br(1,3), h2_br(1,4), h2_br(1,5),
                                    h2_bl(0,0), h2_bl(0,1), h2_bl(0,2), h2_bl(0,3), h2_bl(0,4), h2_bl(0,5),
                                    h2_bl(1,0), h2_bl(1,1), h2_bl(1,2), h2_bl(1,3), h2_bl(1,4), h2_bl(1,5)).finished();


                            /*(*H2) << 1, 0, 0, 0, 0, 0,
                                    0, 1, 0, 0, 0, 0,
                                    0, 0, 1, 0, 0, 0,
                                    0, 0, 0, 1, 0, 0,
                                    0, 0, 0, 0, 1, 0,
                                    0, 0, 0, 0, 0, 1,
                                    0, 0, 0, 0, 0 ,1,
                                    0, 0, 0, 0, 0, 1;*/

                        }


                    Vector8 measurement_error;
                        measurement_error <<    reprojection_tl.x(), reprojection_tl.y(),
                                reprojection_tr.x(), reprojection_tr.y(),
                                reprojection_br.x(), reprojection_br.y(),
                                reprojection_bl.x(), reprojection_bl.y();
                        return measurement_error;

                    }

                } catch( CheiralityException& e) {
                if (H1) *H1 = Matrix::Zero(8,6);
                if (H2) *H2 = Matrix::Zero(8,6);
                if (verboseCheirality_)
                    std::cout << e.what() << ": Landmark "<< DefaultKeyFormatter(this->key2()) <<
                              " moved behind camera " << DefaultKeyFormatter(this->key1()) << std::endl;
                if (throwCheirality_)
                    throw e;
            }

            return Vector8::Constant(2.0 * K_->fx());
        }
        
        
        
       /* void setH(OptionalJacobian<8, 6> H = boost::none,
                gtsam::Matrix26 h_tl = Matrix::Zero(2,6), 
                gtsam::Matrix26 h_tr = Matrix::Zero(2,6), 
                gtsam::Matrix26 h_br = Matrix::Zero(2,6),
                gtsam::Matrix26 h_bl = Matrix::Zero(2,6)){
            
            if(H){
                (*H) <<  h_tl(0,0), h_tl(0,1), h_tl(0,2), h_tl(0,3), h_tl(0,4), h_tl(0,5),
                        h_tl(1,0), h_tl(1,1), h_tl(1,2), h_tl(1,3), h_tl(1,4), h_tl(1,5),
                        h_tr(0,0), h_tr(0,1), h_tr(0,2), h_tr(0,3), h_tr(0,4), h_tr(0,5),
                        h_tr(1,0), h_tr(1,1), h_tr(1,2), h_tr(1,3), h_tr(1,4), h_tr(1,5),
                        h_br(0,0), h_br(0,1), h_br(0,2), h_br(0,3), h_br(0,4), h_br(0,5),
                        h_br(1,0), h_br(1,1), h_br(1,2), h_br(1,3), h_br(1,4), h_br(1,5),
                        h_bl(0,0), h_bl(0,1), h_bl(0,2), h_bl(0,3), h_bl(0,4), h_bl(0,5),
                        h_bl(1,0), h_bl(1,1), h_bl(1,2), h_bl(1,3), h_bl(1,4), h_bl(1,5);
            }
        
        }*/


        /** return the measurement */
        const Vector8& measured() const {
            return measured_;
        }

        /** return the calibration object */
        inline const boost::shared_ptr<CALIBRATION> calibration() const {
            return K_;
        }

        /** return verbosity */
        inline bool verboseCheirality() const { return verboseCheirality_; }

        /** return flag for throwing cheirality exceptions */
        inline bool throwCheirality() const { return throwCheirality_; }

    private:

        /// Serialization function
        friend class boost::serialization::access;
        template<class ARCHIVE>
        void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
            ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
            ar & BOOST_SERIALIZATION_NVP(measured_);
            ar & BOOST_SERIALIZATION_NVP(K_);
            ar & BOOST_SERIALIZATION_NVP(body_P_sensor_);
            ar & BOOST_SERIALIZATION_NVP(throwCheirality_);
            ar & BOOST_SERIALIZATION_NVP(verboseCheirality_);
        }

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    /// traits
    template<class POSE, class LANDMARK, class CALIBRATION>
    struct traits<MarkerFactor<POSE, LANDMARK, CALIBRATION> > :
            public Testable<MarkerFactor<POSE, LANDMARK, CALIBRATION> > {
    };


}







#endif //ISAM2_MARKERFACTOR_H
