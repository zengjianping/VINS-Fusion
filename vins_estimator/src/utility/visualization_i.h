/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#pragma once
#include <eigen3/Eigen/Dense>
#include "../estimator/estimator.h"
#include <fstream>


void pubLatestOdometry(const Eigen::Vector3d &P, const Eigen::Quaterniond &Q, const Eigen::Vector3d &V, double t);

void pubTrackImage(const cv::Mat &imgTrack, const double t);

void printStatistics(const Estimator &estimator, double t);

void pubOdometry(const Estimator &estimator, const string& frame_id, double timestamp);

void pubInitialGuess(const Estimator &estimator, const string& frame_id, double timestamp);

void pubKeyPoses(const Estimator &estimator, const string& frame_id, double timestamp);

void pubCameraPose(const Estimator &estimator, const string& frame_id, double timestamp);

void pubPointCloud(const Estimator &estimator, const string& frame_id, double timestamp);

void pubTF(const Estimator &estimator, const string& frame_id, double timestamp);

void pubKeyframe(const Estimator &estimator);

void pubRelocalization(const Estimator &estimator);

void pubCar(const Estimator & estimator, const string& frame_id, double timestamp);

