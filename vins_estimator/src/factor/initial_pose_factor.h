/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 * Author: Qin Tong (qintonguav@gmail.com)
 *******************************************************/

#pragma once
#include <ceres/ceres.h>
#include <Eigen/Dense>


class InitialPoseFactor : public ceres::SizedCostFunction<6, 7>
{
  public:
    InitialPoseFactor(const Eigen::Vector3d &_P, const Eigen::Quaterniond &_Q);
    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;

    void check(double **parameters);
    Eigen::Vector3d init_P;
    Eigen::Quaterniond init_Q;
    Eigen::Matrix<double, 6, 6> sqrt_info;
};
