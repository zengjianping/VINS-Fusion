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


class InitialBiasFactor : public ceres::SizedCostFunction<6, 9>
{
  public:
    InitialBiasFactor(const Eigen::Vector3d &_Ba, const Eigen::Vector3d &_Bg);
    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;

    Eigen::Vector3d init_Ba, init_Bg;
    Eigen::Matrix<double, 6, 6> sqrt_info;
};

