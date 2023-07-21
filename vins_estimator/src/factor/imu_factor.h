/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#pragma once
#include <iostream>
#include <eigen3/Eigen/Dense>
#include <ceres/ceres.h>
#include "integration_base.h"


class IMUFactor : public ceres::SizedCostFunction<15, 7, 9, 7, 9>
{
  public:
    IMUFactor() = delete;
    IMUFactor(IntegrationBase* _pre_integration);
    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;

    IntegrationBase* pre_integration;
};


