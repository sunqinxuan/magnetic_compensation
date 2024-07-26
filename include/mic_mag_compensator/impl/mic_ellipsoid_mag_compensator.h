/*
 * Magnetic Interference Compensation
 *
 * Copyright (C) 2024 Qinxuan Sun. All rights reserved.
 *
 *     Author : Qinxuan Sun
 *    Contact : sunqinxuan@outlook.com
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */
#ifndef MIC_ELLIPSOID_MAG_COMPENSATOR
#define MIC_ELLIPSOID_MAG_COMPENSATOR

#include <Eigen/Eigenvalues>
#include "mic_mag_compensator/mic_mag_compensator.h"

MIC_NAMESPACE_START

class MicEllipsoidMagCompensator;
using mic_ellipsoid_mag_compensator_t = MicEllipsoidMagCompensator;

class MicEllipsoidMagCompensator : public MicMagCompensator
{
public:
    MicEllipsoidMagCompensator() : MicMagCompensator() {}
    virtual ~MicEllipsoidMagCompensator() = default;

    virtual ret_t calibrate() override;
    virtual ret_t compenste() override;

protected:
    /* Ellipsoid fitting algorithm
     *
     * Inputs:
     *   x = nx1 column vector of x coordinates of input data.
     *   y = nx1 column vector of y coordinates of input data.
     *   z = nx1 column vector of y coordinates of input data.
     *
     * Output:
     *   u = [a,b,c,f,g,h,p,q,r,d], a vector corresponding to coefficients of
     *       the general quadric surface given by equation,
     *       ax2 + by2 + cz2 + 2fyz + 2gxz + 2hxy + 2px + 2qy + 2rz + d = 0.
     *
     * Source:
     *   [1] Li - Least Square Ellipsoid Fitting (2004)
     * */
    ret_t ellipsoid_fitting(
        const std::vector<float64_t> &x,
        const std::vector<float64_t> &y,
        const std::vector<float64_t> &z,
        std::vector<float64_t> &coeffs);

    // computation of the compensation model coefficients;
    ret_t compute_model_coeffs(
        const std::vector<float64_t> &ellipsoid_coeffs,
        const float64_t mag_earth_intensity,
        matrix_3f_t &D_tilde_inv,
        vector_3f_t &o_hat);
};

MIC_NAMESPACE_END

#endif