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
#ifndef MIC_ELLIPSOID_NAV_MAG_COMPENSATOR
#define MIC_ELLIPSOID_NAV_MAG_COMPENSATOR

#include <Eigen/Eigenvalues>
#include <Eigen/Dense>
#include <ceres/ceres.h>
#include "mic_mag_compensator/impl/mic_ellipsoid_mag_compensator.h"

MIC_NAMESPACE_START

class MicEllipsoidNavMagCompensator;
using mic_ellipsoid_nav_mag_compensator_t = MicEllipsoidNavMagCompensator;

/** \brief MicEllipsoidNavMagCompensator .
 *
 * \author Qinxuan Sun, Yansong Gong
 * \ingroup compensation
 */
class MicEllipsoidNavMagCompensator : public MicEllipsoidMagCompensator
{
public:
    /** \brief Empty constructor. */
    MicEllipsoidNavMagCompensator();

    /** \brief destructor. */
    virtual ~MicEllipsoidNavMagCompensator() = default;

    matrix_xf_t get_kf_cov() { return _theta_cov; }

protected:
    /** \brief Compensate using the ellipsoid-based model
     * (only if the working state is set to \a MIC_MAG_COMPENSATE_CALIBRATED).
     * The model coefficients are updated while compensating.
     * \param[in] ts the timestamp at which the compensation result is required
     * \param[out] out output the compensated result at time \a ts
     */
    virtual ret_t do_compenste(const float64_t ts, mic_mag_t &out) override;

    virtual ret_t deserialize(json_t &node) override;

    /** \brief A kalman filter-based update of the model coefficients.
     * \param[in] ts the timestamp at which the model is updated
     */
    ret_t KF_update(const float64_t ts);

protected:
    /** \brief The coefficients for KF-based model update. */
    vector_xf_t _theta;

    /** \brief The covariance for KF-based model update. */
    matrix_xf_t _theta_cov;

    /** \brief The covariance of the Gaussian noise for prediction model. */
    matrix_xf_t _state_noise_cov;

    /** \brief The covariance of the Gaussian noise for measurement model. */
    matrix_xf_t _measure_noise_cov;
};

MIC_NAMESPACE_END

#endif