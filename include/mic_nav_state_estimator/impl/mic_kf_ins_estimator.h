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
#ifndef MIC_KF_INTERNAL_NAVIGATION_SYSTEM
#define MIC_KF_INTERNAL_NAVIGATION_SYSTEM

#include "kf-gins/insmech.h"
#include "common/rotation.h"
#include "mic_nav_state_estimator/mic_nav_state_estimator.h"

MIC_NAMESPACE_START

class MicKFINSEstimator;
using mic_kf_ins_estimator_t = MicKFINSEstimator;

/** \brief @b MicKFINSEstimator provides a Kalman filter-based implementation
 *  of the state estimation algorithm.
 *
 * \author Qinxuan Sun, Yansong Gong
 * \ingroup compensation
 */
class MicKFINSEstimator : public MicNavStateEstimator
{
public:
    /** \brief Empty constructor. */
    MicKFINSEstimator() = default;
    /** \brief destructor. */
    virtual ~MicKFINSEstimator() = default;

protected:
    /** \brief (TODO) Implementing the Kalman filter-based navigation state estimation algorithm.
     * \param[in] ts the timestamp of the added data
     * \param[in] imu_data the IMU data used to update the state
     * \param[out] nav_state output the updated navigation state
     */
    virtual ret_t update(
        const float64_t ts,
        const mic_imu_t &imu_data,
        mic_nav_state_t &nav_state) override;
};

MIC_NAMESPACE_END

#endif
