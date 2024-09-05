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
#ifndef MIC_NAVIGATION_SYSTEM
#define MIC_NAVIGATION_SYSTEM

#include "common/mic_prerequisite.h"
#include "data_storer/mic_data_storer.h"

MIC_NAMESPACE_START

using mic_nav_state_storer_t =
    mic_data_storer_t<mic_nav_state_t, mic_imu_t, mic_gnss_t>;

class MicNavStateEstimator;
using mic_nav_state_estimator_t = MicNavStateEstimator;
using mic_nav_state_estimator_unique_ptr = std::unique_ptr<MicNavStateEstimator>;

/** \brief @b MicNavStateEstimator provides a base interface of the state estimation algorithm.
 *
 * \author Qinxuan Sun, Yansong Gong
 * \ingroup compensation
 */
class MicNavStateEstimator
{
public:
    /** \brief Empty constructor. */
    MicNavStateEstimator() = default;
    /** \brief destructor. */
    ~MicNavStateEstimator() = default;

    /** \brief Add IMU data to the storer and update the current state.
     * If the state is successfully updated, set the current timestamp to ts
     * and return \a ret_t::MIC_RET_SUCCESSED.
     * \param[in] ts the timestamp of the added data
     * \param[in] imu_data the added IMU data
     */
    ret_t add_imu_data(
        const float64_t ts,
        const mic_imu_t &imu_data);

    /** \brief Add GNSS data to the storer.
     * \param[in] ts the timestamp of the added data
     * \param[in] gnss_data the added GNSS data
     */
    ret_t add_gnss_data(
        const float64_t ts,
        const mic_gnss_t &gnss_data);

    /** \brief Set the current state of the estimator.
     * \param[in] ts the timestamp of the updated state
     * \param[in] nav_state the navigation state used to update the estimator
     */
    ret_t set_nav_state(
        const float64_t ts,
        const mic_nav_state_t &nav_state);

    /** \brief Get the current state of the estimator.
     * \param[in] ts the timestamp of the state to get
     * \param[out] nav_state output the current state of the estimator
     */
    ret_t get_nav_state(
        const float64_t ts,
        mic_nav_state_t &nav_state);

protected:
    /** \brief Interface to update the state of the estimator,
     * propagating from \a _curr_time_stamp to \a ts.
     * \param[in] ts the timestamp of the added data
     * \param[in] imu_data the IMU data used to update the state
     * \param[out] nav_state output the updated navigation state
     */
    virtual ret_t update(
        const float64_t ts,
        const mic_imu_t &imu_data,
        mic_nav_state_t &nav_state) = 0;

    /** \brief The timestamp of the current time step. */
    float64_t _curr_time_stamp;

    /** \brief The timestamp of the last time step. */
    float64_t _last_time_stamp;

    /** \brief The data storer including the navigation state, IMU and GNSS data. */
    mic_nav_state_storer_t _data_storer;
};

MIC_NAMESPACE_END

#endif
