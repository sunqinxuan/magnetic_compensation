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

class MicNavStateEstimator
{
public:
    MicNavStateEstimator() = default;
    ~MicNavStateEstimator() = default;

    // mic_nav_state_storer_t& get_data();

    ret_t add_imu_data(
        const float64_t ts,
        const mic_imu_t &imu_data);

    ret_t add_gnss_data(
        const float64_t ts,
        const mic_gnss_t &gnss_data);

    ret_t set_nav_state(
        const float64_t ts,
        const mic_nav_state_t &nav_state);

    ret_t get_nav_state(
        const float64_t ts,
        mic_nav_state_t &nav_state);

protected:
    /* propagate from _curr_time_stamp to ts; */
    virtual ret_t update(
        const float64_t ts,
        const mic_imu_t &imu_data,
        mic_nav_state_t &nav_state) = 0;

    // TODO:
    // virtual mic_nav_state_t update() = 0;

    float64_t _curr_time_stamp;
    float64_t _last_time_stamp;
    // mic_nav_state_t _curr_nav_state;

    /* data */
    mic_nav_state_storer_t _data_storer;
};

MIC_NAMESPACE_END

#endif
