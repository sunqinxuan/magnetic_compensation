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
#include "mic_nav_state_estimator/mic_nav_state_estimator.h"

MIC_NAMESPACE_START

ret_t MicNavStateEstimator::add_imu_data(
    const float64_t ts,
    const mic_imu_t &imu_data)
{
    if (ts < _curr_time_stamp)
    {
        return ret_t::MIC_RET_FAILED;
    }
    _data_storer.add_data<mic_imu_t>(ts,imu_data);
    
    mic_imu_t last_imu_data;
    bool_t res = _data_storer.get_data<mic_imu_t>(_curr_time_stamp, last_imu_data);
    if (res)
    {
        _last_time_stamp = _curr_time_stamp;
        _curr_time_stamp = ts;
        _curr_nav_state = update_nav_state();
    }
    return ret_t::MIC_RET_SUCCESSED;
}

ret_t MicNavStateEstimator::add_gnss_data(
    const float64_t ts,
    const mic_gnss_t &gnss_data)
{
    if (ts < _time_stamp)
    {
        return ret_t::MIC_RET_FAILED;
    }
    mic_imu_t last_imu_data;
    bool_t res = _data_storer.get_data<mic_imu_t>(_time_stamp, last_imu_data);
    if (res)
    {
        _curr_nav_state = update_nav_state();
        _time_stamp = ts;
    }
    return ret_t::MIC_RET_SUCCESSED;
}

ret_t MicNavStateEstimator::set_nav_state(
    const float64_t ts,
    const mic_nav_state_t &nav_state)
{
    ret_t ret = ret_t::MIC_RET_FAILED;
    if (ts > _time_stamp)
    {
        _time_stamp = ts;
        _curr_nav_state = nav_state;
        ret = ret_t::MIC_RET_SUCCESSED;
    }
    _data_storer.add_data<mic_nav_state_t>(ts, nav_state);
    return ret;
}

ret_t MicNavStateEstimator::get_nav_state(
    const float64_t ts,
    mic_nav_state_t &nav_state)
{
    mic_nav_state_t tmp_nav_state;
    bool_t res = _data_storer.get_data<mic_nav_state_t>(ts, tmp_nav_state);
    if (res)
    {
        nav_state = tmp_nav_state;
        return ret_t::MIC_RET_SUCCESSED;
    }
    return ret_t::MIC_RET_FAILED;
};

MIC_NAMESPACE_END