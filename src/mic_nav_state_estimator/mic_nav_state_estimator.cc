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

    mic_nav_state_t nav_state;
    ret_t res = propagate(ts, imu_data, nav_state);

    if (res == ret_t::MIC_RET_SUCCESSED)
    {
        _data_storer.add_data<mic_imu_t>(ts, imu_data);
        _data_storer.add_data<mic_nav_state_t>(ts, nav_state);

        _last_time_stamp = _curr_time_stamp;
        _curr_time_stamp = ts;
    }

    return res;

    // mic_imu_t last_imu_data;
    // bool_t res = _data_storer.get_data<mic_imu_t>(_last_time_stamp, last_imu_data);
    // if (res)
    // {
    //     _curr_nav_state = update_nav_state();
    // }
    // return ret_t::MIC_RET_SUCCESSED;
}

ret_t MicNavStateEstimator::add_gnss_data(
    const float64_t ts,
    const mic_gnss_t &gnss_data)
{
    // TODO: 
    // process gnss data;
    // update();

    /*     if (ts < _curr_time_stamp)
        {
            return ret_t::MIC_RET_FAILED;
        }
        mic_imu_t last_imu_data;
        bool_t res = _data_storer.get_data<mic_imu_t>(_time_stamp, last_imu_data);
        if (res)
        {
            _curr_nav_state = update_nav_state();
            _time_stamp = ts;
        } */

    _data_storer.add_data<mic_gnss_t>(ts, gnss_data);
    return ret_t::MIC_RET_SUCCESSED;
}

ret_t MicNavStateEstimator::set_nav_state(
    const float64_t ts,
    const mic_nav_state_t &nav_state)
{
    ret_t ret = ret_t::MIC_RET_FAILED;
    if (ts > _curr_time_stamp)
    {
        _data_storer.add_data<mic_nav_state_t>(ts, nav_state);
        _last_time_stamp = _curr_time_stamp;
        _curr_time_stamp = ts;
        ret = ret_t::MIC_RET_SUCCESSED;
    }
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