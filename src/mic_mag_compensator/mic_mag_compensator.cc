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
#include "mic_mag_compensator/mic_mag_compensator.h"

#include "mic_nav_state_estimator/impl/mic_kf_ins_estimator.h"

MIC_NAMESPACE_START

MicMagCompensator::MicMagCompensator()
{
    init_nav_state_estimator();
}

MicMagCompensator::~MicMagCompensator()
{
}

void MicMagCompensator::init_nav_state_estimator()
{
    _nav_state_estimator = std::make_unique<mic_kf_ins_estimator_t>();
}

mic_mag_storer_t& MicMagCompensator::get_data_storer()
{
    return _data_storer;
}

mic_nav_state_estimator_t &MicMagCompensator::get_nav_state_estimator()
{
    if (_nav_state_estimator == nullptr)
        init_nav_state_estimator();
    return *_nav_state_estimator;
}

ret_t MicMagCompensator::add_mag_flux(
    const float64_t ts,
    const mic_mag_flux_t &mag_flux_data)
{
    _data_storer.add_data<mic_mag_flux_t>(ts, mag_flux_data);
    _curr_time_stamp=ts;
    return ret_t::MIC_RET_SUCCESSED;
}

ret_t MicMagCompensator::add_mag_op(
    const float64_t ts,
    const mic_mag_op_t &mag_op_data)
{
    _data_storer.add_data<mic_mag_op_t>(ts, mag_op_data);
    return ret_t::MIC_RET_SUCCESSED;
}

ret_t MicMagCompensator::calibrate()
{
    notify(*this);
    auto data_range = _data_storer.get_data_range<mic_mag_flux_t>(0.0,_curr_time_stamp);
    
    // how to get all the mag data in _data_storer?

    if (true) // judge data size
    {
        // calibration
    }
    return ret_t::MIC_RET_FAILED;
}

// ret_t MicMagCompensator::compenste()
// {
//     // for observer updating
//     notify(*this);
//     return ret_t::MIC_RET_FAILED;
// }

ret_t MicMagCompensator::serialize(json_t &node)
{
    return ret_t::MIC_RET_FAILED;
}

ret_t MicMagCompensator::deserialize(json_t &node)
{
    return ret_t::MIC_RET_FAILED;
}

MIC_NAMESPACE_END