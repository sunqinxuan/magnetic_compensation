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
#include "mic_mag_compensator/impl/mic_ellipsoid_mag_compensator.h"

MIC_NAMESPACE_START

ret_t MicEllipsoidMagCompensator::calibrate()
{
    notify(*this);
    ret_t ret = ret_t::MIC_RET_FAILED;

    auto data_range = _data_storer.get_data_range<mic_mag_flux_t>(0.0, _curr_time_stamp + 1);
    auto it_start = data_range.first;
    auto it_end = data_range.second;

    for (auto it = it_start; it != it_end; ++it)
    {
        mic_mag_flux_t mag_flux = it->second;
    }


    return ret;
}

ret_t MicEllipsoidMagCompensator::compenste()
{
    // for observer updating
    notify(*this);
    return ret_t::MIC_RET_FAILED;
}

MIC_NAMESPACE_END