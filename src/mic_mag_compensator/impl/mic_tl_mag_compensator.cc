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
#include "mic_mag_compensator/impl/mic_tl_mag_compensator.h"

// debug
#include <iostream>
using namespace std;

MIC_NAMESPACE_START

MicTLMagCompensator::MicTLMagCompensator() : MicMagCompensator()
{
    _tl_model = std::make_shared<mic_tolles_lawson_t>();
}

ret_t MicTLMagCompensator::calibrate()
{
    ret_t ret = ret_t::MIC_RET_FAILED;

    auto data_range = _mag_measure_storer.get_data_range<mic_mag_flux_t>(0.0, _curr_time_stamp + 1);
    auto it_start = data_range.first;
    auto it_end = data_range.second;

    // TODO: mag_x.resize(_data_storer.size())
    std::vector<float64_t> mag_x, mag_y, mag_z;
    std::vector<float64_t> mag_earth_x, mag_earth_y, mag_earth_z;
    for (auto it = it_start; it != it_end; ++it)
    {
        float64_t ts = it->first;
        mic_mag_flux_t mag_flux = it->second;
        mic_nav_state_t nav_state;
        mic_mag_flux_t mag_flux_truth;
        _mag_truth_storer.get_data<mic_mag_flux_t>(ts, mag_flux_truth);
        mag_x.push_back(mag_flux.vector(0));
        mag_y.push_back(mag_flux.vector(1));
        mag_z.push_back(mag_flux.vector(2));
        mag_earth_x.push_back(mag_flux_truth.vector(0));
        mag_earth_y.push_back(mag_flux_truth.vector(1));
        mag_earth_z.push_back(mag_flux_truth.vector(2));
    }

    _tl_model->createCoeff_Vector(_tl_coeffs, mag_x, mag_y, mag_z, mag_earth_x, mag_earth_y, mag_earth_z);

    // debug
    ofstream fp_tl("TL_beta.txt");
    for (size_t i = 0; i < _tl_coeffs.size(); ++i)
    {
        fp_tl << _tl_coeffs[i] << endl;
    }
    fp_tl.close();

    notify(*this);
    return ret;
}

ret_t MicTLMagCompensator::compenste()
{
    // for observer updating
    notify(*this);
    return ret_t::MIC_RET_FAILED;
}

MIC_NAMESPACE_END