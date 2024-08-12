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
#include "mic_mag_compensator/obeserver/mic_state_logger.h"

MIC_NAMESPACE_START

void MicStateLogger::update(mic_mag_compensator_t &comp)
{
    // MIC_LOG_BASIC_INFO("output state!");

    // float64_t ts = comp.get_curr_time();
    // MIC_LOG_BASIC_INFO("current time: %f", ts);

    // mic_mag_storer_t data_storer = comp.get_data_storer();
    // auto data_range = data_storer.get_data_range<mic_mag_flux_t>(0.0, ts);
    // auto it_start = data_range.first;
    // auto it_end = data_range.second;

    // for (auto it = it_start; it != it_end; ++it)
    // {
    //     float64_t time = it->first;
    //     mic_mag_flux_t mag_flux = it->second;
    //     // MIC_LOG_BASIC_INFO("%f\t%f\t%f\t%f", time, mag_flux.vector(0), mag_flux.vector(1), mag_flux.vector(2));
    // }
}

MIC_NAMESPACE_END