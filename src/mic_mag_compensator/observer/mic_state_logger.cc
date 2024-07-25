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
    MIC_LOG_BASIC_INFO("output state!");

    float64_t ts = comp.get_curr_time();
    mic_mag_storer_t data_storer = comp.get_data_storer();

    char_t char_array[50];
    sprintf(char_array, "current time: %f", ts);

    // MIC_LOG_BASIC_INFO("current time: ", ts);
    MIC_LOG_BASIC_INFO(char_array);
}

MIC_NAMESPACE_END