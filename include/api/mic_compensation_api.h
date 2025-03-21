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

#ifndef MIC_API
#define MIC_API

#include "api/mic_base.h"

namespace mic
{
    ret_t mic_init_worker(
        const std::string model,
        const std::string model_file,
        const std::string log_file = "./data/mic.log",
        const std::string config_file = "./etc/config_compensation.json");

    ret_t mic_add_data(
        const float64_t timestamp,
        const mic_mag_t &mag,
        const mic_mag_t &mag_truth);

    ret_t mic_compensate(
        const float64_t timestamp,
        mic_mag_t &out);

    matrix_xf_t mic_get_cov();

}

#endif