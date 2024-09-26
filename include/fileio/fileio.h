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
#ifndef MIC_FILEIO
#define MIC_FILEIO

#include <string>
#include <sstream>
#include <fstream>
#include <iostream>
#include "common/mic_prerequisite.h"
#include "mic_mag_compensator/mic_mag_compensator.h"

MIC_NAMESPACE_START

// data field:
// time, mag_op, flux_xyz, mag_op_truth, flux_truth_xyz(igrf_ned), ins_pry

ret_t get_line_data(const std::vector<float64_t> &data_line, mic_mag_t &mag)
{
    if (data_line.size() < 5)
    {
        MIC_LOG_ERR("failed to load mag data from line");
        return ret_t::MIC_RET_FAILED;
    }
    float64_t ts = data_line[0];
    float64_t op_value = data_line[1];
    float64_t flux_x = data_line[2];
    float64_t flux_y = data_line[3];
    float64_t flux_z = data_line[4];

    mag.time_stamp = ts;
    mag.vector << flux_x, flux_y, flux_z;
    mag.value = op_value;

    return ret_t::MIC_RET_SUCCESSED;
}

ret_t read_line(std::ifstream &infile, std::vector<float64_t> &data)
{
    std::string line;
    if (std::getline(infile, line))
    {
        std::istringstream iss(line);
        data.clear();
        float64_t tmp;
        while (iss >> tmp)
        {
            data.push_back(tmp);
        }
        return ret_t::MIC_RET_SUCCESSED;
    }
    return ret_t::MIC_RET_FAILED;
}

ret_t load_data(std::string file_name, mic_mag_compensator_shared_ptr mag_compensator_ptr)
{
    std::ifstream infile(file_name);
    if (!infile.is_open())
    {
        // MIC_LOG_ERR("failed open file %s", file_name);
        return ret_t::MIC_RET_FAILED;
    }

    while (true)
    {
        // float64_t ts, op_value, flux_x, flux_y, flux_z,
        //     op_truth, igrf_north, igrf_east, igrf_down,
        //     ins_pitch, ins_roll, ins_yaw;
        // infile >> ts >> op_value >> flux_x >> flux_y >> flux_z >>
        //     op_truth >> igrf_north >> igrf_east >> igrf_down >>
        //     ins_pitch >> ins_roll >> ins_yaw;
        // if (infile.eof())
        //     break;
        std::vector<float64_t> data_line;
        if (read_line(infile, data_line) == ret_t::MIC_RET_FAILED)
            break;
        float64_t ts = data_line[0];
        float64_t op_value = data_line[1];
        float64_t flux_x = data_line[2];
        float64_t flux_y = data_line[3];
        float64_t flux_z = data_line[4];
        float64_t op_truth = data_line[5];
        float64_t igrf_north = data_line[6];
        float64_t igrf_east = data_line[7];
        float64_t igrf_down = data_line[8];
        float64_t ins_pitch = data_line[9];
        float64_t ins_roll = data_line[10];
        float64_t ins_yaw = data_line[11];

        mic_mag_t mag, mag_truth;
        mic_nav_state_t nav_state;
        nav_state.time_stamp = ts;
        matrix_3f_t R_NE;
        R_NE << 0, 1, 0,
            1, 0, 0,
            0, 0, -1;
        nav_state.attitude = quaternionf_t(R_NE *
                                           MicUtils::euler2dcm2(
                                               MicUtils::deg2rad(ins_roll),
                                               MicUtils::deg2rad(ins_pitch),
                                               MicUtils::deg2rad(ins_yaw)));
        mag.time_stamp = ts;
        mag.vector << flux_x, flux_y, flux_z;
        mag.value = op_value;
        mag_truth.time_stamp = ts;
        mag_truth.vector << igrf_north, igrf_east, igrf_down;
        mag_truth.value = op_truth;

        mag_compensator_ptr->add_data(ts, mag, nav_state);
        mag_compensator_ptr->add_data_truth(ts, mag_truth); // TODO
    }
    infile.close();
    return ret_t::MIC_RET_SUCCESSED;
}

MIC_NAMESPACE_END

#endif