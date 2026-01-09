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
#include "GeoMag/Core.hpp"

MIC_NAMESPACE_START

using namespace geomag;

// data field:
// % timestamp,
// % mag_in, x_in, y_in, z_in,
// % mag_out, x_out, y_out, z_out,
// % quspin,
// % ins_pitch(rad), ins_roll(rad), ins_yaw(rad),
// % lat(rad), lon(rad), alt(m),
// % mag_map, x_map, y_map, z_map

ret_t get_line_data(const std::vector<float64_t> &data_line, mic_mag_t &mag, mic_mag_t &mag_truth, mic_nav_state_t &nav_state)
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
    float64_t op_truth = data_line[5];
    float64_t flux_x_truth = data_line[6];
    float64_t flux_y_truth = data_line[7];
    float64_t flux_z_truth = data_line[8];
    float64_t ins_pitch = data_line[9];
    float64_t ins_roll = data_line[10];
    float64_t ins_yaw = data_line[11];

    nav_state.time_stamp = ts;
    matrix_3f_t rotation2NED = matrix_3f_t::Identity();
    std::string nav_frame = MIC_CONFIG_GET(std::string, "navigation_frame");
    std::string euler_seq = MIC_CONFIG_GET(std::string, "euler_angle_sequence");
    if (nav_frame == "ENU")
    {
        rotation2NED << 0, 1, 0,
            1, 0, 0,
            0, 0, -1;
    }
    nav_state.attitude = quaternionf_t(rotation2NED *
                                       MicUtils::euler2dcm(
                                           MicUtils::deg2rad(ins_roll),
                                           MicUtils::deg2rad(ins_pitch),
                                           MicUtils::deg2rad(ins_yaw), euler_seq));

    mag.time_stamp = ts;
    mag.vector << flux_x, flux_y, flux_z;
    mag.value = op_value;

    mag_truth.time_stamp = ts;
    mag_truth.vector << flux_x_truth, flux_y_truth, flux_z_truth;
    mag_truth.value = op_truth;

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

ret_t load_data(std::string file_name, mic_mag_compensator_shared_ptr mag_compensator_ptr, int32_t flag_outcabin)
{
    std::ifstream infile(file_name);
    if (!infile.is_open())
    {
        // MIC_LOG_ERR("failed open file %s", file_name);
        return ret_t::MIC_RET_FAILED;
    }

    std::string nav_frame = MIC_CONFIG_GET(std::string, "navigation_frame");
    std::string euler_seq = MIC_CONFIG_GET(std::string, "euler_angle_sequence");

    MIC_LOG_DEBUG_INFO("flag_outcabin = %d", flag_outcabin);

    std::ofstream fp("debug.txt", std::ios::out);
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
        float64_t op_value = data_line[1]; // mag_in
        float64_t flux_x = data_line[2];   // x_in
        float64_t flux_y = data_line[3];   // y_in
        float64_t flux_z = data_line[4];   // z_in
        if (flag_outcabin == 1)
        {
            op_value = data_line[5]; // mag_out
            flux_x = data_line[6];   // x_out
            flux_y = data_line[7];   // y_out
            flux_z = data_line[8];   // z_out
        }
        float64_t op_truth = data_line[9]; // quspin
        // float64_t igrf_north = data_line[6];
        // float64_t igrf_east = data_line[7];
        // float64_t igrf_down = data_line[8];
        float64_t ins_pitch = data_line[10]; // rad
        float64_t ins_roll = data_line[11];  // rad
        float64_t ins_yaw = data_line[12];   // rad
        float64_t lat = data_line[13];       // rad
        float64_t lon = data_line[14];       // rad
        float64_t alt = data_line[15];       // m

        mic_mag_t mag, mag_truth;
        mic_nav_state_t nav_state;
        nav_state.time_stamp = ts;
        matrix_3f_t rotation2NED = matrix_3f_t::Identity();
        if (nav_frame == "ENU")
        {
            rotation2NED << 0, 1, 0,
                1, 0, 0,
                0, 0, -1;
        }
        nav_state.attitude = quaternionf_t(rotation2NED * MicUtils::euler2dcm(ins_roll, ins_pitch, ins_yaw, euler_seq));

        // for simulation:
        // nav_state.attitude = quaternionf_t(rotation2NED * MicUtils::euler2dcm(ins_roll, ins_pitch, ins_yaw, euler_seq).inverse());

        // fp << std::endl
        //    << nav_state.attitude.matrix() << std::endl; // R_nb

        DateTime date("2015-06-01T00:00:00.000");
        auto gmag = GeoMagFlux{MagFluxUnit::NanoTesla};
        auto position = Wgs84{date, Radian{lon}, Radian{lat}, alt};
        auto bf = gmag(position);
        auto b = MagFluxComponent{bf};

        mag.time_stamp = ts;
        mag.vector << flux_x, flux_y, flux_z;
        mag.value = op_value;
        mag_truth.time_stamp = ts;
        mag_truth.vector << b.north, b.east, b.down;
        // for simulation:
        // mag_truth.vector << 0, 0, op_truth;
        mag_truth.value = op_truth;
        // fp << std::fixed << ts<< std::endl;

        mag_compensator_ptr->add_data(ts, mag, nav_state);
        mag_compensator_ptr->add_data_truth(ts, mag_truth); // TODO
    }
    fp.close();
    infile.close();
    return ret_t::MIC_RET_SUCCESSED;
}

MIC_NAMESPACE_END

#endif