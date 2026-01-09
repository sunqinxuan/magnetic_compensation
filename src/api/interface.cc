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

#include "api/interface.h"
#include "fileio/fileio.h"
#include "mic_mag_compensator/mic_mag_compensator.h"
#include "mic_mag_compensator/impl/mic_ellipsoid_nav_mag_compensator.h"
#include "GeoMag/Core.hpp"

using namespace std;

static std::shared_ptr<mic_ellipsoid_nav_mag_compensator_t> _mic_compensator = nullptr;
static std::string _task_data_file = "task.txt";

bool initialize()
{
    mic_logger_t::initialize(mic_logger_type_t::MIC_BASH_FILE_LOGGER, "./mic.log");
    mic_config_t::initialize(mic_config_type_t::MIC_CONFIG_JSON,
                             "./etc/config_compensation.json");
    mic_logger_t::set_log_level(
        static_cast<mic_log_level_t>(MIC_CONFIG_GET(int32_t, "log_level")));

    _mic_compensator = std::make_shared<mic_ellipsoid_nav_mag_compensator_t>();
    return true;
}

bool loadCalibData(const std::string &calib_data_file)
{
    // if (_mic_compensator == nullptr)
    // {
    //     _mic_compensator = std::make_shared<mic_ellipsoid_nav_mag_compensator_t>();
    // }
    if (load_data(calib_data_file, _mic_compensator, 0) == ret_t::MIC_RET_FAILED)
    {
        MIC_LOG_ERR("failed to load calib data file %s", calib_data_file);
        return false;
    }
    return true;
}

bool loadTaskData(const std::string &task_data_file)
{
    _task_data_file = task_data_file;
    std::ifstream infile(task_data_file);
    if (!infile.is_open())
    {
        MIC_LOG_ERR("failed open file %s", task_data_file);
        return false;
    }
    std::string nav_frame = MIC_CONFIG_GET(std::string, "navigation_frame");
    std::string euler_seq = MIC_CONFIG_GET(std::string, "euler_angle_sequence");

    // std::ofstream fp("igrf.txt", std::ios::out);
    while (true)
    {
        // % output file format:
        // % timestamp,
        // % mag_in, x_in, y_in, z_in,
        // % ins_pitch(rad), ins_roll(rad), ins_yaw(rad),
        // % lat(rad), lon(rad), alt(m),
        // % quspin(-1)
        std::vector<float64_t> data_line;
        if (read_line(infile, data_line) == ret_t::MIC_RET_FAILED)
            break;
        float64_t ts = data_line[0];
        float64_t op_value = data_line[1];  // mag_in
        float64_t flux_x = data_line[2];    // x_in
        float64_t flux_y = data_line[3];    // y_in
        float64_t flux_z = data_line[4];    // z_in
        float64_t ins_pitch = data_line[5]; // rad
        float64_t ins_roll = data_line[6];  // rad
        float64_t ins_yaw = data_line[7];   // rad
        float64_t lat = data_line[8];       // rad
        float64_t lon = data_line[9];       // rad
        float64_t alt = data_line[10];      // m
        float64_t op_truth = data_line[11]; // quspin

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

        DateTime date("2024-12-31T00:00:00.000");
        auto gmag = GeoMagFlux{MagFluxUnit::NanoTesla};
        auto position = Wgs84{date, Radian{lon}, Radian{lat}, alt};
        auto bf = gmag(position);
        auto b = MagFluxComponent{bf};

        mag.time_stamp = ts;
        mag.vector << flux_x, flux_y, flux_z;
        mag.value = op_value;
        mag_truth.time_stamp = ts;
        mag_truth.vector << b.north, b.east, b.down;
        if (op_truth == -1)
        {
            mag_truth.value = b.total;
        }
        else
        {
            mag_truth.value = op_truth;
        }

        // fp << std::fixed << ts << "\t" << lat << "\t" << lon << "\t" << alt << "\t" << b.north << "\t" << b.east << "\t" << b.down << std::endl;

        _mic_compensator->add_data(ts, mag, nav_state);
        _mic_compensator->add_data_truth(ts, mag_truth); // TODO
    }
    // fp.close();
    infile.close();

    return true;
}

bool calibModel(const std::string &model_file)
{
    if (_mic_compensator->calibrate() == ret_t::MIC_RET_FAILED)
    {
        MIC_LOG_ERR("calibration error!");
        return false;
    }
    if (_mic_compensator->save_model(model_file) == ret_t::MIC_RET_FAILED)
    {
        MIC_LOG_ERR("save model error!");
        return false;
    }
    return true;
}

bool loadModel(const std::string &model_file)
{
    // if (_mic_compensator == nullptr)
    // {
    //     _mic_compensator = std::make_shared<mic_ellipsoid_nav_mag_compensator_t>();
    // }
    if (_mic_compensator->load_model(model_file) == ret_t::MIC_RET_FAILED)
    {
        MIC_LOG_ERR("load model error!");
        return false;
    }
    return true;
}

bool compensate(const std::string &comp_data_file)
{
    std::ifstream infile(_task_data_file);
    std::ofstream outfile(comp_data_file);
    // if (!infile.is_open())
    // {
    //     MIC_LOG_ERR("failed open file %s", _task_data_file);
    //     return false;
    // }
    std::string nav_frame = MIC_CONFIG_GET(std::string, "navigation_frame");
    std::string euler_seq = MIC_CONFIG_GET(std::string, "euler_angle_sequence");

    // std::ofstream fp("igrf.txt", std::ios::out);
    while (true)
    {
        // % output file format:
        // % timestamp,
        // % mag_in, x_in, y_in, z_in,
        // % ins_pitch(rad), ins_roll(rad), ins_yaw(rad),
        // % lat(rad), lon(rad), alt(m),
        // % quspin(-1)
        std::vector<float64_t> data_line;
        if (read_line(infile, data_line) == ret_t::MIC_RET_FAILED)
            break;
        float64_t ts = data_line[0];
        float64_t op_value = data_line[1];  // mag_in
        float64_t flux_x = data_line[2];    // x_in
        float64_t flux_y = data_line[3];    // y_in
        float64_t flux_z = data_line[4];    // z_in
        float64_t ins_pitch = data_line[5]; // rad
        float64_t ins_roll = data_line[6];  // rad
        float64_t ins_yaw = data_line[7];   // rad
        float64_t lat = data_line[8];       // rad
        float64_t lon = data_line[9];       // rad
        float64_t alt = data_line[10];      // m
        float64_t op_truth = data_line[11]; // quspin

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

        DateTime date("2024-12-31T00:00:00.000");
        auto gmag = GeoMagFlux{MagFluxUnit::NanoTesla};
        auto position = Wgs84{date, Radian{lon}, Radian{lat}, alt};
        auto bf = gmag(position);
        auto b = MagFluxComponent{bf};

        mag.time_stamp = ts;
        mag.vector << flux_x, flux_y, flux_z;
        mag.value = op_value;
        mag_truth.time_stamp = ts;
        mag_truth.vector << b.north, b.east, b.down;
        if (op_truth == -1)
        {
            mag_truth.value = b.total;
        }
        else
        {
            mag_truth.value = op_truth;
        }

        // fp << std::fixed << ts << "\t" << lat << "\t" << lon << "\t" << alt << "\t" << b.north << "\t" << b.east << "\t" << b.down << std::endl;

        // _mic_compensator->add_data(ts, mag, nav_state);
        // _mic_compensator->add_data_truth(ts, mag_truth);
        mic_mag_t mag_out;
        _mic_compensator->compenste(ts, mag_out, mag, mag_truth, nav_state);
        outfile << std::fixed << ts << "\t" << mag_out.value << "\t" << mag_out.vector.transpose() << std::endl;
    }
    // fp.close();
    infile.close();
    outfile.close();

    return true;
}