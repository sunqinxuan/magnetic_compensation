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
#include <fstream>
#include "mic_mag_compensator/mic_mag_compensator.h"
#include "mic_nav_state_estimator/impl/mic_kf_ins_estimator.h"

MIC_NAMESPACE_START

MicMagCompensator::MicMagCompensator()
{
    // init_nav_state_estimator();
    _state = mic_state_t::MIC_MAG_COMPENSATE_UNCALIBRATED;
    _version = "0.0.1";
}

MicMagCompensator::~MicMagCompensator()
{
}

// void MicMagCompensator::init_nav_state_estimator()
// {
//     _nav_state_estimator = std::make_unique<mic_kf_ins_estimator_t>();
// }

mic_mag_storer_t &MicMagCompensator::get_data_storer()
{
    return _mag_measure_storer;
}

// mic_nav_state_estimator_t &MicMagCompensator::get_nav_state_estimator()
// {
//     if (_nav_state_estimator == nullptr)
//         init_nav_state_estimator();
//     return *_nav_state_estimator;
// }

ret_t MicMagCompensator::add_mag_flux(
    const float64_t ts,
    const mic_mag_flux_t &mag_flux_data)
{
    _mag_measure_storer.add_data<mic_mag_flux_t>(ts, mag_flux_data);
    _curr_time_stamp = ts;
    return ret_t::MIC_RET_SUCCESSED;
}

ret_t MicMagCompensator::add_mag_op(
    const float64_t ts,
    const mic_mag_op_t &mag_op_data)
{
    _mag_measure_storer.add_data<mic_mag_op_t>(ts, mag_op_data);
    return ret_t::MIC_RET_SUCCESSED;
}

ret_t MicMagCompensator::add_mag_flux_truth(
    const float64_t ts,
    const mic_mag_flux_t &mag_flux_data)
{
    _mag_truth_storer.add_data<mic_mag_flux_t>(ts, mag_flux_data);
    // _curr_time_stamp = ts;
    return ret_t::MIC_RET_SUCCESSED;
}

ret_t MicMagCompensator::add_mag_op_truth(
    const float64_t ts,
    const mic_mag_op_t &mag_op_data)
{
    _mag_truth_storer.add_data<mic_mag_op_t>(ts, mag_op_data);
    return ret_t::MIC_RET_SUCCESSED;
}

ret_t MicMagCompensator::add_nav_state(
    const float64_t ts,
    const mic_nav_state_t &nav_state)
{
    _nav_state_storer.add_data<mic_nav_state_t>(ts, nav_state);
    return ret_t::MIC_RET_SUCCESSED;
}

ret_t MicMagCompensator::calibrate()
{
    auto ret = do_calibrate();
    if (ret == ret_t::MIC_RET_SUCCESSED)
    {
        _state = mic_state_t::MIC_MAG_COMPENSATE_CALIBRATED;
    }
    return ret;
}

ret_t MicMagCompensator::compenste(
    const float64_t ts, mic_mag_flux_t &out)
{
    if (_state == mic_state_t::MIC_MAG_COMPENSATE_UNCALIBRATED)
    {
        MIC_LOG_ERR("[MIC] mic compensator is not calirated!");
        return ret_t::MIC_RET_FAILED;
    }
    return do_compenste(ts, out);
}

ret_t MicMagCompensator::load_model(const std::string filename)
{
    std::ifstream map_file(filename);
    auto is_load_map = static_cast<bool_t>(map_file.is_open());
    json_t map_json;
    if (is_load_map)
    {
        map_json = json_t::from_cbor(map_file, true, false);
    }
    else
    {
        MIC_LOG_ERR("[MIC] mic model does not exisit!");
    }
    is_load_map = static_cast<bool_t>(!map_json.empty());
    ret_t ret = ret_t::MIC_RET_FAILED;
    if (is_load_map)
    {
        ret = deserialize(map_json);
        MIC_LOG_BASIC_INFO("[MIC] load mic model:\n%s", map_json.dump(4).c_str());
    }
    if (is_load_map && ret == ret_t::MIC_RET_SUCCESSED)
    {
        _state = mic_state_t::MIC_MAG_COMPENSATE_CALIBRATED;
        ret = ret_t::MIC_RET_SUCCESSED;
    }
    return ret;
}

ret_t MicMagCompensator::save_model(const std::string filename)
{
    std::ofstream map_file(filename);
    if (!map_file.is_open())
    {
        MIC_LOG_ERR("[MIC] mic model path does not exisit!");
        return ret_t::MIC_RET_FAILED;
    }
    json_t map_json;
    ret_t ret = serialize(map_json);
    if (ret == ret_t::MIC_RET_SUCCESSED)
    {
        MIC_LOG_BASIC_INFO("[MIC] save mic model:\n%s", map_json.dump(4).c_str());
        auto cbor = json_t::to_cbor(map_json);
        map_file.write((char*)cbor.data(), cbor.size() * sizeof(uint8_t));
    }
    map_file.close();
    return ret;
}

ret_t MicMagCompensator::serialize(json_t &node)
{
    // store something ...
    node["version"] = _version;
    return ret_t::MIC_RET_SUCCESSED;
}

ret_t MicMagCompensator::deserialize(json_t &node)
{
    // load something ...
    _version = node["version"];
    return ret_t::MIC_RET_SUCCESSED;
}

MIC_NAMESPACE_END