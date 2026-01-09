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
    _version = "1.1.0";
    _rmse_sq.setZero();
}

MicMagCompensator::~MicMagCompensator()
{
}

// void MicMagCompensator::init_nav_state_estimator()
// {
//     _nav_state_estimator = std::make_unique<mic_kf_ins_estimator_t>();
// }

mic_mag_storer_t &MicMagCompensator::get_data_storer_measure()
{
    return _mag_measure_storer;
}

mic_mag_storer_t &MicMagCompensator::get_data_storer_truth()
{
    return _mag_truth_storer;
}

mic_mag_storer_t &MicMagCompensator::get_data_storer_comp()
{
    return _mag_comp_storer;
}

ret_t MicMagCompensator::update_rmse_sq()
{
    if (_mag_comp_storer.get_data_size<mic_mag_t>() == 0)
    {
        return ret_t::MIC_RET_FAILED;
    }
    if (_mag_comp_storer.get_data_size<mic_mag_t>() == 1)
    {
        mic_mag_t mag_comp, mag_truth;
        mic_nav_state_t nav_state;
        if (_mag_comp_storer.get_data<mic_mag_t>(_curr_time_stamp, mag_comp) &&
            _mag_truth_storer.get_data<mic_mag_t>(_curr_time_stamp, mag_truth) &&
            _mag_measure_storer.get_data<mic_nav_state_t>(_curr_time_stamp, nav_state))
        {
            matrix_3f_t R_nb = nav_state.attitude.matrix();
            vector_3f_t mag_b = R_nb.transpose() * mag_truth.vector.normalized() * mag_truth.value;
            _rmse_sq(0) = (mag_truth.value - mag_comp.value) * (mag_truth.value - mag_comp.value);
            _rmse_sq(1) = (mag_b(0) - mag_comp.vector(0)) * (mag_b(0) - mag_comp.vector(0));
            _rmse_sq(2) = (mag_b(1) - mag_comp.vector(1)) * (mag_b(1) - mag_comp.vector(1));
            _rmse_sq(3) = (mag_b(2) - mag_comp.vector(2)) * (mag_b(2) - mag_comp.vector(2));
            return ret_t::MIC_RET_SUCCESSED;
        }
        else
        {
            return ret_t::MIC_RET_FAILED;
        }
    }
    else
    {
        mic_mag_t mag_comp, mag_truth;
        mic_nav_state_t nav_state;
        if (_mag_comp_storer.get_data<mic_mag_t>(_curr_time_stamp, mag_comp) &&
            _mag_truth_storer.get_data<mic_mag_t>(_curr_time_stamp, mag_truth) &&
            _mag_measure_storer.get_data<mic_nav_state_t>(_curr_time_stamp, nav_state))
        {
            int N = _mag_comp_storer.get_data_size<mic_mag_t>();
            matrix_3f_t R_nb = nav_state.attitude.matrix();
            vector_3f_t mag_b = R_nb.transpose() * mag_truth.vector.normalized() * mag_truth.value;
            float64_t delta_t = (mag_truth.value - mag_comp.value) * (mag_truth.value - mag_comp.value);
            float64_t delta_x = (mag_b(0) - mag_comp.vector(0)) * (mag_b(0) - mag_comp.vector(0));
            float64_t delta_y = (mag_b(1) - mag_comp.vector(1)) * (mag_b(1) - mag_comp.vector(1));
            float64_t delta_z = (mag_b(2) - mag_comp.vector(2)) * (mag_b(2) - mag_comp.vector(2));
            _rmse_sq(0) = _rmse_sq(0) * (N - 1) / N + delta_t / N;
            _rmse_sq(1) = _rmse_sq(1) * (N - 1) / N + delta_x / N;
            _rmse_sq(2) = _rmse_sq(2) * (N - 1) / N + delta_y / N;
            _rmse_sq(3) = _rmse_sq(3) * (N - 1) / N + delta_z / N;
            return ret_t::MIC_RET_SUCCESSED;
        }
        else
        {
            return ret_t::MIC_RET_FAILED;
        }
    }
    return ret_t::MIC_RET_FAILED;
}

// mic_nav_state_estimator_t &MicMagCompensator::get_nav_state_estimator()
// {
//     if (_nav_state_estimator == nullptr)
//         init_nav_state_estimator();
//     return *_nav_state_estimator;
// }

ret_t MicMagCompensator::add_data(
    const float64_t ts,
    const mic_mag_t &mag_data,
    const mic_nav_state_t &nav_state)
{
    _curr_time_stamp = ts;
    _mag_measure_storer.add_data<mic_mag_t>(ts, mag_data);
    if (nav_state.time_stamp > 0)
        _mag_measure_storer.add_data<mic_nav_state_t>(ts, nav_state);
    return ret_t::MIC_RET_SUCCESSED;
}

ret_t MicMagCompensator::add_data_truth(
    const float64_t ts,
    const mic_mag_t &mag_data,
    const mic_nav_state_t &nav_state)
{
    _mag_truth_storer.add_data<mic_mag_t>(ts, mag_data);
    if (nav_state.time_stamp > 0)
        _mag_truth_storer.add_data<mic_nav_state_t>(ts, nav_state);
    return ret_t::MIC_RET_SUCCESSED;
}

ret_t MicMagCompensator::calibrate()
{
    auto ret = do_calibrate();
    if (ret == ret_t::MIC_RET_SUCCESSED)
    {
        _state = mic_state_t::MIC_MAG_COMPENSATE_CALIBRATED;
    }
    notify(*this);
    return ret;
}

ret_t MicMagCompensator::compenste(
    const float64_t ts, mic_mag_t &out,
    const mic_mag_t &mag,
    const mic_mag_t &mag_truth,
    const mic_nav_state_t &nav_state)
{
    auto ret = ret_t::MIC_RET_FAILED;
    if (_state == mic_state_t::MIC_MAG_COMPENSATE_UNCALIBRATED)
    {
        MIC_LOG_ERR("[MIC] mic compensator is not calirated!");
        return ret;
    }
    if (_state == mic_state_t::MIC_MAG_COMPENSATE_CALIBRATED)
    {
        _state = mic_state_t::MIC_MAG_COMPENSATE_COMPENSATING;
        add_data(ts, mag, nav_state);
        add_data_truth(ts, mag_truth);
        ret = do_compenste(ts, out);
        _mag_comp_storer.add_data<mic_mag_t>(ts, out);
        // update_rmse_sq();
        // notify(*this);
        _state = mic_state_t::MIC_MAG_COMPENSATE_CALIBRATED;
        return ret;
    }
    else // mic_state_t::MIC_MAG_COMPENSATE_COMPENSATING
    {
        return ret;
    }
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
        MIC_LOG_DEBUG_INFO("[MIC] load mic model:\n%s", map_json.dump(4).c_str());
    }
    if (is_load_map && ret == ret_t::MIC_RET_SUCCESSED)
    {
        _state = mic_state_t::MIC_MAG_COMPENSATE_CALIBRATED;
        ret = ret_t::MIC_RET_SUCCESSED;
        notify(*this);
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
        MIC_LOG_DEBUG_INFO("[MIC] save mic model:\n%s", map_json.dump(4).c_str());
        auto cbor = json_t::to_cbor(map_json);
        map_file.write((char *)cbor.data(), cbor.size() * sizeof(uint8_t));
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