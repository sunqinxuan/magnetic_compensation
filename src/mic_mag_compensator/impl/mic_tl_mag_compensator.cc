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

ret_t MicTLMagCompensator::do_calibrate()
{
    auto data_range = _mag_measure_storer.get_data_range<mic_mag_t>(0.0, _curr_time_stamp + 1);
    auto it_start = data_range.first;
    auto it_end = data_range.second;

    std::vector<float64_t> mag_x, mag_y, mag_z;
    std::vector<float64_t> mag_op, mag_op_truth;
    // std::vector<float64_t> mag_earth_x, mag_earth_y, mag_earth_z;
    for (auto it = it_start; it != it_end; ++it)
    {
        float64_t ts = it->first;
        mic_mag_t mag = it->second;
        mic_mag_t mag_truth;
        _mag_truth_storer.get_data<mic_mag_t>(ts, mag_truth);
        mag_x.push_back(mag.vector(0));
        mag_y.push_back(mag.vector(1));
        mag_z.push_back(mag.vector(2));
        mag_op.push_back(mag.value);
        mag_op_truth.push_back(mag_truth.value);
        // mag_earth_x.push_back(mag_truth.vector(0));
        // mag_earth_y.push_back(mag_truth.vector(1));
        // mag_earth_z.push_back(mag_truth.vector(2));
    }

    std::vector<float64_t> tl_beta;
    // mag_op_truth.clear(); /////
    if (!_tl_model->createCoeff(tl_beta, mag_x, mag_y, mag_z, mag_op_truth, mag_op))
    {
        MIC_LOG_ERR("failed to calibrate TL model");
        return ret_t::MIC_RET_FAILED;
    }
    // if (tl_beta.size() != 18)
    //     return ret_t::MIC_RET_FAILED;
    _tl_coeffs = vector_xf_t::Zero(tl_beta.size());
    for (size_t i = 0; i < tl_beta.size(); ++i)
    {
        _tl_coeffs(i) = tl_beta[i];
    }

    // debug
    ofstream fp_tl("TL_beta.txt");
    fp_tl << _tl_coeffs;
    fp_tl.close();

    notify(*this);
    return ret_t::MIC_RET_SUCCESSED;
}

ret_t MicTLMagCompensator::do_compenste(const float64_t ts, mic_mag_t &out)
{
    matrix_xf_t A_matrix;
    std::vector<std::vector<double>> TL_A;

    std::vector<double> mag_x, mag_y, mag_z, mag_op;
    auto data_range = _mag_measure_storer.get_data_range<mic_mag_t>(0.0, ts + 1);
    auto it_start = data_range.first;
    auto it_end = data_range.second;
    for (auto it = it_start; it != it_end; ++it)
    {
        mic_mag_t mag = it->second;
        mag_x.push_back(mag.vector(0));
        mag_y.push_back(mag.vector(1));
        mag_z.push_back(mag.vector(2));
        mag_op.push_back(mag.value);
    }

    // if only one measure is provided,
    // the derivative terms in matrix A will be zero;
    // that is to say, no eddy current interference is considered;
    if (!_tl_model->createMatrixA(TL_A, mag_x, mag_y, mag_z, mag_op))
    {
        MIC_LOG_ERR("failed to construct TL matrix A!");
        return ret_t::MIC_RET_FAILED;
    }

    A_matrix = matrix_xf_t::Identity(mag_x.size(), TL_A.size());
    for (size_t i = 0; i < TL_A.size(); ++i)
    {
        for (size_t j = 0; j < TL_A[i].size(); ++j)
        {
            A_matrix(j, i) = TL_A[i][j];
        }
    }

    // vector_xf_t mag_op_vector=vector_xf_t::Zero(mag_op.size());
    vector_xf_t A_matrix_deltaN = A_matrix.bottomRows(1).transpose();
    out.value = mag_op.back() - _tl_coeffs.dot(A_matrix_deltaN);

    // vector_xf_t out_vector = vector_xf_t::Zero(A_matrix.rows());
    // out_vector = A_matrix * _tl_coeffs;
    // out.value = out_vector(out_vector.rows() - 1);
    // cout << "out_vector = " << endl
    //      << out_vector.transpose() << endl;
    // cout << "out.value = " << out.value << endl;

    // for observer updating
    notify(*this);
    return ret_t::MIC_RET_SUCCESSED;
}

ret_t MicTLMagCompensator::serialize(json_t &node)
{
    std::vector<double> beta(_tl_coeffs.rows());
    for (int i = 0; i < 18; i++)
    {
        beta[i] = _tl_coeffs(i);
    }
    node["tl_model"]["coeff_beta"] = beta;
    return MicMagCompensator::serialize(node);
}

ret_t MicTLMagCompensator::deserialize(json_t &node)
{
    std::vector<double> beta = node["tl_model"]["coeff_beta"];
    // if (beta.size() != 18)
    //     return ret_t::MIC_RET_FAILED;
    _tl_coeffs = vector_xf_t::Zero(beta.size());
    for (int i = 0; i < beta.size(); i++)
    {
        _tl_coeffs(i) = beta[i];
    }
    return MicMagCompensator::deserialize(node);
}

MIC_NAMESPACE_END