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
#include "mic_mag_compensator/impl/mic_cabin_mag_compensator.h"

// debug
#include <iostream>
#include <fstream>
using namespace std;

MIC_NAMESPACE_START

MicCabinMagCompensator::MicCabinMagCompensator() : MicMagCompensator()
{
    _mag_ellipsoid_ptr = std::make_shared<mic_ellipsoid_mag_compensator_t>();
}

ret_t MicCabinMagCompensator::do_calibrate()
{
    auto data_range = _mag_measure_storer.get_data_range<mic_mag_t>(0.0, _curr_time_stamp + 1);
    auto it_start = data_range.first;
    auto it_end = data_range.second;

    std::vector<vector_3f_t> mag_vec, mag_n_vec;
    std::vector<float64_t> mag_n_value;
    for (auto it = it_start; it != it_end; ++it)
    {
        float64_t ts = it->first;
        mic_mag_t mag = it->second;
        mic_nav_state_t nav_state;
        mic_mag_t mag_truth;
        // if (_mag_measure_storer.get_data<mic_nav_state_t>(ts, nav_state) &&
        if (_mag_truth_storer.get_data<mic_mag_t>(ts, mag_truth))
        {
            mag_vec.push_back(mag.vector);
            // mag_n_vec.push_back(nav_state.attitude.matrix().transpose() * mag_truth.vector);
            mag_n_vec.push_back(mag_truth.vector);
            mag_n_value.push_back(mag_truth.value);
        }
    }
    if (mag_vec.size() < 20)
        return ret_t::MIC_RET_FAILED;

    // double mag_earth_intensity = MIC_CONFIG_GET(float64_t, "mag_earth_intensity");
    double mag_earth_intensity = std::accumulate(mag_n_value.begin(), mag_n_value.end(), 0);
    mag_earth_intensity /= mag_n_value.size();
    double mag_sum = 0;
    for (size_t i = 0; i < mag_n_vec.size(); ++i)
    {
        mag_sum += mag_n_vec[i].norm();
    }
    mag_earth_intensity = mag_sum / mag_n_vec.size();
    MIC_LOG_DEBUG_INFO("mag_earth_intensity = %f", mag_earth_intensity);

    // std::vector<float64_t> ellipsoid_coeffs;
    if (_mag_ellipsoid_ptr->fit_ellipsoid_model(mag_vec, mag_earth_intensity, _D_tilde_inv, _o_hat) == ret_t::MIC_RET_FAILED)
    {
        MIC_LOG_ERR("ellipsoid fitting error!");
        return ret_t::MIC_RET_FAILED;
    }

    // debug
    ofstream fp_d("D_tilde_inv.txt"), fp_o("o_hat.txt");
    fp_d << fixed << _D_tilde_inv;
    fp_o << fixed << _o_hat;
    fp_d.close();
    fp_o.close();

    matrix_3f_t R_hat = matrix_3f_t::Identity();
    const size_t N = mag_vec.size();
    std::vector<vector_3f_t> p_left(N), p_right(N);
    for (size_t i = 0; i < N; ++i)
    {
        p_left[i] = _D_tilde_inv * (mag_vec[i] - _o_hat);
        p_right[i] = mag_n_vec[i];
    }
    // if (init_value_estimate(mag_vec, mag_n_vec, R_nb, _D_tilde_inv, _o_hat, R_hat) == ret_t::MIC_RET_FAILED)
    if (_mag_ellipsoid_ptr->fit_ls_rotation(p_left, p_right, R_hat) == ret_t::MIC_RET_FAILED)
    {
        MIC_LOG_ERR("failed to compute intial R_hat!");
        return ret_t::MIC_RET_FAILED;
    }

    // debug
    ofstream fp_R("R_hat.txt");
    fp_R << fixed << R_hat;
    fp_R.close();
    // cout << "R_hat = " << endl
    //      << R_hat << endl;

    quaternionf_t quat;
    if (R_hat.determinant() > 0)
    {
        // quat = quaternionf_t(R_hat);
        quat = quaternionf_t(matrix_3f_t::Identity());
        if (ceres_optimize(mag_vec, mag_n_vec, _D_tilde_inv, _o_hat, quat) == ret_t::MIC_RET_FAILED)
        {
            MIC_LOG_ERR("failed to get R_opt by ceres optimization!");
            return ret_t::MIC_RET_FAILED;
        }

        // debug
        _R_opt = quat.toRotationMatrix();
        // cout << "R_opt = " << endl
        //      << _R_opt << endl;
        ofstream fp_R("R_opt.txt");
        fp_R << fixed << _R_opt;
        fp_R.close();
    }
    else
    {
        MIC_LOG_ERR("det(R_hat) != 1");
        return ret_t::MIC_RET_FAILED;
    }

    notify(*this);
    return ret_t::MIC_RET_SUCCESSED;
}

ret_t MicCabinMagCompensator::do_compenste(const float64_t ts, mic_mag_t &out)
{
    matrix_3f_t matrix = _R_opt.transpose() * _D_tilde_inv;
    vector_3f_t offset = _o_hat;

    mic_mag_t in;
    if (_mag_measure_storer.get_data<mic_mag_t>(ts, in))
    {
        out.vector = matrix * (in.vector - offset);
        out.value = out.vector.norm();
        notify(*this);
        return ret_t::MIC_RET_SUCCESSED;
    }
    else
    {
        notify(*this);
        return ret_t::MIC_RET_FAILED;
    }
}

ret_t MicCabinMagCompensator::serialize(json_t &node)
{
    cout << fixed << std::setprecision(2);
    cout << "calibrated model coefficients:\n\n"
         << "coeff_D: \n"
         << _D_tilde_inv.inverse() << endl
         << endl;
    cout << "coeff_o: \n"
         << _o_hat.transpose() << endl
         << endl;

    std::vector<double> D(9), R(9), o(3);
    for (int i = 0; i < 3; i++)
    {
        o[i] = _o_hat(i);
        for (int j = 0; j < 3; j++)
        {
            D[i * 3 + j] = _D_tilde_inv(i, j);
            R[i * 3 + j] = _R_opt(i, j);
        }
    }
    node["ellipsoid_model"]["coeff_D_inv"] = D;
    node["ellipsoid_model"]["coeff_R"] = R;
    node["ellipsoid_model"]["coeff_o"] = o;
    return MicMagCompensator::serialize(node);
}

ret_t MicCabinMagCompensator::deserialize(json_t &node)
{
    std::vector<double> D = node["ellipsoid_model"]["coeff_D_inv"];
    std::vector<double> R = node["ellipsoid_model"]["coeff_R"];
    std::vector<double> o = node["ellipsoid_model"]["coeff_o"];
    if (D.size() != 9 || R.size() != 9 || o.size() != 3)
        return ret_t::MIC_RET_FAILED;
    for (int i = 0; i < 3; i++)
    {
        _o_hat(i) = o[i];
        for (int j = 0; j < 3; j++)
        {
            _D_tilde_inv(i, j) = D[i * 3 + j];
            _R_opt(i, j) = R[i * 3 + j];
        }
    }
    return MicMagCompensator::deserialize(node);
}

ret_t MicCabinMagCompensator::ceres_optimize(
    const std::vector<vector_3f_t> &mag,
    const std::vector<vector_3f_t> &mag_1,
    const matrix_3f_t &D_tilde_inv,
    const vector_3f_t &o_hat,
    quaternionf_t &quat)
{
    if (mag.size() != mag_1.size())
        return ret_t::MIC_RET_FAILED;

    ceres::Problem problem;
    ceres::LossFunction *loss_function = new ceres::HuberLoss(1.0);
    ceres::LocalParameterization *quat_param =
        new ceres::EigenQuaternionParameterization;

    for (size_t k = 0; k < mag.size(); ++k)
    {
        vector_3f_t mag_r = mag[k];
        vector_3f_t mag_c = mag_1[k];

        ceres::CostFunction *cost_function = CabinCostFunctionCreator::Create(
            D_tilde_inv, o_hat, mag_r, mag_c);

        problem.AddResidualBlock(cost_function, loss_function, quat.coeffs().data());
        problem.SetParameterization(quat.coeffs().data(), quat_param);
    }

    ceres::Solver::Summary summary;
    ceres::Solver::Options options;
    options.max_num_iterations = 50;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
    options.minimizer_progress_to_stdout = true;

    ceres::Solve(options, &problem, &summary);
    // std::cout << summary.BriefReport() << std::endl;
    std::cout << summary.FullReport() << std::endl;

    if (summary.IsSolutionUsable())
        return ret_t::MIC_RET_SUCCESSED;
    else
        return ret_t::MIC_RET_FAILED;
}

MIC_NAMESPACE_END