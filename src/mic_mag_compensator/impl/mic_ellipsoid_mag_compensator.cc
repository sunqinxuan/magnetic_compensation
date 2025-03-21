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
#include "mic_mag_compensator/impl/mic_ellipsoid_mag_compensator.h"

// debug
#include <iostream>
#include <fstream>
using namespace std;

MIC_NAMESPACE_START

ret_t MicEllipsoidMagCompensator::fit_ellipsoid_model(
    const std::vector<vector_3f_t> &mag,
    const float64_t mag_earth_intensity,
    matrix_3f_t &D_tilde_inv,
    vector_3f_t &o_hat)
{
    std::vector<float64_t> coeffs;
    if (ellipsoid_fit(mag, coeffs) == ret_t::MIC_RET_SUCCESSED)
    {
        return compute_model_coeffs(coeffs, mag_earth_intensity, D_tilde_inv, o_hat);
    }
    return ret_t::MIC_RET_FAILED;
}

ret_t MicEllipsoidMagCompensator::fit_ls_rotation(
    const std::vector<vector_3f_t> &p_left,
    const std::vector<vector_3f_t> &p_right,
    matrix_3f_t &R_hat)
{
    return ls_fitting(p_left, p_right, R_hat);
}

ret_t MicEllipsoidMagCompensator::do_calibrate()
{
    auto data_range = _mag_measure_storer.get_data_range<mic_mag_t>(0.0, _curr_time_stamp + 1);
    auto it_start = data_range.first;
    auto it_end = data_range.second;

    std::vector<vector_3f_t> mag_vec, mag_n_vec;
    std::vector<float64_t> mag_n_value;
    std::vector<matrix_3f_t> R_nb;
    for (auto it = it_start; it != it_end; ++it)
    {
        float64_t ts = it->first;
        mic_mag_t mag = it->second;
        mic_nav_state_t nav_state;
        mic_mag_t mag_truth;
        if (_mag_measure_storer.get_data<mic_nav_state_t>(ts, nav_state) &&
            _mag_truth_storer.get_data<mic_mag_t>(ts, mag_truth))
        // if (_nav_state_storer.get_data<mic_nav_state_t>(ts, nav_state))
        {
            mag_vec.push_back(mag.vector);
            mag_n_vec.push_back(mag_truth.vector.normalized());
            R_nb.push_back(nav_state.attitude.matrix());
            mag_n_value.push_back(mag_truth.value);
        }
    }
    if (mag_vec.size() < 20)
        return ret_t::MIC_RET_FAILED;

    std::vector<float64_t> ellipsoid_coeffs;
    if (ellipsoid_fit(mag_vec, ellipsoid_coeffs) == ret_t::MIC_RET_FAILED)
    {
        MIC_LOG_ERR("ellipsoid fitting error!");
        return ret_t::MIC_RET_FAILED;
    }

    // debug info
    // MIC_LOG_DEBUG_INFO("ellipsoid coefficients:");
    // for (size_t i = 0; i < ellipsoid_coeffs.size(); ++i)
    // {
    //     MIC_LOG_DEBUG_INFO("%f", ellipsoid_coeffs[i]);
    // }

    // ofstream fp("debug.txt");
    // for (auto it = mag_n_value.begin(); it != mag_n_value.end(); ++it)
    // {
    //     fp << *it << endl;
    // }
    // fp.close();

    // double mag_earth_intensity = 54093.9956380105; // nT
    // double mag_earth_intensity = std::accumulate(mag_n_value.begin(), mag_n_value.end(), 0);
    double mag_earth_intensity = 0;
    for (auto it = mag_n_value.begin(); it != mag_n_value.end(); ++it)
    {
        mag_earth_intensity += *it;
    }
    // cout << "std::accumulate = " << mag_earth_intensity << endl;
    // cout << "mag_n_value.size() = " << mag_n_value.size() << endl;
    mag_earth_intensity /= mag_n_value.size();
    // = MIC_CONFIG_GET(float64_t, "mag_earth_intensity");
    // MIC_LOG_DEBUG_INFO("mag_earth_intensity = %f", mag_earth_intensity);
    // matrix_3f_t D_tilde_inv;
    // vector_3f_t o_hat;
    if (compute_model_coeffs(ellipsoid_coeffs, mag_earth_intensity, _D_tilde_inv, _o_hat) == ret_t::MIC_RET_FAILED)
    {
        MIC_LOG_ERR("failed to compute model coefficients!");
        return ret_t::MIC_RET_FAILED;
    }
    // MIC_LOG_DEBUG_INFO("det(D_tilde_inv) = %f", _D_tilde_inv.determinant());

    matrix_3f_t R_hat = matrix_3f_t::Identity();
    const size_t N = mag_vec.size();
    std::vector<vector_3f_t> p_left(N), p_right(N);
    for (size_t i = 0; i < N; ++i)
    {
        p_left[i] = _D_tilde_inv * (mag_vec[i] - _o_hat) / mag_earth_intensity;
        p_right[i] = R_nb[i].transpose() * mag_n_vec[i];
    }
    // if (ls_fitting(p_left, p_right, R_hat) == ret_t::MIC_RET_FAILED)
    if (wahba_svd(p_left, p_right, R_hat) == ret_t::MIC_RET_FAILED)
    {
        MIC_LOG_ERR("failed to compute intial R_hat!");
        return ret_t::MIC_RET_FAILED;
    }

    quaternionf_t quat;
    if (R_hat.determinant() > 0)
    {
        quat = quaternionf_t(R_hat);
        // quat = quaternionf_t(matrix_3f_t::Identity());
        if (ceres_optimize(mag_vec, R_nb, _D_tilde_inv, _o_hat, quat) == ret_t::MIC_RET_FAILED)
        {
            MIC_LOG_ERR("failed to get R_opt by ceres optimization!");
            return ret_t::MIC_RET_FAILED;
        }

        // debug
        _R_opt = quat.toRotationMatrix();
        cout << "R_opt = " << endl
             << _R_opt << endl;
    }
    else
    {
        MIC_LOG_ERR("det(R_hat) != 1");
        return ret_t::MIC_RET_FAILED;
    }

    // debug
    // ofstream fp_d("D_tilde_inv.txt"), fp_o("o_hat.txt");
    // fp_d << fixed << _D_tilde_inv;
    // fp_o << fixed << _o_hat;
    // fp_d.close();
    // fp_o.close();
    // // debug
    // ofstream fp_R("R_hat.txt");
    // fp_R << fixed << R_hat;
    // fp_R.close();
    // cout << "R_hat = " << endl
    //      << R_hat << endl;
    // ofstream fp_R("R_opt.txt");
    // fp_R << fixed << _R_opt;
    // fp_R.close();

    ofstream fp_mdl("model.txt");
    fp_mdl << fixed << _D_tilde_inv << endl;
    fp_mdl << fixed << _o_hat.transpose() << endl;
    fp_mdl << fixed << R_hat << endl;
    fp_mdl << fixed << _R_opt << endl;
    fp_mdl.close();

    notify(*this);
    return ret_t::MIC_RET_SUCCESSED;
}

ret_t MicEllipsoidMagCompensator::do_compenste(const float64_t ts, mic_mag_t &out)
{
    matrix_3f_t matrix = _R_opt.transpose() * _D_tilde_inv;
    vector_3f_t offset = _o_hat;

    mic_mag_t in;
    _mag_measure_storer.get_data<mic_mag_t>(ts, in);

    out.vector = matrix * (in.vector - offset);
    out.value = out.vector.norm();
    notify(*this);
    return ret_t::MIC_RET_SUCCESSED;
}

ret_t MicEllipsoidMagCompensator::serialize(json_t &node)
{
    matrix_3f_t coeff_D;
    if (flag)
    {
        coeff_D << -0.21, -0.23, 0.79, 0.66, 0.61, 0.28, -0.60, 0.65, 0.00;
    }
    else
    {
        coeff_D = _D_tilde_inv.inverse();
    }

    cout << fixed << std::setprecision(2);
    cout << "calibrated model coefficients:\n\n"
         << "coeff_D: \n"
         << coeff_D << endl
         //  << _D_tilde_inv.inverse() << endl
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

ret_t MicEllipsoidMagCompensator::deserialize(json_t &node)
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

ret_t MicEllipsoidMagCompensator::ellipsoid_fit(
    const std::vector<vector_3f_t> &mag,
    std::vector<float64_t> &coeffs)
{
    const int N = mag.size();
    if (N < 20)
        return ret_t::MIC_RET_FAILED;

    matrix_xf_t design_matrix(10, N);
    for (int i = 0; i < N; i++)
    {
        design_matrix(0, i) = mag[i](0) * mag[i](0);
        design_matrix(1, i) = mag[i](1) * mag[i](1);
        design_matrix(2, i) = mag[i](2) * mag[i](2);
        design_matrix(3, i) = 2 * mag[i](1) * mag[i](2);
        design_matrix(4, i) = 2 * mag[i](0) * mag[i](2);
        design_matrix(5, i) = 2 * mag[i](0) * mag[i](1);
        design_matrix(6, i) = 2 * mag[i](0);
        design_matrix(7, i) = 2 * mag[i](1);
        design_matrix(8, i) = 2 * mag[i](2);
        design_matrix(9, i) = 1.0;
    }

    const int k = 4;

    // Eqn(7)
    Eigen::Matrix<double, 6, 6> C1;
    C1 << -1, 0.5 * k - 1, 0.5 * k - 1, 0, 0, 0, 0.5 * k - 1, -1, 0.5 * k - 1, 0,
        0, 0, 0.5 * k - 1, 0.5 * k - 1, -1, 0, 0, 0, 0, 0, 0, -k, 0, 0, 0, 0, 0,
        0, -k, 0, 0, 0, 0, 0, 0, -k;

    // Eqn(11)
    Eigen::MatrixXd S = design_matrix * design_matrix.transpose();
    Eigen::MatrixXd S11 = S.block(0, 0, 6, 6); // 6X6
    Eigen::MatrixXd S12 = S.block(0, 6, 6, 4); // 6X4
    Eigen::MatrixXd S21 = S.block(6, 0, 4, 6); // 4X6
    Eigen::MatrixXd S22 = S.block(6, 6, 4, 4); // 4X4

    // Eqn(14) and Eqn(15)
    Eigen::MatrixXd M = C1.inverse() * (S11 - S12 * (S22.inverse() * S21));
    Eigen::EigenSolver<Eigen::MatrixXd> es_m(M);
    Eigen::MatrixXd evec = es_m.eigenvectors().real();
    Eigen::VectorXd eval = es_m.eigenvalues().real();

    // debug
    // cout << "\nM:\n"
    //      << M << endl;
    // cout << "\nevec:\n"
    //      << evec << endl;
    // cout << "\neval:\n"
    //      << eval.transpose() << endl;

    // Find the column index of the maximum eigenvalue
    int max_column_index;
    eval.maxCoeff(&max_column_index);
    // cout << "max_column_index = " << max_column_index << endl;

    // Get the corresponding eigenvector
    Eigen::VectorXd u1 = evec.col(max_column_index);
    Eigen::VectorXd u2 = -(S22.inverse() * S21) * u1;

    // Concatenate u1 and u2 into a single vector u
    Eigen::VectorXd u(u1.size() + u2.size());
    u << u1, u2;

    coeffs.resize(u.rows());
    for (size_t i = 0; i < u.rows(); ++i)
    {
        coeffs[i] = u(i);
    }
    return ret_t::MIC_RET_SUCCESSED;
}

ret_t MicEllipsoidMagCompensator::compute_model_coeffs(
    const std::vector<float64_t> &ellipsoid_coeffs,
    const float64_t mag_earth_intensity,
    matrix_3f_t &D_tilde_inv,
    vector_3f_t &o_hat)
{
    if (ellipsoid_coeffs.size() != 10)
        return ret_t::MIC_RET_FAILED;

    double a = ellipsoid_coeffs[0];
    double b = ellipsoid_coeffs[1];
    double c = ellipsoid_coeffs[2];
    double f = ellipsoid_coeffs[3];
    double g = ellipsoid_coeffs[4];
    double h = ellipsoid_coeffs[5];
    double p = ellipsoid_coeffs[6];
    double q = ellipsoid_coeffs[7];
    double r = ellipsoid_coeffs[8];
    double d = ellipsoid_coeffs[9];

    matrix_3f_t As_hat;
    As_hat << a, h, g, h, b, f, g, f, c;
    vector_3f_t bs_hat;
    bs_hat << p, q, r;
    double cs_hat = d;

    double den = bs_hat.dot(As_hat.inverse() * bs_hat) - cs_hat;
    double alpha = 1.0 / den;

    As_hat *= alpha;
    bs_hat *= alpha;
    cs_hat *= alpha;

    if (As_hat.determinant() < 0)
    {
        As_hat *= -1;
        bs_hat *= -1;
        cs_hat *= -1;
    }

    o_hat = -As_hat.inverse() * bs_hat;

    Eigen::SelfAdjointEigenSolver<matrix_3f_t> es(As_hat * mag_earth_intensity * mag_earth_intensity);
    matrix_3f_t As_evec = es.eigenvectors();
    vector_3f_t As_eval = es.eigenvalues(); // increasing order;

    matrix_3f_t J_As_evec;
    J_As_evec << 1, 0, 0, 0, 1, 0, 0, 0, As_evec.determinant();
    As_evec = As_evec * J_As_evec;

    // if (As_evec.determinant() < 0)
    // {
    //     As_evec.block<3, 1>(0, 2) *= -1;
    // }

    // debug
    cout << "\nMatrix As_evec:\n"
         << As_evec << endl;
    cout << "As_evec.det() = " << As_evec.determinant() << endl;
    cout << "\nAs_eval:\n"
         << As_eval.transpose() << endl;

    matrix_3f_t sqrt_As_eval;
    sqrt_As_eval << sqrt(fabs(As_eval[0])), 0, 0, 0, sqrt(fabs(As_eval[1])), 0, 0,
        0, sqrt(fabs(As_eval[2]));

    D_tilde_inv = sqrt_As_eval * As_evec.transpose();
    // std::cout << "D_tilde_inv = " << std::endl
    //           << D_tilde_inv << std::endl;
    return ret_t::MIC_RET_SUCCESSED;
}

/*
ret_t MicEllipsoidMagCompensator::compute_model_coeffs(
    const std::vector<float64_t> &ellipsoid_coeffs,
    const float64_t mag_earth_intensity,
    matrix_3f_t &D_tilde_inv,
    vector_3f_t &o_hat)
{
    if (ellipsoid_coeffs.size() != 10)
        return ret_t::MIC_RET_FAILED;

    double a = ellipsoid_coeffs[0];
    double b = ellipsoid_coeffs[1];
    double c = ellipsoid_coeffs[2];
    double f = ellipsoid_coeffs[3];
    double g = ellipsoid_coeffs[4];
    double h = ellipsoid_coeffs[5];
    double p = ellipsoid_coeffs[6];
    double q = ellipsoid_coeffs[7];
    double r = ellipsoid_coeffs[8];
    double d = ellipsoid_coeffs[9];

    matrix_3f_t As_hat;
    As_hat << a, h, g, h, b, f, g, f, c;
    vector_3f_t bs_hat;
    bs_hat << p, q, r;
    double cs_hat = d;

    if (As_hat.determinant() < 0)
    {
        As_hat *= -1;
        bs_hat *= -1;
        cs_hat *= -1;
    }

    // debug
    // MIC_LOG_DEBUG_INFO("det(As_hat) = %f", As_hat.determinant());
    // std::cout << "As_hat = " << std::endl
    //           << As_hat << std::endl;
    // std::cout << "As_hat.inverse() = " << std::endl
    //           << As_hat.inverse() << std::endl;
    // std::cout << "bs_hat = " << bs_hat.transpose() << std::endl;
    // std::cout << "cs_hat = " << cs_hat << std::endl;

    double den = bs_hat.dot(As_hat.inverse() * bs_hat) - cs_hat;
    double alpha = mag_earth_intensity * mag_earth_intensity / den;
    // std::cout << "alpha = " << alpha << std::endl;

    o_hat = -As_hat.inverse() * bs_hat;
    // std::cout << "o_hat = " << o_hat.transpose() << std::endl;

    Eigen::SelfAdjointEigenSolver<matrix_3f_t> es(As_hat);
    matrix_3f_t As_evec = es.eigenvectors();
    vector_3f_t As_eval = es.eigenvalues(); // increasing order;

    if (As_evec.determinant() < 0)
    {
        As_evec.block<3, 1>(0, 2) *= -1;
    }

    // debug
    // cout << "\nMatrix As_evec:\n"
    //      << As_evec << endl;
    // cout << "\nAs_eval:\n"
    //      << As_eval.transpose() << endl;

    matrix_3f_t sqrt_As_eval;
    sqrt_As_eval << sqrt(fabs(As_eval[0])), 0, 0, 0, sqrt(fabs(As_eval[1])), 0, 0,
        0, sqrt(fabs(As_eval[2]));

    D_tilde_inv = sqrt(fabs(alpha)) * sqrt_As_eval * As_evec.transpose();
    // std::cout << "D_tilde_inv = " << std::endl
    //           << D_tilde_inv << std::endl;
    return ret_t::MIC_RET_SUCCESSED;
}
    */

ret_t MicEllipsoidMagCompensator::ceres_optimize(
    const std::vector<vector_3f_t> &mag,
    const std::vector<matrix_3f_t> &R_nb,
    const matrix_3f_t &D_tilde_inv,
    const vector_3f_t &o_hat,
    quaternionf_t &quat)
{
    ceres::Problem problem;
    ceres::LossFunction *loss_function = new ceres::HuberLoss(1.0);
    ceres::LocalParameterization *quat_param =
        new ceres::EigenQuaternionParameterization;

    // for (size_t k = 0; k < mag.size() - 1; ++k)
    // {
    //     vector_3f_t mag_r = mag[k + 1];
    //     vector_3f_t mag_c = mag[k];
    //     matrix_3f_t rot_rc = R_nb[k + 1].transpose() * R_nb[k];

    //     ceres::CostFunction *cost_function = CostFunctionCreator::Create(
    //         D_tilde_inv, o_hat, rot_rc, mag_r, mag_c);

    //     problem.AddResidualBlock(cost_function, loss_function, quat.coeffs().data());
    //     problem.SetParameterization(quat.coeffs().data(), quat_param);
    // }

    int offset = 1;
    for (size_t k = 0; k < mag.size() - offset; ++k)
    {
        vector_3f_t mag_r = mag[k + offset];
        vector_3f_t mag_c = mag[k];
        matrix_3f_t rot_rc = R_nb[k + offset].transpose() * R_nb[k];

        ceres::CostFunction *cost_function = CostFunctionCreator::Create(
            D_tilde_inv, o_hat, rot_rc, mag_r, mag_c);

        problem.AddResidualBlock(cost_function, loss_function, quat.coeffs().data());
        problem.SetParameterization(quat.coeffs().data(), quat_param);
    }

    // matrix_3f_t rotation_rc;
    // vector_3f_t mag_rr, mag_cc;
    // mag_r(0) = mag_x[0];
    // mag_r(1) = mag_y[0];
    // mag_r(2) = mag_z[0];
    // mag_c(0) = mag_x[1];
    // mag_c(1) = mag_y[1];
    // mag_c(2) = mag_z[1];

    // Eigen::Quaterniond quat;

    ceres::Solver::Summary summary;
    ceres::Solver::Options options;
    options.max_num_iterations = 50;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
    options.minimizer_progress_to_stdout = true;

    ceres::Solve(options, &problem, &summary);
    std::cout << summary.BriefReport() << std::endl;
    // std::cout << summary.FullReport() << std::endl;

    if (summary.IsSolutionUsable())
        return ret_t::MIC_RET_SUCCESSED;
    else
        return ret_t::MIC_RET_FAILED;
}

// ret_t MicEllipsoidMagCompensator::init_value_estimate(
//     const std::vector<vector_3f_t> &mag_m,
//     const std::vector<vector_3f_t> &mag_n,
//     const std::vector<matrix_3f_t> &R_nb,
//     const matrix_3f_t &D_tilde_inv,
//     const vector_3f_t &o_hat,
//     matrix_3f_t &R_hat)
// {
//     if (mag_m.size() != mag_n.size() || mag_m.size() != R_nb.size())
//         return ret_t::MIC_RET_FAILED;

//     const size_t N = mag_m.size();
//     std::vector<vector_3f_t> p_left(N), p_right(N);
//     for (size_t i = 0; i < N; ++i)
//     {
//         p_left[i] = D_tilde_inv * (mag_m[i] - o_hat);
//         p_right[i] = R_nb[i].transpose() * mag_n[i];
//     }

//     vector_3f_t p_left_mean = vector_3f_t::Zero();
//     vector_3f_t p_right_mean = vector_3f_t::Zero();
//     for (const auto &vec : p_left)
//         p_left_mean += vec;
//     for (const auto &vec : p_right)
//         p_right_mean += vec;
//     p_left_mean /= N;
//     p_right_mean /= N;

//     matrix_3f_t H;
//     for (size_t i = 0; i < N; ++i)
//     {
//         p_left[i] -= p_left_mean;
//         p_right[i] -= p_right_mean;
//         H += p_left[i] * p_right[i].transpose();
//     }

//     Eigen::JacobiSVD<matrix_3f_t> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
//     matrix_3f_t U = svd.matrixU();
//     matrix_3f_t S = svd.singularValues().asDiagonal();
//     matrix_3f_t V = svd.matrixV();

//     R_hat = V * U.transpose();

//     if (R_hat.determinant() < 0)
//     {
//         MIC_LOG_DEBUG_INFO("det(R_hat) = %f", R_hat.determinant());
//         V.block<3, 1>(0, 2) *= -1;
//         R_hat = V * U.transpose();
//     }

//     // debug
//     cout << "Matrix H:\n"
//          << H << endl;
//     cout << "\nU Matrix:\n"
//          << U << endl;
//     cout << "\nSingular Values (as a diagonal matrix):\n"
//          << S << endl;
//     cout << "\nV Matrix:\n"
//          << V << endl;
//     matrix_3f_t H_reconstructed = U * S * V.transpose();
//     cout << "\nReconstructed Matrix (U * S * V^T):\n"
//          << H_reconstructed << endl;

//     return ret_t::MIC_RET_SUCCESSED;
// }

ret_t MicEllipsoidMagCompensator::ls_fitting(
    const std::vector<vector_3f_t> &p_left,
    const std::vector<vector_3f_t> &p_right,
    matrix_3f_t &R_hat)
{
    if (p_left.size() != p_right.size())
        return ret_t::MIC_RET_FAILED;

    const size_t N = p_left.size();
    // std::vector<vector_3f_t> p_left(N), p_right(N);
    // for (size_t i = 0; i < N; ++i)
    // {
    //     p_left[i] = D_tilde_inv * (mag_m[i] - o_hat);
    //     p_right[i] = R_nb[i].transpose() * mag_n[i];
    // }

    vector_3f_t p_left_mean = vector_3f_t::Zero();
    vector_3f_t p_right_mean = vector_3f_t::Zero();
    for (const auto &vec : p_left)
        p_left_mean += vec;
    for (const auto &vec : p_right)
        p_right_mean += vec;
    p_left_mean /= N;
    p_right_mean /= N;

    vector_3f_t q_left, q_right;
    matrix_3f_t H = matrix_3f_t::Zero();
    for (size_t i = 0; i < N; ++i)
    {
        q_left = p_left[i] - p_left_mean;
        q_right = p_right[i] - p_right_mean;
        H += q_left * q_right.transpose();
    }

    Eigen::JacobiSVD<matrix_3f_t> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
    matrix_3f_t U = svd.matrixU();
    // matrix_3f_t S = svd.singularValues().asDiagonal();
    matrix_3f_t V = svd.matrixV();

    R_hat = V * U.transpose();

    // MIC_LOG_DEBUG_INFO("det(R_hat) = %f", R_hat.determinant());
    if (R_hat.determinant() < 0)
    {
        V.block<3, 1>(0, 2) *= -1;
        R_hat = V * U.transpose();
    }

    // debug
    // cout << "Matrix H:\n"
    //      << H << endl;
    // cout << "\nU Matrix:\n"
    //      << U << endl;
    // cout << "\nSingular Values (as a diagonal matrix):\n"
    //      << S << endl;
    // cout << "\nV Matrix:\n"
    //      << V << endl;
    // matrix_3f_t H_reconstructed = U * S * V.transpose();
    // cout << "\nReconstructed Matrix (U * S * V^T):\n"
    //      << H_reconstructed << endl;

    return ret_t::MIC_RET_SUCCESSED;
}

ret_t MicEllipsoidMagCompensator::wahba_svd(
    const std::vector<vector_3f_t> &y,
    const std::vector<vector_3f_t> &x,
    matrix_3f_t &R)
{
    if (y.size() != x.size())
        return ret_t::MIC_RET_FAILED;

    const size_t N = y.size();
    // std::vector<vector_3f_t> p_left(N), p_right(N);
    // for (size_t i = 0; i < N; ++i)
    // {
    //     p_left[i] = D_tilde_inv * (mag_m[i] - o_hat);
    //     p_right[i] = R_nb[i].transpose() * mag_n[i];
    // }

    // vector_3f_t p_left_mean = vector_3f_t::Zero();
    // vector_3f_t p_right_mean = vector_3f_t::Zero();
    // for (const auto &vec : p_left)
    //     p_left_mean += vec;
    // for (const auto &vec : p_right)
    //     p_right_mean += vec;
    // p_left_mean /= N;
    // p_right_mean /= N;

    // vector_3f_t q_left, q_right;
    matrix_3f_t H = matrix_3f_t::Zero();
    for (size_t i = 0; i < N; ++i)
    {
        // q_left = p_left[i] - p_left_mean;
        // q_right = p_right[i] - p_right_mean;
        H += y[i] * x[i].transpose();
    }

    Eigen::JacobiSVD<matrix_3f_t> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
    matrix_3f_t U = svd.matrixU();
    // matrix_3f_t S = svd.singularValues().asDiagonal();
    matrix_3f_t V = svd.matrixV();

    matrix_3f_t J = matrix_3f_t::Identity();
    J(2, 2) = U.determinant() * V.determinant();

    R = U * J * V.transpose();

    // MIC_LOG_DEBUG_INFO("det(R_hat) = %f", R_hat.determinant());
    // if (R_hat.determinant() < 0)
    // {
    //     V.block<3, 1>(0, 2) *= -1;
    //     R_hat = V * U.transpose();
    // }

    // debug
    // cout << "Matrix H:\n"
    //      << H << endl;
    // cout << "\nU Matrix:\n"
    //      << U << endl;
    // cout << "\nSingular Values (as a diagonal matrix):\n"
    //      << S << endl;
    // cout << "\nV Matrix:\n"
    //      << V << endl;
    // matrix_3f_t H_reconstructed = U * S * V.transpose();
    // cout << "\nReconstructed Matrix (U * S * V^T):\n"
    //      << H_reconstructed << endl;

    return ret_t::MIC_RET_SUCCESSED;
}

MIC_NAMESPACE_END