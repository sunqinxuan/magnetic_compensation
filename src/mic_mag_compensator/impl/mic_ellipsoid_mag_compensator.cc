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

ret_t MicEllipsoidMagCompensator::do_calibrate()
{
    ret_t ret = ret_t::MIC_RET_FAILED;

    auto data_range = _mag_measure_storer.get_data_range<mic_mag_flux_t>(0.0, _curr_time_stamp + 1);
    auto it_start = data_range.first;
    auto it_end = data_range.second;

    std::vector<vector_3f_t> mag_vec, mag_n_vec;
    std::vector<matrix_3f_t> R_nb;
    for (auto it = it_start; it != it_end; ++it)
    {
        float64_t ts = it->first;
        mic_mag_flux_t mag_flux = it->second;
        mic_nav_state_t nav_state;
        mic_mag_flux_t mag_flux_truth;
        _mag_truth_storer.get_data<mic_mag_flux_t>(ts, mag_flux_truth);
        if(_nav_state_storer.get_data<mic_nav_state_t>(ts,nav_state))
        // if (_nav_state_estimator->get_nav_state(ts, nav_state) == ret_t::MIC_RET_SUCCESSED)
        {
            mag_vec.push_back(mag_flux.vector);
            mag_n_vec.push_back(mag_flux_truth.vector);
            R_nb.push_back(nav_state.attitude.matrix());
        }
    }

    std::vector<float64_t> ellipsoid_coeffs;
    ellipsoid_fit(mag_vec, ellipsoid_coeffs);

    // debug info
    MIC_LOG_DEBUG_INFO("ellipsoid coefficients:");
    for (size_t i = 0; i < ellipsoid_coeffs.size(); ++i)
    {
        MIC_LOG_DEBUG_INFO("%f", ellipsoid_coeffs[i]);
    }

    // double mag_earth_intensity = 54093.9956380105; // nT
    double mag_earth_intensity = MIC_CONFIG_GET(float64_t, "mag_earth_intensity");
    MIC_LOG_DEBUG_INFO("mag_earth_intensity = %f", mag_earth_intensity);
    // matrix_3f_t D_tilde_inv;
    // vector_3f_t o_hat;
    compute_model_coeffs(ellipsoid_coeffs, mag_earth_intensity, _D_tilde_inv, _o_hat);

    // debug
    ofstream fp_d("D_tilde_inv.txt"), fp_o("o_hat.txt");
    fp_d << fixed << _D_tilde_inv;
    fp_o << fixed << _o_hat;
    fp_d.close();
    fp_o.close();

    matrix_3f_t R_hat;
    init_value_estimate(mag_vec, mag_n_vec, R_nb, _D_tilde_inv, _o_hat, R_hat);

    // debug
    ofstream fp_R("R_hat.txt");
    fp_R << fixed << R_hat;
    fp_R.close();
    MIC_LOG_DEBUG_INFO("det(R_hat) = %f", R_hat.determinant());
    cout << "R_hat = " << endl
         << R_hat << endl;

    quaternionf_t quat;
    if (R_hat.determinant() > 0)
    {
        quat = quaternionf_t(R_hat);
        ceres_optimize(mag_vec, R_nb, _D_tilde_inv, _o_hat, quat);

        // debug
        matrix_3f_t R_opt = quat.toRotationMatrix();
        cout << "R_opt = " << endl
             << R_opt << endl;
        ofstream fp_R("R_opt.txt");
        fp_R << fixed << R_opt;
        fp_R.close();
    }
    else
    {
        MIC_LOG_ERR("det(R_hat) != 1");
    }

    notify(*this);
    return ret;
}

ret_t MicEllipsoidMagCompensator::do_compenste(const mic_mag_flux_t &in, mic_mag_flux_t &out)
{
    // for observer updating
    notify(*this);
    return ret_t::MIC_RET_FAILED;
}

ret_t MicEllipsoidMagCompensator::ellipsoid_fit(
    const std::vector<vector_3f_t> &mag,
    std::vector<float64_t> &coeffs)
{
    const int N = mag.size();
    if (N < 20)
        return ret_t::MIC_RET_FAILED;

    Eigen::MatrixXd design_matrix(10, N);
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

    // Find the column index of the maximum eigenvalue
    int max_column_index;
    eval.maxCoeff(&max_column_index);

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

ret_t MicEllipsoidMagCompensator::serialize(json_t &node)
{
    // TODO
    // ...
    return MicMagCompensator::serialize(node);
}

ret_t MicEllipsoidMagCompensator::deserialize(json_t &node)
{
    // TODO
    // ...
    return MicMagCompensator::deserialize(node);
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

    matrix_3f_t sqrt_As_eval;
    sqrt_As_eval << sqrt(fabs(As_eval[0])), 0, 0, 0, sqrt(fabs(As_eval[1])), 0, 0,
        0, sqrt(fabs(As_eval[2]));

    D_tilde_inv = sqrt(fabs(alpha)) * sqrt_As_eval * As_evec.transpose();
    // std::cout << "D_tilde_inv = " << std::endl
    //           << D_tilde_inv << std::endl;
    return ret_t::MIC_RET_SUCCESSED;
}

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

    for (size_t k = 0; k < mag.size() - 1; ++k)
    {
        vector_3f_t mag_r = mag[k + 1];
        vector_3f_t mag_c = mag[k];
        matrix_3f_t rot_rc = R_nb[k + 1].transpose() * R_nb[k];

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
    options.max_num_iterations = 10;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
    options.minimizer_progress_to_stdout = true;

    ceres::Solve(options, &problem, &summary);
    std::cout << summary.FullReport() << std::endl;

    if (summary.IsSolutionUsable())
        return ret_t::MIC_RET_SUCCESSED;
    else
        return ret_t::MIC_RET_FAILED;
}

ret_t MicEllipsoidMagCompensator::init_value_estimate(
    const std::vector<vector_3f_t> &mag_m,
    const std::vector<vector_3f_t> &mag_n,
    const std::vector<matrix_3f_t> &R_nb,
    const matrix_3f_t &D_tilde_inv,
    const vector_3f_t &o_hat,
    matrix_3f_t &R_hat)
{
    if (mag_m.size() != mag_n.size() || mag_m.size() != R_nb.size())
        return ret_t::MIC_RET_FAILED;

    const size_t N = mag_m.size();
    std::vector<vector_3f_t> p_left(N), p_right(N);
    for (size_t i = 0; i < N; ++i)
    {
        p_left[i] = D_tilde_inv * (mag_m[i] - o_hat);
        p_right[i] = R_nb[i].transpose() * mag_n[i];
    }

    vector_3f_t p_left_mean = vector_3f_t::Zero();
    vector_3f_t p_right_mean = vector_3f_t::Zero();
    for (const auto &vec : p_left)
        p_left_mean += vec;
    for (const auto &vec : p_right)
        p_right_mean += vec;
    p_left_mean /= N;
    p_right_mean /= N;

    matrix_3f_t H;
    for (size_t i = 0; i < N; ++i)
    {
        p_left[i] -= p_left_mean;
        p_right[i] -= p_right_mean;
        H += p_left[i] * p_right[i].transpose();
    }

    Eigen::JacobiSVD<matrix_3f_t> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
    matrix_3f_t U = svd.matrixU();
    matrix_3f_t S = svd.singularValues().asDiagonal();
    matrix_3f_t V = svd.matrixV();

    R_hat = V * U.transpose();

    // debug
    cout << "Matrix H:\n"
         << H << endl;
    cout << "\nU Matrix:\n"
         << U << endl;
    cout << "\nSingular Values (as a diagonal matrix):\n"
         << S << endl;
    cout << "\nV Matrix:\n"
         << V << endl;
    matrix_3f_t H_reconstructed = U * S * V.transpose();
    cout << "\nReconstructed Matrix (U * S * V^T):\n"
         << H_reconstructed << endl;

    return ret_t::MIC_RET_SUCCESSED;
}

MIC_NAMESPACE_END