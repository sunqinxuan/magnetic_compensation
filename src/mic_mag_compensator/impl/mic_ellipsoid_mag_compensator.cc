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
using namespace std;

MIC_NAMESPACE_START

ret_t MicEllipsoidMagCompensator::calibrate()
{
    ret_t ret = ret_t::MIC_RET_FAILED;

    auto data_range = _data_storer.get_data_range<mic_mag_flux_t>(0.0, _curr_time_stamp + 1);
    auto it_start = data_range.first;
    auto it_end = data_range.second;

    // TODO: mag_x.resize(_data_storer.size())
    std::vector<float64_t> mag_x, mag_y, mag_z, ellipsoid_coeffs;
    for (auto it = it_start; it != it_end; ++it)
    {
        mic_mag_flux_t mag_flux = it->second;
        mag_x.push_back(mag_flux.vector(0));
        mag_y.push_back(mag_flux.vector(1));
        mag_z.push_back(mag_flux.vector(2));
    }
    ellipsoid_fitting(mag_x, mag_y, mag_z, ellipsoid_coeffs);

    // debug info
    MIC_LOG_DEBUG_INFO("ellipsoid coefficients:");
    for (size_t i = 0; i < ellipsoid_coeffs.size(); ++i)
    {
        MIC_LOG_DEBUG_INFO("%f", ellipsoid_coeffs[i]);
    }

    double mag_earth_intensity = 54093.9956380105; // nT
    matrix_3f_t D_tilde_inv;
    vector_3f_t o_hat;
    compute_model_coeffs(ellipsoid_coeffs, mag_earth_intensity, D_tilde_inv, o_hat);

    notify(*this);
    return ret;
}

ret_t MicEllipsoidMagCompensator::compenste()
{
    // for observer updating
    notify(*this);
    return ret_t::MIC_RET_FAILED;
}

ret_t MicEllipsoidMagCompensator::ellipsoid_fitting(
    const std::vector<float64_t> &x,
    const std::vector<float64_t> &y,
    const std::vector<float64_t> &z,
    std::vector<float64_t> &coeffs)
{

    if (x.size() != y.size() || x.size() != z.size())
        return ret_t::MIC_RET_FAILED;

    const int N = x.size();

    Eigen::MatrixXd design_matrix(10, N);
    for (int i = 0; i < N; i++)
    {
        design_matrix(0, i) = x[i] * x[i];
        design_matrix(1, i) = y[i] * y[i];
        design_matrix(2, i) = z[i] * z[i];
        design_matrix(3, i) = 2 * y[i] * z[i];
        design_matrix(4, i) = 2 * x[i] * z[i];
        design_matrix(5, i) = 2 * x[i] * y[i];
        design_matrix(6, i) = 2 * x[i];
        design_matrix(7, i) = 2 * y[i];
        design_matrix(8, i) = 2 * z[i];
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

    Eigen::Matrix3d As_hat;
    As_hat << a, h, g, h, b, f, g, f, c;
    Eigen::Vector3d bs_hat;
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

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(As_hat);
    Eigen::Matrix3d As_evec = es.eigenvectors();
    Eigen::Vector3d As_eval = es.eigenvalues(); // increasing order;

    Eigen::Matrix3d sqrt_As_eval;
    sqrt_As_eval << sqrt(fabs(As_eval[0])), 0, 0, 0, sqrt(fabs(As_eval[1])), 0, 0,
        0, sqrt(fabs(As_eval[2]));

    D_tilde_inv = sqrt(fabs(alpha)) * sqrt_As_eval * As_evec.transpose();
    // std::cout << "D_tilde_inv = " << std::endl
    //           << D_tilde_inv << std::endl;
    return ret_t::MIC_RET_SUCCESSED;
}

MIC_NAMESPACE_END