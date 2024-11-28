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
#include "mic_mag_compensator/impl/mic_cabin_nav_mag_compensator.h"

// debug
#include <iostream>
#include <fstream>
using namespace std;

MIC_NAMESPACE_START

MicCabinNavMagCompensator::MicCabinNavMagCompensator() : MicCabinMagCompensator()
{
    std::vector<float64_t> R_k = MIC_CONFIG_GET(std::vector<float64_t>, "KF_state_noise");
    vector_xf_t R_k_eigen = vector_xf_t::Map(R_k.data(), R_k.size());
    _state_noise_cov = matrix_xf_t::Identity(12, 12);
    _state_noise_cov.diagonal() = R_k_eigen;
    cout << "_state_noise_cov: " << endl
         << _state_noise_cov << endl;

    std::vector<float64_t> Q_k = MIC_CONFIG_GET(std::vector<float64_t>, "KF_measure_noise");
    vector_xf_t Q_k_eigen = vector_xf_t::Map(Q_k.data(), Q_k.size());
    _measure_noise_cov = matrix_xf_t::Identity(3, 3);
    _measure_noise_cov.diagonal() = Q_k_eigen;
    cout << "_measure_noise_cov: " << endl
         << _measure_noise_cov << endl;
}

ret_t MicCabinNavMagCompensator::deserialize(json_t &node)
{
    // std::vector<double> D = node["ellipsoid_model"]["coeff_D_inv"];
    // std::vector<double> R = node["ellipsoid_model"]["coeff_R"];
    // std::vector<double> o = node["ellipsoid_model"]["coeff_o"];
    // if (D.size() != 9 || R.size() != 9 || o.size() != 3)
    //     return ret_t::MIC_RET_FAILED;
    // for (int i = 0; i < 3; i++)
    // {
    //     _o_hat(i) = o[i];
    //     for (int j = 0; j < 3; j++)
    //     {
    //         _D_tilde_inv(i, j) = D[i * 3 + j];
    //         _R_opt(i, j) = R[i * 3 + j];
    //     }
    // }
    // return MicMagCompensator::deserialize(node);

    ret_t ret = MicCabinMagCompensator::deserialize(node);

    matrix_3f_t C = _D_tilde_inv.inverse() * _R_opt;
    cout << "C: " << endl
         << C << endl;
    cout << "o: " << _o_hat.transpose() << endl;
    vector_xf_t C_cols = Eigen::Map<vector_xf_t>(C.data(), C.size());
    _theta = vector_xf_t::Zero(12);
    _theta << _o_hat, C_cols;
    cout << "theta: " << _theta.transpose() << endl;

    std::vector<float64_t> init_cov = MIC_CONFIG_GET(std::vector<float64_t>, "KF_init_cov");
    // Eigen::Map<vector_xf_t> init_cov_eigen(init_cov.data(), init_cov.size());
    vector_xf_t init_cov_eigen = vector_xf_t::Map(init_cov.data(), init_cov.size());
    // cout<<"init_cov_eigen: "<<init_cov_eigen.transpose()<<endl;
    _theta_cov = matrix_xf_t::Identity(12, 12);
    _theta_cov.diagonal() = init_cov_eigen;
    cout << "_theta_cov: " << endl
         << _theta_cov << endl;

    return ret;
}

ret_t MicCabinNavMagCompensator::do_compenste(const float64_t ts, mic_mag_t &out)
{
    mic_mag_t in;
    if (_mag_measure_storer.get_data<mic_mag_t>(ts, in))
    {
        KF_update(ts);
        // cout<<"KF covariance:\t"<<_theta_cov.diagonal().transpose()<<endl;

        matrix_3f_t matrix;
        matrix << _theta(3), _theta(6), _theta(9),
            _theta(4), _theta(7), _theta(10),
            _theta(5), _theta(8), _theta(11);
        vector_3f_t offset = _theta.head(3);

        // cout << endl
        //      << "matrix: " << endl
        //      << matrix << endl;
        // cout << "offset: " << offset.transpose() << endl
        //      << endl;

        out.vector = matrix.inverse() * (in.vector - offset);
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

ret_t MicCabinNavMagCompensator::KF_update(const float64_t ts)
{
    mic_mag_t mag, mag_truth;
    if (_mag_measure_storer.get_data<mic_mag_t>(ts, mag) && _mag_truth_storer.get_data<mic_mag_t>(ts, mag_truth))
    {
        vector_3f_t m_k = mag_truth.value * mag_truth.vector.normalized();
        vector_3f_t y_k = mag.value * mag.vector.normalized();

        matrix_xf_t H_m_k = matrix_xf_t::Identity(3, 12);
        H_m_k.block<3, 3>(0, 3) = matrix_3f_t::Identity() * m_k(0);
        H_m_k.block<3, 3>(0, 6) = matrix_3f_t::Identity() * m_k(1);
        H_m_k.block<3, 3>(0, 9) = matrix_3f_t::Identity() * m_k(2);
        // cout<<"H_m_k = "<<endl<<H_m_k<<endl;

        /* theta_k_bar=theta_{k-1} */
        vector_xf_t theta_k_bar=_theta;
        /* Sigma_k_bar=Sigma_{k-1}+R_k */
        matrix_xf_t theta_cov_k_bar=_theta_cov+_state_noise_cov;

        /* K_k=Sigma_k_bar*h(m)^T*[h(m)*Sigma_k_bar*h(m)^T+Q_k]^{-1} */
        matrix_xf_t tmp=H_m_k*theta_cov_k_bar*H_m_k.transpose()+_measure_noise_cov;
        matrix_xf_t kalman_gain=theta_cov_k_bar*H_m_k.transpose()*tmp.inverse();

        /* theta_k=theta_k_bar+K_k[y_k-h(m)*theta_k_bar] */
        vector_xf_t theta_k=theta_k_bar+kalman_gain*(y_k-H_m_k*theta_k_bar);

        /* Sigma_k=[I-K_k*h(m)]*Sigma_k_bar */
        matrix_xf_t theta_cov_k=(matrix_xf_t::Identity(12,12)-kalman_gain*H_m_k)*theta_cov_k_bar;

        // update model;
        _theta=theta_k;
        _theta_cov=theta_cov_k;

        return ret_t::MIC_RET_SUCCESSED;
    }
    else
    {
        return ret_t::MIC_RET_FAILED;
    }
}

MIC_NAMESPACE_END