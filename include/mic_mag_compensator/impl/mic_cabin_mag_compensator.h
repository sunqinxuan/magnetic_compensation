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
#ifndef MIC_CABIN_MAG_COMPENSATOR
#define MIC_CABIN_MAG_COMPENSATOR

#include <Eigen/Eigenvalues>
#include <Eigen/Dense>
#include <ceres/ceres.h>
#include "mic_mag_compensator/mic_mag_compensator.h"
#include "mic_mag_compensator/impl/mic_ellipsoid_mag_compensator.h"

MIC_NAMESPACE_START

class MicCabinMagCompensator;
using mic_cabin_mag_compensator_t = MicCabinMagCompensator;

class CabinCostFunctionCreator
{
public:
    CabinCostFunctionCreator(const matrix_3f_t &coeff_D_tilde_inv,
                             const vector_3f_t &coeff_o_hat,
                             const vector_3f_t &mag_r,
                             const vector_3f_t &mag_c)
        : coeff_D_tilde_inv_(coeff_D_tilde_inv), coeff_o_hat_(coeff_o_hat),
          mag_r_(mag_r), mag_c_(mag_c) {}

    template <typename T>
    bool operator()(const T *const quaternion, T *residual_ptr) const
    {
        Eigen::Map<const Eigen::Quaternion<T>> q(quaternion);
        Eigen::Matrix<T, 3, 3> R = q.toRotationMatrix();

        Eigen::Matrix<T, 3, 3> D_tilde_inv = coeff_D_tilde_inv_.template cast<T>();
        Eigen::Matrix<T, 3, 1> o_hat = coeff_o_hat_.template cast<T>();
        Eigen::Matrix<T, 3, 1> mag_r = mag_r_.template cast<T>();
        Eigen::Matrix<T, 3, 1> mag_c = mag_c_.template cast<T>();

        Eigen::Matrix<T, 3, 1> mag_r1 = D_tilde_inv * R * mag_c + o_hat;

        Eigen::Map<Eigen::Matrix<T, 3, 1>> residual(residual_ptr);
        residual = mag_r - mag_r1;

        return true;
    }

    static ceres::CostFunction *Create(const matrix_3f_t &coeff_D_tilde_inv,
                                       const vector_3f_t &coeff_o_hat,
                                       const vector_3f_t &mag_r,
                                       const vector_3f_t &mag_c)
    {
        return new ceres::AutoDiffCostFunction<CabinCostFunctionCreator, 3, 4>(
            new CabinCostFunctionCreator(coeff_D_tilde_inv, coeff_o_hat,
                                         mag_r, mag_c));
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
    matrix_3f_t coeff_D_tilde_inv_;
    vector_3f_t coeff_o_hat_;
    vector_3f_t mag_r_, mag_c_;
};

class MicCabinMagCompensator : public MicMagCompensator
{
public:
    MicCabinMagCompensator(); // : MicMagCompensator() {}
    virtual ~MicCabinMagCompensator() = default;

protected:
    virtual ret_t do_calibrate() override;
    virtual ret_t do_compenste(const mic_mag_flux_t &in, mic_mag_flux_t &out) override;

    virtual ret_t serialize(json_t &node) override;
    virtual ret_t deserialize(json_t &node) override;

    /* Ellipsoid fitting algorithm
     *
     * Inputs:
     *   x = nx1 column vector of x coordinates of input data.
     *   y = nx1 column vector of y coordinates of input data.
     *   z = nx1 column vector of y coordinates of input data.
     *
     * Output:
     *   u = [a,b,c,f,g,h,p,q,r,d], a vector corresponding to coefficients of
     *       the general quadric surface given by equation,
     *       ax2 + by2 + cz2 + 2fyz + 2gxz + 2hxy + 2px + 2qy + 2rz + d = 0.
     *
     * Source:
     *   [1] Li - Least Square Ellipsoid Fitting (2004)
     * */
    // ret_t ellipsoid_fit(
    //     const std::vector<vector_3f_t> &mag,
    //     std::vector<float64_t> &coeffs);

    // computation of the compensation model coefficients;
    // ret_t compute_model_coeffs(
    //     const std::vector<float64_t> &ellipsoid_coeffs,
    //     const float64_t mag_earth_intensity,
    //     matrix_3f_t &D_tilde_inv,
    //     vector_3f_t &o_hat);

    /* ceres optimization of orthogonal matrix R=V*R^{mb};
     * mag - magnetic measurements (from flux);
     * R_nb - rotations from body frame to navigation frame (from ins);
     * D_tilde_inv, o_hat - model coefficients;
     * quat - the orthogonal matrix to be optimized R=V*R^{mb};
     */
    ret_t ceres_optimize(
        const std::vector<vector_3f_t> &mag,
        const std::vector<vector_3f_t> &mag_1,
        const matrix_3f_t &D_tilde_inv,
        const vector_3f_t &o_hat,
        quaternionf_t &quat);

    // // estimate initial value of orthogonal matrix R=V*R^{mb};
    // ret_t init_value_estimate(
    //     const std::vector<vector_3f_t> &mag_m,
    //     const std::vector<vector_3f_t> &mag_n,
    //     const matrix_3f_t &D_tilde_inv,
    //     const vector_3f_t &o_hat,
    //     matrix_3f_t &R_hat);

protected:
    mic_ellipsoid_mag_compensator_shared_ptr _mag_ellipsoid_ptr;

    matrix_3f_t _D_tilde_inv;
    vector_3f_t _o_hat;
    matrix_3f_t _R_opt;
};

MIC_NAMESPACE_END

#endif