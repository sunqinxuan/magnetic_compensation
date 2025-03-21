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

        Eigen::Matrix<T, 3, 1> mag_r1 = D_tilde_inv.inverse() * R * mag_c + o_hat;

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

/** \brief MicCabinMagCompensator is a magnetic compensation algorithm
 * specifically designed for applying the magnetic measured at the tail stinger
 * as the baseline during calibration.
 *
 * The corresponding compensation algorithm is originally proposed by Qinxuan Sun
 * in https://sunqinxuan.github.io/projects/2024-07-09-compensation.
 *
 * \author Qinxuan Sun, Yansong Gong
 * \ingroup compensation
 */
class MicCabinMagCompensator : public MicMagCompensator
{
public:
    /** \brief Empty constructor. */
    MicCabinMagCompensator();

    /** \brief destructor. */
    virtual ~MicCabinMagCompensator() = default;

protected:
    /** \brief Implementation of the calibration algorithm of the compensation model
     * based on the ellipsoid fitting and the orientaion estimation. */
    virtual ret_t do_calibrate() override;

    /** \brief Compensate using the ellipsoid-based model
     * (only if the working state is set to \a MIC_MAG_COMPENSATE_CALIBRATED).
     * \param[in] ts the timestamp at which the compensation result is required
     * \param[out] out output the compensated result at time \a ts
     */
    virtual ret_t do_compenste(const float64_t ts, mic_mag_t &out) override;

    virtual ret_t serialize(json_t &node) override;
    virtual ret_t deserialize(json_t &node) override;

    /** \brief Optimize the model parameters R=V*R^{mm'} using Ceres library.
     * \param[in] mag magnetic measurements obtained in cabin
     * \param[in] mag_1 magnetic measurements obtained on tail stinger
     * \param[in] D_tilde_inv, o_hat model coefficients
     * \param[out] quat the output quaternion corresponding to the orthogonal matrix R=V*R^{mb}
     */
    ret_t ceres_optimize(
        const std::vector<vector_3f_t> &mag,
        const std::vector<vector_3f_t> &mag_1,
        const matrix_3f_t &D_tilde_inv,
        const vector_3f_t &o_hat,
        quaternionf_t &quat);

protected:
    /** \brief The pointer to the \a MicEllipsoidMagCompensator object. */
    mic_ellipsoid_mag_compensator_shared_ptr _mag_ellipsoid_ptr;

    /** \brief The compensation model coefficients. */
    matrix_3f_t _D_tilde_inv;

    /** \brief The compensation model coefficients. */
    vector_3f_t _o_hat;

    /** \brief The compensation model coefficients. */
    matrix_3f_t _R_opt;
};

MIC_NAMESPACE_END

#endif