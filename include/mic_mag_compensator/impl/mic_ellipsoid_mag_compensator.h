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
#ifndef MIC_ELLIPSOID_MAG_COMPENSATOR
#define MIC_ELLIPSOID_MAG_COMPENSATOR

#include <Eigen/Eigenvalues>
#include <Eigen/Dense>
#include <ceres/ceres.h>
#include <iomanip>
#include "mic_mag_compensator/mic_mag_compensator.h"

MIC_NAMESPACE_START

class MicEllipsoidMagCompensator;
using mic_ellipsoid_mag_compensator_t = MicEllipsoidMagCompensator;
using mic_ellipsoid_mag_compensator_shared_ptr = std::shared_ptr<MicEllipsoidMagCompensator>;

class CostFunctionCreator
{
public:
    CostFunctionCreator(const matrix_3f_t &coeff_D_tilde_inv,
                        const vector_3f_t &coeff_o_hat,
                        const matrix_3f_t &rotation_r_c,
                        const vector_3f_t &mag_r,
                        const vector_3f_t &mag_c)
        : coeff_D_tilde_inv_(coeff_D_tilde_inv), coeff_o_hat_(coeff_o_hat),
          rotation_r_c_(rotation_r_c), mag_r_(mag_r), mag_c_(mag_c) {}

    template <typename T>
    bool operator()(const T *const quaternion, T *residual_ptr) const
    {
        Eigen::Map<const Eigen::Quaternion<T>> q(quaternion);
        Eigen::Matrix<T, 3, 3> R = q.toRotationMatrix();

        Eigen::Matrix<T, 3, 3> D_tilde_inv = coeff_D_tilde_inv_.template cast<T>();
        Eigen::Matrix<T, 3, 1> o_hat = coeff_o_hat_.template cast<T>();
        Eigen::Matrix<T, 3, 3> R_rc = rotation_r_c_.template cast<T>();
        Eigen::Matrix<T, 3, 1> mag_r = mag_r_.template cast<T>();
        Eigen::Matrix<T, 3, 1> mag_c = mag_c_.template cast<T>();

        Eigen::Matrix<T, 3, 1> mag_r1 = D_tilde_inv.inverse() * R * R_rc * R.transpose() * D_tilde_inv * (mag_c - o_hat) + o_hat;

        Eigen::Map<Eigen::Matrix<T, 3, 1>> residual(residual_ptr);
        residual = mag_r - mag_r1;

        return true;
    }

    static ceres::CostFunction *Create(const matrix_3f_t &coeff_D_tilde_inv,
                                       const vector_3f_t &coeff_o_hat,
                                       const matrix_3f_t &rotation_r_c,
                                       const vector_3f_t &mag_r,
                                       const vector_3f_t &mag_c)
    {
        return new ceres::AutoDiffCostFunction<CostFunctionCreator, 3, 4>(
            new CostFunctionCreator(coeff_D_tilde_inv, coeff_o_hat, rotation_r_c,
                                    mag_r, mag_c));
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
    matrix_3f_t coeff_D_tilde_inv_;
    vector_3f_t coeff_o_hat_;
    matrix_3f_t rotation_r_c_; // rotation from b_k (c) to b_k+1 (r);
    vector_3f_t mag_r_, mag_c_;
};

/** \brief MicEllipsoidMagCompensator is a magnetic compensation algorithm based on
 * the ellipsoid fitting and orientation estimation.
 *
 * The corresponding compensation algorithm is originally proposed by Qinxuan Sun
 * in https://sunqinxuan.github.io/projects/2024-07-09-compensation.
 *
 * \code
 * mic_mag_compensator_shared_ptr compensator = std::make_shared<mic_ellipsoid_mag_compensator_t>();
 * compensator->load_model("model_ellipsoid.mdl");
 * compensator->add_data(ts, mag);
 * compensator->compenste(ts, out);
 * std::cout << out.vector << std::endl;
 * \endcode
 *
 * \author Qinxuan Sun, Yansong Gong
 * \ingroup compensation
 */
class MicEllipsoidMagCompensator : public MicMagCompensator
{
public:
    /** \brief Empty constructor. */
    MicEllipsoidMagCompensator() : MicMagCompensator() { flag = false; }

    /** \brief destructor. */
    virtual ~MicEllipsoidMagCompensator() = default;

    /** \brief Call the ellipsoid fitting algirithm to compute the compensation model
     * coefficients \a D_tilde_inv and \a o_hat using the input magnetic data.
     * \param[in] mag sequence of the measured magnetic vectors
     * \param[in] mag_earth_intensity the earth magnetic intensity in the calibration region
     * \param[out] D_tilde_inv output model coefficients
     * \param[out] o_hat output model coefficients
     */
    ret_t fit_ellipsoid_model(
        const std::vector<vector_3f_t> &mag,
        const float64_t mag_earth_intensity,
        matrix_3f_t &D_tilde_inv,
        vector_3f_t &o_hat);

    /** \brief Call the least-sqares fitting algirithm to compute the relative rotation
     * between two point sets.
     * \param[in] p_left the point set in reference frame
     * \param[in] p_right the point set to be transformed
     * \param[out] R_hat the resultant rotation matrix
     */
    ret_t fit_ls_rotation(
        const std::vector<vector_3f_t> &p_left,
        const std::vector<vector_3f_t> &p_right,
        matrix_3f_t &R_hat);

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

    /** \brief Fit an ellipsoid using the measured magnetic data.
     * The algirithm is presented in https://ieeexplore.ieee.org/document/1290055.
     * \param[in] mag sequence of the magnetic measurements
     * \param[out] coeffs a vector u = [a,b,c,f,g,h,p,q,r,d] corresponding to
     * coefficients of the general quadric surface given by equation,
     * ax2 + by2 + cz2 + 2fyz + 2gxz + 2hxy + 2px + 2qy + 2rz + d = 0.
     */
    ret_t ellipsoid_fit(
        const std::vector<vector_3f_t> &mag,
        std::vector<float64_t> &coeffs);

    /** \brief Compute the model parameters using the fitted ellipsoid coefficients.
     * \param[in] ellipsoid_coeffs sequence of the measured magnetic vectors
     * \param[in] mag_earth_intensity the earth magnetic intensity in the calibration region
     * \param[out] D_tilde_inv output model coefficients
     * \param[out] o_hat output model coefficients
     */
    ret_t compute_model_coeffs(
        const std::vector<float64_t> &ellipsoid_coeffs,
        const float64_t mag_earth_intensity,
        matrix_3f_t &D_tilde_inv,
        vector_3f_t &o_hat);

    /** \brief Optimize the model parameters R=V*R^{mb} using Ceres library.
     * \param[in] mag magnetic measurements
     * \param[in] R_nb rotations from body frame to navigation frame (from ins)
     * \param[in] D_tilde_inv, o_hat model coefficients
     * \param[out] quat the output quaternion corresponding to the orthogonal matrix R=V*R^{mb}
     */
    ret_t ceres_optimize(
        const std::vector<vector_3f_t> &mag,
        const std::vector<matrix_3f_t> &R_nb,
        const matrix_3f_t &D_tilde_inv,
        const vector_3f_t &o_hat,
        quaternionf_t &quat);

    /** \brief Implement the least-sqares fitting algirithm to compute the relative rotation
     * between two point sets, as described in https://ieeexplore.ieee.org/document/4767965.
     * \param[in] p_left the point set in reference frame
     * \param[in] p_right the point set to be transformed
     * \param[out] R_hat the resultant rotation matrix
     */
    ret_t ls_fitting(
        const std::vector<vector_3f_t> &p_left,
        const std::vector<vector_3f_t> &p_right,
        matrix_3f_t &R_hat);

    /** \brief Implement the SVD-based solution of the Wahba's problem.
     * For more details, please refer to :
     * Markley, F. Landis.
     * "Attitude determination using vector observations and the singular value decomposition."
     * Journal of the Astronautical Sciences36.3 (1988): 245-258.
     * \param[in] y the point set in reference frame
     * \param[in] x the point set to be transformed
     * \param[out] R_hat the resultant rotation matrix
     */
    ret_t wahba_svd(
        const std::vector<vector_3f_t> &y,
        const std::vector<vector_3f_t> &x,
        matrix_3f_t &R);

protected:
    /** \brief The compensation model coefficients. */
    matrix_3f_t _D_tilde_inv;

    /** \brief The compensation model coefficients. */
    vector_3f_t _o_hat;

    /** \brief The compensation model coefficients. */
    matrix_3f_t _R_opt;
};

MIC_NAMESPACE_END

#endif