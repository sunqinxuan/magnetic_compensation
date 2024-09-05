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

#ifndef MAGNAV_MAGCMP_TL_HPP_
#define MAGNAV_MAGCMP_TL_HPP_

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <unordered_set>
#include <vector>
#include <numeric>

#include "filter/butterworth.hpp"
#include "filter/filter.hpp"

namespace tl
{
  /** \brief \b TollesLawson provide the implementation of the Tolles-Lawson Algorithm.
   *
   * The estimation of the model coefficients including the total constraint-based method
   * (which is the traditional one) and the component constraint-based method (which is
   * originally designed by Q. Sun).
   *
   * Refer to https://sunqinxuan.github.io/projects/2024-05-06-mag-compensation for more details.
   *
   * \author Qinxuan Sun, Yansong Gong
   * \ingroup compensation
   */
  class TollesLawson
  {
  public:
    /** \ingroup enums
     * Enum for the FDM scheme used to calculate the derivatives in TL-model. */
    enum FDMscheme
    {
      /** 1st derivative 1st-order backward difference. */
      BACKWARD,
      /** 1st derivative 1st-order forward  difference. */
      FORWARD,
      /** 1st derivative 2nd-order central  difference. */
      CENTRAL,
      /** 1st derivative 2nd-order backward difference. */
      BACKWARD2,
      /** 1st derivative 2nd-order forward  difference. */
      FORWARD2,
      /** 4th derivative central difference. */
      FOURTH
    };

    /** \ingroup enums
     * Enum for the terms used for the TL model coefficients. */
    enum TLterm
    {
      /** The permanent magnetic moment term. */
      PERMANENT,
      /** The induced magnetic moment term. */
      INDUCED,
      /** The eddy current magnetic moment term. */
      EDDY,
      /** The FDM magnetic moment term. */
      FDM,
      /** The bias magnetic moment term. */
      BIAS
    };

    /** \brief Create Hash values for the key of unordered map  */
    struct HashTLterm
    {
      size_t operator()(const TLterm &p) const
      {
        return size_t(p);
      }
    };

    /** \brief Empty constructor. */
    TollesLawson() {}

    /** \brief Create Tolles-Lawson `A` matrix using vector magnetometer measurements.
     * \param[out] TL_A_ output Tolles-Lawson `A` matrix (N*18)
     * \param[in] Bx,By,Bz sequences of the triaxial compoenents of the magnetic measurements
     * \param[in] Bt (provided or empty) the magnetic intensities measured by optically pumped magnetometers
     * \param[in] terms (optional) the terms used in TL model coefficients (default: {PERMANENT,INDUCED, EDDY})
     */
    bool createMatrixA(std::vector<std::vector<double>> &TL_A_,
                       const std::vector<double> &Bx,
                       const std::vector<double> &By,
                       const std::vector<double> &Bz, std::vector<double> &Bt,
                       const std::unordered_set<TLterm, HashTLterm> &terms = {PERMANENT,
                                                                              INDUCED, EDDY});

    /** \brief Create component-based Tolles-Lawson `A` matrix using vector magnetometer measurements.
     * \param[out] TL_A_ output Tolles-Lawson `A` matrix (3N*18)
     * \param[in] Bx,By,Bz sequences of the triaxial compoenents of the magnetic measurements
     * \param[in] terms (optional) the terms used in TL model coefficients (default: {PERMANENT,INDUCED, EDDY})
     */
    bool createMatrixA_component(
        std::vector<std::vector<double>> &TL_A_, const std::vector<double> &Bx,
        const std::vector<double> &By, const std::vector<double> &Bz,
        const std::unordered_set<TLterm, HashTLterm> &terms = {PERMANENT, INDUCED, EDDY});

    /** \brief Create Tolles-Lawson coefficients using vector and scalar magnetometer
     * measurements with or without a bandpass filter.
     * \param[out] TL_beta_ output Tolles-Lawson coefficients
     * \param[in] Bx,By,Bz sequences of the triaxial compoenents of the magnetic measurements
     * \param[in] Be (provided or empty) the baseline earth magnetic field; the band-pass filter is applied if empty
     * \param[in] Bt (provided or empty) the magnetic intensities measured by optically pumped magnetometers
     * \param[in] terms (optional) the terms used in TL model coefficients (default: {PERMANENT,INDUCED, EDDY})
     * \param[in] lambda \lambda*Identity is added to ATA matrix in the linear regression algorithm
     * \param[in] pass1,pass2,fs,pole,trim parameters for the band-pass filter
     */
    bool createCoeff(
        std::vector<double> &TL_beta_, const std::vector<double> &Bx,
        const std::vector<double> &By, const std::vector<double> &Bz,
        const std::vector<double> &Be, std::vector<double> &Bt,
        const std::unordered_set<TLterm, HashTLterm> &terms = {PERMANENT, INDUCED, EDDY},
        const double lambda = 0.0, const double pass1 = 0.1, const double pass2 = 0.9,
        const double fs = 10.0, const int pole = 4, const int trim = 20);

    /** \brief Create component-based Tolles-Lawson coefficients using vector and scalar magnetometer measurements.
     * \param[out] TL_beta_ output Tolles-Lawson coefficients
     * \param[in] Bx,By,Bz sequences of the triaxial compoenents of the magnetic measurements
     * \param[in] Bex,Bey,Bez the baseline earth magnetic field, represented as triaxial components
     * \param[in] terms (optional) the terms used in TL model coefficients (default: {PERMANENT,INDUCED, EDDY})
     */
    bool createCoeff_component(
        std::vector<double> &TL_beta_, const std::vector<double> &Bx,
        const std::vector<double> &By, const std::vector<double> &Bz,
        const std::vector<double> &Bex, const std::vector<double> &Bey,
        const std::vector<double> &Bez,
        const std::unordered_set<TLterm, HashTLterm> &terms = {PERMANENT, INDUCED, EDDY});

  private:
    /** \brief Finite difference method (FDM) on vector of input data.
     * \param[out] dif output results
     * \param[in] x input data
     * \param[in] scheme enum paramter to assign a FDM scheme
     */
    bool fdm(std::vector<double> &dif, const std::vector<double> &x,
             const TollesLawson::FDMscheme scheme = CENTRAL);

    /** \brief Filter the input data using Butterworth bandpass filter.
     * \param[out] y output results
     * \param[in] x input data
     * \param[in] pass1 (optional) first passband frequency [Hz]
     * \param[in] pass2 (optional) second passband frequency [Hz]
     * \param[in] pole (optional) number of poles for Butterworth filter
     * \param[in] trim (optional) number of elements to trim after filtering
     */
    bool bwbp_filter(std::vector<double> &y, const std::vector<double> &x,
                     const double pass1, const double pass2, const int pole,
                     const int trim);

    /** \brief Linear regression to solve the least-square problem `Ax=b`.
     * \param[out] coeff output model coefficients
     * \param[out] residual output fitting residuals
     * \param[in] bb length-`N` observed data vector
     * \param[in] AA `N*Nf` input data matrix (`Nf` is number of features)
     * \param[in] lambda ridge parameter
     */
    void linear_regression(std::vector<double> &coeff,
                           std::vector<double> &residual,
                           const std::vector<double> &bb,
                           const std::vector<std::vector<double>> &AA,
                           const double lambda);

    /** \brief Combine the components into vectors and concatenate them.
     * \param[out] result output concatenated vector
     * \param[in] x,y,z sequences of triaxial components
     */
    void stack_vector(std::vector<double> &result, const std::vector<double> &x,
                      const std::vector<double> &y, const std::vector<double> &z);

    /** \brief Compute the root mean square error given a residual sequence.
     * \param[in] residual input residuals 
     */
    double rmse(const std::vector<double> &residual);

    /** \brief output degeneration problem to terminal for debugging.
     * \param[in] TL_A input TL `A` matrix 
     */
    void degeneration(const std::vector<std::vector<double>> &TL_A);

  private:
    // A -  Tolles-Lawson `A` matrix
    // std::vector<std::vector<double>> TL_A_;
    // std::vector<std::vector<double>> TL_A_filt_;

    // std::vector<double> TL_beta_;

    // [Bx, By, Bz] - vector magnetometer measurements [nT]
    // std::vector<double> &Bx;
    // std::vector<double> &By;
    // std::vector<double> &Bz;

    // B - scalar magnetometer measurements [nT]
    // std::vector<double> &B;

    // Bt - magnitude of vector magnetometer measurements or scalar magnetometer
    // measurements for modified Tolles-Lawson [nT]
    //	std::vector<double> &Bt;

    // B_dot - finite differences of total field vector [nT]
    // std::vector<double> Bx_dot;
    // std::vector<double> By_dot;
    // std::vector<double> Bz_dot;
  };
} // namespace magnav

#endif
