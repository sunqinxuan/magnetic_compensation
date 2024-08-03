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
#include <unordered_set>
#include <vector>
#include <numeric> 

#include "filter/butterworth.hpp"
#include "filter/filter.hpp"

namespace tl {
class TollesLawson {
public:
  enum FDMscheme {
    BACKWARD,  // 1st derivative 1st-order backward difference
    FORWARD,   //  1st derivative 1st-order forward  difference
    CENTRAL,   //  1st derivative 2nd-order central  difference
    BACKWARD2, // 1st derivative 2nd-order backward difference
    FORWARD2,  // 1st derivative 2nd-order forward  difference
    FOURTH     //   4th derivative central difference
  };

  enum TLterm { PERMANENT, INDUCED, EDDY, FDM, BIAS };

  TollesLawson() {}

  /*
   * createMatrixA()
   * Create Tolles-Lawson `A` matrix using vector magnetometer measurements.
   * Optionally returns the magnitude and derivatives of total field.
   */
  bool createMatrixA(std::vector<std::vector<double>> &TL_A_,
                     const std::vector<double> &Bx,
                     const std::vector<double> &By,
                     const std::vector<double> &Bz, std::vector<double> &Bt,
                     const std::unordered_set<TLterm> &terms = {PERMANENT,
                                                                INDUCED, EDDY},
                     const double Bt_scale = 50000.0);
  bool createMatrixA_Vector(
      std::vector<std::vector<double>> &TL_A_, const std::vector<double> &Bx,
      const std::vector<double> &By, const std::vector<double> &Bz,
      const std::unordered_set<TLterm> &terms = {PERMANENT, INDUCED, EDDY});

  /*
   * createCoeff()
   * Create Tolles-Lawson coefficients using vector and scalar magnetometer
   * measurements and a bandpass, low-pass or high-pass filter.
   */
  double createCoeff(
      std::vector<double> &TL_beta_, const std::vector<double> &Bx,
      const std::vector<double> &By, const std::vector<double> &Bz,
      const std::vector<double> &B, const std::vector<double> &Be,
      std::vector<double> &Bt,
      const std::unordered_set<TLterm> &terms = {PERMANENT, INDUCED, EDDY},
      const double lambda = 0.0, const double pass1 = 0.1,
      const double pass2 = 0.9, const double fs = 10.0, const int pole = 4,
      const int trim = 20, const double Bt_scale = 50000.0);
  double createCoeff_Vector(
      std::vector<double> &TL_beta_, const std::vector<double> &Bx,
      const std::vector<double> &By, const std::vector<double> &Bz,
      const std::vector<double> &Bex, const std::vector<double> &Bey,
      const std::vector<double> &Bez,
      const std::unordered_set<TLterm> &terms = {PERMANENT, INDUCED, EDDY});

private:
  /*
   * fdm()
   * Finite difference method (FDM) on vector of input data.
   */
  bool fdm(std::vector<double> &dif, const std::vector<double> &x,
           const TollesLawson::FDMscheme scheme = CENTRAL);

  bool bwbp_filter(std::vector<double> &y, const std::vector<double> &x,
                   const double pass1, const double pass2, const int pole,
                   const int trim);

  void linear_regression(std::vector<double> &coeff,
                         std::vector<double> &residual,
                         const std::vector<double> &bb,
                         const std::vector<std::vector<double>> &AA,
                         const double lambda);

  void stack_vector(std::vector<double> &result, const std::vector<double> &x,
                    const std::vector<double> &y, const std::vector<double> &z);

  // void getFilterCoeffAB(int n, double lowcut, double highcut, int fs,
  //                      std::vector<double> &acof_vec,
  //                      std::vector<double> &bcof_vec);

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
