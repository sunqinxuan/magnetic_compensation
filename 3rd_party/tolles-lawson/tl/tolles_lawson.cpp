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

#include "tl/tolles_lawson.hpp"

namespace tl
{

  bool TollesLawson::createMatrixA_component(
      std::vector<std::vector<double>> &TL_A_, const std::vector<double> &Bx,
      const std::vector<double> &By, const std::vector<double> &Bz,
      const std::unordered_set<TLterm, HashTLterm> &terms)
  {
    if (!(Bx.size() == By.size() && Bx.size() == Bz.size()))
    {
      return false;
    }
    if (terms.empty())
    {
      return false;
    }

    int N = Bx.size();

    std::vector<double> Bx_dot, By_dot, Bz_dot;
    fdm(Bx_dot, Bx);
    fdm(By_dot, By);
    fdm(Bz_dot, Bz);

    TL_A_.clear();

    if (terms.find(PERMANENT) != terms.end())
    {
      TL_A_.resize(3);
      TL_A_[0].clear();
      TL_A_[1].clear();
      TL_A_[2].clear();
      for (int i = 0; i < N; i++)
      {
        TL_A_[0].push_back(1);
        TL_A_[0].push_back(0);
        TL_A_[0].push_back(0);

        TL_A_[1].push_back(0);
        TL_A_[1].push_back(1);
        TL_A_[1].push_back(0);

        TL_A_[2].push_back(0);
        TL_A_[2].push_back(0);
        TL_A_[2].push_back(1);
      }
    }

    if (terms.find(INDUCED) != terms.end())
    {
      int idx = TL_A_.size();
      TL_A_.resize(idx + 6);
      TL_A_[idx].clear();
      TL_A_[idx + 1].clear();
      TL_A_[idx + 2].clear();
      TL_A_[idx + 3].clear();
      TL_A_[idx + 4].clear();
      TL_A_[idx + 5].clear();
      for (int i = 0; i < N; i++)
      {
        TL_A_[idx].push_back(Bx[i]);
        TL_A_[idx].push_back(0);
        TL_A_[idx].push_back(0);

        TL_A_[idx + 1].push_back(By[i]);
        TL_A_[idx + 1].push_back(Bx[i]);
        TL_A_[idx + 1].push_back(0);

        TL_A_[idx + 2].push_back(Bz[i]);
        TL_A_[idx + 2].push_back(0);
        TL_A_[idx + 2].push_back(Bx[i]);

        TL_A_[idx + 3].push_back(0);
        TL_A_[idx + 3].push_back(By[i]);
        TL_A_[idx + 3].push_back(0);

        TL_A_[idx + 4].push_back(0);
        TL_A_[idx + 4].push_back(Bz[i]);
        TL_A_[idx + 4].push_back(By[i]);

        TL_A_[idx + 5].push_back(0);
        TL_A_[idx + 5].push_back(0);
        TL_A_[idx + 5].push_back(Bz[i]);
      }
    }

    if (terms.find(EDDY) != terms.end())
    {
      int idx = TL_A_.size();
      TL_A_.resize(idx + 9);
      TL_A_[idx].clear();
      TL_A_[idx + 1].clear();
      TL_A_[idx + 2].clear();
      TL_A_[idx + 3].clear();
      TL_A_[idx + 4].clear();
      TL_A_[idx + 5].clear();
      TL_A_[idx + 6].clear();
      TL_A_[idx + 7].clear();
      TL_A_[idx + 8].clear();
      for (int i = 0; i < N; i++)
      {
        TL_A_[idx].push_back(Bx_dot[i]);
        TL_A_[idx].push_back(0);
        TL_A_[idx].push_back(0);

        TL_A_[idx + 1].push_back(By_dot[i]);
        TL_A_[idx + 1].push_back(0);
        TL_A_[idx + 1].push_back(0);

        TL_A_[idx + 2].push_back(Bz_dot[i]);
        TL_A_[idx + 2].push_back(0);
        TL_A_[idx + 2].push_back(0);

        TL_A_[idx + 3].push_back(0);
        TL_A_[idx + 3].push_back(Bx_dot[i]);
        TL_A_[idx + 3].push_back(0);

        TL_A_[idx + 4].push_back(0);
        TL_A_[idx + 4].push_back(By_dot[i]);
        TL_A_[idx + 4].push_back(0);

        TL_A_[idx + 5].push_back(0);
        TL_A_[idx + 5].push_back(Bz_dot[i]);
        TL_A_[idx + 5].push_back(0);

        TL_A_[idx + 6].push_back(0);
        TL_A_[idx + 6].push_back(0);
        TL_A_[idx + 6].push_back(Bx_dot[i]);

        TL_A_[idx + 7].push_back(0);
        TL_A_[idx + 7].push_back(0);
        TL_A_[idx + 7].push_back(By_dot[i]);

        TL_A_[idx + 8].push_back(0);
        TL_A_[idx + 8].push_back(0);
        TL_A_[idx + 8].push_back(Bz_dot[i]);
      }
    }

    return true;
  }

  bool TollesLawson::createCoeff_component(
      std::vector<double> &TL_beta_, const std::vector<double> &Bx,
      const std::vector<double> &By, const std::vector<double> &Bz,
      const std::vector<double> &Bex, const std::vector<double> &Bey,
      const std::vector<double> &Bez, const std::unordered_set<TLterm, HashTLterm> &terms)
  {
    if (!(Bx.size() == By.size() && Bx.size() == Bz.size()))
    {
      return false;
    }
    if (terms.empty())
    {
      return false;
    }

    std::vector<std::vector<double>> TL_A_;
    if (!createMatrixA_component(TL_A_, Bx, By, Bz, terms))
    {
      return false;
    }

    std::vector<double> Bt, Be;
    stack_vector(Bt, Bx, By, Bz);
    stack_vector(Be, Bex, Bey, Bez);

    std::vector<std::vector<double>> TL_A_filt_;
    std::vector<double> B_filt;

    for (int i = 0; i < Bt.size(); i++)
    {
      B_filt.push_back(Bt[i] - Be[i]);
    }
    TL_A_filt_ = TL_A_;

    degeneration(TL_A_filt_);

    // linear regression to get TL coefficients;
    std::vector<double> residual;
    linear_regression(TL_beta_, residual, B_filt, TL_A_filt_, 0);

    // compute TL fit error variance;
    // double sum = std::accumulate(std::begin(residual), std::end(residual), 0.0);
    // double mean = sum / residual.size();
    // double variance = 0.0;
    // for (size_t i = 0; i < residual.size(); i++)
    // {
    //   variance = variance + pow(residual[i] - mean, 2);
    // }
    // variance = variance / residual.size();
    // double std = sqrt(variance);
    // printf("TL fit residual mean: %f\n", mean);
    // printf("TL fit residual std.: %f\n", std);

    printf("TL fit residual rmse: %f\n", rmse(residual));

    return true;
  }

  bool TollesLawson::createMatrixA(std::vector<std::vector<double>> &TL_A_,
                                   const std::vector<double> &Bx,
                                   const std::vector<double> &By,
                                   const std::vector<double> &Bz,
                                   std::vector<double> &Bt,
                                   const std::unordered_set<TLterm, HashTLterm> &terms,
                                   const double Bt_scale)
  {
    if (!(Bx.size() == By.size() && Bx.size() == Bz.size()))
    {
      return false;
    }
    if (terms.empty())
    {
      return false;
    }

    // Bt.clear(); // TODO
    if (Bt.empty())
    {
      Bt.resize(Bx.size());
      for (size_t i = 0; i < Bx.size(); i++)
      {
        Bt[i] = sqrt(Bx[i] * Bx[i] + By[i] * By[i] + Bz[i] * Bz[i]);
      }
    }
    else
    {
      if (Bx.size() != Bt.size())
      {
        return false;
      }
    }

    int N = Bx.size();
    std::vector<double> Bx_hat(N), By_hat(N), Bz_hat(N);
    for (int i = 0; i < N; i++)
    {
      Eigen::Vector3d bt(Bx[i], By[i], Bz[i]);
      double bt_norm = bt.norm();
      Bx_hat[i] = Bx[i] / bt_norm;
      By_hat[i] = By[i] / bt_norm;
      Bz_hat[i] = Bz[i] / bt_norm;
    }

    std::vector<double> Bx_hat_dot, By_hat_dot, Bz_hat_dot;
    fdm(Bx_hat_dot, Bx_hat);
    fdm(By_hat_dot, By_hat);
    fdm(Bz_hat_dot, Bz_hat);

    std::vector<double> Bx_hat_Bx(N), Bx_hat_By(N), Bx_hat_Bz(N), By_hat_By(N),
        By_hat_Bz(N), Bz_hat_Bz(N), Bx_hat_Bx_dot(N), Bx_hat_By_dot(N),
        Bx_hat_Bz_dot(N), By_hat_Bx_dot(N), By_hat_By_dot(N), By_hat_Bz_dot(N),
        Bz_hat_Bx_dot(N), Bz_hat_By_dot(N), Bz_hat_Bz_dot(N);
    for (int i = 0; i < N; i++)
    {
      Bx_hat_Bx[i] = Bt[i] * Bx_hat[i] * Bx_hat[i];
      Bx_hat_By[i] = Bt[i] * Bx_hat[i] * By_hat[i];
      Bx_hat_Bz[i] = Bt[i] * Bx_hat[i] * Bz_hat[i];
      By_hat_By[i] = Bt[i] * By_hat[i] * By_hat[i];
      By_hat_Bz[i] = Bt[i] * By_hat[i] * Bz_hat[i];
      Bz_hat_Bz[i] = Bt[i] * Bz_hat[i] * Bz_hat[i];

      Bx_hat_Bx_dot[i] = Bt[i] * Bx_hat[i] * Bx_hat_dot[i];
      Bx_hat_By_dot[i] = Bt[i] * Bx_hat[i] * By_hat_dot[i];
      Bx_hat_Bz_dot[i] = Bt[i] * Bx_hat[i] * Bz_hat_dot[i];
      By_hat_Bx_dot[i] = Bt[i] * By_hat[i] * Bx_hat_dot[i];
      By_hat_By_dot[i] = Bt[i] * By_hat[i] * By_hat_dot[i];
      By_hat_Bz_dot[i] = Bt[i] * By_hat[i] * Bz_hat_dot[i];
      Bz_hat_Bx_dot[i] = Bt[i] * Bz_hat[i] * Bx_hat_dot[i];
      Bz_hat_By_dot[i] = Bt[i] * Bz_hat[i] * By_hat_dot[i];
      Bz_hat_Bz_dot[i] = Bt[i] * Bz_hat[i] * Bz_hat_dot[i];
    }

    TL_A_.clear();

    if (terms.find(PERMANENT) != terms.end())
    {
      TL_A_.resize(3);
      TL_A_[0] = Bx_hat;
      TL_A_[1] = By_hat;
      TL_A_[2] = Bz_hat;
    }

    if (terms.find(INDUCED) != terms.end())
    {
      int idx = TL_A_.size();
      TL_A_.resize(idx + 6);
      TL_A_[idx] = Bx_hat_Bx;
      TL_A_[idx + 1] = Bx_hat_By;
      TL_A_[idx + 2] = Bx_hat_Bz;
      TL_A_[idx + 3] = By_hat_By;
      TL_A_[idx + 4] = By_hat_Bz;
      TL_A_[idx + 5] = Bz_hat_Bz;
    }

    if (terms.find(EDDY) != terms.end())
    {
      int idx = TL_A_.size();
      TL_A_.resize(idx + 9);
      TL_A_[idx] = Bx_hat_Bx_dot;
      TL_A_[idx + 1] = Bx_hat_By_dot;
      TL_A_[idx + 2] = Bx_hat_Bz_dot;
      TL_A_[idx + 3] = By_hat_Bx_dot;
      TL_A_[idx + 4] = By_hat_By_dot;
      TL_A_[idx + 5] = By_hat_Bz_dot;
      TL_A_[idx + 6] = Bz_hat_Bx_dot;
      TL_A_[idx + 7] = Bz_hat_By_dot;
      TL_A_[idx + 8] = Bz_hat_Bz_dot;
    }

    if (terms.find(FDM) != terms.end())
    {
      int idx = TL_A_.size();
      TL_A_.resize(idx + 3);
      TL_A_[idx] = Bx_hat_dot;
      TL_A_[idx + 1] = By_hat_dot;
      TL_A_[idx + 2] = Bz_hat_dot;
    }

    if (terms.find(BIAS) != terms.end())
    {
      int idx = TL_A_.size();
      std::vector<double> tmp(N, 1);
      TL_A_.resize(idx + 1, tmp);
    }

    return true;
  }

  bool TollesLawson::createCoeff(
      std::vector<double> &TL_beta_, const std::vector<double> &Bx,
      const std::vector<double> &By, const std::vector<double> &Bz,
      const std::vector<double> &Be, std::vector<double> &Bt,
      const std::unordered_set<TLterm, HashTLterm> &terms,
      const double lambda, const double pass1, const double pass2,
      const double fs, const int pole, const int trim, const double Bt_scale)
  {
    if (!(Bx.size() == By.size() && Bx.size() == Bz.size()))
    {
      return false;
    }
    if (terms.empty())
    {
      return false;
    }

    if (Bt.empty())
    {
      Bt.resize(Bx.size());
      for (size_t i = 0; i < Bx.size(); i++)
      {
        Bt[i] = sqrt(Bx[i] * Bx[i] + By[i] * By[i] + Bz[i] * Bz[i]);
      }
    }
    else
    {
      if (Bx.size() != Bt.size())
      {
        return false;
      }
    }

    bool perform_filter;
    if (Be.size() == Bx.size())
    {
      perform_filter = false;
      printf("[TL] use earth mag!\n");
    }
    else
    {
      if ((pass1 > 0 && pass1 < fs / 2.0) || (pass2 > 0 && pass2 < fs / 2.0))
      {
        perform_filter = true;
        printf("[TL] use band-pass filter!\n");
      }
      else
      {
        printf("[TL] wrong filter coeffs!\n");
        return false;
      }
    }

    std::vector<std::vector<double>> TL_A_;
    if (!createMatrixA(TL_A_, Bx, By, Bz, Bt, terms, Bt_scale))
    {
      return false;
    }

    std::vector<std::vector<double>> TL_A_filt_;
    std::vector<double> B_filt;

    if (perform_filter)
    {
      // if (perform_filter)
      // {
      // bandpass filter;
      // filter columns of matrix A and the measurements B
      // and trim edges;
      TL_A_filt_.resize(TL_A_.size());
      for (size_t i = 0; i < TL_A_.size(); i++)
      {
        bwbp_filter(TL_A_filt_[i], TL_A_[i], pass1, pass2, pole, trim);
      }
      bwbp_filter(B_filt, Bt, pass1, pass2, pole, trim);
      // }
      // else
      // {
      //   TL_A_filt_ = TL_A_;
      //   B_filt = B;
      // }
    }
    else
    {
      for (int i = 0; i < Bt.size(); i++)
      {
        B_filt.push_back(Bt[i] - Be[i]);
      }
      TL_A_filt_ = TL_A_;
    }

    degeneration(TL_A_filt_);

    // linear regression to get TL coefficients;
    std::vector<double> residual;
    linear_regression(TL_beta_, residual, B_filt, TL_A_filt_, lambda);

    // compute TL fit error variance;
    // double sum = std::accumulate(std::begin(residual), std::end(residual), 0.0);
    // double mean = sum / residual.size();
    // double variance = 0.0;
    // for (size_t i = 0; i < residual.size(); i++)
    // {
    //   variance = variance + pow(residual[i] - mean, 2);
    // }
    // variance = variance / residual.size();
    // double std = sqrt(variance);
    // printf("TL fit residual mean: %f\n", mean);
    // printf("TL fit residual std.: %f\n", std);

    printf("TL fit residual rmse: %f\n", rmse(residual));

    return true;
  }

  // sum((itr .- mean(itr)).^2) / (length(itr) - 1)

  bool TollesLawson::fdm(std::vector<double> &dif, const std::vector<double> &x,
                         const TollesLawson::FDMscheme scheme)
  {
    // N = length(x)
    int N = x.size();
    //		std::vector<double> dif(N);
    dif.resize(N);

    if (scheme == BACKWARD && N > 1)
    {
      dif[0] = x[1] - x[0];
      for (int i = 1; i < N; i++)
      {
        dif[i] = x[i] - x[i - 1];
      }
      return true;
    }
    else if (scheme == FORWARD && N > 1)
    {
      dif[N - 1] = x[N - 1] - x[N - 2];
      for (int i = 0; i < N - 1; i++)
      {
        dif[i] = x[i + 1] - x[i];
      }
      return true;
    }
    else if (scheme == CENTRAL && N > 2)
    {
      dif[0] = x[1] - x[0];
      dif[N - 1] = x[N] - x[N - 1];
      for (int i = 1; i < N - 1; i++)
      {
        dif[i] = 0.5 * (x[i + 1] - x[i - 1]);
      }
      return true;
    }
    else if (scheme == BACKWARD2 && N > 3)
    {
      dif[0] = x[1] - x[0];
      dif[1] = x[2] - x[1];
      for (int i = 2; i < N; i++)
      {
        dif[i] = 0.5 * (3.0 * x[i] - 4.0 * x[i - 1] + x[i - 2]);
      }
      return true;
    }
    else if (scheme == FORWARD2 && N > 3)
    {
      dif[N - 1] = x[N - 1] - x[N - 2];
      dif[N - 2] = x[N - 2] - x[N - 3];
      for (int i = 0; i < N - 2; i++)
      {
        dif[i] = 0.5 * (-x[i + 2] + 4.0 * x[i + 1] - 3.0 * x[i]);
      }
      return true;
    }
    else if (scheme == FOURTH && N > 4)
    {
      dif[0] = 0;
      dif[1] = 0;
      dif[N - 1] = 0;
      dif[N - 2] = 0;
      for (int i = 2; i < N - 2; i++)
      {
        dif[i] =
            (x[i - 2] - 4.0 * x[i - 1] + 6.0 * x[i] - 4.0 * x[i + 1] + x[i + 2]) /
            16.0;
      }
      return true;
    }
    else
    {
      std::fill(dif.begin(), dif.end(), 0.0);
      return false;
    }
  }

  bool TollesLawson::bwbp_filter(std::vector<double> &y,
                                 const std::vector<double> &x, const double pass1,
                                 const double pass2, const int pole,
                                 const int trim)
  {
    std::vector<double> bwbpfilter_A = ComputeDenCoeffs(pole, pass1, pass2);
    std::vector<double> bwbpfilter_B =
        ComputeNumCoeffs(pole, pass1, pass2, bwbpfilter_A);

    if (x.size() <= 3 * bwbpfilter_A.size() ||
        x.size() <= 3 * bwbpfilter_B.size())
    {
      return false;
    }

    // TODO
    // to be consistent with the resultes of 'butter' function in matlab;
    bwbpfilter_A.resize(bwbpfilter_A.size() - 1);

    y.clear();
    filtfilt(bwbpfilter_B, bwbpfilter_A, x, y);

    y.erase(y.begin(), y.begin() + trim);
    y.erase(y.end() - trim, y.end());

    return true;
  }

  void TollesLawson::linear_regression(std::vector<double> &coeff,
                                       std::vector<double> &residual,
                                       const std::vector<double> &bb,
                                       const std::vector<std::vector<double>> &AA,
                                       const double lambda)
  {
    const int N = bb.size();
    const int M = AA.size();
    Eigen::MatrixXd A(N, M);
    Eigen::VectorXd b(N);
    for (int i = 0; i < N; i++)
    {
      b(i) = bb[i];
      for (int j = 0; j < M; j++)
      {
        A(i, j) = AA[j][i];
      }
    }
    Eigen::MatrixXd ATA =
        A.transpose() * A + lambda * Eigen::MatrixXd::Identity(M, M);
    // std::cout << "ATA = " << std::endl << ATA << std::endl;
    Eigen::MatrixXd ATb = A.transpose() * b;
    // std::cout << "ATb = " << std::endl << ATb.transpose() << std::endl;
    Eigen::VectorXd x = ATA.llt().solve(ATb);
    // std::cout << "x = " << std::endl << x.transpose() << std::endl;

    coeff.resize(M);
    for (int i = 0; i < M; i++)
    {
      coeff[i] = x(i);
    }

    // linear regression fit error;
    Eigen::VectorXd delta = b - A * x;
    residual.resize(N);
    for (int i = 0; i < N; i++)
    {
      residual[i] = delta(i);
    }
  }

  void TollesLawson::stack_vector(std::vector<double> &result,
                                  const std::vector<double> &x,
                                  const std::vector<double> &y,
                                  const std::vector<double> &z)
  {
    if (!(x.size() == y.size() && x.size() == z.size()))
    {
      return;
    }
    result.clear();
    for (size_t i = 0; i < x.size(); i++)
    {
      result.push_back(x[i]);
      result.push_back(y[i]);
      result.push_back(z[i]);
    }
  }

  double TollesLawson::rmse(const std::vector<double> &residual)
  {
    std::vector<double> res_sq(residual.size());
    for (size_t i = 0; i < residual.size(); ++i)
    {
      res_sq[i] = residual[i] * residual[i];
    }
    double sum = std::accumulate(std::begin(res_sq), std::end(res_sq), 0.0);
    return sqrt(sum / residual.size());
  }

  void TollesLawson::degeneration(const std::vector<std::vector<double>> &TL_A)
  {
    unsigned int dim = TL_A.size();
    unsigned int N = TL_A[0].size();
    Eigen::MatrixXd A_matrix = Eigen::MatrixXd::Identity(N, dim);
    for (size_t i = 0; i < dim; ++i)
    {
      for (size_t j = 0; j < N; ++j)
      {
        A_matrix(j, i) = TL_A[i][j];
      }
    }
    Eigen::MatrixXd ATA = A_matrix.transpose() * A_matrix;
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(ATA);
    // Eigen::MatrixXd evec = es.eigenvectors();
    Eigen::VectorXd eval = es.eigenvalues();
    cout << "ATA eigenvalues: " << endl
         << eval.transpose() << endl;
    // cout<<"ATA eigenvectors: "<<endl<<evec<<endl;

    double sigma_min = sqrt(eval(0));
    double sigma_max = sqrt(eval(dim - 1));
    if (sigma_min > 1e-3)
    {
      double condition_k = sigma_max / sigma_min;
      cout << "condition number = " << condition_k << endl;
    }
    else
    {
      cerr << "errors solving eigenvalues of ATA" << endl;
    }

    // Eigen::JacobiSVD<Eigen::MatrixXd> svd(A_matrix, Eigen::ComputeFullU | Eigen::ComputeFullV);
    // Eigen::JacobiSVD<Eigen::MatrixXd, Eigen::ComputeThinU | Eigen::ComputeThinV> svd(A_matrix);
    // cout << "A_matrix singular values: " << endl
    //      << svd.singularValues().transpose() << endl;
  }

} // namespace magnav
