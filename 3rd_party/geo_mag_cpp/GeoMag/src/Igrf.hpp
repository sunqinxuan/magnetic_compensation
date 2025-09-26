/**
 * @file Igrf.hpp
 * @author fugu133
 * @brief IGRFモデルを用いて磁束密度を計算する何か
 * @version 0.1
 * @date 2023-12-22
 *
 * @copyright Copyright (c) 2023
 *
 */

#pragma once

#include "Coordinate.hpp"
#include "Essential.hpp"
#include "Model.hpp"

GEOMAG_NAMESPACE_BEGIN
class Igrf {
  public:
	/**
	 * @brief デフォルトモデルでモデルを生成する
	 * @ref https://www.ngdc.noaa.gov/IAGA/vmod/coeffs/igrf13coeffs.txt
	 *
	 */
	Igrf() : m_model_set(){};

	/**
	 * @brief モデルセットを指定してモデルを生成する
	 *
	 * @param model_set
	 */
	Igrf(const ModelSet& model_set) : m_model_set(model_set){};

	/**
	 * @brief 入力ストリームからモデルを読み込みモデルを生成する
	 *
	 * @param is
	 */
	Igrf(std::istream& is) : m_model_set(is){};

  private:
	Model m_model;										 // IGRF model
	ModelSet m_model_set;								 // IGRF model set

	/**
	 * @brief 線形補間によりモデルを生成する
	 *
	 * @param dt 作成するモデルの時刻
	 * @param last 補間に用いるモデルのうち、dtよりも前の時刻のモデル
	 * @param next 補間に用いるモデルのうち、dtよりも後の時刻のモデル
	 * @param model 生成されるモデル
	 */
	void interpolateModel(const DateTime& dt, const Model& last, const Model& next, Model& model) {
		auto diff = (dt.fractionalYears() - last.epoch.year()) / (double)(next.epoch.year() - last.epoch.year());
		std::transform(last.coefficients.begin(), last.coefficients.end(), next.coefficients.begin(), model.coefficients.begin(),
					   [diff](double a, double b) { return a + diff * (b - a); });
	}

	/**
	 * @brief 線形外挿によりモデルを生成する
	 *
	 * @param dt 作成するモデルの時刻
	 * @param last 補間に用いるモデルのうち、dtよりも前の時刻のモデル
	 * @param next 補間に用いるモデルのうち、dtよりも後の時刻のモデル
	 * @param model 生成されるモデル
	 */
	void extrapolateModel(const DateTime& dt, const Model& last, const Model& next, Model& model) {
		double diff = dt.fractionalYears() - last.epoch.year();
		// model.coefficients = last.coefficients + diff * next.coefficients;
		std::transform(last.coefficients.begin(), last.coefficients.end(), next.coefficients.begin(), model.coefficients.begin(),
					   [diff](double a, double b) { return a + diff * b; });
	}

	/**
	 * @brief モデルを初期化する
	 *
	 * @param dt 初期化するモデルの時刻
	 */
	void initializeModel(const DateTime& dt) {
		Model last, next;

		// Select model
		m_model_set.select(dt, last, next);

		// interpolate or extrapolate model
		if (next.type != ModelType::Sv) {
			interpolateModel(dt, last, next, m_model);
			m_model.epoch = dt;
			m_model.type = ModelType::Interpolated;
		} else {
			extrapolateModel(dt, last, next, m_model);
			m_model.epoch = dt;
			m_model.type = ModelType::Extrapolated;
		}
	}

	/**
	 * @brief 磁束密度を計算する
	 *
	 * @tparam T 位置情報の型
	 * @param position 座標系情報を持った位置
	 * @param mag_density その位置での磁束密度 [nT]
	 */
	template <typename T>
	void calculateMagDensity(const CoordinateBase<T> position, Eigen::Vector3d& mag_density) {
		constexpr std::size_t max_degree = Model::max_degree;
		constexpr double earth_radius = 6371.2e3; // IGRFはこれ[m]

		double r = position.elements().altitude;					 // distance
		const double phi = position.elements().longitude.radians();	 // longitude
		const double theta = position.elements().latitude.radians(); // latitude

		double cos_theta = std::sin(theta); // colatitude
		double sin_theta = std::cos(theta);
		double cos_delta = 0.0, sin_delta = 0.0;

		if (position.type() == CoordinateType::GeocentricSpherical) {
			cos_delta = 1.0;
			sin_delta = 0.0;
		} else if (position.type() == CoordinateType::Wgs84) {
			constexpr auto a = constant::wgs84_a;
			constexpr auto b = constant::wgs84_b;
			constexpr auto aa = a * a;
			constexpr auto bb = b * b;
			const auto h = r;
			const auto a2sint2 = aa * sin_theta * sin_theta;
			const auto b2cost2 = bb * cos_theta * cos_theta;
			const auto rho2 = a2sint2 + b2cost2;
			const auto rho = std::sqrt(rho2);
			r = std::sqrt((aa * a2sint2 + bb * b2cost2) / rho2 + r * r + 2 * r * rho);
			cos_delta = (h + rho) / r;
			sin_delta = (aa - bb) / rho * sin_theta * cos_theta / r;
			const double cos_theta_gd = cos_theta;
			cos_theta = cos_theta_gd * cos_delta - sin_theta * sin_delta;
			sin_theta = sin_theta * cos_delta + cos_theta_gd * sin_delta;
		} else {
			throw std::runtime_error("Invalid coordinate type");
		}

		std::array<double, max_degree> cos_phi; // cos(m*phi)
		std::array<double, max_degree> sin_phi; // sin(m*phi)
		for (std::size_t m = 1; m <= max_degree; m++) {
			cos_phi[m - 1] = std::cos(m * phi);
			sin_phi[m - 1] = std::sin(m * phi);
		}

		constexpr std::size_t p_size = (max_degree + 1) * (max_degree + 2) / 2;
		std::array<double, p_size> p{0};   // Legendre polynomial
		std::array<double, p_size> d_p{0}; // Derivative of Legendre polynomial
		p[0] = 1;
		p[2] = sin_theta;
		d_p[0] = 0;
		d_p[2] = cos_theta;

		double b_r = 0, b_t = 0, b_p = 0;
		double ratio = (earth_radius / r) * (earth_radius / r);

		// Lag
		int c_idx = 1, n = 0, m = 1;
		for (std::size_t p_idx = 2; p_idx <= p_size; p_idx++) {
			if (n < m) {
				n++;
				m = 0;
				ratio *= earth_radius / r;
			}

			const int p_lag0 = p_idx - 1;
			if (n == m && p_lag0 != 2) {
				const int p_lag1 = p_idx - n - 2;
				const double cof = std::sqrt(1 - 1 / (double)(2 * m));
				p[p_lag0] = cof * sin_theta * p[p_lag1];
				d_p[p_lag0] = cof * (sin_theta * d_p[p_lag1] + cos_theta * p[p_lag1]);
			} else if (p_lag0 != 2) {
				const int p_lag1 = p_idx - n - 1;
				const int p_lag2 = p_idx - 2 * n;
				const double cofl = (2 * n - 1) / std::sqrt(n * n - m * m);
				const double cofr = std::sqrt((n - 1) * (n - 1) - m * m) / std::sqrt(n * n - m * m);
				p[p_lag0] = cofl * cos_theta * p[p_lag1] - cofr * p[p_lag2];
				d_p[p_lag0] = cofl * (cos_theta * d_p[p_lag1] - sin_theta * p[p_lag1]) - cofr * d_p[p_lag2];
			}

			if (m == 0) {
				const double c_lag0 = c_idx - 1;
				const double& gh_cof = m_model.coefficients[c_lag0];
				const double cof = ratio * gh_cof;
				b_r += (n + 1) * cof * p[p_lag0];
				b_t -= cof * d_p[p_lag0];
				c_idx++;
			} else {
				const double m_lag0 = m - 1;
				const double c_lag0 = c_idx - 1;
				const double& gh_cof0 = m_model.coefficients[c_lag0];
				const double& gh_cof1 = m_model.coefficients[c_lag0 + 1];
				const double cof = ratio * (gh_cof0 * cos_phi[m_lag0] + gh_cof1 * sin_phi[m_lag0]);
				b_r += (n + 1) * cof * p[p_lag0];
				b_t -= cof * d_p[p_lag0];
				if (sin_theta == 0.0) {
					b_p -= cos_theta * ratio * (gh_cof1 * cos_phi[m_lag0] - gh_cof0 * sin_phi[m_lag0]) * p[p_lag0];
				} else {
					b_p -= 1 / sin_theta * ratio * m * (gh_cof1 * cos_phi[m_lag0] - gh_cof0 * sin_phi[m_lag0]) * p[p_lag0];
				}
				c_idx += 2;
			}
			m++;
		}
		mag_density << -b_t * cos_delta - b_r * sin_delta, b_p, b_t * sin_delta - b_r * cos_delta;
	}

  protected:
	/**
	 * @brief 位置と磁束密度を更新する
	 *
	 * @param position ECEF座標系での位置ベクトル
	 * @param mag_density その位置での磁束密度 [nT]
	 */
	void updatePositionAndMag(const Ecef& position, Eigen::Vector3d& mag_density) {
		initializeModel(position.epoch());
		calculateMagDensity(position.toGeocentricSpherical(), mag_density);
	}

	/**
	 * @brief 位置と磁束密度を更新する
	 *
	 * @param position WGS84回転楕円座標系での位置
	 * @param mag_density その位置での磁束密度 [nT]
	 */
	void updatePositionAndMag(const Wgs84& position, Eigen::Vector3d& mag_density) {
		initializeModel(position.epoch());
		calculateMagDensity(position, mag_density);
	}
};
GEOMAG_NAMESPACE_END