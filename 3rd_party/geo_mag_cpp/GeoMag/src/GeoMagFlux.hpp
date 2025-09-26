/**
 * @file GeoMagFlux.hpp
 * @author Kaiji Takeuchi
 * @brief
 * @version 0.1
 * @date 2024-01-06
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

#include "Eigen/Geometry"
#include "Igrf.hpp"

GEOMAG_NAMESPACE_BEGIN

enum class MagFluxUnit { NanoTesla, MicroTesla, Tesla, Gauss, Si, Cgs, Mks, Mksa };

class GeoMagFlux : protected Igrf {
  public:
	/**
	 * @brief デフォルトモデルでモデルを生成する
	 * @ref https://www.ngdc.noaa.gov/IAGA/vmod/coeffs/igrf13coeffs.txt
	 *
	 */
	GeoMagFlux(MagFluxUnit unit = MagFluxUnit::Si) : Igrf{} { setScaling(unit); }

	/**
	 * @brief モデルセットを指定してモデルを生成する
	 *
	 * @param model_set
	 */
	GeoMagFlux(const ModelSet& model_set, MagFluxUnit unit = MagFluxUnit::Si) : Igrf(model_set) { setScaling(unit); };

	/**
	 * @brief 入力ストリームからモデルを読み込みモデルを生成する
	 *
	 * @param is
	 */
	GeoMagFlux(std::istream& is, MagFluxUnit unit = MagFluxUnit::Si) : Igrf(is) { setScaling(unit); };

	/**
	 * @brief 任意位置での磁束密度を取得する
	 *
	 * @param position ECEF座標系での位置
	 * @return Eigen::Vector3d 磁束密度
	 */
	Eigen::Vector3d operator()(const Ecef& position) {
		Eigen::Vector3d mag_density;
		updatePositionAndMag(position, mag_density);
		return mag_density * m_unit_scale;
	}

	/**
	 * @brief 任意位置での磁束密度を取得する
	 *
	 * @param position WGS84回転楕円座標系での位置
	 * @return Eigen::Vector3d 磁束密度
	 */
	Eigen::Vector3d operator()(const Wgs84& position) {
		Eigen::Vector3d mag_density;
		updatePositionAndMag(position, mag_density);
		return mag_density * m_unit_scale;
	}

	/**
	 * @brief 任意位置での磁束密度を取得する
	 *
	 * @param dt 時刻
	 * @param position ECEF座標系での位置
	 * @return Eigen::Vector3d 磁束密度
	 */
	Eigen::Vector3d operator()(const DateTime& dt, const Eigen::Vector3d& position) { return operator()(Ecef{dt, position}); }

	/**
	 * @brief 任意位置での磁束密度を取得する
	 *
	 * @param dt 時刻
	 * @param position WGS84回転楕円座標系での位置
	 * @return Eigen::Vector3d 磁束密度
	 */
	Eigen::Vector3d operator()(const DateTime& dt, const Wgs84Position& position) { return operator()(Wgs84{dt, position}); }

	void setOutputUnit(MagFluxUnit unit) { setScaling(unit); }

  private:
	static constexpr double nanotesla_to_tesla = 1.0e-9;	  // [nT] ->
	static constexpr double nanotesla_to_microtesla = 1.0e-3; // [nT] -> [uT]
	static constexpr double nanotesla_to_gauss = 1.0e-5;	  // [nT] -> [G]

	MagFluxUnit m_unit;
	double m_unit_scale;
	std::string m_unit_symbol;

	void setScaling(MagFluxUnit unit) {
		m_unit = unit;
		switch (m_unit) {
			case MagFluxUnit::NanoTesla:
				m_unit_scale = 1.0;
				m_unit_symbol = "nT";
				return;
			case MagFluxUnit::MicroTesla:
				m_unit_scale = nanotesla_to_microtesla;
				m_unit_symbol = "uT";
				return;
			case MagFluxUnit::Tesla:
				m_unit_scale = nanotesla_to_tesla;
				m_unit_symbol = "T";
				return;
			case MagFluxUnit::Gauss:
				m_unit_scale = nanotesla_to_gauss;
				m_unit_symbol = "G";
				return;
			case MagFluxUnit::Si:
				m_unit_scale = nanotesla_to_tesla;
				m_unit_symbol = "T";
				return;
			case MagFluxUnit::Cgs:
				m_unit_scale = nanotesla_to_gauss;
				m_unit_symbol = "G";
				return;
			case MagFluxUnit::Mks:
				m_unit_scale = nanotesla_to_tesla;
				m_unit_symbol = "T";
				return;
			case MagFluxUnit::Mksa:
				m_unit_scale = nanotesla_to_tesla;
				m_unit_symbol = "T";
				return;
			default: return;
		}
	}
};

struct MagFluxComponent {
	double north;
	double east;
	double down;
	double total;
	double horizontal;
	Angle inclination;
	Angle declination;

	MagFluxComponent(const Eigen::Vector3d& mag_density) {
		north = mag_density(0);
		east = mag_density(1);
		down = mag_density(2);
		total = mag_density.norm();
		horizontal = std::sqrt(north * north + east * east);
		inclination = Radian{std::atan2(down, horizontal)};
		declination = Radian{std::atan2(east, north)};
	}
};

GEOMAG_NAMESPACE_END