/**
 * @file Coordinate.hpp
 * @author Kaiji Takeuchi (fugu0220@gmail.com)
 * @brief
 * @version 0.1
 * @date 2023-12-01
 *
 * @copyright Copyright (c) 2023
 *
 */

#pragma once

#include <iostream>

#include "../../../eigen-3.3.9/Eigen/Geometry"
#include "AngleHelper.hpp"
#include "DateTime.hpp"
#include "Essential.hpp"

GEOMAG_NAMESPACE_BEGIN

enum class CoordinateType {
	Icrf,
	Eci,				 // ECI Cartesian (Earth-centered inertial)
	Ecef,				 // ECEF Cartesian (Earth-centered, Earth-fixed)
	GeocentricSpherical, // ECEF Geocentric Spherical
	Wgs84,				 // WGS84 Spheroid
	GeodeticCartesian,	 // WGS84 Geodetic Cartesian
	EclipticSpherical,	 // Ecliptic Spherical
	EclipticCartesian,	 // Ecliptic Cartesian
	EquatorialSpherical, // Equatorial Spherical
	EquatorialCartesian, // Equatorial Cartesian
	Topocentric			 // Topocentric (Azimuth, Elevation, Range)
};

template <class DataType>
class CoordinateBase {
  public:
	CoordinateBase() : m_epoch(DateTime::now()), m_type(CoordinateType::Eci) {}
	CoordinateBase(const DateTime& dt, const DataType& d, CoordinateType typ) : m_epoch(dt), m_data(d), m_type(typ) {}
	const DateTime& epoch() const { return m_epoch; }
	const DataType& elements() const { return m_data; }
	CoordinateType type() const { return m_type; }
	virtual std::string toString() const { return ""; }

	const DataType& operator()() const { return m_data; }

  protected:
	DateTime m_epoch;
	DataType m_data;
	CoordinateType m_type;
};

class Eci;
class Ecef;
using GeocentricCartesian = Ecef;
class GeocentricSpherical;
class Wgs84;
class GeodeticCartesian;
using GeodeticSpheroid = Wgs84;
class EclipticSpherical;
class EclipticCartesian;
class EquatorialSpherical;
using EquatorialCartesian = Eci;
class Topocentric;

class Eci : public CoordinateBase<Eigen::Vector3d> {
  public:
	Eci() : CoordinateBase(DateTime::now(), Eigen::Vector3d::Zero(), CoordinateType::Eci) {}
	Eci(const DateTime& dt, const Eigen::Vector3d& d) : CoordinateBase(dt, d, CoordinateType::Eci) {}
	Eci(const DateTime& dt, double x, double y, double z) : CoordinateBase(dt, Eigen::Vector3d{x, y, z}, CoordinateType::Eci) {}

	const double& x() const { return m_data.x(); }
	const double& y() const { return m_data.y(); }
	const double& z() const { return m_data.z(); }

	Eci toEci() const { return *this; }
	EquatorialSpherical toEquatorialSpherical() const;
	Ecef toEcef() const;
	GeocentricSpherical toGeocentricSpherical() const;
	Wgs84 toWgs84() const;

	std::string toString() const override {
		std::stringstream ss;
		ss << "ECI(t = " << m_epoch.toString() << ", x = " << m_data.x() << " [m], y = " << m_data.y() << " [m], z = " << m_data.z()
		   << " [m])";
		return ss.str();
	}

	friend auto operator<<(std::ostream& os, const Eci& eci) -> std::ostream& {
		os << eci.toString();
		return os;
	}
};

class Ecef : public CoordinateBase<Eigen::Vector3d> {
  public:
	Ecef() : CoordinateBase(DateTime::now(), Eigen::Vector3d::Zero(), CoordinateType::Ecef) {}
	Ecef(const DateTime& dt, const Eigen::Vector3d& d) : CoordinateBase(dt, d, CoordinateType::Ecef) {}
	Ecef(const DateTime& dt, double x, double y, double z) : CoordinateBase(dt, Eigen::Vector3d{x, y, z}, CoordinateType::Ecef) {}

	const double& x() const { return m_data.x(); }
	const double& y() const { return m_data.y(); }
	const double& z() const { return m_data.z(); }

	Eci toEci() const;
	EquatorialSpherical toEquatorialSpherical() const;
	Ecef toEcef() const { return *this; }
	GeocentricSpherical toGeocentricSpherical() const;
	Wgs84 toWgs84() const;

	std::string toString() const override {
		std::stringstream ss;
		ss << "ECEF(t =" << m_epoch.toString() << ", x = " << m_data.x() << " [m], y = " << m_data.y() << " [m], z =" << m_data.z()
		   << " [m])";
		return ss.str();
	}

	friend auto operator<<(std::ostream& os, const Ecef& ecef) -> std::ostream& {
		os << ecef.toString();
		return os;
	}
};

struct GeocentricSphericalPosition {
	Angle longitude;
	Angle latitude;
	double altitude;
};

class GeocentricSpherical : public CoordinateBase<GeocentricSphericalPosition> {
  public:
	GeocentricSpherical()
	  : CoordinateBase(DateTime::now(), GeocentricSphericalPosition{Angle::zero(), Angle::zero(), 0.0},
					   CoordinateType::GeocentricSpherical) {}
	GeocentricSpherical(const DateTime& dt, const GeocentricSphericalPosition& d)
	  : CoordinateBase(dt, d, CoordinateType::GeocentricSpherical) {}
	GeocentricSpherical(const DateTime& dt, const Angle& lon, const Angle& lat, double alt)
	  : CoordinateBase(dt, GeocentricSphericalPosition{lon, lat, alt}, CoordinateType::GeocentricSpherical) {}

	const Angle& longitude() const { return m_data.longitude; }
	const Angle& latitude() const { return m_data.latitude; }
	const double& altitude() const { return m_data.altitude; }

	Ecef toEcef() const;
	Eci toEci() const;
	GeocentricSpherical toGeocentricSpherical() const { return *this; }
	EquatorialSpherical toEquatorialSpherical() const;
	Wgs84 toWgs84() const;

	std::string toString() const override {
		std::stringstream ss;
		ss << "GeocentricSpherical(t = " << m_epoch.toString() << ", Lon = " << m_data.longitude.degrees()
		   << " [deg], Lat = " << m_data.latitude.degrees() << " [deg], Alt = " << m_data.altitude << " [m])";
		return ss.str();
	}

	friend auto operator<<(std::ostream& os, const GeocentricSpherical& geocentric) -> std::ostream& {
		os << geocentric.toString();
		return os;
	}
};

struct Wgs84Position {
	Angle longitude;
	Angle latitude;
	double altitude;
};

class Wgs84 : public CoordinateBase<Wgs84Position> {
  public:
	Wgs84() : CoordinateBase(DateTime::now(), Wgs84Position{Angle::zero(), Angle::zero(), 0.0}, CoordinateType::Wgs84) {}
	Wgs84(const DateTime& dt, const Wgs84Position& d) : CoordinateBase(dt, d, CoordinateType::Wgs84) {}
	Wgs84(const DateTime& dt, const Angle& lon, const Angle& lat, double alt)
	  : CoordinateBase(dt, Wgs84Position{lon, lat, alt}, CoordinateType::Wgs84) {}
	// Wgs84(const DateTime& dt, double lon, double lat, double alt)
	//   : CoordinateBase(dt, Wgs84Position{lon, lat, alt}, CoordinateType::Wgs84) {}

	const Angle& longitude() const { return m_data.longitude; }
	const Angle& latitude() const { return m_data.latitude; }
	const double& altitude() const { return m_data.altitude; }

	Eci toEci() const;
	EquatorialSpherical toEquatorialSpherical() const;
	Ecef toEcef() const;
	GeocentricSpherical toGeocentricSpherical() const;
	Wgs84 toWgs84() const { return *this; }

	std::string toString() const override {
		std::stringstream ss;
		ss << "WGS84(" << m_epoch.toString() << ", Lon = " << m_data.longitude.degrees() << " [deg], Lat = " << m_data.latitude.degrees()
		   << " [deg], Alt = " << m_data.altitude << " [m])";
		return ss.str();
	}

	friend auto operator<<(std::ostream& os, const Wgs84& wgs84) -> std::ostream& {
		os << wgs84.toString();
		return os;
	}
};

struct EclipticSphericalPosition {
	Angle ecliptic_longitude;
	Angle ecliptic_latitude;
	double distance;
};

class EclipticSpherical : public CoordinateBase<EclipticSphericalPosition> {
  public:
	EclipticSpherical()
	  : CoordinateBase(DateTime::now(), EclipticSphericalPosition{Angle::zero(), Angle::zero(), 0.0}, CoordinateType::EclipticSpherical) {}
	EclipticSpherical(const DateTime& dt, const EclipticSphericalPosition& d) : CoordinateBase(dt, d, CoordinateType::EclipticSpherical) {}
	EclipticSpherical(const DateTime& dt, const Angle& lon, const Angle& lat, double r)
	  : CoordinateBase(dt, EclipticSphericalPosition{lon, lat, r}, CoordinateType::EclipticSpherical) {}

	const Angle& longitude() const { return m_data.ecliptic_longitude; }
	const Angle& latitude() const { return m_data.ecliptic_latitude; }
	const double& distance() const { return m_data.distance; }

	EclipticCartesian toEclipticCartesian() const;
	Eci toEci() const;
	EquatorialSpherical toEquatorialSpherical() const;

	std::string toString() const override {
		std::stringstream ss;
		ss << "EclipticSpherical(t = " << m_epoch.toString() << ", Lon = " << m_data.ecliptic_longitude.degrees()
		   << " [deg], Lat = " << m_data.ecliptic_latitude.degrees() << " [deg], R = " << m_data.distance << " [m])";
		return ss.str();
	}

	friend auto operator<<(std::ostream& os, const EclipticSpherical& ecliptic) -> std::ostream& {
		os << ecliptic.toString();
		return os;
	}
};

struct EclipticCartesian : public CoordinateBase<Eigen::Vector3d> {
	EclipticCartesian() : CoordinateBase(DateTime::now(), Eigen::Vector3d::Zero(), CoordinateType::EclipticCartesian) {}
	EclipticCartesian(const DateTime& dt, const Eigen::Vector3d& d) : CoordinateBase(dt, d, CoordinateType::EclipticCartesian) {}
	EclipticCartesian(const DateTime& dt, double x, double y, double z)
	  : CoordinateBase(dt, Eigen::Vector3d{x, y, z}, CoordinateType::EclipticCartesian) {}

	const double& x() const { return m_data.x(); }
	const double& y() const { return m_data.y(); }
	const double& z() const { return m_data.z(); }

	EclipticSpherical toEclipticSpherical() const;
	Eci toEci() const;

	std::string toString() const override {
		std::stringstream ss;
		ss << "EclipticCartesian(t = " << m_epoch.toString() << ", x = " << m_data.x() << " [m], y = " << m_data.y()
		   << " [m], z = " << m_data.z() << " [m])";
		return ss.str();
	}

	friend auto operator<<(std::ostream& os, const EclipticCartesian& ecliptic) -> std::ostream& {
		os << ecliptic.toString();
		return os;
	}
};

struct EquatorialSphericalPosition {
	Angle rightAscension;
	Angle declination;
	double distance;
};

struct EquatorialSpherical : public CoordinateBase<EquatorialSphericalPosition> {
  public:
	EquatorialSpherical()
	  : CoordinateBase(DateTime::now(), EquatorialSphericalPosition{Angle::zero(), Angle::zero(), 0.0},
					   CoordinateType::EquatorialSpherical) {}
	EquatorialSpherical(const DateTime& dt, const EquatorialSphericalPosition& d)
	  : CoordinateBase(dt, d, CoordinateType::EquatorialSpherical) {}
	EquatorialSpherical(const DateTime& dt, const Angle& ra, const Angle& dec, double r)
	  : CoordinateBase(dt, EquatorialSphericalPosition{ra, dec, r}, CoordinateType::EquatorialSpherical) {}

	const Angle& rightAscension() const { return m_data.rightAscension; }
	const Angle& declination() const { return m_data.declination; }
	const double& distance() const { return m_data.distance; }

	std::string toString() const override {
		std::stringstream ss;
		ss << "EquatorialSpherical(t = " << m_epoch.toString() << ", RA = " << m_data.rightAscension.degrees()
		   << " [deg], Dec = " << m_data.declination.degrees() << " [deg], R = " << m_data.distance << " [m])";
		return ss.str();
	}

	EclipticSpherical toEclipticSpherical() const;
	Eci toEci() const;

	friend auto operator<<(std::ostream& os, const EquatorialSpherical& equatorial) -> std::ostream& {
		os << equatorial.toString();
		return os;
	}
};

struct TopocentricPosition {
	Angle azimuth;
	Angle elevation;
	double range;
};

class Topocentric : public CoordinateBase<TopocentricPosition> {
  public:
	Topocentric() : CoordinateBase(DateTime::now(), TopocentricPosition{Angle::zero(), Angle::zero(), 0.0}, CoordinateType::Topocentric) {}
	Topocentric(const DateTime& dt, const TopocentricPosition& d) : CoordinateBase(dt, d, CoordinateType::Topocentric) {}
	Topocentric(const DateTime& dt, const Angle& az, const Angle& el, double r)
	  : CoordinateBase(dt, TopocentricPosition{az, el, r}, CoordinateType::Topocentric) {}

	const Angle& azimuth() const { return m_data.azimuth; }
	const Angle& elevation() const { return m_data.elevation; }
	const double& range() const { return m_data.range; }

	std::string toString() const override {
		std::stringstream ss;
		ss << "AER(t = " << m_epoch.toString() << ", Az = " << m_data.azimuth.degrees() << " [deg], El = " << m_data.elevation.degrees()
		   << " [deg], R = " << m_data.range << " [m])";
		return ss.str();
	}

	friend auto operator<<(std::ostream& os, const Topocentric& topocentric) -> std::ostream& {
		os << topocentric.toString();
		return os;
	}
};

inline Ecef Eci::toEcef() const {
	const double theta = m_epoch.greenwichSiderealTime().radians();
	const double x = m_data.x() * std::cos(theta) + m_data.y() * std::sin(theta);
	const double y = -m_data.x() * std::sin(theta) + m_data.y() * std::cos(theta);
	const double z = m_data.z();
	return Ecef(m_epoch, Eigen::Vector3d{x, y, z});
}

inline GeocentricSpherical Eci::toGeocentricSpherical() const {
	return toEcef().toGeocentricSpherical();
}

inline Wgs84 Eci::toWgs84() const {
	return toEcef().toWgs84();
}

inline EquatorialSpherical Eci::toEquatorialSpherical() const {
	const double r = m_data.norm();
	const double theta = std::atan2(m_data.y(), m_data.x()); // right ascension
	const double phi = std::asin(m_data.z() / r);			 // geocentric latitude
	return EquatorialSpherical(m_epoch, EquatorialSphericalPosition{Radian(theta), Radian(phi), r});
}

inline Eci Ecef::toEci() const {
	const double theta = m_epoch.greenwichSiderealTime().radians();
	const double x = m_data.x() * std::cos(theta) - m_data.y() * std::sin(theta);
	const double y = m_data.x() * std::sin(theta) + m_data.y() * std::cos(theta);
	const double z = m_data.z();

	return Eci(m_epoch, Eigen::Vector3d{x, y, z});
}

inline Ecef GeocentricSpherical::toEcef() const {
	const double cos_theta = m_data.latitude.cos();
	const double sin_theta = m_data.latitude.sin();
	const double cos_phi = m_data.longitude.cos();
	const double sin_phi = m_data.longitude.sin();

	const double x = cos_theta * cos_phi;
	const double y = cos_theta * sin_phi;
	const double z = sin_theta;

	return Ecef(m_epoch, m_data.altitude * Eigen::Vector3d{x, y, z});
}

inline GeocentricSpherical Ecef::toGeocentricSpherical() const {
	const double p = std::sqrt(m_data.x() * m_data.x() + m_data.y() * m_data.y());
	const double theta = std::atan2(m_data.z(), p);
	const double phi = std::atan2(m_data.y(), m_data.x());
	const double r = m_data.norm();
	return GeocentricSpherical(m_epoch, GeocentricSphericalPosition{Radian(phi), Radian(theta), r});
}

inline Wgs84 Ecef::toWgs84() const {
	constexpr double a = constant::wgs84_a;
	constexpr double b = constant::wgs84_b;
	constexpr double e2 = (a * a - b * b) / (a * a);

	const double p = std::sqrt(m_data.x() * m_data.x() + m_data.y() * m_data.y());

	double phi = std::atan2(p, m_data.z()); // geocentric latitude
	double lat = phi;
	std::int32_t i = 0;
	do {
		phi = lat;
		const double sin_phi = std::sin(phi);
		const double N = a / std::sqrt(1 - e2 * sin_phi * sin_phi);
		lat = std::atan2(m_data.z() + N * e2 * sin_phi, p);
		i++;
	} while (std::abs(lat - phi) > 1e-10 && i < 10); // 4回くらいで収束する

	const double lon = std::atan2(m_data.y(), m_data.x());
	const double alt = p / std::cos(phi) - a / std::sqrt(1 - e2 * std::sin(phi) * std::sin(phi));

	return Wgs84(m_epoch, Wgs84Position{Radian(lon), Radian(lat), alt});
}

inline EquatorialSpherical Ecef::toEquatorialSpherical() const {
	return toEci().toEquatorialSpherical();
}

inline Ecef Wgs84::toEcef() const {
	// constexpr double a = constant::wgs84_a;
	// constexpr double b = constant::wgs84_b;
	// constexpr double e2 = (a * a - b * b) / (a * a);
	// const double N = a / std::sqrt(1 - e2 * m_data.latitude.sin() * m_data.latitude.sin());
	// const double R = (N + m_data.altitude) * m_data.latitude.cos();
	// const double Re = (N * (1 - e2) + m_data.altitude);
	// const double x = R * m_data.longitude.cos();
	// const double y = R * m_data.longitude.sin();
	// const double z = Re * m_data.latitude.sin();
	// return Ecef(m_epoch, Eigen::Vector3d{x, y, z});
	constexpr double a = constant::wgs84_a;
	constexpr double b = constant::wgs84_b;
	constexpr double e2 = 1 - b * b / (a * a);
	const double cos_phi = m_data.latitude.cos();
	const double sin_phi = m_data.latitude.sin();
	const double cos_theta = m_data.longitude.cos();
	const double sin_theta = m_data.longitude.sin();
	const double N = a / std::sqrt(1 - e2 * sin_phi * sin_phi);
	const double x = (N + m_data.altitude) * cos_phi * cos_theta;
	const double y = (N + m_data.altitude) * cos_phi * sin_theta;
	const double z = (N * (1 - e2) + m_data.altitude) * sin_phi;
	return Ecef(m_epoch, Eigen::Vector3d{x, y, z});
}

inline GeocentricSpherical Wgs84::toGeocentricSpherical() const {
	return toEcef().toGeocentricSpherical();
}

inline Eci Wgs84::toEci() const {
	return toEcef().toEci();
}

inline EquatorialSpherical Wgs84::toEquatorialSpherical() const {
	return toEci().toEquatorialSpherical();
}

inline EquatorialSpherical EclipticSpherical::toEquatorialSpherical() const {
	const double T = (m_epoch.j2000() + m_epoch.deltaT().totalDays()) / constant::jd_century;
	const double Omega = AngleHelper::degreeToWrapRadian(125.04 - 1934.136 * T); // Longitude of ascending node
	const double epsilon = AngleHelper::degreeToWrapRadian(23 + (26 + Polynomial::deg3(T, 21.448, 46.8150, 0.00059, -0.001813) / 60) / 60 +
														   0.00256 * std::cos(Omega)); // Obliquity of the ecliptic

	const double alpha = AngleHelper::wrapRadian(
	  std::atan2(m_data.ecliptic_longitude.sin() * std::cos(epsilon) - m_data.ecliptic_latitude.tan() * std::sin(epsilon),
				 m_data.ecliptic_longitude.cos()));
	const double delta = std::asin(m_data.ecliptic_latitude.sin() * std::cos(epsilon) +
								   m_data.ecliptic_latitude.cos() * std::sin(epsilon) * m_data.ecliptic_longitude.sin());
	return EquatorialSpherical(m_epoch, EquatorialSphericalPosition{Radian(alpha), Radian(delta), m_data.distance});
}

inline EclipticSpherical EquatorialSpherical::toEclipticSpherical() const {
	const double T = (m_epoch.j2000() + m_epoch.deltaT().totalDays()) / constant::jd_century;
	const double Omega = AngleHelper::degreeToWrapRadian(125.04 - 1934.136 * T); // Longitude of ascending node
	const double epsilon = AngleHelper::degreeToWrapRadian(23 + (26 + Polynomial::deg3(T, 21.448, 46.8150, 0.00059, -0.001813) / 60) / 60 +
														   0.00256 * std::cos(Omega)); // Obliquity of the ecliptic

	const double lon = AngleHelper::wrapRadian(std::atan2(
	  m_data.rightAscension.sin() * std::cos(epsilon) + m_data.declination.tan() * std::sin(epsilon), m_data.rightAscension.cos()));
	const double lat =
	  std::asin(m_data.declination.sin() * std::cos(epsilon) - m_data.declination.cos() * std::sin(epsilon) * m_data.rightAscension.sin());
	return EclipticSpherical(m_epoch, EclipticSphericalPosition{Radian{lon}, Radian{lat}, m_data.distance});
}

inline EclipticCartesian EclipticSpherical::toEclipticCartesian() const {
	return EclipticCartesian(m_epoch, m_data.distance * Eigen::Vector3d{m_data.ecliptic_longitude.cos() * m_data.ecliptic_latitude.cos(),
																		m_data.ecliptic_longitude.sin() * m_data.ecliptic_latitude.cos(),
																		m_data.ecliptic_latitude.sin()});
}

inline Eci EclipticSpherical::toEci() const {
	return toEclipticCartesian().toEci();
}

inline EclipticSpherical EclipticCartesian::toEclipticSpherical() const {
	const double lon = AngleHelper::wrapRadian(std::atan2(m_data.y(), m_data.x()));
	const double lat = std::asin(m_data.z() / m_data.norm());
	return EclipticSpherical(m_epoch, EclipticSphericalPosition{Radian{lon}, Radian{lat}, m_data.norm()});
}

inline Eci EclipticCartesian::toEci() const {
	const double T = (m_epoch.j2000() + m_epoch.deltaT().totalDays()) / constant::jd_century;
	const double Omega = AngleHelper::degreeToWrapRadian(125.04 - 1934.136 * T); // Longitude of ascending node
	const double epsilon = AngleHelper::degreeToWrapRadian(23 + (26 + Polynomial::deg3(T, 21.448, 46.8150, 0.00059, -0.001813) / 60) / 60 +
														   0.00256 * std::cos(Omega)); // Obliquity of the ecliptic

	const double s_eps = std::sin(epsilon), c_eps = std::cos(epsilon);

	return Eci(m_epoch, Eigen::Vector3d{m_data.x(), m_data.y() * c_eps - m_data.z() * s_eps, m_data.y() * s_eps + m_data.z() * c_eps});
}

GEOMAG_NAMESPACE_END