/**
 * @file AngleHelper.hpp
 * @author Kaiji Takeuchi (fugu0220@gmail.com)
 * @brief
 * @version 0.1
 * @date 2023-12-01
 *
 * @copyright Copyright (c) 2023
 *
 */

#pragma once

#include <cmath>
#include <iomanip>
#include <sstream>

#include "Essential.hpp"

GEOMAG_NAMESPACE_BEGIN

/**
 * @brief 角度の単位を表す列挙型
 *
 */
enum class AngleUnit {
	Degree,
	Radian,
	Hour,
	Arcmin,
	Arcsec,
	Hms,
	Dms,
};

struct AngleHelper {
	static auto degreeToRadian(double degree) -> double { return degree * constant::pi / 180.0; }

	static auto radianToDegree(double radian) -> double { return radian * 180.0 / constant::pi; }

	static auto degreeToHour(double degree) -> double { return degree / 15.0; }

	static auto hourToDegree(double hour) -> double { return hour * 15.0; }

	static auto radianToHour(double radian) -> double { return radian * 12.0 / constant::pi; }

	static auto hourToRadian(double hour) -> double { return hour * constant::pi / 12.0; }

	static auto degreeToArcmin(double degree) -> double { return degree * 60.0; }

	static auto arcminToDegree(double arcmin) -> double { return arcmin / 60.0; }

	static auto radianToArcmin(double radian) -> double { return radian * 60.0 * 180.0 / constant::pi; }

	static auto arcminToRadian(double arcmin) -> double { return arcmin * constant::pi / 180.0 / 60.0; }

	static auto degreeToArcsec(double degree) -> double { return degree * 3600.0; }

	static auto arcsecToDegree(double arcsec) -> double { return arcsec / 3600.0; }

	static auto radianToArcsec(double radian) -> double { return radian * 3600.0 * 180.0 / constant::pi; }

	static auto arcsecToRadian(double arcsec) -> double { return arcsec * constant::pi / 180.0 / 3600.0; }

	static auto wrapDegree(double degree) -> double {
		double wrapped_degree = std::fmod(degree, 360.0);
		if (wrapped_degree < 0.0) {
			wrapped_degree += 360.0;
		}
		return wrapped_degree;
	}

	static auto degreeToWrapRadian(double degree) -> double { return wrapRadian(degreeToRadian(degree)); }

	static auto wrapRadian(double radian) -> double {
		double wrapped_radian = std::fmod(radian, constant::pi2);
		if (wrapped_radian < 0.0) {
			wrapped_radian += constant::pi2;
		}
		return wrapped_radian;
	}

	static auto radianToWrapDegree(double radian) -> double { return wrapDegree(radianToDegree(radian)); }
};

/**
 * @brief DMS形式の角度を表す構造体
 *
 */
struct DmsAngle {
	int degree;	   // 度
	int arcmin;	   // 分
	double arcsec; // 秒

	DmsAngle(int d, int m, double s) : degree(d), arcmin(m), arcsec(s) {}
};

/**
 * @brief HMS形式の角度を表す構造体
 *
 */
struct HmsAngle {
	int hour;	   // 時
	int minute;	   // 分
	double second; // 秒

	HmsAngle(int h, int m, double s) : hour(h), minute(m), second(s) {}
};

/**
 * @brief 角度を表すクラス
 *
 */
class Angle {
  public:
	/**
	 * @brief Construct a new Angle object
	 *
	 */
	Angle() : m_angle_radian(0.0) {}

	/**
	 * @brief Construct a new Angle object
	 *
	 * @param angle 角度
	 * @param unit 角度の単位
	 */
	Angle(double angle, AngleUnit unit) : m_angle_radian(0.0) { setAngle(angle, unit); }

	/**
	 * @brief Construct a new Angle object
	 *
	 * @param hms HMS形式の角度
	 */
	Angle(const HmsAngle& hms) : m_angle_radian(0.0) { setAngle(hms); }

	/**
	 * @brief Construct a new Angle object
	 *
	 * @param dms DMS形式の角度
	 */
	Angle(const DmsAngle& dms) : m_angle_radian(0.0) { setAngle(dms); }

	/**
	 * @brief 弧度法での角度を返す
	 *
	 */
	auto radians() const -> double { return m_angle_radian; }

	/**
	 * @brief 度数法での角度を返す
	 *
	 */
	auto degrees() const -> double { return AngleHelper::radianToDegree(m_angle_radian); }

	/**
	 * @brief 時角での角度を返す
	 *
	 */
	auto hours() const -> double { return AngleHelper::radianToHour(m_angle_radian); }

	/**
	 * @brief 分角での角度を返す
	 *
	 * @return double
	 */
	auto arcmins() const -> double { return AngleHelper::radianToArcmin(m_angle_radian); }

	/**
	 * @brief 秒角での角度を返す
	 *
	 * @return double
	 */
	auto arcsecs() const -> double { return AngleHelper::radianToArcsec(m_angle_radian); }

	/**
	 * @brief DMS形式での角度を返す
	 *
	 * @return Dms
	 */
	auto dms() const -> DmsAngle {
		double deg, min, sec;
		sec = std::modf(std::modf(AngleHelper::wrapDegree(degrees()), &deg) * 60.0, &min) * 60.0;
		return {static_cast<int>(deg), static_cast<int>(min), sec};
	}

	/**
	 * @brief HMS形式での角度を返す
	 *
	 * @return Hms
	 */
	auto hms() const -> HmsAngle {
		double h, min, sec;
		sec = std::modf(std::modf(AngleHelper::wrapDegree(degrees()) / 15.0, &h) * 60.0, &min) * 60.0;
		return {static_cast<int>(h), static_cast<int>(min), sec};
	}

	/**
	 * @brief 角度を設定する
	 *
	 * @param angle 角度
	 * @param unit 角度の単位
	 */
	auto setAngle(double angle, AngleUnit unit) -> void {
		switch (unit) {
			case AngleUnit::Degree: m_angle_radian = AngleHelper::degreeToRadian(angle); break;
			case AngleUnit::Radian: m_angle_radian = angle; break;
			case AngleUnit::Hour: m_angle_radian = AngleHelper::hourToRadian(angle); break;
			case AngleUnit::Arcmin: m_angle_radian = AngleHelper::arcminToRadian(angle); break;
			case AngleUnit::Arcsec: m_angle_radian = AngleHelper::arcsecToRadian(angle); break;
			default: break;
		}
	}

	/**
	 * @brief 角度を設定する
	 *
	 * @param hms HMS形式の角度
	 */
	auto setAngle(const HmsAngle& hms) -> void {
		m_angle_radian = AngleHelper::hourToRadian(hms.hour + hms.minute / 60.0 + hms.second / 3600.0);
	}

	/**
	 * @brief 角度を設定する
	 *
	 * @param dms DMS形式の角度
	 */
	auto setAngle(const DmsAngle& dms) -> void {
		m_angle_radian = AngleHelper::degreeToRadian(dms.degree + dms.arcmin / 60.0 + dms.arcsec / 3600.0);
	}

	/**
	 * @brief 0 <= θ < 2π の範囲で正規化する
	 *
	 */
	auto normalize() -> void { m_angle_radian = AngleHelper::wrapRadian(m_angle_radian); }

	/**
	 * @brief -π <= θ < π の範囲で正規化する
	 *
	 * @return Angle
	 */
	auto semiNormalize() -> void {
		normalize();
		if (m_angle_radian > constant::pi) {
			m_angle_radian -= constant::pi2;
		}
	}

	/**
	 * @brief 0 <= θ < 2π の範囲で正規化した角度を返す
	 *
	 * @return Angle
	 */
	auto normalized() const -> Angle { return Angle(AngleHelper::wrapRadian(m_angle_radian), AngleUnit::Radian); }

	/**
	 * @brief -π <= θ < π の範囲で正規化した角度を返す
	 *
	 * @return Angle
	 */
	auto semiNormalized() const -> Angle {
		auto angle = normalized();
		if (angle.m_angle_radian > constant::pi) {
			angle.m_angle_radian -= constant::pi2;
		}
		return angle;
	}

	/**
	 * @brief 角度を文字列で返す
	 *
	 * @param unit 角度の単位
	 * @return std::string
	 */
	auto toString(AngleUnit unit = AngleUnit::Degree, const int precision = 4) const -> std::string {
		std::stringstream ss;
		ss << std::fixed << std::setprecision(precision);

		switch (unit) {
			case AngleUnit::Degree: ss << degrees() << "°"; break;
			case AngleUnit::Radian: ss << radians() << " rad"; break;
			case AngleUnit::Hour: ss << hours() << " h"; break;
			case AngleUnit::Arcmin: ss << arcmins() << "'"; break;
			case AngleUnit::Arcsec: ss << arcsecs() << R"(")"; break;
			case AngleUnit::Hms: {
				auto hms = this->hms();
				ss << hms.hour << "h" << hms.minute << "m" << hms.second << "s";
				break;
			}
			case AngleUnit::Dms: {
				auto dms = this->dms();
				ss << dms.degree << "°" << dms.arcmin << "'" << dms.arcsec << R"(")";
				break;
			}
			default: break;
		}

		return ss.str();
	}

	/**
	 * @brief 正弦を返す
	 *
	 * @return double
	 */
	auto sin() const -> double { return std::sin(m_angle_radian); }

	/**
	 * @brief 余弦を返す
	 *
	 * @return double
	 */
	auto cos() const -> double { return std::cos(m_angle_radian); }

	/**
	 * @brief 正接を返す
	 *
	 * @return double
	 */
	auto tan() const -> double { return std::tan(m_angle_radian); }

	/**
	 * @brief 正弦を返す
	 *
	 * @param angle 角度
	 * @return double
	 */
	static auto sin(const Angle& angle) -> double { return angle.sin(); }

	/**
	 * @brief 余弦を返す
	 *
	 * @param angle 角度
	 * @return double
	 */
	static auto cos(const Angle& angle) -> double { return angle.cos(); }

	/**
	 * @brief 正接を返す
	 *
	 * @param angle 角度
	 * @return double
	 */
	static auto tan(const Angle& angle) -> double { return angle.tan(); }

	/**
	 * @brief 逆正弦を返す
	 *
	 * @return Angle
	 */
	static auto asin(const double val) -> Angle { return Angle(std::asin(val), AngleUnit::Radian); }

	/**
	 * @brief 逆余弦を返す
	 *
	 * @return Angle
	 */
	static auto acos(const double val) -> Angle { return Angle(std::acos(val), AngleUnit::Radian); }

	/**
	 * @brief 逆正接を返す
	 *
	 * @return Angle
	 */
	static auto atan(const double val) -> Angle { return Angle(std::atan(val), AngleUnit::Radian); }

	/**
	 * @brief 逆正接を返す
	 *
	 * @return Angle
	 */
	static auto atan2(const double y, const double x) -> Angle { return Angle(std::atan2(y, x), AngleUnit::Radian); }

	/**
	 * @brief 角度0を返す
	 *
	 * @return Angle
	 */
	static auto zero() -> Angle { return Angle(0.0, AngleUnit::Radian); }

  private:
	double m_angle_radian;

	friend auto operator+(const Angle& lhs, const Angle& rhs) -> Angle {
		return Angle(lhs.m_angle_radian + rhs.m_angle_radian, AngleUnit::Radian);
	}

	friend auto operator-(const Angle& lhs, const Angle& rhs) -> Angle {
		return Angle(lhs.m_angle_radian - rhs.m_angle_radian, AngleUnit::Radian);
	}

	friend auto operator*(const Angle& lhs, double rhs) -> Angle { return Angle(lhs.m_angle_radian * rhs, AngleUnit::Radian); }

	friend auto operator*(double lhs, const Angle& rhs) -> Angle { return Angle(lhs * rhs.m_angle_radian, AngleUnit::Radian); }

	friend auto operator/(const Angle& lhs, double rhs) -> Angle { return Angle(lhs.m_angle_radian / rhs, AngleUnit::Radian); }

	friend auto operator==(const Angle& lhs, const Angle& rhs) -> bool { return lhs.m_angle_radian == rhs.m_angle_radian; }

	friend auto operator!=(const Angle& lhs, const Angle& rhs) -> bool { return lhs.m_angle_radian != rhs.m_angle_radian; }

	friend auto operator<(const Angle& lhs, const Angle& rhs) -> bool { return lhs.m_angle_radian < rhs.m_angle_radian; }

	friend auto operator<=(const Angle& lhs, const Angle& rhs) -> bool { return lhs.m_angle_radian <= rhs.m_angle_radian; }

	friend auto operator>(const Angle& lhs, const Angle& rhs) -> bool { return lhs.m_angle_radian > rhs.m_angle_radian; }

	friend auto operator>=(const Angle& lhs, const Angle& rhs) -> bool { return lhs.m_angle_radian >= rhs.m_angle_radian; }

	friend auto operator<<(std::ostream& os, const Angle& angle) -> std::ostream& {
		os << angle.degrees() << "°";
		return os;
	}

	friend auto operator>>(std::istream& is, Angle& angle) -> std::istream& {
		double value;
		is >> value;
		angle.setAngle(value, AngleUnit::Degree);
		return is;
	}

	friend auto operator+(const Angle& angle) -> Angle { return angle; }

	friend auto operator-(const Angle& angle) -> Angle { return Angle(-angle.m_angle_radian, AngleUnit::Radian); }

	friend auto operator+=(Angle& lhs, const Angle& rhs) -> Angle& {
		lhs.m_angle_radian += rhs.m_angle_radian;
		return lhs;
	}

	friend auto operator-=(Angle& lhs, const Angle& rhs) -> Angle& {
		lhs.m_angle_radian -= rhs.m_angle_radian;
		return lhs;
	}

	friend auto operator*=(Angle& lhs, double rhs) -> Angle& {
		lhs.m_angle_radian *= rhs;
		return lhs;
	}

	friend auto operator/=(Angle& lhs, double rhs) -> Angle& {
		lhs.m_angle_radian /= rhs;
		return lhs;
	}
};

class Degree : public Angle {
  public:
	Degree() : Angle() {}
	Degree(double angle) : Angle(angle, AngleUnit::Degree) {}
};

class Radian : public Angle {
  public:
	Radian() : Angle() {}
	Radian(double angle) : Angle(angle, AngleUnit::Radian) {}
};

class NormalizedAngle : public Angle {
  public:
	NormalizedAngle() : Angle() {}
	NormalizedAngle(double angle) : Angle(constant::pi2 * angle, AngleUnit::Radian) {}
};

class DoyAngle : public Angle {
  public:
	DoyAngle() : Angle() {}
	DoyAngle(double doy) : Angle(constant::pi2 * doy / constant::days_per_nonleap_year, AngleUnit::Radian) {}
	DoyAngle(int year, double doy)
	  : Angle(constant::pi2 * doy / (isLeapYear(year) ? constant::days_per_leap_year : constant::days_per_nonleap_year),
			  AngleUnit::Radian) {}

  private:
	bool isLeapYear(int year) const { return (year % 4 == 0 && year % 100 != 0) || year % 400 == 0; }
};

class HourAngle : public Angle {
  public:
	HourAngle() : Angle() {}
	HourAngle(double angle) : Angle(angle, AngleUnit::Hour) {}
};

class Arcmin : public Angle {
  public:
	Arcmin() : Angle() {}
	Arcmin(double angle) : Angle(angle, AngleUnit::Arcmin) {}
};

class Arcsec : public Angle {
  public:
	Arcsec() : Angle() {}
	Arcsec(double angle) : Angle(angle, AngleUnit::Arcsec) {}
};

class Hms : public Angle {
  public:
	Hms() : Angle() {}
	Hms(const HmsAngle& hms) : Angle(hms) {}
	Hms(int h, int m, double s) : Angle(HmsAngle(h, m, s)) {}
};

class Dms : public Angle {
  public:
	Dms() : Angle() {}
	Dms(const DmsAngle& dms) : Angle(dms) {}
	Dms(int d, int m, double s) : Angle(DmsAngle(d, m, s)) {}
};

GEOMAG_NAMESPACE_END
