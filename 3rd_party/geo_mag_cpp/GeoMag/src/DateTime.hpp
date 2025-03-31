/**
 * @file DateTime.hpp
 * @author Kaiji Takeuchi (fugu0220@gmail.com)
 * @brief
 * @version 0.1
 * @date 2023-11-30
 *
 * @copyright Copyright (c) 2023
 *
 */

#pragma once

// clang-format off
#include <iomanip>
#include <sstream>
#include <chrono>
#include <iostream>

#include "Essential.hpp"
#include "AngleHelper.hpp"
#include "TimeSpan.hpp"
#include "Polynomial.hpp"
// clang-format on

GEOMAG_NAMESPACE_BEGIN

class DateTime {

  public:
	DateTime() { initialize(1, 1, 1, 0, 0, 0, 0); }

	// /**
	//  * @brief Construct a new Date Time object
	//  *
	//  * @param dt
	//  */
	// DateTime(const DateTime& dt) : m_ticks(dt.m_ticks) {}

	/**
	 * @brief Construct a new Date Time object
	 *
	 * @param year 年
	 * @param month 月
	 * @param day 日
	 * @param hour 時
	 * @param minute 分
	 * @param second 秒
	 * @param microsecond マイクロ秒
	 */

	DateTime(int year, int month, int day, int hour, int minute, int second, int microsecond) {
		initialize(year, month, day, hour, minute, second, microsecond);
	}

	/**
	 * @brief Construct a new Date Time object
	 *
	 * @param year 年
	 * @param month 月
	 * @param day 日
	 * @param hour 時
	 * @param minute 分
	 * @param second 秒
	 */
	DateTime(int year, int month, int day, int hour, int minute, int second) : DateTime(year, month, day, hour, minute, second, 0) {}

	/**
	 * @brief Construct a new Date Time object
	 *
	 * @param year 年
	 * @param month 月
	 * @param day 日
	 * @param hour 時
	 * @param minute 分
	 * @param second 秒
	 */
	DateTime(int year, int month, int day, int hour, int minute, double second)
	  : DateTime(year, month, day, hour, minute, static_cast<int>(second),
				 static_cast<int>((second - static_cast<int>(second)) * 1000000)) {}

	/**
	 * @brief Construct a new Date Time object
	 *
	 * @param year 年
	 * @param day_of_year 1年の中での日付
	 */
	DateTime(int year, double day_of_year) { m_ticks = TimeSpan(absoluteDay(year, day_of_year) * constant::ticks_per_day).ticks(); }

	/**
	 * @brief Construct a new Date Time object
	 *
	 * @param date_time ISO8601形式の日付文字列
	 */
	DateTime(const std::string& date_time) { initialize(date_time); }

	/**
	 * @brief Construct a new Date Time object
	 *
	 * @param ticks ティック数
	 */
	DateTime(std::int64_t ticks) : m_ticks(ticks) {}

	/**
	 * @brief 年成分を取得する
	 * @return int 年成分 [year]
	 */
	int year() const {
		int year, month, day;
		pushDate(year, month, day);
		return year;
	}

	/**
	 * @brief 月成分を取得する
	 * @return int 月成分 [month]
	 */
	int month() const {
		int year, month, day;
		pushDate(year, month, day);
		return month;
	}

	/**
	 * @brief 日成分を取得する
	 * @return int 日成分 [day]
	 */
	int day() const {
		int year, month, day;
		pushDate(year, month, day);
		return day;
	}

	/**
	 * @brief 時成分を取得する
	 * @return int 時成分 [hour]
	 */
	int hour() const { return static_cast<int>(m_ticks % constant::ticks_per_day / constant::ticks_per_hour); }

	/**
	 * @brief 分成分を取得する
	 * @return int 分成分 [minute]
	 */
	int minute() const { return static_cast<int>(m_ticks % constant::ticks_per_hour / constant::ticks_per_minute); }

	/**
	 * @brief 秒成分を取得する
	 * @return int 秒成分 [second]
	 */
	int second() const { return static_cast<int>(m_ticks % constant::ticks_per_minute / constant::ticks_per_second); }

	/**
	 * @brief マイクロ秒成分を取得する
	 * @return int マイクロ秒成分 [microsecond]
	 */
	int microsecond() const { return static_cast<int>(m_ticks % constant::ticks_per_second / constant::ticks_per_microsecond); }

	/**
	 * @brief ティック数を取得する
	 *
	 * @return std::int64_t ティック数
	 */
	std::int64_t ticks() const { return m_ticks; }

	/**
	 * @brief ユリウス日を取得する
	 *
	 * @return double ユリウス日 [day]
	 */
	auto julianDay() const -> double { return TimeSpan(m_ticks).totalDays() + constant::jd_at_gc_era; }

	/**
	 * @brief 修正ユリウス日を取得する
	 *
	 * @return double 修正ユリウス日 [day]
	 */
	auto modifiedJulianDay() const -> double { return julianDay() - constant::jd_at_mjd_epoch; }

	/**
	 * @brief J2000.0からの経過時間を取得する
	 *
	 * @return double J2000.0からの経過時間 [day]
	 */
	auto j2000() const -> double { return julianDay() - constant::jd_at_j2000_epoch; }

	/**
	 * @brief Unixエポックからの経過時間を取得する
	 *
	 * @return double Unixエポックからの経過時間 [s]
	 */
	auto unixTime() const -> double { return (m_ticks - constant::ticks_at_unix_epoch) / static_cast<double>(constant::ticks_per_second); }

	/**
	 * @brief グレゴリオ暦での通算年数を取得する
	 *
	 * @return 通算年数
	 */
	auto fractionalYears() const -> double {
		int year, month, day;
		pushDate(year, month, day);
		long time_part_ticks = m_ticks - absoluteDay(year, month, day) * constant::ticks_per_day;
		double days = dayOfYear(year, month, day) + time_part_ticks / static_cast<double>(constant::ticks_per_day);
		return (double)year + (days - 1) / (isLeapYear(year) ? constant::days_per_leap_year : constant::days_per_nonleap_year);
	}

	/**
	 * @brief 現在時刻を取得する
	 *
	 * @return DateTime
	 */
	static auto now() -> DateTime {
		auto now = std::chrono::system_clock::now();
		auto now_time = std::chrono::system_clock::to_time_t(now);
		auto now_tm = *std::gmtime(&now_time);
		return DateTime(now_tm.tm_year + 1900, now_tm.tm_mon + 1, now_tm.tm_mday, now_tm.tm_hour, now_tm.tm_min, now_tm.tm_sec);
	}

	/**
	 * @brief グリニッジ恒星時を取得する
	 *
	 * @return Angle グリニッジ恒星時
	 */
	auto greenwichSiderealTime() const -> Angle {
		// julian date of previous midnight
		double jd = julianDay();											   // julian date
		double jd0 = std::floor(jd + 0.5) - 0.5;							   // julian date of previous midnight
		double t = (jd0 - constant::jd_at_j2000_epoch) / constant::jd_century; // julian centuries since epoch
		double jdf = jd - jd0;												   // fraction
		double gt = 24110.54841 + t * (8640184.812866 + t * (0.093104 - t * 6.2E-6));
		gt += jdf * 1.00273790935 * constant::seconds_per_day;

		return Radian{AngleHelper::degreeToWrapRadian(gt / 240.0)};
	}

	/**
	 * @brief 地方恒星時を取得する
	 *
	 * @param longitude 経度
	 * @return Angle 地方恒星時
	 */
	auto localSiderealTime(const Angle& longitude) const -> Angle { return greenwichSiderealTime() + longitude; }

	/**
	 * @brief ΔTを取得する
	 * @note https://eclipse.gsfc.nasa.gov/SEhelp/deltatpoly2004.html
	 * @return auto
	 */
	auto deltaT() const -> TimeSpan {
		const int years = year();
		double y = (double)years + ((double)month() - 0.5) / 12;

		if (years < -500) {
			return TimeSpan(Polynomial::deg2((y - 1820) / 100, -20, 0, 32), TimeUnit::Seconds);
		} else if (band(years, -500, 500)) {
			return TimeSpan(Polynomial::deg6(y / 100, 10583.6, -1014.41, 33.78311, -5.952053, -0.1798452, 0.022174192, 0.0090316521),
							TimeUnit::Seconds);
		} else if (band(years, 500, 1600)) {
			return TimeSpan(Polynomial::deg6((y - 1000) / 100, 1574.2, -556.01, 71.23472, 0.319781, -0.8503463, -0.005050998, 0.0083572073),
							TimeUnit::Seconds);
		} else if (band(years, 1600, 1700)) {
			return TimeSpan(Polynomial::deg3(y - 1600, 120, -0.9808, -0.01532, 1.0 / 7129.0), TimeUnit::Seconds);
		} else if (band(years, 1700, 1800)) {
			return TimeSpan(Polynomial::deg4(y - 1700, 8.83, 0.1603, -0.0059285, 0.00013336, -1.0 / 1174000.0), TimeUnit::Seconds);
		} else if (band(years, 1800, 1860)) {
			return TimeSpan(
			  Polynomial::deg7(y - 1800, 13.72, -0.332447, 0.0068612, 0.0041116, -0.00037436, 0.0000121272, -0.0000001699, 0.000000000875),
			  TimeUnit::Seconds);
		} else if (band(years, 1860, 1900)) {
			return TimeSpan(Polynomial::deg5(y - 1860, 7.62, 0.5737, -0.251754, 0.01680668, -0.0004473624, 1.0 / 233174.0),
							TimeUnit::Seconds);
		} else if (band(years, 1900, 1920)) {
			return TimeSpan(Polynomial::deg4(y - 1900, -2.79, 1.494119, -0.0598939, 0.0061966, -0.000197), TimeUnit::Seconds);
		} else if (band(years, 1920, 1941)) {
			return TimeSpan(Polynomial::deg3(y - 1920, 21.20, 0.84493, -0.076100, 0.0020936), TimeUnit::Seconds);
		} else if (band(years, 1941, 1961)) {
			return TimeSpan(Polynomial::deg3(y - 1950, 29.07, 0.407, -1.0 / 233.0, 1.0 / 2547.0), TimeUnit::Seconds);
		} else if (band(years, 1961, 1986)) {
			return TimeSpan(Polynomial::deg3(y - 1975, 45.45, 1.067, -1.0 / 260.0, -1.0 / 718.0), TimeUnit::Seconds);
		} else if (band(years, 1986, 2005)) {
			return TimeSpan(Polynomial::deg5(y - 2000, 63.86, 0.3345, -0.060374, 0.0017275, 0.000651814, 0.00002373599), TimeUnit::Seconds);
		} else if (band(years, 2005, 2050)) {
			return TimeSpan(Polynomial::deg2(y - 2000, 62.92, 0.32217, 0.005589), TimeUnit::Seconds);
		} else if (band(years, 2050, 2150)) {
			return TimeSpan(Polynomial::deg2((y - 1820) / 100, -20 - 0.5628 * (2150 - y), 0, 32), TimeUnit::Seconds);
		} else {
			return TimeSpan(Polynomial::deg2((y - 1820) / 100, -20, 0, 32), TimeUnit::Seconds);
		}
	}

	/**
	 * @brief 均時差を取得する
	 * @param delta_time ΔT
	 * @remark (平均 7 sec程の誤差あり)
	 *
	 */
	auto equationOfTime(const TimeSpan delta_time) const -> Angle {
		const double T = (j2000() + delta_time.totalDays()) / constant::jd_century; // Julian centuries since J2000
		const double L0 = AngleHelper::degreeToWrapRadian(Polynomial::deg2(T, 280.46646, 36000.76983, 0.0003032)); // Mean longitude
		const double M = AngleHelper::degreeToWrapRadian(Polynomial::deg2(T, 357.52911, 35999.05029, -0.0001537)); // Mean anomaly
		const double C =
		  AngleHelper::degreeToWrapRadian((Polynomial::deg2(T, 1.914602, 0.004817, 0.000014) * std::sin(M) +
										   (0.019993 - T * 0.000101) * std::sin(2 * M) + 0.000289 * std::sin(3 * M))); // Equation of center
		const double Lt = AngleHelper::wrapRadian(L0 + C);													   // True longitude
		const double omega = AngleHelper::degreeToWrapRadian(125.04 - 1934.136 * T);		  // Longitude of ascending node
		const double La =
		  AngleHelper::wrapRadian(Lt - AngleHelper::degreeToRadian(0.00569 - 0.00478 * std::sin(omega))); // Apparent longitude

		return Degree{-1.91466647 * std::sin(M) - 0.019994643 * std::sin(2 * M) + 2.466 * std::sin(2 * La) - 0.0053 * sin(4 * La)};
	}

	/**
	 * @brief 均時差を取得する
	 * @remark (平均 7 sec程の誤差あり)
	 *
	 */
	auto equationOfTime() -> Angle { return equationOfTime(deltaT()); }

	/**
	 * @brief グリニッジ太陽時を取得する
	 *
	 * @param delta_time ΔT
	 * @return Angle グリニッジ太陽時
	 */
	auto greenwichSolarTime(const TimeSpan delta_time) const -> Angle {
		return HourAngle{secondsOfDay() / constant::seconds_per_hour + equationOfTime(delta_time).hours()}.normalized();
	};

	/**
	 * @brief グリニッジ太陽時を取得する
	 *
	 * @return Angle グリニッジ太陽時
	 */
	auto greenwichSolarTime() -> Angle { return greenwichSolarTime(deltaT()); }

	/**
	 * @brief 地方太陽時を取得する
	 *
	 * @param longitude 経度
	 * @param delta_time ΔT
	 * @return Angle 地方太陽時
	 */
	auto localSolarTime(const Angle& longitude, const TimeSpan delta_time) const -> Angle {
		return (greenwichSolarTime(delta_time) + longitude).normalized();
	}

	/**
	 * @brief 地方太陽時を取得する
	 *
	 * @param longitude 経度
	 * @return Angle 地方太陽時
	 */
	auto localSolarTime(const Angle& longitude) -> Angle { return localSolarTime(longitude, deltaT()); }

	/**
	 * @brief ISO8601形式文字列に変換する
	 *
	 * @return std::string ISO8601形式文字列
	 */
	auto toString() const -> std::string {
		std::stringstream ss;
		int year, month, day;
		pushDate(year, month, day);
		ss << std::setfill('0') << std::setw(4) << year << "-" << std::setw(2) << month << "-" << std::setw(2) << day << "T" << std::setw(2)
		   << hour() << ":" << std::setw(2) << minute() << ":" << std::setw(2) << second() << "." << std::setw(6) << microsecond() << "Z";
		return ss.str();
	}

	auto add(std::int64_t ticks) -> DateTime { return DateTime(m_ticks + ticks); }

	auto add(const TimeSpan& ts) -> DateTime { return DateTime(m_ticks + ts.ticks()); }

	auto addYears(const int years) -> DateTime { return addMonths(years * 12); }

	auto addMonths(const int months) -> DateTime {
		int year, month, day;
		pushDate(year, month, day);

		month += months % 12;
		year += months / 12;

		if (month < 1) {
			month += 12;
			--year;
		} else if (month > 12) {
			month -= 12;
			++year;
		}

		day = std::min(day, constant::dyas_in_mounth[isLeapYear(year)][month]);

		return DateTime(year, month, day, 0, 0, 0).add(timeOfDay());
	}

	auto addDays(const double days) -> DateTime { return addMicroseconds(days * constant::microseconds_per_day); }

	auto addHours(const double hours) -> DateTime { return addMicroseconds(hours * constant::microseconds_per_hour); }

	auto addMinutes(const double minutes) -> DateTime { return addMicroseconds(minutes * constant::microseconds_per_minute); }

	auto addSeconds(const double seconds) -> DateTime { return addMicroseconds(seconds * constant::microseconds_per_second); }

	auto addMicroseconds(const double microseconds) -> DateTime {
		return addTicks(static_cast<std::int64_t>(microseconds * constant::ticks_per_microsecond));
	}

	auto addTicks(const std::int64_t ticks) -> DateTime { return DateTime{m_ticks + ticks}; }

	friend auto operator<<(std::ostream& os, const DateTime& dt) -> std::ostream& { return os << dt.toString(); }

	int dayOfYear() const {
		int year, month, day;
		pushDate(year, month, day);
		return dayOfYear(year, month, day);
	}

	double secondsOfDay() const { return TimeSpan(m_ticks % constant::ticks_per_day).totalSeconds(); }

	static DateTime max() { return DateTime(std::numeric_limits<std::int64_t>::max()); }

	static DateTime min() { return DateTime(0); }

  private:
	std::int64_t m_ticks;

	/**
	 * @brief 閏年判定
	 *
	 * @param year 年
	 * @return true 閏年
	 * @return false 平年
	 */
	auto isLeapYear(int year) const -> bool { return (year % 4 == 0 && year % 100 != 0) || year % 400 == 0; }

	/**
	 * @brief 年の範囲チェック
	 *
	 * @param year 年
	 * @return true Pass
	 * @return false NG
	 */
	auto validateYearRange(int year) const -> bool { return year >= 1 && year <= 9999; }

	/**
	 * @brief 月の範囲チェック
	 *
	 * @param month 月
	 * @return true Pass
	 * @return false NG
	 */
	auto validateMonthRange(int month) const -> bool { return month >= 1 && month <= 12; }

	/**
	 * @brief 日付の範囲チェック
	 *
	 * @param day 日
	 * @return true Pass
	 * @return false NG
	 */
	auto validateDate(int year, int month, int day) const -> bool {
		if (!validateYearRange(year)) {
			return false;
		}
		if (!validateMonthRange(month)) {
			return false;
		}
		if (day > constant::dyas_in_mounth[isLeapYear(year)][month]) {
			return false;
		}
		return true;
	}

	/**
	 * @brief 時の範囲チェック
	 *
	 * @param hour 時
	 * @return true Pass
	 * @return false NG
	 */
	auto validateHourRange(int hour) const -> bool { return hour >= 0 && hour <= 23; }

	/**
	 * @brief 分の範囲チェック
	 *
	 * @param minute 分
	 * @return true Pass
	 * @return false NG
	 */
	auto validateMinuteRange(int minute) const -> bool { return minute >= 0 && minute <= 59; }

	/**
	 * @brief 秒の範囲チェック
	 *
	 * @param second 秒
	 * @return true Pass
	 * @return false NG
	 */
	auto validateSecondRange(int second) const -> bool { return second >= 0 && second <= 59; }

	/**
	 * @brief マイクロ秒の範囲チェック
	 *
	 * @param microsecond マイクロ秒
	 * @return true Pass
	 * @return false NG
	 */
	auto validateMicrosecondRange(int microsecond) const -> bool { return microsecond >= 0 && microsecond <= 999999; }

	/**
	 * @brief 時間の範囲チェック
	 *
	 * @param days 日
	 * @param hours 時
	 * @param minutes 分
	 * @param seconds 秒
	 * @param microseconds マイクロ秒
	 *
	 * @return true Pass
	 * @return false NG
	 */
	auto validateTime(int hour, int minute, int second, int microsecond) const -> bool {
		if (!validateHourRange(hour)) {
			return false;
		}
		if (!validateMinuteRange(minute)) {
			return false;
		}
		if (!validateSecondRange(second)) {
			return false;
		}
		if (!validateMicrosecondRange(microsecond)) {
			return false;
		}
		return true;
	}

	auto dayOfYear(int year, int month, int day) const -> int {
		if (!validateDate(year, month, day)) {
			throw DateTimeException("Date range is invalid", DateTimeException::InvalidDate);
		}
		return day + constant::lap_days_in_month[isLeapYear(year)][month];
	}

	auto absoluteDay(int year, int month, int day) const -> int {
		const int prev_year = year - 1;
		return dayOfYear(year, month, day) - 1 + prev_year * constant::days_per_nonleap_year + prev_year / 4 - prev_year / 100 +
			   prev_year / 400;
	}

	auto absoluteDay(int year, double day_of_year) const -> double {
		const int prev_year = year - 1;
		return static_cast<double>(prev_year * constant::days_per_nonleap_year + prev_year / 4 - prev_year / 100 + prev_year / 400) +
			   day_of_year - 1.0;
	}

	auto timeOfDay() const -> TimeSpan { return TimeSpan(m_ticks % constant::ticks_per_day); }

	auto initialize(int year, int month, int day, int hour, int minute, int second, int microsecond) -> void {
		if (!validateDate(year, month, day)) {
			throw DateTimeException("Date range is invalid", DateTimeException::InvalidDate);
		}
		if (!validateTime(hour, minute, second, microsecond)) {
			throw DateTimeException("Time range is invalid", DateTimeException::InvalidTime);
		}

		m_ticks = TimeSpan(absoluteDay(year, month, day), hour, minute, second, microsecond).ticks();
	}

	auto initialize(const std::string& date_time) -> void {
		std::int32_t year, month, day, hour, minute, second, microsecond;
		year = iso8601BlocktoInt(date_time, 0, 4);
		month = iso8601BlocktoInt(date_time, 5, 7);
		day = iso8601BlocktoInt(date_time, 8, 10);
		if (date_time.length() <= 10) {
			initialize(year, month, day, 0, 0, 0, 0);
		} else {
			hour = iso8601BlocktoInt(date_time, 11, 13);
			minute = iso8601BlocktoInt(date_time, 14, 16);

			std::string sec_tz = date_time.substr(17);

			// タイムゾーンの位置を探す
			std::size_t tz_pos = 0;
			while (tz_pos < sec_tz.length()) {
				if (sec_tz[tz_pos] == 'Z' || sec_tz[tz_pos] == '+' || sec_tz[tz_pos] == '-') {
					break;
				}
				tz_pos++;
			}

			iso8601BlocktoDecimal(sec_tz, 0, tz_pos - 1, second, microsecond);

			if (tz_pos == sec_tz.length()) {
				initialize(year, month, day, hour, minute, second, microsecond);
			} else {
				std::string tz = sec_tz.substr(tz_pos);
				if (tz[0] == 'Z' || tz == "+00:00" || tz == "-00:00" || tz == "UTC" || tz == "GMT") {
					initialize(year, month, day, hour, minute, second, microsecond);
				} else {
					int tz_hour = iso8601BlocktoInt(tz, 1, 3);
					int tz_minute = iso8601BlocktoInt(tz, 4, 6);
					initialize(year, month, day, hour, minute, second, microsecond);
					if (tz[0] == '-') {
						m_ticks += TimeSpan(tz_hour, tz_minute, 0).ticks();
					} else {
						m_ticks -= TimeSpan(tz_hour, tz_minute, 0).ticks();
					}
				}
			}
		}
	}

	auto iso8601BlocktoInt(const std::string& str, int begin, int end) const -> int {
		int value = 0;
		for (int i = begin; i < end; i++) {
			if (str[i] < '0' || str[i] > '9') {
				throw DateTimeException("Invalid integer string", DateTimeException::InvalidIso8601Format);
			}
			value = value * 10 + (str[i] - '0');
		}
		return value;
	}

	auto iso8601BlocktoDecimal(const std::string& str, int begin, int end, int& integer, int& decimal) const -> double {
		double ret = 0;
		std::int32_t div = 1;
		std::int32_t decimal_point_pos = begin;

		while (decimal_point_pos <= end) {
			if (str[decimal_point_pos] == '.') {
				break;
			}
			decimal_point_pos++;
		}

		integer = iso8601BlocktoInt(str, begin, decimal_point_pos);
		decimal = 0;

		if (decimal_point_pos < end) {
			decimal = iso8601BlocktoInt(str, decimal_point_pos + 1, end + 1);
			for (int i = decimal_point_pos + 1; i <= end; i++) {
				div *= 10;
			}
		}

		ret = static_cast<double>(integer) + static_cast<double>(decimal) / static_cast<double>(div);

		decimal = decimal * (1000'000 / div);

		return ret;
	}

	auto pushDate(int& year, int& month, int& day) const -> void {
		int total_days = static_cast<int>(m_ticks / constant::ticks_per_day);

		// 年
		{
			// 4世紀単位の数
			int num_4cent = total_days / 146097;
			total_days -= num_4cent * 146097;

			// 1世紀単位の数
			int num_1cent = total_days / 36524;
			if (num_1cent == 4) {
				// 閏世紀末日
				num_1cent = 3;
			}
			total_days -= num_1cent * 36524;

			// 4年単位の数
			int num_4year = total_days / 1461;
			total_days -= num_4year * 1461;

			// 1年単位の数
			int num_year = total_days / 365;
			if (num_year == 4) {
				/*
				 * 閏年末日
				 */
				num_year = 3;
			}
			total_days -= num_year * 365;

			year = (num_4cent * 400) + (num_1cent * 100) + (num_4year * 4) + num_year + 1;
		}

		// 月
		{
			const auto& dyas_in_mounth = constant::dyas_in_mounth[isLeapYear(year)];
			month = 1;
			while (total_days >= dyas_in_mounth[month] && month <= 12) {
				total_days -= dyas_in_mounth[month++];
			}
		}

		// 日
		day = total_days + 1;
	}

	auto band(const double x, const double l, const double r) const -> bool { return x >= l && x < r; }

	friend auto operator+(const DateTime& dt, TimeSpan ts) -> DateTime { return DateTime(dt.ticks() + ts.ticks()); }

	friend auto operator-(const DateTime& dt, const TimeSpan& ts) -> DateTime { return DateTime(dt.ticks() - ts.ticks()); }

	friend auto operator-(const DateTime& dt1, const DateTime& dt2) -> TimeSpan { return TimeSpan(dt1.ticks() - dt2.ticks()); }

	friend auto operator+=(DateTime& dt, const TimeSpan& ts) -> DateTime& {
		dt = dt + ts;
		return dt;
	}

	friend auto operator-=(DateTime& dt, const TimeSpan& ts) -> DateTime& {
		dt = dt - ts;
		return dt;
	}

	friend auto operator==(const DateTime& dt1, const DateTime& dt2) -> bool { return dt1.ticks() == dt2.ticks(); }

	friend auto operator>(const DateTime& dt1, const DateTime& dt2) -> bool { return dt1.ticks() > dt2.ticks(); }

	friend auto operator>=(const DateTime& dt1, const DateTime& dt2) -> bool { return dt1.ticks() >= dt2.ticks(); }

	friend auto operator!=(const DateTime& dt1, const DateTime& dt2) -> bool { return dt1.ticks() != dt2.ticks(); }

	friend auto operator<(const DateTime& dt1, const DateTime& dt2) -> bool { return dt1.ticks() < dt2.ticks(); }

	friend auto operator<=(const DateTime& dt1, const DateTime& dt2) -> bool { return dt1.ticks() <= dt2.ticks(); }
};

GEOMAG_NAMESPACE_END