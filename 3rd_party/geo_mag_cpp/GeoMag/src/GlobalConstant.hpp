/**
 * @file GlobalConstatnt.hpp
 * @author Kaiji Takeuchi
 * @brief
 * @version 0.1
 * @date 2023-11-27
 *
 * @copyright Copyright (c) 2023
 *
 */

#pragma once

#include <cmath>
#include <cstdint>

#include "Macro.hpp"

GEOMAG_NAMESPACE_BEGIN
namespace constant {
	namespace {
		/* 時間 */
		constexpr std::int64_t ticks_per_day = 86400000000LL;
		constexpr std::int64_t ticks_per_hour = 3600000000LL;
		constexpr std::int64_t ticks_per_minute = 60000000LL;
		constexpr std::int64_t ticks_per_second = 1000000LL;
		constexpr std::int64_t ticks_per_millisecond = 1000LL;
		constexpr std::int64_t ticks_per_microsecond = 1LL;
		constexpr std::int64_t microseconds_per_day = 86400000000LL;
		constexpr std::int64_t microseconds_per_hour = 3600000000LL;
		constexpr std::int64_t microseconds_per_minute = 60000000LL;
		constexpr std::int64_t microseconds_per_second = 1000000LL;
		constexpr std::int64_t microseconds_per_millisecond = 1000LL;
		constexpr std::int64_t seconds_per_day = 86400LL;
		constexpr std::int64_t seconds_per_hour = 3600LL;
		constexpr std::int64_t seconds_per_minute = 60LL;
		constexpr std::int64_t minutes_per_day = 1440LL;
		constexpr std::int64_t minutes_per_hour = 60LL;
		constexpr std::int64_t hours_per_day = 24LL;
		constexpr std::int64_t days_per_week = 7LL;
		constexpr std::int64_t days_per_nonleap_year = 365LL;
		constexpr std::int64_t days_per_leap_year = 366LL;

		/* エポック */
		constexpr std::int64_t ticks_at_unix_epoch = 62135596800000000LL;	// [ticks] at 1970-01-01T00:00:00Z
		constexpr std::int64_t ticks_at_j2000_epoch = 630822816000000000LL; // [ticks] at 2000-01-01T12:00:00Z

		/* ユリウス日 */
		constexpr double jd_at_mjd_epoch = 2400000.5;	// [day] at 1858-11-17T00:00:00Z (JD 2400000.5)
		constexpr double jd_at_j1900_epoch = 2415020.0; // [day] at 1900-01-01T12:00:00Z (JD 2415020.0)
		constexpr double jd_at_j2000_epoch = 2451545.0; // [day] at 2000-01-01T12:00:00Z (JD 2451545.0)
		constexpr double jd_at_unix_epoch = 2440587.5;	// [day] at 1970-01-01T00:00:00Z (JD 2440587.5)
		constexpr double jd_at_gc_era = 1721425.5;		// [day] at 0001-01-01T00:00:00Z (JD 1721425.5)
		constexpr double jd_century = 36525.0;			// [day] 1 century = 36525 days

		/* グレゴリオ暦 */
		const std::int32_t dyas_in_mounth[2][13] = {{
													  // non-leap year
													  0,
													  31, // January
													  28, // February
													  31, // March
													  30, // April
													  31, // May
													  30, // June
													  31, // July
													  31, // August
													  30, // September
													  31, // October
													  30, // November
													  31, // December
													},
													{
													  // leap year
													  0,
													  31, // January
													  29, // February
													  31, // March
													  30, // April
													  31, // May
													  30, // June
													  31, // Julys
													  31, // August
													  30, // September
													  31, // October
													  30, // November
													  31, // December
													}};	  // 月当たりの日数 [day]

		const std::int32_t lap_days_in_month[2][13] = {{
														 // non-leap year
														 0,
														 0,	  // January
														 31,  // February
														 59,  // March
														 90,  // April
														 120, // May
														 151, // June
														 181, // July
														 212, // August
														 243, // September
														 273, // October
														 304, // November
														 334, // December
													   },
													   {
														 // leap year
														 0,
														 0,	  // January
														 31,  // February
														 60,  // March
														 91,  // April
														 121, // May
														 152, // June
														 182, // July
														 213, // August
														 244, // September
														 274, // October
														 305, // November
														 335, // December
													   }};	  // 月当たりの日数 [day]

		/* 数学 */
		constexpr double pi = 3.141592653589793238462643383279502884197169399375105820974944592307816406286;
		constexpr double pi2 = 6.283185307179586476925286766559005768394338798750211641949889184615632812572;
		constexpr double pi_2 = 1.570796326794896619231321691639751442098584699687552910487472296153203203143;
		constexpr double pi_4 = 0.785398163397448309615660845819875721049292349843776455243736148076601601572;
		constexpr double pi_180 = 0.017453292519943295769236907684886127134428718885417254560971914401527859419;
		constexpr double tow_third = 2.0 / 3.0;
		constexpr double thdt = 4.37526908801129966e-3; // 3 * pi / 86400.0

		/* 太陽 */
		constexpr double au = 149597870700; // 天文単位 [m]

		/* 地球の形状と重力 */
		constexpr double ae = 1.0;				   // 地球赤道半径 [normalized]
		constexpr double f = 1.0 / 298.257223563;  // 地球扁平率
		constexpr double xkmper = 6378.137;		   // 地球赤道半径 [km]
		constexpr double mu = 398600.8;			   // 地球中心重力定数 GM [km^3/s^2]
		constexpr double wgs84_a = 6378137.0;	   // WGS84楕円体の長半径 [m]
		constexpr double wgs84_b = 6356752.314245; // WGS84楕円体の短半径 [m]

		/* 摂動係数 */
		constexpr double xke = 0.0743669161331734132; // 60 / sqrt(ae^3/mu)
		constexpr double xj2 = 1.082616e-3;			  // j2項
		constexpr double xj3 = -2.53881e-6;			  // j3項
		constexpr double xj4 = -1.65597e-6;			  // j4項
		constexpr double ck2 = 0.5 * xj2 * ae * ae;
		constexpr double ck4 = -0.375 * xj4 * ae * ae * ae * ae;
		constexpr double a3ovk2 = -xj3 / ck2 * ae * ae * ae; // -j3 / ck2 * ae^3

		/* SGP4密度定数 */
		constexpr double qo = 120.0;
		constexpr double so = 78.0;
		constexpr double qoms2t = 1.88027916e-9;	   // (qo - so) * ae / xkmper) ^ 4
		constexpr double s = ae * (1.0 + so / xkmper); // s = ae*(1+f) = 1.01222928

	} // namespace
} // namespace constant
GEOMAG_NAMESPACE_END