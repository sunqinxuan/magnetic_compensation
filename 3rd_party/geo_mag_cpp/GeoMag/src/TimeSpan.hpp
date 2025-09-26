/**
 * @file TimeSpan.hpp
 * @author Kaiji Takeuchi (fugu0220@gmail.com)
 * @brief 時間の長さを表すクラス
 * @remarks 最小単位はマイクロ秒
 * @version 0.1
 * @date 2023-11-30
 *
 * @copyright Copyright Kaiji Takeuchi(c) 2023
 *
 */

#pragma once

#include "Essential.hpp"

GEOMAG_NAMESPACE_BEGIN

enum class TimeUnit {
	Years,
	Months,
	Days,
	Hours,
	Minutes,
	Seconds,
	Milliseconds,
	Microseconds,
};

/**
 * @brief 時間の長さを表すクラス
 * @remarks 最小単位はマイクロ秒
 */
class TimeSpan {
  public:
	/**
	 * @brief Construct a new Time Span object
	 *
	 * @param ticks ティック数 [us]
	 */
	explicit TimeSpan(std::int64_t ticks) : m_ticks(ticks) {}

	/**
	 * @brief Construct a new Time Span object
	 *
	 * @param hours 時 [h]
	 * @param minutes 分 [min]
	 * @param seconds 秒 [sec]
	 */

	TimeSpan(int hours, int minutes, int seconds) { calculateTicks(0, hours, minutes, seconds, 0); }

	/**
	 * @brief Construct a new Time Span object
	 *
	 * @param days 日 [day]
	 * @param hours 時 [h]
	 * @param minutes 分 [min]
	 * @param seconds 秒 [sec]
	 */
	TimeSpan(int days, int hours, int minutes, int seconds) { calculateTicks(days, hours, minutes, seconds, 0); }

	/**
	 * @brief Construct a new Time Span object
	 *
	 * @param days 日 [day]
	 * @param hours 時 [h]
	 * @param minutes 分 [min]
	 * @param seconds 秒 [sec]
	 * @param microseconds マイクロ秒 [us]
	 */
	TimeSpan(int days, int hours, int minutes, int seconds, int microseconds) {
		calculateTicks(days, hours, minutes, seconds, microseconds);
	}

	/**
	 * @brief Construct a new Time Span object
	 *
	 * @param time 時間 [unit]
	 * @param unit 時間単位
	 */
	TimeSpan(double time, TimeUnit unit) {
		switch (unit) {
			case TimeUnit::Days: m_ticks = static_cast<int64_t>(time * constant::ticks_per_day); break;
			case TimeUnit::Hours: m_ticks = static_cast<int64_t>(time * constant::ticks_per_hour); break;
			case TimeUnit::Minutes: m_ticks = static_cast<int64_t>(time * constant::ticks_per_minute); break;
			case TimeUnit::Seconds: m_ticks = static_cast<int64_t>(time * constant::ticks_per_second); break;
			case TimeUnit::Milliseconds: m_ticks = static_cast<int64_t>(time * constant::ticks_per_millisecond); break;
			case TimeUnit::Microseconds: m_ticks = static_cast<int64_t>(time * constant::ticks_per_microsecond); break;
			default: m_ticks = 0; break;
		}
	}

	/**
	 * @brief 日数を取得する
	 *
	 * @return int 日数 [day]
	 */
	auto days() const -> int { return static_cast<int>(m_ticks / constant::ticks_per_day); }

	/**
	 * @brief 時間を取得する
	 *
	 * @return int 時間数 [h]
	 */
	auto hours() const -> int { return static_cast<int>(m_ticks % constant::ticks_per_day / constant::ticks_per_hour); }

	/**
	 * @brief 分を取得する
	 *
	 * @return int 分数 [min]
	 */
	auto minutes() const -> int { return static_cast<int>(m_ticks % constant::ticks_per_hour / constant::ticks_per_minute); }

	/**
	 * @brief 秒を取得する
	 *
	 * @return int 秒数 [s]
	 */
	auto seconds() const -> int { return static_cast<int>(m_ticks % constant::ticks_per_minute / constant::ticks_per_second); }

	/**
	 * @brief ミリ秒を取得する
	 *
	 * @return int 秒数 [ms]
	 */
	auto milliseconds() const -> int { return static_cast<int>(m_ticks % constant::ticks_per_second / constant::ticks_per_millisecond); }

	/**
	 * @brief マイクロ秒を取得する
	 *
	 * @return int 秒数 [us]
	 */
	auto microseconds() const -> int { return static_cast<int>(m_ticks % constant::ticks_per_second / constant::ticks_per_microsecond); }

	/**
	 * @brief ティック数を取得する
	 *
	 * @return std::uint64_t ティック数
	 */
	auto ticks() const -> std::int64_t { return m_ticks; }

	/**
	 * @brief 経過日数を取得する
	 *
	 * @return double 経過日数 [day]
	 */
	double totalDays() const { return static_cast<double>(m_ticks) / constant::ticks_per_day; }

	/**
	 * @brief 経過時間数を取得する
	 *
	 * @return double 経過時間数 [h]
	 */
	double totalHours() const { return static_cast<double>(m_ticks) / constant::ticks_per_hour; }

	/**
	 * @brief 経過分数を取得する
	 *
	 * @return double 経過分数 [min]
	 */
	double totalMinutes() const { return static_cast<double>(m_ticks) / constant::ticks_per_minute; }

	/**
	 * @brief 経過秒数を取得する
	 *
	 * @return double 経過秒数 [s]
	 */
	double totalSeconds() const { return static_cast<double>(m_ticks) / constant::ticks_per_second; }

	/**
	 * @brief 経過ミリ秒数を取得する
	 *
	 * @return double 経過ミリ秒数 [ms]
	 */
	double totalMilliseconds() const { return static_cast<double>(m_ticks) / constant::ticks_per_millisecond; }

	/**
	 * @brief 経過マイクロ秒数を取得する
	 *
	 * @return double 経過マイクロ秒数 [us]
	 */
	double totalMicroseconds() const { return static_cast<double>(m_ticks) / constant::ticks_per_microsecond; }

  private:
	int64_t m_ticks;

	/**
	 * @brief 日数、時間、分、秒、マイクロ秒からティック数を計算する
	 *
	 * @param days 日数 [day]
	 * @param hours 時間数 [h]
	 * @param minutes 分数 [min]
	 * @param seconds 秒数 [s]
	 * @param microseconds マイクロ秒数 [us]
	 */
	void calculateTicks(int days, int hours, int minutes, int seconds, int microseconds) {
		m_ticks = 0;
		m_ticks += days * constant::ticks_per_day;
		m_ticks += hours * constant::ticks_per_hour;
		m_ticks += minutes * constant::ticks_per_minute;
		m_ticks += seconds * constant::ticks_per_second;
		m_ticks += microseconds * constant::ticks_per_microsecond;
	}

	friend auto operator+(const TimeSpan& ts1, const TimeSpan& ts2) { return TimeSpan{ts1.ticks() + ts2.ticks()}; }

	friend auto operator-(const TimeSpan& ts1, const TimeSpan& ts2) { return TimeSpan{ts1.ticks() - ts2.ticks()}; }

	friend auto operator==(const TimeSpan& ts1, TimeSpan& ts2) { return ts1.ticks() == ts2.ticks(); }

	friend auto operator>(const TimeSpan& ts1, const TimeSpan& ts2) { return ts1.ticks() > ts2.ticks(); }

	friend auto operator>=(const TimeSpan& ts1, const TimeSpan& ts2) { return ts1.ticks() >= ts2.ticks(); }

	friend auto operator!=(const TimeSpan& ts1, const TimeSpan& ts2) { return ts1.ticks() != ts2.ticks(); }

	friend auto operator<(const TimeSpan& ts1, const TimeSpan& ts2) { return ts1.ticks() < ts2.ticks(); }

	friend auto operator<=(const TimeSpan& ts1, const TimeSpan& ts2) { return ts1.ticks() <= ts2.ticks(); }
};

class Days : public TimeSpan {
  public:
	Days() : TimeSpan(0) {}
	Days(double days) : TimeSpan(days, TimeUnit::Days) {}
};

class Hours : public TimeSpan {
  public:
	Hours() : TimeSpan(0) {}
	Hours(double hours) : TimeSpan(hours, TimeUnit::Hours) {}
};

class Minutes : public TimeSpan {
  public:
	Minutes() : TimeSpan(0) {}
	Minutes(double minutes) : TimeSpan(minutes, TimeUnit::Minutes) {}
};

class Seconds : public TimeSpan {
  public:
	Seconds() : TimeSpan(0) {}
	Seconds(double seconds) : TimeSpan(seconds, TimeUnit::Seconds) {}
};

class Milliseconds : public TimeSpan {
  public:
	Milliseconds() : TimeSpan(0) {}
	Milliseconds(double milliseconds) : TimeSpan(milliseconds, TimeUnit::Milliseconds) {}
};

class Microseconds : public TimeSpan {
  public:
	Microseconds() : TimeSpan(0) {}
	Microseconds(double microseconds) : TimeSpan(microseconds, TimeUnit::Microseconds) {}
};

GEOMAG_NAMESPACE_END
