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
#ifndef MIC_PREREQUISITE
#define MIC_PREREQUISITE

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <thread>
#include <stdio.h>
#include "api/mic_base.h"
#include "json.hpp"

#define MIC_NAMESPACE_START namespace mic {
#define MIC_NAMESPACE_END }

#define USING_NAMESPACE_MIC using namespace mic

#define MIC_MIN_FLOAT64 std::numeric_limits<float64_t>::min()
#define MIC_MAX_FLOAT64 std::numeric_limits<float64_t>::max()
#define MIC_MIN_FLOAT32 std::numeric_limits<float32_t>::min()
#define MIC_MAX_FLOAT32 std::numeric_limits<float32_t>::max()
#define MIC_MIN_INT32 std::numeric_limits<int32_t>::min()
#define MIC_MAX_INT32 std::numeric_limits<int32_t>::max()

#define MIC_EPSILON (1e-3f)
#define MIC_SMALLER_EPSILON (1e-6f)
#define MIC_SMALLEST_EPSILON (1e-8f)

#define MIC_IS_FLOAT_ZERO(x) (fabs(x - 0.) < MIC_SMALLER_EPSILON)

#define MIC_PI (3.14159265358979323846)
#define MIC_HALF_PI (1.57079632679489661923)
#define MIC_DOUBLE_PI (6.28318530717958647692)

#define MIC_REF(obj) std::ref(obj)
#define MIC_SAFE_DELETE(p) do { if (p) {delete p; p = nullptr;}} while(0)

#define mic_time_t std::chrono::steady_clock::time_point
#define MIC_NOW() (std::chrono::steady_clock::now())
#define MIC_TIME_COST(start, end) \
    (std::chrono::duration_cast<std::chrono::duration<float64_t> > \
            (end - start).count())
#define MIC_TIME_COST_TO_NOW(time) MIC_TIME_COST(time, MIC_NOW())
#define MIC_TIMER_START(_X) \
        auto _X##_start = std::chrono::steady_clock::now(); \
        auto _X##_stop = _X##_start
#define MIC_TIMER_STOP(_X) \
        do \
        { \
            _X##_stop = std::chrono::steady_clock::now(); \
            auto time_used = \
            std::chrono::duration_cast<std::chrono::duration<float64_t> > \
            (_X##_stop - _X##_start); \
            MIC_LOG_DEBUG_INFO("[MIC] "#_X" takes %.3f s\n", time_used.count()); \
        } while(0)
#define MIC_TIMER_COUNT(_X) \
        std::chrono::duration_cast<std::chrono::duration<float64_t> > \
        (_X##_stop - _X##_start).count()


MIC_NAMESPACE_START

using thread_t = std::thread;
using json_t = nlohmann::json;

class MicLogger;
using mic_logger_t = MicLogger;

class MicConfig;
using mic_config_t = MicConfig;

template <typename T>
class MicObserver;

template <typename T>
using mic_observer_t = MicObserver<T>;

template <typename T>
using mic_observer_shared_ptr = std::shared_ptr<mic_observer_t<T> >;

template <typename T>
class MicObservable;

class MicUtils;
using mic_utils = MicUtils;

MIC_NAMESPACE_END

#endif