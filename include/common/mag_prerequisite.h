#ifndef MAG_PREREQUISITE
#define MAG_PREREQUISITE

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <thread>
#include <stdio.h>
#include "api/mag_base.h"
#include "json.hpp"

#define MAG_NAMESPACE_START namespace mag {
#define MAG_NAMESPACE_END }

#define USING_NAMESPACE_MAG using namespace mag

#define MAG_MIN_FLOAT64 std::numeric_limits<float64_t>::min()
#define MAG_MAX_FLOAT64 std::numeric_limits<float64_t>::max()
#define MAG_MIN_FLOAT32 std::numeric_limits<float32_t>::min()
#define MAG_MAX_FLOAT32 std::numeric_limits<float32_t>::max()
#define MAG_MIN_INT32 std::numeric_limits<int32_t>::min()
#define MAG_MAX_INT32 std::numeric_limits<int32_t>::max()

#define MAG_EPSILON (1e-3f)
#define MAG_SMALLER_EPSILON (1e-6f)
#define MAG_SMALLEST_EPSILON (1e-8f)

#define MAG_IS_FLOAT_ZERO(x) (fabs(x - 0.) < MAG_SMALLER_EPSILON)

#define MAG_PI (3.14159265358979323846)
#define MAG_HALF_PI (1.57079632679489661923)
#define MAG_DOUBLE_PI (6.28318530717958647692)

#define MAG_REF(obj) std::ref(obj)
#define MAG_SAFE_DELETE(p) do { if (p) {delete p; p = nullptr;}} while(0)

#define mag_time_t std::chrono::steady_clock::time_point
#define MAG_NOW() (std::chrono::steady_clock::now())
#define MAG_TIME_COST(start, end) \
    (std::chrono::duration_cast<std::chrono::duration<float64_t> > \
            (end - start).count())
#define MAG_TIME_COST_TO_NOW(time) MAG_TIME_COST(time, MAG_NOW())
#define MAG_TIMER_START(_X) \
        auto _X##_start = std::chrono::steady_clock::now(); \
        auto _X##_stop = _X##_start
#define MAG_TIMER_STOP(_X) \
        do \
        { \
            _X##_stop = std::chrono::steady_clock::now(); \
            auto time_used = \
            std::chrono::duration_cast<std::chrono::duration<float64_t> > \
            (_X##_stop - _X##_start); \
            MAG_LOG_DEBUG_INFO("[MAG] "#_X" takes %.3f s\n", time_used.count()); \
        } while(0)
#define MAG_TIMER_COUNT(_X) \
        std::chrono::duration_cast<std::chrono::duration<float64_t> > \
        (_X##_stop - _X##_start).count()


MAG_NAMESPACE_START

using thread_t = std::thread;
using json_t = nlohmann::json;

class MagLogger;
using mag_logger_t = MagLogger;

class MagConfig;
using mag_config_t = MagConfig;

template <typename T>
class MagObserver;

template <typename T>
using mag_observer_t = MagObserver<T>;

template <typename T>
using mag_observer_shared_ptr = std::shared_ptr<mag_observer_t<T> >;

template <typename T>
class MagObservable;

class MagUtils;
using mag_utils = MagUtils;

MAG_NAMESPACE_END

#endif