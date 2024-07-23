#ifndef MAG_DATA_PROCESSOR
#define MAG_DATA_PROCESSOR

#include "common/mag_prerequisite.h"
#include "common/mag_config.h"
#include <map>
#include <vector>
#include <tuple>
#include <map>
#include <type_traits>

MAG_NAMESPACE_START

template<typename... Types>
class MagDataStorer;
template<typename... Types>
using mag_data_storer_t = MagDataStorer<Types...>;

// 辅助结构，用于在std::tuple中根据类型获取索引
template<typename T, typename Tuple>
struct tuple_index;

template<typename T, typename... Types>
struct tuple_index<T, std::tuple<T, Types...> > {
    static constexpr std::size_t value = 0;
};

template<typename T, typename U, typename... Types>
struct tuple_index<T, std::tuple<U, Types...> > {
    static constexpr std::size_t value = 1 + tuple_index<T, std::tuple<Types...>>::value;
};

template<typename... Types>
class MagDataStorer
{
public:
    MagDataStorer()
    {
        _time_sync_tolerance = MAG_CONFIG_GET(float64_t, "time_sync_tolerance");
        _data_remain_time = MAG_CONFIG_GET(float64_t, "data_remain_time");
    }
    ~MagDataStorer() {}

    // 用于输入数据的函数模板
    template<typename T>
    bool_t add_data(const float64_t ts, const T& data)
    {
        constexpr std::size_t index = tuple_index<T, std::tuple<Types...>>::value;
        auto& data_map = std::get<index>(_data_storage);
        data_map[ts] = data;
        auto it = data_map.begin();
        while (it != data_map.end() && it->first < ts - _data_remain_time)
        {
            it = data_map.erase(it);
        }
        return true;
    }

    // 用于获取数据的函数模板
    template<typename T>
    auto get_data_range(const float64_t start_ts, const float64_t end_ts)
    {
        constexpr std::size_t index = tuple_index<T, std::tuple<Types...>>::value;
        auto& data_map = std::get<index>(_data_storage);
        auto start_it = data_map.end();
        auto end_it = data_map.end();
        if (end_ts >= start_ts)
        {
            start_it = data_map.lower_bound(start_ts);
            if (start_it != data_map.begin()) start_it--;
            end_it = data_map.lower_bound(end_ts);
        }
        return std::make_pair(start_it, end_it);
    }

    // 用于获取数据的函数模板
    template<typename... Args>
    bool_t get_data(const float64_t ts, Args&... data)
    {
        return get_data_impl(ts, data...);
    }

private:
    float32_t _time_sync_tolerance;
    float32_t _data_remain_time;
    std::tuple<std::map<float64_t, Types>...> _data_storage;

    // 递归处理每一个数据类型
    template<typename T, typename... Rest>
    bool_t get_data_impl(const float64_t ts, T& data, Rest&... rest)
    {
        constexpr std::size_t index = tuple_index<T, std::tuple<Types...> >::value;
        auto& data_map = std::get<index>(_data_storage);
        auto precise_iter = data_map.find(ts);
        if (precise_iter != data_map.end())
        {
            data = precise_iter->second;
            return get_data_impl(ts, rest...);
        }
        auto rough_iter_1 = data_map.lower_bound(ts);
        auto rough_iter_2 = rough_iter_1;
        if (rough_iter_2 != data_map.begin())
        {
            rough_iter_2--;
        }
        if (rough_iter_1 != data_map.end()
            && fabs(ts - rough_iter_1->first) < _time_sync_tolerance)
        {
            data = rough_iter_1->second;
            return get_data_impl(ts, rest...);
        }
        else if (rough_iter_2 != data_map.end()
            && fabs(ts - rough_iter_2->first) < _time_sync_tolerance)
        {
            data = rough_iter_2->second;
            return get_data_impl(ts, rest...);
        }
        return false;
    }

    // 递归基例
    bool_t get_data_impl(const float64_t ts)
    {
        return true;
    }
};

MAG_NAMESPACE_END

// #include "data_storer/impl/mag_data_storer.hpp"

#endif