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
#ifndef MIC_DATA_PROCESSOR
#define MIC_DATA_PROCESSOR

#include "common/mic_prerequisite.h"
#include "common/mic_config.h"
#include <map>
#include <vector>
#include <tuple>
#include <map>
#include <type_traits>

MIC_NAMESPACE_START

template<typename... Types>
class MicDataStorer;
template<typename... Types>
using mic_data_storer_t = MicDataStorer<Types...>;

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
class MicDataStorer
{
public:
    MicDataStorer()
    {
        _time_sync_tolerance = MIC_CONFIG_GET(float64_t, "time_sync_tolerance");
        _data_remain_time = MIC_CONFIG_GET(float64_t, "data_remain_time");
    }
    ~MicDataStorer() {}

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

MIC_NAMESPACE_END

// #include "data_storer/impl/mic_data_storer.hpp"

#endif