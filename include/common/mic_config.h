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
#ifndef MIC_CONFIGURE
#define MIC_CONFIGURE

#include "common/mic_prerequisite.h"
#include "common/mic_logger.h"
#include <typeinfo>
#include <unordered_map>

MIC_NAMESPACE_START

using mic_config_unique_ptr = std::unique_ptr<mic_config_t>;

template<typename T>
struct MicParamType
{
    static std::string info()
    {
        return typeid(T).name();
    }
};
template<typename T>
using mic_param_type_t = MicParamType<T>;

enum class MicConfigType : uint8_t
{
    MIC_CONFIG_JSON = 0,
};
using mic_config_type_t = MicConfigType;

#define MIC_CONFIG_GET(type, key) \
    mic_config_t::get_instance().get<type>(key)

#define MIC_CONFIG_SET(type, key, value) \
    mic_config_t::get_instance().set<type>(key, value)

class MicConfig
{
protected:
    MicConfig (); // private constructor makes a singleton

    static mic_config_unique_ptr _config;

    json_t _json;

    #define REGISTER_PARAM(key, type) \
        do \
        { \
            _param_registry[key] = mic_param_type_t<type>::info(); \
        } while (0)

    std::unordered_map<std::string, std::string> _param_registry;

    void from_json_node(json_t& node);

    template <typename T>
    bool_t check_key(const std::string& key)
    {
        if (_json.find(key) == _json.end())
        {
            MIC_LOG_ERR(
                "[MIC] mic can not get the config key: %s\n", key.c_str());
            return false;
        }
        else if (_param_registry.find(key) == _param_registry.end())
        {
            MIC_LOG_ERR(
                "[MIC] mic have not registered config key: %s\n", key.c_str());
            return false;
        }
        else if (_param_registry[key] != typeid(T).name())
        {
            MIC_LOG_ERR(
                "[MIC] mic can not get the same type of config key: %s\n", key.c_str());
            return false;
        }
        return true;
    }

public:
    ~MicConfig() {};  // close the file when deconstructing

    // set a new config file
    static void initialize(
        const mic_config_type_t type,
        const std::string filename = "./etc/default_config.json");

    static mic_config_t& get_instance();

    static const std::string to_str();
    static ret_t from_str(const std::string& str);

    // access the parameter values
    template <typename T>
    T get(const std::string key)
    {
        if (check_key<T>(key)) return _json[key];
        return T();
    }

    template <typename T>
    void set(const std::string key, T value)
    {
        if (check_key<T>(key)) _json[key] = value;
    }

};

class MicJsonConfig : public MicConfig
{
public:
    MicJsonConfig(
        const std::string filename = "./etc/default_config.json");
    ~MicJsonConfig() {};
};
using mic_json_config_t = MicJsonConfig;

MIC_NAMESPACE_END

#endif