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
#include "common/mic_config.h"
#include "common/mic_logger.h"
#include <fstream>

MIC_NAMESPACE_START

MicConfig::MicConfig()
{
    REGISTER_PARAM("log_level", int32_t);
    REGISTER_PARAM("time_sync_tolerance", float64_t);
    REGISTER_PARAM("data_remain_time", float64_t);
    REGISTER_PARAM("mag_earth_intensity", float64_t);
    REGISTER_PARAM("load_file_name", std::string);
}

void MicConfig::initialize(
    const mic_config_type_t type,
    const std::string filename)
{
    if (_config != nullptr)
    {
        return;
    }
    switch (type)
    {
    default:
    case mic_config_type_t::MIC_CONFIG_JSON:
        _config.reset(new mic_json_config_t(filename));
        break;
    }
}

mic_config_t& MicConfig::get_instance()
{
    if (_config == nullptr)
    {
        MIC_LOG_WARN(
            "[MIC] MIC configure is not initialized!\n");
        _config.reset(new mic_json_config_t());
    }
    return *_config;
}

const std::string MicConfig::to_str()
{
    return get_instance()._json.dump(4);
}

ret_t MicConfig::from_str(const std::string& str)
{
    json_t node = json_t::parse(str);
    if (node.empty())
    {
        return ret_t::MIC_RET_FAILED;
    }
    auto& config = get_instance();
    config.from_json_node(node);
    return ret_t::MIC_RET_SUCCESSED;
}

void MicConfig::from_json_node(json_t& node)
{
    for (auto it = _param_registry.begin(); it != _param_registry.end(); it++)
    {
        auto& key = it->first;
        if (node.find(key) == node.end())
        {
            MIC_LOG_ERR(
                "[MIC] mic can not find the config key: %s", key.c_str());
            continue;
        }
        _json[key] = node[key];
    }
}

mic_config_unique_ptr MicConfig::_config = nullptr;

MicJsonConfig::MicJsonConfig(const std::string filename) : MicConfig()
{
    std::ifstream map_file(filename);
    if (!map_file.is_open())
    {
        MIC_LOG_ERR(
            "[MIC] mic config file, %s does not exist!", filename.c_str());
        return;
    }
    json_t json = json_t::parse(map_file, nullptr, 0, 1);
    from_json_node(json);
}

MIC_NAMESPACE_END