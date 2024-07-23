#include "common/mag_config.h"
#include "common/mag_logger.h"
#include <fstream>

MAG_NAMESPACE_START

MagConfig::MagConfig()
{
    REGISTER_PARAM("log_level", int32_t);
    REGISTER_PARAM("time_sync_tolerance", float64_t);
    REGISTER_PARAM("data_remain_time", float64_t);
}

void MagConfig::initialize(
    const mag_config_type_t type,
    const std::string filename)
{
    if (_config != nullptr)
    {
        return;
    }
    switch (type)
    {
    default:
    case mag_config_type_t::MAG_CONFIG_JSON:
        _config.reset(new mag_json_config_t(filename));
        break;
    }
}

mag_config_t& MagConfig::get_instance()
{
    if (_config == nullptr)
    {
        MAG_LOG_WARN(
            "[MAG] MAG configure is not initialized!\n");
        _config.reset(new mag_json_config_t());
    }
    return *_config;
}

const std::string MagConfig::to_str()
{
    return get_instance()._json.dump(4);
}

ret_t MagConfig::from_str(const std::string& str)
{
    json_t node = json_t::parse(str);
    if (node.empty())
    {
        return ret_t::MAG_RET_FAILED;
    }
    auto& config = get_instance();
    config.from_json_node(node);
    return ret_t::MAG_RET_SUCCESSED;
}

void MagConfig::from_json_node(json_t& node)
{
    for (auto it = _param_registry.begin(); it != _param_registry.end(); it++)
    {
        auto& key = it->first;
        if (node.find(key) == node.end())
        {
            MAG_LOG_ERR(
                "[MAG] mag can not find the config key: %s", key.c_str());
            continue;
        }
        _json[key] = node[key];
    }
}

mag_config_unique_ptr MagConfig::_config = nullptr;

MagJsonConfig::MagJsonConfig(const std::string filename) : MagConfig()
{
    std::ifstream map_file(filename);
    if (!map_file.is_open())
    {
        MAG_LOG_ERR(
            "[MAG] mag config file, %s does not exist!", filename.c_str());
        return;
    }
    json_t json = json_t::parse(map_file, nullptr, 0, 1);
    from_json_node(json);
}

MAG_NAMESPACE_END