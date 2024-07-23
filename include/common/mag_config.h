#ifndef MAG_CONFIGURE
#define MAG_CONFIGURE

#include "common/mag_prerequisite.h"
#include "common/mag_logger.h"
#include <typeinfo>
#include <unordered_map>

MAG_NAMESPACE_START

using mag_config_unique_ptr = std::unique_ptr<mag_config_t>;

template<typename T>
struct MagParamType
{
    static std::string info()
    {
        return typeid(T).name();
    }
};
template<typename T>
using mag_param_type_t = MagParamType<T>;

enum class MagConfigType : uint8_t
{
    MAG_CONFIG_JSON = 0,
};
using mag_config_type_t = MagConfigType;

#define MAG_CONFIG_GET(type, key) \
    mag_config_t::get_instance().get<type>(key)

#define MAG_CONFIG_SET(type, key, value) \
    mag_config_t::get_instance().set<type>(key, value)

class MagConfig
{
protected:
    MagConfig (); // private constructor makes a singleton

    static mag_config_unique_ptr _config;

    json_t _json;

    #define REGISTER_PARAM(key, type) \
        do \
        { \
            _param_registry[key] = mag_param_type_t<type>::info(); \
        } while (0)

    std::unordered_map<std::string, std::string> _param_registry;

    void from_json_node(json_t& node);

    template <typename T>
    bool_t check_key(const std::string& key)
    {
        if (_json.find(key) == _json.end())
        {
            MAG_LOG_ERR(
                "[MAG] mag can not get the config key: %s\n", key.c_str());
            return false;
        }
        else if (_param_registry.find(key) == _param_registry.end())
        {
            MAG_LOG_ERR(
                "[MAG] mag have not registered config key: %s\n", key.c_str());
            return false;
        }
        else if (_param_registry[key] != typeid(T).name())
        {
            MAG_LOG_ERR(
                "[MAG] mag can not get the same type of config key: %s\n", key.c_str());
            return false;
        }
        return true;
    }

public:
    ~MagConfig() {};  // close the file when deconstructing

    // set a new config file
    static void initialize(
        const mag_config_type_t type,
        const std::string filename = "./etc/default_config.json");

    static mag_config_t& get_instance();

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

class MagJsonConfig : public MagConfig
{
public:
    MagJsonConfig(
        const std::string filename = "./etc/default_config.json");
    ~MagJsonConfig() {};
};
using mag_json_config_t = MagJsonConfig;

MAG_NAMESPACE_END

#endif