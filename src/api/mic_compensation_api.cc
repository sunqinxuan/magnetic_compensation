#include "api/mic_compensation_api.h"
#include "common/mic_utils.h"
#include "common/mic_config.h"
#include "common/mic_logger.h"
#include "mic_mag_compensator/mic_mag_compensator.h"
#include "mic_mag_compensator/impl/mic_ellipsoid_mag_compensator.h"
#include "mic_mag_compensator/impl/mic_tl_mag_compensator.h"
#include "mic_mag_compensator/obeserver/mic_state_logger.h"

namespace mic
{

    static mic_mag_compensator_shared_ptr _mic_compensator = nullptr;

    static mic_observer_shared_ptr<mic_mag_compensator_t> _mic_logger = nullptr;

    ret_t mic_init_worker(
        const std::string model_file,
        const std::string log_file,
        const std::string config_file)
    {
        mic_logger_t::initialize(
            mic_logger_type_t::MIC_BASH_FILE_LOGGER, log_file);
        mic_config_t::initialize(
            mic_config_type_t::MIC_CONFIG_JSON, config_file);
        mic_logger_t::set_log_level(
            static_cast<mic_log_level_t>(MIC_CONFIG_GET(int32_t, "log_level")));

        auto comp_method = MIC_CONFIG_GET(std::string, "compensation_method");

        if ("tl" == comp_method)  // "tl"
        {
            _mic_compensator = std::make_shared<mic_tl_mag_compensator_t>();
        }
        else if ("ellipsoid" == comp_method) // "ellipsoid"
        {
            _mic_compensator = std::make_shared<mic_ellipsoid_mag_compensator_t>();
        }
        else
        {
            MIC_LOG_ERR("[MIC] MIC compensation method is not supported!");
            return ret_t::MIC_RET_FAILED;
        }

        auto comp_logger = std::make_shared<mic_state_logger_t>();
        _mic_compensator->subscrible(comp_logger);

        _mic_compensator->load_model(model_file);

        return ret_t::MIC_RET_SUCCESSED;
    }

    ret_t mic_compensate(
        const mic_mag_flux_t &in, mic_mag_flux_t &out)
    {
        if (_mic_compensator == nullptr)
        {
            MIC_LOG_ERR("[MIC] MIC worker is not initialized!");
            return ret_t::MIC_RET_FAILED;
        }
        return _mic_compensator->compenste(in, out);
    }


}