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

#include "api/mic_compensation_api.h"
#include "common/mic_utils.h"
#include "common/mic_config.h"
#include "common/mic_logger.h"
#include "mic_mag_compensator/mic_mag_compensator.h"
// #include "mic_mag_compensator/impl/mic_ellipsoid_mag_compensator.h"
#include "mic_mag_compensator/impl/mic_ellipsoid_nav_mag_compensator.h"
// #include "mic_mag_compensator/impl/mic_tl_mag_compensator.h"
// #include "mic_mag_compensator/impl/mic_tl_component_mag_compensator.h"
// #include "mic_mag_compensator/impl/mic_cabin_mag_compensator.h"
// #include "mic_mag_compensator/impl/mic_cabin_nav_mag_compensator.h"
#include "mic_mag_compensator/obeserver/mic_state_logger.h"
#include "mic_mag_compensator/obeserver/mic_state_evaluator.h"

namespace mic
{

    // static mic_mag_compensator_shared_ptr _mic_compensator = nullptr;
    static std::shared_ptr<mic_ellipsoid_mag_compensator_t> _mic_compensator = nullptr;

    static std::shared_ptr<mic_state_logger_t> _mic_logger = nullptr;
    static std::shared_ptr<mic_state_evaluator_t> _mic_evaluator = nullptr;

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

        // auto comp_method = MIC_CONFIG_GET(std::string, "compensation_method");

        /*
        if ("tl" == model)
        {
            _mic_compensator = std::make_shared<mic_tl_mag_compensator_t>();
        }
        else if ("tlc" == model)
        {
            _mic_compensator = std::make_shared<mic_tl_component_mag_compensator_t>();
        }
        else if ("ellipsoid" == model)
        {
            _mic_compensator = std::make_shared<mic_ellipsoid_mag_compensator_t>();
        }
        else if ("cabin" == model)
        {
            _mic_compensator = std::make_shared<mic_cabin_mag_compensator_t>();
        }
        else if ("cabin_nav" == model)
        {
            _mic_compensator = std::make_shared<mic_cabin_nav_mag_compensator_t>();
        }
        else
        {
            MIC_LOG_ERR("[MIC] MIC compensation model is not supported!");
            return ret_t::MIC_RET_FAILED;
        }
        */
        _mic_compensator = std::make_shared<mic_ellipsoid_mag_compensator_t>();

        _mic_logger = std::make_shared<mic_state_logger_t>();
        _mic_compensator->subscrible(_mic_logger);
        _mic_evaluator = std::make_shared<mic_state_evaluator_t>();
        _mic_compensator->subscrible(_mic_evaluator);

        _mic_compensator->load_model(model_file);

        return ret_t::MIC_RET_SUCCESSED;
    }

    // ret_t mic_add_data(
    //     const double timestamp,
    //     const mic_mag_t &mag,
    //     const mic_mag_t &mag_truth,
    //     const mic_nav_state_t &nav_state)
    // {
    //     _mic_compensator->add_data(timestamp, mag, nav_state);
    //     _mic_compensator->add_data_truth(timestamp, mag_truth);
    //     // _mic_evaluator->add_ground_truth(timestamp, mag_truth);
    //     return ret_t::MIC_RET_SUCCESSED;
    // }

    ret_t mic_compensate(const double timestamp, mic_mag_t &out,
                         const mic_mag_t &mag,
                         const mic_mag_t &mag_truth,
                         const mic_nav_state_t &nav_state)
    {
        if (_mic_compensator == nullptr)
        {
            MIC_LOG_ERR("[MIC] MIC worker is not initialized!");
            return ret_t::MIC_RET_FAILED;
        }
        return _mic_compensator->compenste(timestamp, out, mag, mag_truth, nav_state);
    }

    // matrix_xf_t mic_get_cov() { return _mic_compensator->get_kf_cov(); }

    ret_t mic_pry2navstate(MicNavState nav_state, float64_t pitch, float64_t roll, float64_t yaw)
    {
        matrix_3f_t rotation2NED = matrix_3f_t::Identity();
        std::string nav_frame = MIC_CONFIG_GET(std::string, "navigation_frame");
        std::string euler_seq = MIC_CONFIG_GET(std::string, "euler_angle_sequence");
        if (nav_frame == "ENU")
        {
            rotation2NED << 0, 1, 0,
                1, 0, 0,
                0, 0, -1;
        }
        nav_state.attitude = quaternionf_t(rotation2NED *
                                           MicUtils::euler2dcm(
                                               MicUtils::deg2rad(roll),
                                               MicUtils::deg2rad(pitch),
                                               MicUtils::deg2rad(yaw), euler_seq));
        return ret_t::MIC_RET_SUCCESSED;
    }

}