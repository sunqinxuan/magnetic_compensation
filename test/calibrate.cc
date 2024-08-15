#include <iostream>
#include <fstream>
#include <ctime>
#include "common/mic_prerequisite.h"
#include "common/mic_logger.h"
#include "common/mic_config.h"
#include "data_storer/mic_data_storer.h"
#include "mic_mag_compensator/mic_mag_compensator.h"
#include "mic_mag_compensator/obeserver/mic_state_logger.h"
#include "mic_mag_compensator/impl/mic_ellipsoid_mag_compensator.h"
#include "mic_mag_compensator/impl/mic_tl_mag_compensator.h"
#include "mic_mag_compensator/impl/mic_cabin_mag_compensator.h"

USING_NAMESPACE_MIC;

int main(int argc, char *argv[])
{
    mic_logger_t::initialize(mic_logger_type_t::MIC_BASH_FILE_LOGGER, "./mic.log");
    mic_config_t::initialize(mic_config_type_t::MIC_CONFIG_JSON,
                             "./etc/default_config.json");
    mic_logger_t::set_log_level(
        static_cast<mic_log_level_t>(MIC_CONFIG_GET(int32_t, "log_level")));

    std::string filename = MIC_CONFIG_GET(std::string, "load_file_name");
    std::ifstream infile(filename);
    if (!infile.is_open())
    {
        MIC_LOG_ERR("Error: Could not open file %s", filename);
        return -1;
    }

    mic_mag_compensator_shared_ptr mag_compensator_ptr;
    auto comp_method = MIC_CONFIG_GET(std::string, "compensation_method");
    if ("tl" == comp_method)
    {
        mag_compensator_ptr = std::make_shared<mic_tl_mag_compensator_t>();
    }
    else if ("ellipsoid" == comp_method)
    {
        mag_compensator_ptr = std::make_shared<mic_ellipsoid_mag_compensator_t>();
    }
    else if ("cabin" == comp_method)
    {
        mag_compensator_ptr = std::make_shared<mic_cabin_mag_compensator_t>();
    }
    else
    {
        MIC_LOG_ERR("[MIC] MIC compensation method is not supported!");
        return -1;
    }

    auto comp_logger = std::make_shared<mic_state_logger_t>();
    mag_compensator_ptr->subscrible(comp_logger);

    float64_t ts, flux_x, flux_y, flux_z,
        ins_pitch, ins_roll, ins_yaw,
        igrf_north, igrf_east, igrf_down;
    mic_mag_flux_t mag_flux, mag_flux_truth;
    mic_nav_state_t nav_state;

    while (true)
    {
        infile >> ts >> flux_x >> flux_y >> flux_z >> ins_pitch >> ins_roll >> ins_yaw >> igrf_north >> igrf_east >> igrf_down;
        if (infile.eof())
            break;
        // if (line < 1002.10)
        {
            nav_state.time_stamp = ts;
            nav_state.attitude = quaternionf_t(
                MicUtils::euler2dcm(
                    MicUtils::deg2rad(ins_roll),
                    MicUtils::deg2rad(ins_pitch),
                    MicUtils::deg2rad(ins_yaw)));
            mag_flux.time_stamp = ts;
            mag_flux.vector << flux_x, flux_y, flux_z;
            mag_flux_truth.time_stamp = ts;
            mag_flux_truth.vector << igrf_north, igrf_east, igrf_down;

            mag_compensator_ptr->add_mag_flux(ts, mag_flux);
            mag_compensator_ptr->add_mag_flux_truth(ts, mag_flux_truth);
            mag_compensator_ptr->add_nav_state(ts, nav_state);
        }
    }
    infile.close();

    mag_compensator_ptr->calibrate();

    // time_t tt = time(nullptr); // milliseconds from 1970;
    // struct tm *cur_tm = localtime(&tt);
    // std::stringstream tm_str;
    // tm_str << cur_tm->tm_year + 1900 << "-"
    //        << cur_tm->tm_mon << "-"
    //        << cur_tm->tm_mday << "-"
    //        << cur_tm->tm_hour << "-"
    //        << cur_tm->tm_min << "-"
    //        << cur_tm->tm_sec;

    // mag_compensator_ptr->save_model("./" + tm_str.str() + ".mdl");
    mag_compensator_ptr->save_model("./mic_model_" + comp_method + ".mdl");

    // mic_mag_flux_t mag_flux_comp;
    // mag_compensator_ptr->compenste(mag_flux, mag_flux_comp);

    return 0;
}
