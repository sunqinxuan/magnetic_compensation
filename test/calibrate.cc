#include <iostream>
#include <fstream>
#include <ctime>
#include <gflags/gflags.h>
#include "common/mic_prerequisite.h"
#include "common/mic_logger.h"
#include "common/mic_config.h"
#include "data_storer/mic_data_storer.h"
#include "mic_mag_compensator/mic_mag_compensator.h"
#include "mic_mag_compensator/obeserver/mic_state_logger.h"
#include "mic_mag_compensator/impl/mic_ellipsoid_mag_compensator.h"
#include "mic_mag_compensator/impl/mic_tl_mag_compensator.h"
#include "mic_mag_compensator/impl/mic_tl_component_mag_compensator.h"
#include "mic_mag_compensator/impl/mic_cabin_mag_compensator.h"
#include "fileio/fileio.h"

USING_NAMESPACE_MIC;

DEFINE_string(model, "ellipsoid", "ellipsoid, tl, tlc or cabin");
DEFINE_string(file, "Flt1002_1002.2.txt", "file to load data");

int main(int argc, char *argv[])
{
    google::ParseCommandLineFlags(&argc, &argv, true);
    mic_logger_t::initialize(mic_logger_type_t::MIC_BASH_FILE_LOGGER, "./mic.log");
    mic_config_t::initialize(mic_config_type_t::MIC_CONFIG_JSON,
                             "./etc/config_calibration.json");
    mic_logger_t::set_log_level(
        static_cast<mic_log_level_t>(MIC_CONFIG_GET(int32_t, "log_level")));

    mic_mag_compensator_shared_ptr mag_compensator_ptr;
    if ("tl" == FLAGS_model)
    {
        mag_compensator_ptr = std::make_shared<mic_tl_mag_compensator_t>();
    }
    else if("tlc"==FLAGS_model)
    {
        mag_compensator_ptr = std::make_shared<mic_tl_component_mag_compensator_t>();
    }
    else if ("ellipsoid" == FLAGS_model)
    {
        mag_compensator_ptr = std::make_shared<mic_ellipsoid_mag_compensator_t>();
    }
    else if ("cabin" == FLAGS_model)
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

    if(load_data(FLAGS_file,mag_compensator_ptr)==ret_t::MIC_RET_FAILED)
    {
        MIC_LOG_ERR("failed to load data file %s", FLAGS_file);
        return -1;
    }

    mag_compensator_ptr->calibrate();
    mag_compensator_ptr->save_model("./mic_model_" + FLAGS_model + ".mdl");

    google::ShutDownCommandLineFlags();
    return 0;
}
