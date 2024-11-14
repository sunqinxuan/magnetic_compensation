#include <iostream>
#include <string>
#include <fstream>
#include <ctime>
#include <gflags/gflags.h>
#include <chrono>
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
#include "mic_mag_compensator/impl/mic_cabin_nav_mag_compensator.h"
#include "fileio/fileio.h"

USING_NAMESPACE_MIC;
using namespace std;

DEFINE_string(model, "ellipsoid", "ellipsoid, tl, tlc or cabin");
DEFINE_string(file, "Flt1002_1002.02.txt", "file to load data");
DEFINE_string(out, "mic_model.mdl", "file to save compensation model");

int main(int argc, char *argv[])
{
    google::ParseCommandLineFlags(&argc, &argv, true);
    mic_logger_t::initialize(mic_logger_type_t::MIC_BASH_FILE_LOGGER, "./mic.log");
    mic_config_t::initialize(mic_config_type_t::MIC_CONFIG_JSON,
                             "./etc/config_calibration.json");
    mic_logger_t::set_log_level(
        static_cast<mic_log_level_t>(MIC_CONFIG_GET(int32_t, "log_level")));

    // if(FLAGS_file.substr(0, 3) == "sim")
    // {
    //     FLAGS_model="ellipsoid";
    // }
    // else
    // {
    //     FLAGS_model="cabin";
    // }

    mic_mag_compensator_shared_ptr mag_compensator_ptr;
    if ("tl" == FLAGS_model)
    {
        mag_compensator_ptr = std::make_shared<mic_tl_mag_compensator_t>();
    }
    else if ("tlc" == FLAGS_model)
    {
        mag_compensator_ptr = std::make_shared<mic_tl_component_mag_compensator_t>();
    }
    else if ("ellipsoid" == FLAGS_model)
    {
        mag_compensator_ptr = std::make_shared<mic_ellipsoid_mag_compensator_t>();
        // if(FLAGS_file.substr(15,1)=="5")
        // {
        //     mag_compensator_ptr->setFlag();
        // }
    }
    else if ("cabin" == FLAGS_model)
    {
        mag_compensator_ptr = std::make_shared<mic_cabin_mag_compensator_t>();
    }
    else if ("nav" == FLAGS_model)
    {
        mag_compensator_ptr = std::make_shared<mic_cabin_nav_mag_compensator_t>();
    }
    else
    {
        MIC_LOG_ERR("[MIC] MIC compensation method is not supported!");
        return -1;
    }

    auto comp_logger = std::make_shared<mic_state_logger_t>();
    mag_compensator_ptr->subscrible(comp_logger);

    //
    const int totalSteps = 50;
    const std::string greenText = "\033[32m"; // 深绿色
    const std::string resetText = "\033[0m";  // 重置颜色

    std::cout << endl
              << "Loading file: " << FLAGS_file << "\n";

    for (int step = 0; step <= totalSteps; ++step)
    {
        std::cout << "\r" << greenText << "[";
        int progress = step * 100 / totalSteps;
        for (int i = 0; i < totalSteps; ++i)
        {
            if (i < step)
                std::cout << "#";
            else
                std::cout << " ";
        }
        std::cout << "] " << progress << "%" << resetText;
        std::cout.flush();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    std::cout << "\nFile loaded successfully!" << std::endl
              << endl;

    //

    if (load_data(FLAGS_file, mag_compensator_ptr) == ret_t::MIC_RET_FAILED)
    {
        MIC_LOG_ERR("failed to load data file %s", FLAGS_file);
        return -1;
    }

    cout << "Start model calibration ... " << endl;
    auto start = std::chrono::high_resolution_clock::now();
    mag_compensator_ptr->calibrate();
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> duration = end - start;
    cout << endl
         << "Calibration done!" << endl;
    std::cout << "Total runtime: " << duration.count() << " ms" << std::endl
              << endl;


    string name = "./out/" + FLAGS_out;
    mag_compensator_ptr->save_model(name);
    cout << "Compensation model saved: " << name << endl
         << endl;

    google::ShutDownCommandLineFlags();
    return 0;
}
