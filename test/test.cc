#include <iostream>
#include <fstream>
#include "common/mic_prerequisite.h"
#include "common/mic_logger.h"
#include "common/mic_config.h"
#include "data_storer/mic_data_storer.h"
#include "mic_mag_compensator/mic_mag_compensator.h"
#include "mic_mag_compensator/obeserver/mic_state_logger.h"
#include "mic_mag_compensator/impl/mic_ellipsoid_mag_compensator.h"
#include "mic_mag_compensator/impl/mic_tl_mag_compensator.h"
#include "ceres/ceres.h"

USING_NAMESPACE_MIC;
using namespace std;

int main(int argc, char *argv[])
{
    mic_logger_t::initialize(mic_logger_type_t::MIC_BASH_FILE_LOGGER, "./mic.log");
    mic_config_t::initialize(mic_config_type_t::MIC_CONFIG_JSON,
                             "./etc/default_config.json");
    mic_logger_t::set_log_level(
        static_cast<mic_log_level_t>(MIC_CONFIG_GET(int32_t, "log_level")));

    // ceres::Solver::Options options;
    // ceres::Solver::Summary summary;
    // ceres::Problem problem;
    // ceres::Solve(options, &problem, &summary);
    // MIC_LOG_BASIC_INFO("final_cost: %lf!\n", summary.final_cost);

    std::string filename = MIC_CONFIG_GET(std::string, "load_file_name");
    std::ifstream infile(filename);
    if (!infile.is_open())
    {
        std::cerr << "Error: Could not open file " << filename << std::endl;
        return -1;
    }

    float64_t ts, flux_x, flux_y, flux_z,
        ins_pitch, ins_roll, ins_yaw,
        igrf_north, igrf_east, igrf_down;
    mic_mag_flux_t mag_flux, mag_flux_truth;
    mic_nav_state_t nav_state;
    // std::ofstream fp("debug.txt");
    // mic_ellipsoid_mag_compensator_t mag_compensator;

    mic_mag_compensator_shared_ptr mag_compensator_ptr;
    if (MIC_CONFIG_GET(std::string, "compensation_method") == "tl")
    {
        mag_compensator_ptr = std::make_shared<mic_tl_mag_compensator_t>();
    }
    else // "ellipsoid"
    {
        mag_compensator_ptr = std::make_shared<mic_ellipsoid_mag_compensator_t>();
    }

    auto comp_logger = std::make_shared<mic_state_logger_t>();
    // mag_compensator.subscrible(comp_logger);
    mag_compensator_ptr->subscrible(comp_logger);

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

            // mag_compensator.add_mag_flux(ts, mag_flux);
            // mag_compensator.add_mag_flux_truth(ts, mag_flux_truth);
            // mag_compensator.get_nav_state_estimator().set_nav_state(ts, nav_state);

            mag_compensator_ptr->add_mag_flux(ts, mag_flux);
            mag_compensator_ptr->add_mag_flux_truth(ts, mag_flux_truth);
            mag_compensator_ptr->add_nav_state(ts,nav_state);
            // mag_compensator_ptr->get_nav_state_estimator().set_nav_state(ts, nav_state);
        }

        // fp << std::fixed << line << "\t" << ts << "\t"
        //    << flux_x << "\t" << flux_y << "\t"
        //    << flux_z << "\t" << ins_pitch << "\t"
        //    << ins_roll << "\t" << ins_yaw << "\t"
        //    << std::endl;
    }
    infile.close();
    // fp.close();

    // mag_compensator.calibrate();
    mag_compensator_ptr->calibrate();

    mic_mag_flux_t mag_flux_comp;
    mag_compensator_ptr->compenste(mag_flux, mag_flux_comp);

    return 0;
}

int main1()
{
    mic_logger_t::initialize(mic_logger_type_t::MIC_BASH_FILE_LOGGER, "./mic.log");
    mic_config_t::initialize(mic_config_type_t::MIC_CONFIG_JSON,
                             "../etc/default_config.json");
    mic_logger_t::set_log_level(
        static_cast<mic_log_level_t>(MIC_CONFIG_GET(int32_t, "log_level")));

    MicDataStorer<float64_t, int32_t, std::string> storer;
    storer.add_data<float64_t>(1.0, 2.0);
    storer.add_data<int32_t>(1.01, 2);
    storer.add_data<std::string>(0.999, "test");

    float64_t f = 0.0;
    int32_t i = 0;
    std::string str = "";
    auto res = storer.get_data(1.0, f, i, str);
    std::cout << "res: " << res << std::endl;
    std::cout << "f: " << f << " i: " << i << " str: " << str << std::endl;

    MicDataStorer<std::string, int32_t, float64_t> storer2;
    storer2.add_data<float64_t>(1.0, 3214312.0);
    storer2.add_data<int32_t>(1.019, 5);
    storer2.add_data<std::string>(0.999, "test2");

    int32_t data = 0;
    storer2.get_data<int32_t>(1.019, data);
    std::cout << "result: " << data << std::endl;

    res = storer2.get_data(1.0, str, i, f);
    std::cout << "res: " << res << std::endl;
    std::cout << "f: " << f << " i: " << i << " str: " << str << std::endl;

    // mic_mag_compensator_t comp;
    // auto comp_logger = std::make_shared<mic_state_logger_t>();
    // comp.subscrible(comp_logger);

    for (size_t i = 0; i < 100; i++)
    {
        // comp.compenste();
    }

    return 0;
}
