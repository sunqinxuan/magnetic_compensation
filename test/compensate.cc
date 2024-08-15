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
#include "api/mic_compensation_api.h"

USING_NAMESPACE_MIC;
using namespace std;

int main(int argc, char *argv[])
{
    // initialize the compensator;
    auto comp_method = MIC_CONFIG_GET(std::string, "compensation_method");
    mic_init_worker("./mic_model_" + comp_method + ".mdl");

    // create output file for compensation results;
    time_t tt = time(nullptr); // milliseconds from 1970;
    struct tm *cur_tm = localtime(&tt);
    std::stringstream tm_str;
    tm_str << cur_tm->tm_year + 1900 << "-"
           << cur_tm->tm_mon + 1 << "-"
           << cur_tm->tm_mday << "-"
           << cur_tm->tm_hour << "-"
           << cur_tm->tm_min << "-"
           << cur_tm->tm_sec;
    std::ofstream outfile("./output_" + tm_str.str() + ".txt");
    if (!outfile.is_open())
    {
        MIC_LOG_ERR("Error: Could not open file %s", "./" + tm_str.str() + ".txt");
        return -1;
    }

    // load data from file;
    std::string filename = MIC_CONFIG_GET(std::string, "load_file_name");
    std::ifstream infile(filename);
    if (!infile.is_open())
    {
        MIC_LOG_ERR("Error: Could not open file %s", filename);
        return -1;
    }
    float64_t ts, flux_x, flux_y, flux_z,
        ins_pitch, ins_roll, ins_yaw,
        igrf_north, igrf_east, igrf_down;
    mic_mag_flux_t mag_flux, mag_flux_truth;
    mic_nav_state_t nav_state;
    while (true)
    {
        infile >> ts >> flux_x >> flux_y >> flux_z >>
            ins_pitch >> ins_roll >> ins_yaw >>
            igrf_north >> igrf_east >> igrf_down;
        if (infile.eof())
            break;
            
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

        // do compensation and save results to file;
        mic_mag_flux_t mag_out;
        mic_compensate(mag_flux, mag_out);
        outfile << mag_out.vector.transpose() << std::endl;
    }
    infile.close();
    outfile.close();

    return 0;
}
