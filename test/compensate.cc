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
#include "api/mic_compensation_api.h"
#include "fileio/fileio.h"

USING_NAMESPACE_MIC;
using namespace std;

DEFINE_string(model, "ellipsoid", "ellipsoid, tl, tlc or cabin");
DEFINE_string(file, "Flt1002_1002.2.txt", "file to load data");

int main(int argc, char *argv[])
{
    google::ParseCommandLineFlags(&argc, &argv, true);

    // initialize the compensator;
    mic_init_worker(FLAGS_model, "./mic_model_" + FLAGS_model + ".mdl");

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
    std::ifstream infile(FLAGS_file);
    if (!infile.is_open())
    {
        MIC_LOG_ERR("Error: Could not open file %s", FLAGS_file);
        return -1;
    }
    while (true)
    {
        std::vector<float64_t> data_line;
        if (read_line(infile, data_line) == ret_t::MIC_RET_FAILED)
            break;

        mic_mag_t mag;
        if (get_line_data(data_line, mag) == ret_t::MIC_RET_SUCCESSED)
        {
            float64_t ts = mag.time_stamp;
            mic_add_data(ts, mag);

            // do compensation and save results to file;
            mic_mag_t mag_out;
            mic_compensate(ts, mag_out);
            outfile << std::fixed << ts << "\t" << mag_out.value << "\t" << mag_out.vector.transpose() << std::endl;
            cout << std::fixed << ts << "\t" << mag_out.value << "\t" << mag_out.vector.transpose() << std::endl;
        }
    }
    infile.close();
    outfile.close();

    google::ShutDownCommandLineFlags();
    return 0;
}
