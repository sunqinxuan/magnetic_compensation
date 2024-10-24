#include <iostream>
#include <fstream>
#include <ctime>
#include <gflags/gflags.h>
#include <chrono>
#include <thread>
#include <iomanip>
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

// DEFINE_string(model, "ellipsoid", "ellipsoid, tl, tlc or cabin");
// DEFINE_string(modelfile, "mic_model_ellipsoid_1002_02.mdl", "model file");
DEFINE_string(model, "mic_model_ellipsoid_1002_02.mdl", "model file");
DEFINE_string(file, "Flt1002_1002.2.txt", "file to load data");
DEFINE_string(out, "output.txt", "file to output compensation results");

int main(int argc, char *argv[])
{
    google::ParseCommandLineFlags(&argc, &argv, true);

    // initialize the compensator;
    // mic_init_worker(FLAGS_model, "./mic_model_" + FLAGS_model + ".mdl");mic_init_worker(FLAGS_model, "./mic_model_" + FLAGS_model + ".mdl");
    // mic_init_worker(FLAGS_model, "./out/"+FLAGS_modelfile);
    mic_init_worker("ellipsoid", "./out/" + FLAGS_model);

    google::CommandLineFlagInfo info;
    GetCommandLineFlagInfo("output", &info);
    std::string output_file_name;
    // if (info.is_default)
    // {
    //     // output_file_name = "output_" + FLAGS_model + ".txt";
    //     output_file_name = "output_" + FLAGS_file;
    // }
    // else
    // {
        output_file_name = "./out/" + FLAGS_out;
    // }
    MIC_LOG_BASIC_INFO("output file name: %s", output_file_name.c_str());

    // create output file for compensation results;
    // time_t tt = time(nullptr); // milliseconds from 1970;
    // struct tm *cur_tm = localtime(&tt);
    // std::stringstream tm_str;
    // tm_str << cur_tm->tm_year + 1900 << "-"
    //        << cur_tm->tm_mon + 1 << "-"
    //        << cur_tm->tm_mday << "-"
    //        << cur_tm->tm_hour << "-"
    //        << cur_tm->tm_min << "-"
    //        << cur_tm->tm_sec;
    // std::ofstream outfile("./output_" + tm_str.str() + ".txt");

    std::ofstream outfile(output_file_name);
    if (!outfile.is_open())
    {
        MIC_LOG_ERR("Error: Could not open file %s", output_file_name.c_str());
        return -1;
    }

    //
    cout << endl
         << "Loaded compensation model file: " << FLAGS_model << endl;

    const int totalSteps = 50;
    const std::string greenText = "\033[32m";  // 深绿色
    const std::string yellowText = "\033[33m"; // 深黄色
    const std::string resetText = "\033[0m";   // 重置颜色

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
    cout << "Real-time compensation output format:  " << endl
         << "timestamp\tmagnetic intensity\tx-axis component\ty-axis component\tz-axis component" << std::endl;

    using namespace std::chrono;

    // 初始化计时器
    auto start_time = high_resolution_clock::now();
    int refresh_count = 0;
    double frequency = 0.0;

    // load data from file;
    std::ifstream infile(FLAGS_file);
    if (!infile.is_open())
    {
        MIC_LOG_ERR("Error: Could not open file %s", FLAGS_file.c_str());
        return -1;
    }
    std::cout << std::endl;
    while (true)
    {
        // 当前时间
        auto now = high_resolution_clock::now();
        duration<double> elapsed_seconds = now - start_time;

        // 计算刷新频率
        refresh_count++;
        frequency = refresh_count / elapsed_seconds.count();
        // if (frequency > 500)
        //     frequency = 200;

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
            // cout << "Real-time compensation: " << std::setw(3) << std::fixed
            //      << ts << "\t" << mag_out.value << "\t" << mag_out.vector.transpose() << "\r" << std::flush;

            std::cout << "\r" << "\033[K";
            cout << "Real-time compensation: " << std::setw(3) << std::fixed
                 << ts << "\t" << mag_out.value << "\t" << mag_out.vector.transpose() << yellowText
                 << "| Frequency: " << std::fixed << std::setprecision(2)
                 << frequency << " Hz" << resetText;
            cout.flush();

            // cout << "Real-time compensation: " << std::setw(3) << std::fixed
            //      << ts << "\t" << mag_out.value << "\t" << mag_out.vector.transpose() << "\033[K" << std::endl;
            // cout << "Refresh Frequency: " << std::fixed << std::setprecision(2)
            //      << frequency << " Hz\033[K" << std::flush;
            // std::cout << "\033[2A"; // 移动光标到两行上方

            // 每秒刷新一次
            std::this_thread::sleep_for(milliseconds(1));
        }
    }
    std::cout << std::endl
              << std::endl
              << "Compensation Process Finish! " << std::endl
              << endl;
    infile.close();
    outfile.close();

    cout << "Compensation results saved: " << output_file_name << endl
         << endl;

    google::ShutDownCommandLineFlags();
    return 0;
}
