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

DEFINE_string(modelfile, "mic_model.mdl", "model file");

int main(int argc, char *argv[])
{
    google::ParseCommandLineFlags(&argc, &argv, true);

    // 初始化补偿器，输入补偿模型文件
    mic_init_worker("ellipsoid", FLAGS_modelfile);

    // timestamp: 时间戳，单位s
    float64_t timestamp = 24000.000000;
    // mag_op: 舱内光泵磁强计测量值，单位nT
    float64_t mag_op = 55484.875000;
    // mag_x,mag_y,mag_z: 舱内磁通门磁强计三轴测量值，单位nT
    float64_t mag_x = -24032.530000;
    float64_t mag_y = -47619.720000;
    float64_t mag_z = -15278.270000;

    std::vector<float64_t> data_line;
    data_line.push_back(timestamp);
    data_line.push_back(mag_op);
    data_line.push_back(mag_x);
    data_line.push_back(mag_y);
    data_line.push_back(mag_z);

    mic_mag_t mag;
    if (get_line_data(data_line, mag) == ret_t::MIC_RET_SUCCESSED)
    {
        float64_t ts = mag.time_stamp;
        mic_mag_t mag_out;

        // 将一组数据输入补偿器
        mic_add_data(ts, mag);

        // 实时磁补偿并输出补偿结果
        mic_compensate(ts, mag_out);
        std::cout << " - real-time compensation: " << std::fixed
             << ts << "\t" << mag_out.value << "\t" << mag_out.vector.transpose()<< std::endl;
    }

    google::ShutDownCommandLineFlags();
    return 0;
}
