#include <iostream>
#include <fstream>
#include <ctime>
#include <gflags/gflags.h>
#include "api/mic_compensation_api.h"

using namespace mic;
using namespace std;

DEFINE_string(modelfile, "mic_model.mdl", "model file");

int main(int argc, char *argv[])
{
    google::ParseCommandLineFlags(&argc, &argv, true);

    // 初始化补偿器，输入补偿模型文件
    mic_init_worker("ellipsoid", FLAGS_modelfile);
    cout << "model: " << FLAGS_modelfile << endl;

    // ts: 时间戳，单位s
    float64_t ts = 24000.000000;
    // mag_op: 舱内光泵磁强计测量值，单位nT
    float64_t mag_op = 55484.875000;
    // mag_x,mag_y,mag_z: 舱内磁通门磁强计三轴测量值，单位nT
    float64_t mag_x = -24032.530000;
    float64_t mag_y = -47619.720000;
    float64_t mag_z = -15278.270000;

    mic_mag_t mag;
    mag.time_stamp = ts;
    mag.value = mag_op;
    mag.vector << mag_x, mag_y, mag_z;

    // 将一组数据输入补偿器
    mic_add_data(ts, mag);

    // 实时磁补偿并输出补偿结果
    mic_mag_t mag_out;
    mic_compensate(ts, mag_out);
    std::cout << " - real-time compensation: " << std::fixed
              << ts << "\t" << mag_out.value << "\t" << mag_out.vector.transpose() << std::endl;

    google::ShutDownCommandLineFlags();
    return 0;
}
