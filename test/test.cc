#include <iostream>
#include "common/mag_prerequisite.h"
#include "common/mag_logger.h"
#include "common/mag_config.h"
#include "data_storer/mag_data_storer.h"
#include "mag_compensator/mag_compensator.h"
#include "mag_compensator/obeserver/mag_state_logger.h"

USING_NAMESPACE_MAG;

int main()
{
    mag_logger_t::initialize(mag_logger_type_t::MAG_BASH_FILE_LOGGER, "./mag.log");
    mag_config_t::initialize(mag_config_type_t::MAG_CONFIG_JSON,
        "./etc/default_config.json");
    mag_logger_t::set_log_level(
        static_cast<mag_log_level_t>(MAG_CONFIG_GET(int32_t, "log_level")));

    MagDataStorer<float64_t, int32_t, std::string> storer;
    storer.add_data<float64_t>(1.0, 2.0);
    storer.add_data<int32_t>(1.01, 2);
    storer.add_data<std::string>(0.999, "test");

    float64_t f = 0.0;
    int32_t i = 0;
    std::string str = "";
    auto res = storer.get_data(1.0, f, i, str);
    std::cout << "res: " << res << std::endl;
    std::cout << "f: " << f << " i: " << i << " str: " << str << std::endl;

    MagDataStorer<std::string, int32_t, float64_t> storer2;
    storer2.add_data<float64_t>(1.0, 3214312.0);
    storer2.add_data<int32_t>(1.019, 5);
    storer2.add_data<std::string>(0.999, "test2");

    int32_t data = 0;
    storer2.get_data<int32_t>(1.019, data);
    std::cout << "result: " << data << std::endl;

    res = storer2.get_data(1.0, str, i, f);
    std::cout << "res: " << res << std::endl;
    std::cout << "f: " << f << " i: " << i << " str: " << str << std::endl;

    mag_compensator_t comp;
    auto comp_logger = std::make_shared<mag_state_logger_t>();
    comp.subscrible(comp_logger);

    for (size_t i = 0; i < 100; i++)
    {
        comp.compenste();
    }

    return 0;
}