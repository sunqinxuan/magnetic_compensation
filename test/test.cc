#include <iostream>
#include <fstream>
#include "common/mic_prerequisite.h"
#include "common/mic_logger.h"
#include "common/mic_config.h"
#include "data_storer/mic_data_storer.h"
#include "mic_compensator/mic_compensator.h"
#include "mic_compensator/obeserver/mic_state_logger.h"

USING_NAMESPACE_MIC;


int main(int argc, char *argv[])
{


    return 0;
}

ret_t loadMITData(const std::string &filename)
{
    std::ifstream infile(filename);
    if (!infile.is_open()) {
        std::cerr << "Error: Could not open file " << filename << std::endl;
        return ret_t::MIC_RET_FAILED;
    }

    // Variable to hold each line of the file
    std::string line;

    // Read the file line by line
    while (std::getline(infile, line)) {
        // Process each line (e.g., print it to the console)
        std::cout << line << std::endl;
    }

    // Close the file stream
    infile.close();

    // Inform the user that reading is complete
    std::cout << "Finished reading from file " << filename << std::endl;
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

    mic_compensator_t comp;
    auto comp_logger = std::make_shared<mic_state_logger_t>();
    comp.subscrible(comp_logger);

    for (size_t i = 0; i < 100; i++)
    {
        comp.compenste();
    }

    return 0;
}

