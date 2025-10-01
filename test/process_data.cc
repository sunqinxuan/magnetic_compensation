#include <iostream>
#include <sstream>
#include <fstream>
#include <vector>
#include <math.h>
#include <map>
#include <time.h>
#include <iomanip>

using float64_t = double;

bool read_data(
    std::string filename,
    std::map<float64_t, std::vector<float64_t>> &data)
{
    std::ifstream state_file(filename);
    if (!state_file.is_open())
    {
        printf("File is not existed, %s!\n", filename.c_str());
        return false;
    }

    std::string state_line;

    // 先读取第一行确定列数
    if (!std::getline(state_file, state_line))
    {
        printf("File is empty!\n");
        return false;
    }

    std::istringstream first_line_stream(state_line);
    float64_t ts;
    first_line_stream >> ts;

    // 更高效的方式统计列数
    size_t column_count = 0;
    std::string temp;
    while (first_line_stream >> temp)
    {
        column_count++;
    }

    if (column_count == 0)
    {
        printf("No data columns found!\n");
        return false;
    }

    // 重置文件指针
    state_file.clear();
    state_file.seekg(0);

    // 预先分配一个vector用于复用
    std::vector<float64_t> line_data;
    line_data.reserve(column_count);

    while (std::getline(state_file, state_line))
    {
        std::istringstream line_stream(state_line);
        if (!(line_stream >> ts))
            continue;

        line_data.clear();

        // 直接读取到预分配的vector中
        float64_t value;
        for (size_t i = 0; i < column_count && line_stream >> value; ++i)
        {
            line_data.push_back(value);
        }

        if (line_data.size() == column_count)
        {
            // 使用原地构造，避免不必要的拷贝
            data.emplace(std::piecewise_construct,
                        std::forward_as_tuple(ts),
                        std::forward_as_tuple(std::move(line_data)));
        }
    }
    return true;
}

template <typename T>
bool find_result_by_time_stamp(
    const float64_t& ts,
    std::map<float64_t, T>& map,
    T& result,
    float64_t tolerant_time)
{
    auto precise_iter = map.find(ts);
    if (precise_iter != map.end())
    {
        result = precise_iter->second;
        return true;
    }
    bool has_anchor = false;
    auto rough_iter_1 = map.lower_bound(ts);
    auto rough_iter_2 = rough_iter_1;
    if (rough_iter_2 != map.begin())
    {
        rough_iter_2--;
    }
    if (rough_iter_1 != map.end()
        && fabs(ts - rough_iter_1->first) < tolerant_time)
    {
        result = rough_iter_1->second;
        has_anchor = true;
    }
    else if (rough_iter_2 != map.end()
        && fabs(ts - rough_iter_2->first) < tolerant_time)
    {
        result = rough_iter_2->second;
        has_anchor = true;
    }
    return has_anchor;
}

int main(int argc, char **argv)
{
    if (argc < 2)
    {
        std::cerr << "Usage: " << argv[0] << " <file1> [file2] ..." << std::endl;
        return 1;
    }

    clock_t start = clock();

    std::vector<std::map<float64_t, std::vector<float64_t>>> all_data;
    all_data.reserve(argc - 1);

    for (int i = 1; i < argc; ++i)
    {
        std::map<float64_t, std::vector<float64_t>> file_data;

        if (read_data(argv[i], file_data))
        {
            all_data.emplace_back(std::move(file_data));
            std::cout << "Read " << all_data.back().size() << " points from " << argv[i] << std::endl;
        }
        else
        {
            std::cerr << "Error reading: " << argv[i] << std::endl;
        }
    }

    std::cout << "Successfully processed " << all_data.size() << "/" << (argc - 1) << " files" << std::endl;

    if (all_data.empty())
    {
        std::cerr << "Data empty!" << std::endl;
    }

    auto& first_data = all_data[0];
    std::vector<std::ofstream> output_files;
    output_files.resize(first_data.size());
    for (size_t i = 0; i < all_data.size(); ++i)
    {
        std::string filename = std::string(argv[i + 1]) + "_new.txt";
        output_files[i].open(filename);
        output_files[i] << std::fixed << std::setprecision(6);
    }
    for (auto& it : first_data)
    {
        auto ts = it.first;
        std::vector<float64_t> result;
        for (size_t i = 0; i < all_data.size(); ++i)
        {
            if (!find_result_by_time_stamp(ts, all_data[i], result, 100.0))
            {
                std::cerr << "Error: No data found for timestamp " << ts << " in file " << argv[i + 1] << std::endl;
            }
            output_files[i] << ts << " ";
            for (auto& value : result)
            {
                output_files[i] << value << " ";
            }
            output_files[i] << std::endl;
        }
    }

    for (size_t i = 0; i < output_files.size(); ++i)
    {
        output_files[i].close();
    }

    clock_t end = clock();
    std::cout << "Time used: " << (end - start) / (float64_t)CLOCKS_PER_SEC << "s" << std::endl;

    return 0;
}