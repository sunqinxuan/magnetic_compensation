/*
 * Magnetic Interference Compensation
 *
 * Copyright (C) 2024 Qinxuan Sun. All rights reserved.
 *
 *     Author : Qinxuan Sun
 *    Contact : sunqinxuan@outlook.com
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef MIC_INTERFACE
#define MIC_INTERFACE

#include "api/mic_base.h"
// #include "api/mic_compensation_api.h"

using namespace mic;

// 读取标定数据：参数1：待读入标定数据的的文件名及路径（string格式）
// 读取任务数据：参数1：待读入任务数据的的文件名及路径（string格式）
// 生成磁干扰补偿模型：参数1：待保存模型的文件名及路径（string格式）
// 加载磁干扰补偿模型：参数1：待保加载模型的文件名及路径（string格式）
// 磁干扰补偿：参数1：补偿后数据的文件名及路径（string格式），其他参数可以根据现在的代码自己定
// 再给一组示例数据

bool initialize();

bool loadCalibData(const std::string &calib_data_file);

bool loadTaskData(const std::string &task_data_file);

bool calibModel(const std::string &model_file);

bool loadModel(const std::string &model_file);

bool compensate(const std::string &comp_data_file);

#endif