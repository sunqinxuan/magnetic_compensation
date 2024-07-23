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
#include "common/mic_logger.h"
#include <stdarg.h>

MIC_NAMESPACE_START

#define MIC_MAX_LOG_MSG_SIZE (4096)

void MIC_LOG_BASIC_INFO(const char_t* info, ...)
{
    va_list args;
    va_start(args, info);
    mic_logger_t::log(
        mic_log_kind_t::MIC_LOG_INFO,
        mic_log_level_t::MIC_LOG_BASIC,
        info,
        args);
    va_end(args);
}

void MIC_LOG_DEBUG_INFO(const char_t* info, ...)
{
    va_list args;
    va_start(args, info);
    mic_logger_t::log(
        mic_log_kind_t::MIC_LOG_INFO,
        mic_log_level_t::MIC_LOG_DEBUG,
        info,
        args);
    va_end(args);
}

void MIC_LOG_WARN(const char_t* info, ...)
{
    va_list args;
    va_start(args, info);
    mic_logger_t::log(
        mic_log_kind_t::MIC_LOG_WARN,
        mic_log_level_t::MIC_LOG_BASIC,
        info,
        args);
    va_end(args);
}

void MIC_LOG_ERR(const char_t* info, ...)
{
    va_list args;
    va_start(args, info);
    mic_logger_t::log(
        mic_log_kind_t::MIC_LOG_ERR,
        mic_log_level_t::MIC_LOG_BASIC,
        info,
        args);
    va_end(args);
}

void MicLogger::initialize(mic_logger_type_t type, std::string filename, mic_log_level_t log_level)
{
    if (_logger != nullptr)
    {
        return;
    }
    switch (type)
    {
    case mic_logger_type_t::MIC_BASH_PRINTER:
        _logger.reset(new mic_bash_printer_t());
        break;
    case mic_logger_type_t::MIC_FILE_WRITER:
        _logger.reset(new mic_file_writer_t(filename));
        break;
    case mic_logger_type_t::MIC_BASH_FILE_LOGGER:
    default:
        _logger.reset(new mic_bash_file_logger_t(filename));
        break;
    }
    _log_level = log_level;
}

void MicLogger::set_log_level(mic_log_level_t log_level)
{
    _log_level = log_level;
}

void MicLogger::log_config()
{
    // std::string config_log =
    //     MIC_CONFIG_LOG_BEGIN + std::string("\n") +
    //     mic_config_t::get_instance().to_str() + std::string("\n") +
    //     MIC_CONFIG_LOG_END + std::string("\n");
    // MIC_LOG_BASIC_INFO(config_log.c_str());
}

void MicLogger::log(
    mic_log_kind_t log_type,
    mic_log_level_t log_level,
    const char_t* info,
    va_list args)
{
    if (_logger == nullptr || log_level > _log_level)
    {
        return;
    }
    char_t message[MIC_MAX_LOG_MSG_SIZE];
    vsnprintf(message, MIC_MAX_LOG_MSG_SIZE, info, args);
    switch (log_type)
    {
    case mic_log_kind_t::MIC_LOG_ERR:
        _logger->log_err(message);
        break;
    case mic_log_kind_t::MIC_LOG_WARN:
        _logger->log_warn(message);
        break;
    case mic_log_kind_t::MIC_LOG_INFO:
        _logger->log_info(message);
        break;
    default:
        break;
    }
}

void MicBashPrinter::log_info(const char_t* info)
{
    printf("#MIC INFO#  %s\n", info);
}

void MicBashPrinter::log_warn(const char_t* info)
{
    printf("#MIC WARN#  %s\n", info);
}

void MicBashPrinter::log_err(const char_t* info)
{
    printf("#MIC ERR#  %s\n", info);
}

MicFileWriter::MicFileWriter(std::string filename)
{
    _file = fopen(filename.c_str(), "w");
}

MicFileWriter::~MicFileWriter()
{
    if (_file != nullptr) fclose(_file);
}

void MicFileWriter::log_info(const char_t* info)
{
    if (_file != nullptr) fprintf(_file, "#MIC INFO#  %s\n", info);
}

void MicFileWriter::log_warn(const char_t* info)
{
    if (_file != nullptr) fprintf(_file, "#MIC INFO#  %s\n", info);
}

void MicFileWriter::log_err(const char_t* info)
{
    if (_file != nullptr) fprintf(_file, "#MIC INFO#  %s\n", info);
}

MicBashFileLogger::MicBashFileLogger(std::string filename)
: _bash_printer(), _file_logger(filename)
{}

MicBashFileLogger::~MicBashFileLogger()
{}

void MicBashFileLogger::log_info(const char_t* info)
{
    _bash_printer.log_info(info);
    _file_logger.log_info(info);
}

void MicBashFileLogger::log_warn(const char_t* info)
{
    _bash_printer.log_warn(info);
    _file_logger.log_warn(info);
}

void MicBashFileLogger::log_err(const char_t* info)
{
    _bash_printer.log_err(info);
    _file_logger.log_err(info);
}

std::unique_ptr<mic_logger_t> MicLogger::_logger = nullptr;
mic_log_level_t MicLogger::_log_level = mic_log_level_t::MIC_LOG_BASIC;


MIC_NAMESPACE_END