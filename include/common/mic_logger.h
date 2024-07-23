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
#ifndef MIC_LOGGER
#define MIC_LOGGER

#include "common/mic_prerequisite.h"

MIC_NAMESPACE_START

// for log parser
#define MIC_STATE_LOG_TITLE ("[MIC] R: ")
#define MIC_CONFIG_LOG_BEGIN ("[MIC] C:")
#define MIC_CONFIG_LOG_END  (":C")

class MicLogger;
using mic_logger_t = MicLogger;

class MicBashPrinter;
using mic_bash_printer_t = MicBashPrinter;

class MicFileWriter;
using mic_file_writer_t = MicFileWriter;
using mic_file_ptr = FILE*;

class MicBashFileLogger;
using mic_bash_file_logger_t = MicBashFileLogger;

void MIC_LOG_WARN(const char_t* info, ...);
void MIC_LOG_ERR(const char_t* info, ...);
void MIC_LOG_DEBUG_INFO(const char_t* info, ...);
void MIC_LOG_BASIC_INFO(const char_t* info, ...);

enum class MicLoggerType : uint8_t
{
    MIC_BASH_PRINTER = 0,
    MIC_FILE_WRITER = 1,
    MIC_BASH_FILE_LOGGER = 2,
};
using mic_logger_type_t = MicLoggerType;

enum class MicLogLevel
{
    MIC_LOG_NO = 0,
    MIC_LOG_BASIC = 1,
    MIC_LOG_DEBUG = 2,
};
using mic_log_level_t = MicLogLevel;

enum class MicLogKind
{
    MIC_LOG_ERR = 0,
    MIC_LOG_WARN = 1,
    MIC_LOG_INFO = 2
};
using mic_log_kind_t = MicLogKind;

class MicLogger
{
public:
    virtual ~MicLogger() {}
    MicLogger(const MicLogger& ) = delete;
    MicLogger(MicLogger&& ) = delete;
    MicLogger& operator = (const MicLogger& ) = delete;
    MicLogger& operator = (MicLogger&& ) = delete;

    static void initialize(
        mic_logger_type_t type,
        std::string filename = "",
        mic_log_level_t log_level = mic_log_level_t::MIC_LOG_BASIC);
    static void set_log_level(mic_log_level_t log_level);
    static void log(
        mic_log_kind_t log_type, mic_log_level_t log_level, const char_t* info, va_list args);

    static void log_config();

protected:
    MicLogger() = default;

    virtual void log_info(const char_t* info) = 0;
    virtual void log_warn(const char_t* info) = 0;
    virtual void log_err(const char_t* info) = 0;

    static std::unique_ptr<mic_logger_t> _logger;
    static mic_log_level_t _log_level;
};


class MicBashPrinter : public MicLogger
{
public:
    friend MicBashFileLogger;

    MicBashPrinter() = default;
    virtual ~MicBashPrinter() {};
    MicBashPrinter(const MicBashPrinter& ) = delete;
    MicBashPrinter(MicBashPrinter&& ) = delete;
    MicBashPrinter& operator = (const MicBashPrinter& ) = delete;
    MicBashPrinter& operator = (MicBashPrinter&& ) = delete;

protected:
    virtual void log_info(const char_t* info) override;
    virtual void log_warn(const char_t* info) override;
    virtual void log_err(const char_t* info) override;
};

class MicFileWriter : public MicLogger
{
public:
    friend MicBashFileLogger;

    MicFileWriter(std::string filename);
    virtual ~MicFileWriter();
    MicFileWriter(const MicFileWriter& ) = delete;
    MicFileWriter(MicFileWriter&& ) = delete;
    MicFileWriter& operator = (const MicFileWriter& ) = delete;
    MicFileWriter& operator = (MicFileWriter&& ) = delete;

protected:
    virtual void log_info(const char_t* info) override;
    virtual void log_warn(const char_t* info) override;
    virtual void log_err(const char_t* info) override;

    mic_file_ptr _file;
};

class MicBashFileLogger : public MicLogger
{
public:
    MicBashFileLogger(std::string filename);
    virtual ~MicBashFileLogger();
    MicBashFileLogger(const MicBashFileLogger& ) = delete;
    MicBashFileLogger(MicBashFileLogger&& ) = delete;
    MicBashFileLogger& operator = (const MicBashFileLogger& ) = delete;
    MicBashFileLogger& operator = (MicBashFileLogger&& ) = delete;

protected:
    virtual void log_info(const char_t* info) override;
    virtual void log_warn(const char_t* info) override;
    virtual void log_err(const char_t* info) override;

    mic_bash_printer_t _bash_printer;
    mic_file_writer_t _file_logger;
};

MIC_NAMESPACE_END

#endif