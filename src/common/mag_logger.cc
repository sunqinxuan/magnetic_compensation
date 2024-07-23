#include "common/mag_logger.h"
#include <stdarg.h>

MAG_NAMESPACE_START

#define MAG_MAX_LOG_MSG_SIZE (4096)

void MAG_LOG_BASIC_INFO(const char_t* info, ...)
{
    va_list args;
    va_start(args, info);
    mag_logger_t::log(
        mag_log_kind_t::MAG_LOG_INFO,
        mag_log_level_t::MAG_LOG_BASIC,
        info,
        args);
    va_end(args);
}

void MAG_LOG_DEBUG_INFO(const char_t* info, ...)
{
    va_list args;
    va_start(args, info);
    mag_logger_t::log(
        mag_log_kind_t::MAG_LOG_INFO,
        mag_log_level_t::MAG_LOG_DEBUG,
        info,
        args);
    va_end(args);
}

void MAG_LOG_WARN(const char_t* info, ...)
{
    va_list args;
    va_start(args, info);
    mag_logger_t::log(
        mag_log_kind_t::MAG_LOG_WARN,
        mag_log_level_t::MAG_LOG_BASIC,
        info,
        args);
    va_end(args);
}

void MAG_LOG_ERR(const char_t* info, ...)
{
    va_list args;
    va_start(args, info);
    mag_logger_t::log(
        mag_log_kind_t::MAG_LOG_ERR,
        mag_log_level_t::MAG_LOG_BASIC,
        info,
        args);
    va_end(args);
}

void MagLogger::initialize(mag_logger_type_t type, std::string filename, mag_log_level_t log_level)
{
    if (_logger != nullptr)
    {
        return;
    }
    switch (type)
    {
    case mag_logger_type_t::MAG_BASH_PRINTER:
        _logger.reset(new mag_bash_printer_t());
        break;
    case mag_logger_type_t::MAG_FILE_WRITER:
        _logger.reset(new mag_file_writer_t(filename));
        break;
    case mag_logger_type_t::MAG_BASH_FILE_LOGGER:
    default:
        _logger.reset(new mag_bash_file_logger_t(filename));
        break;
    }
    _log_level = log_level;
}

void MagLogger::set_log_level(mag_log_level_t log_level)
{
    _log_level = log_level;
}

void MagLogger::log_config()
{
    // std::string config_log =
    //     MAG_CONFIG_LOG_BEGIN + std::string("\n") +
    //     mag_config_t::get_instance().to_str() + std::string("\n") +
    //     MAG_CONFIG_LOG_END + std::string("\n");
    // MAG_LOG_BASIC_INFO(config_log.c_str());
}

void MagLogger::log(
    mag_log_kind_t log_type,
    mag_log_level_t log_level,
    const char_t* info,
    va_list args)
{
    if (_logger == nullptr || log_level > _log_level)
    {
        return;
    }
    char_t message[MAG_MAX_LOG_MSG_SIZE];
    vsnprintf(message, MAG_MAX_LOG_MSG_SIZE, info, args);
    switch (log_type)
    {
    case mag_log_kind_t::MAG_LOG_ERR:
        _logger->log_err(message);
        break;
    case mag_log_kind_t::MAG_LOG_WARN:
        _logger->log_warn(message);
        break;
    case mag_log_kind_t::MAG_LOG_INFO:
        _logger->log_info(message);
        break;
    default:
        break;
    }
}

void MagBashPrinter::log_info(const char_t* info)
{
    printf("#MAG INFO#  %s\n", info);
}

void MagBashPrinter::log_warn(const char_t* info)
{
    printf("#MAG WARN#  %s\n", info);
}

void MagBashPrinter::log_err(const char_t* info)
{
    printf("#MAG ERR#  %s\n", info);
}

MagFileWriter::MagFileWriter(std::string filename)
{
    _file = fopen(filename.c_str(), "w");
}

MagFileWriter::~MagFileWriter()
{
    if (_file != nullptr) fclose(_file);
}

void MagFileWriter::log_info(const char_t* info)
{
    if (_file != nullptr) fprintf(_file, "#MAG INFO#  %s\n", info);
}

void MagFileWriter::log_warn(const char_t* info)
{
    if (_file != nullptr) fprintf(_file, "#MAG INFO#  %s\n", info);
}

void MagFileWriter::log_err(const char_t* info)
{
    if (_file != nullptr) fprintf(_file, "#MAG INFO#  %s\n", info);
}

MagBashFileLogger::MagBashFileLogger(std::string filename)
: _bash_printer(), _file_logger(filename)
{}

MagBashFileLogger::~MagBashFileLogger()
{}

void MagBashFileLogger::log_info(const char_t* info)
{
    _bash_printer.log_info(info);
    _file_logger.log_info(info);
}

void MagBashFileLogger::log_warn(const char_t* info)
{
    _bash_printer.log_warn(info);
    _file_logger.log_warn(info);
}

void MagBashFileLogger::log_err(const char_t* info)
{
    _bash_printer.log_err(info);
    _file_logger.log_err(info);
}

std::unique_ptr<mag_logger_t> MagLogger::_logger = nullptr;
mag_log_level_t MagLogger::_log_level = mag_log_level_t::MAG_LOG_BASIC;


MAG_NAMESPACE_END