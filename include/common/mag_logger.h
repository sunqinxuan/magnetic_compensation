#ifndef MAG_LOGGER
#define MAG_LOGGER

#include "common/mag_prerequisite.h"

MAG_NAMESPACE_START

// for log parser
#define MAG_STATE_LOG_TITLE ("[MAG] R: ")
#define MAG_CONFIG_LOG_BEGIN ("[MAG] C:")
#define MAG_CONFIG_LOG_END  (":C")

class MagLogger;
using mag_logger_t = MagLogger;

class MagBashPrinter;
using mag_bash_printer_t = MagBashPrinter;

class MagFileWriter;
using mag_file_writer_t = MagFileWriter;
using mag_file_ptr = FILE*;

class MagBashFileLogger;
using mag_bash_file_logger_t = MagBashFileLogger;

void MAG_LOG_WARN(const char_t* info, ...);
void MAG_LOG_ERR(const char_t* info, ...);
void MAG_LOG_DEBUG_INFO(const char_t* info, ...);
void MAG_LOG_BASIC_INFO(const char_t* info, ...);

enum class MagLoggerType : uint8_t
{
    MAG_BASH_PRINTER = 0,
    MAG_FILE_WRITER = 1,
    MAG_BASH_FILE_LOGGER = 2,
};
using mag_logger_type_t = MagLoggerType;

enum class MagLogLevel
{
    MAG_LOG_NO = 0,
    MAG_LOG_BASIC = 1,
    MAG_LOG_DEBUG = 2,
};
using mag_log_level_t = MagLogLevel;

enum class MagLogKind
{
    MAG_LOG_ERR = 0,
    MAG_LOG_WARN = 1,
    MAG_LOG_INFO = 2
};
using mag_log_kind_t = MagLogKind;

class MagLogger
{
public:
    virtual ~MagLogger() {}
    MagLogger(const MagLogger& ) = delete;
    MagLogger(MagLogger&& ) = delete;
    MagLogger& operator = (const MagLogger& ) = delete;
    MagLogger& operator = (MagLogger&& ) = delete;

    static void initialize(
        mag_logger_type_t type,
        std::string filename = "",
        mag_log_level_t log_level = mag_log_level_t::MAG_LOG_BASIC);
    static void set_log_level(mag_log_level_t log_level);
    static void log(
        mag_log_kind_t log_type, mag_log_level_t log_level, const char_t* info, va_list args);

    static void log_config();

protected:
    MagLogger() = default;

    virtual void log_info(const char_t* info) = 0;
    virtual void log_warn(const char_t* info) = 0;
    virtual void log_err(const char_t* info) = 0;

    static std::unique_ptr<mag_logger_t> _logger;
    static mag_log_level_t _log_level;
};


class MagBashPrinter : public MagLogger
{
public:
    friend MagBashFileLogger;

    MagBashPrinter() = default;
    virtual ~MagBashPrinter() {};
    MagBashPrinter(const MagBashPrinter& ) = delete;
    MagBashPrinter(MagBashPrinter&& ) = delete;
    MagBashPrinter& operator = (const MagBashPrinter& ) = delete;
    MagBashPrinter& operator = (MagBashPrinter&& ) = delete;

protected:
    virtual void log_info(const char_t* info) override;
    virtual void log_warn(const char_t* info) override;
    virtual void log_err(const char_t* info) override;
};

class MagFileWriter : public MagLogger
{
public:
    friend MagBashFileLogger;

    MagFileWriter(std::string filename);
    virtual ~MagFileWriter();
    MagFileWriter(const MagFileWriter& ) = delete;
    MagFileWriter(MagFileWriter&& ) = delete;
    MagFileWriter& operator = (const MagFileWriter& ) = delete;
    MagFileWriter& operator = (MagFileWriter&& ) = delete;

protected:
    virtual void log_info(const char_t* info) override;
    virtual void log_warn(const char_t* info) override;
    virtual void log_err(const char_t* info) override;

    mag_file_ptr _file;
};

class MagBashFileLogger : public MagLogger
{
public:
    MagBashFileLogger(std::string filename);
    virtual ~MagBashFileLogger();
    MagBashFileLogger(const MagBashFileLogger& ) = delete;
    MagBashFileLogger(MagBashFileLogger&& ) = delete;
    MagBashFileLogger& operator = (const MagBashFileLogger& ) = delete;
    MagBashFileLogger& operator = (MagBashFileLogger&& ) = delete;

protected:
    virtual void log_info(const char_t* info) override;
    virtual void log_warn(const char_t* info) override;
    virtual void log_err(const char_t* info) override;

    mag_bash_printer_t _bash_printer;
    mag_file_writer_t _file_logger;
};

MAG_NAMESPACE_END

#endif