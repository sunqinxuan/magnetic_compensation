#ifndef MAG_STATE_LOGGER
#define MAG_STATE_LOGGER

#include "common/mag_utils.h"

#include "mag_compensator/mag_compensator.h"

MAG_NAMESPACE_START

class MagStateLogger;
using mag_state_logger_t = class MagStateLogger;

class MagStateLogger : public MagObserver<MagCompensator>
{
public:
    MagStateLogger() = default;
    ~MagStateLogger() = default;

    virtual void update(mag_compensator_t& comp) override;

};

MAG_NAMESPACE_END

#endif