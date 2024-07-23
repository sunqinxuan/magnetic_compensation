#ifndef MAG_KF_GLOBAL_INTERNAL_NAVIGATION_SYSTEM
#define MAG_KF_GLOBAL_INTERNAL_NAVIGATION_SYSTEM

#include "mag_ins/mag_ins.h"

MAG_NAMESPACE_START

class MagKalmanFilterGlobalIns;
using mag_kf_gins_t = MagKalmanFilterGlobalIns;

class MagKalmanFilterGlobalIns : public MagIns
{
public:
    MagKalmanFilterGlobalIns() = default;
    virtual ~MagKalmanFilterGlobalIns() = default;

protected:

    virtual mag_pose_t update_pose(
        const mag_pose_t& last_pose,
        const mag_ins_data_t& last_ins_data,
        const mag_ins_data_t& curr_ins_data) override;
};

MAG_NAMESPACE_END

#endif

