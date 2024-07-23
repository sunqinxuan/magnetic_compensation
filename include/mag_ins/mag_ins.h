#ifndef MAG_INTERNAL_NAVIGATION_SYSTEM
#define MAG_INTERNAL_NAVIGATION_SYSTEM

#include "common/mag_prerequisite.h"
#include "data_storer/mag_data_storer.h"

MAG_NAMESPACE_START

using mag_ins_data_storer_t =
    mag_data_storer_t<mag_ins_data_t, mag_pose_t>;

class MagIns;
using mag_ins_t = MagIns;
using mag_ins_unique_ptr = std::unique_ptr<MagIns>;

class MagIns
{
public:
    MagIns() = default;
    ~MagIns() = default;

    mag_ins_data_storer_t& get_data();

    ret_t add_ins_data(
        const float64_t ts,
        const mag_ins_data_t& ins_data);

    ret_t set_pose(
        const float64_t ts,
        const mag_pose_t& pose);

    ret_t get_pose(
        const float64_t ts,
        mag_pose_t& pose);

protected:

    virtual mag_pose_t update_pose(
        const mag_pose_t& last_pose,
        const mag_ins_data_t& last_ins_data,
        const mag_ins_data_t& curr_ins_data) = 0;

    float64_t _time_stamp;
    mag_pose_t _curr_pose;

    /* data */
    mag_ins_data_storer_t _data_storer;
};

MAG_NAMESPACE_END

#endif

