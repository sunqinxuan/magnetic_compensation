#include "mag_ins/mag_ins.h"

MAG_NAMESPACE_START

ret_t MagIns::add_ins_data(
    const float64_t ts,
    const mag_ins_data_t& ins_data)
{
    if (ts < _time_stamp)
    {
        return ret_t::MAG_RET_FAILED;
    }
    mag_ins_data_t last_ins_data;
    bool_t res = _data_storer.get_data<mag_ins_data_t>(_time_stamp, last_ins_data);
    if (res)
    {
        _curr_pose = update_pose(_curr_pose, last_ins_data, ins_data);
        _time_stamp = ts;
    }
    return ret_t::MAG_RET_SUCCESSED;
}

ret_t MagIns::set_pose(
    const float64_t ts,
    const mag_pose_t& pose)
{
    ret_t ret = ret_t::MAG_RET_FAILED;
    if (ts > _time_stamp)
    {
        _time_stamp = ts;
        _curr_pose = pose;
        ret = ret_t::MAG_RET_SUCCESSED;
    }
    _data_storer.add_data<mag_pose_t>(ts, pose);
    return ret;
}

ret_t MagIns::get_pose(
    const float64_t ts,
    mag_pose_t& pose)
{
    mag_pose_t tmp_pose;
    bool_t res = _data_storer.get_data<mag_pose_t>(ts, tmp_pose);
    if (res)
    {
        pose = tmp_pose;
        return ret_t::MAG_RET_SUCCESSED;
    }
    return ret_t::MAG_RET_FAILED;
};



MAG_NAMESPACE_END