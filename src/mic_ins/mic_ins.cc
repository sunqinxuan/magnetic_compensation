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
#include "mic_ins/mic_ins.h"

MIC_NAMESPACE_START

ret_t MicIns::add_ins_data(
    const float64_t ts,
    const mic_ins_data_t& ins_data)
{
    if (ts < _time_stamp)
    {
        return ret_t::MIC_RET_FAILED;
    }
    mic_ins_data_t last_ins_data;
    bool_t res = _data_storer.get_data<mic_ins_data_t>(_time_stamp, last_ins_data);
    if (res)
    {
        _curr_pose = update_pose(_curr_pose, last_ins_data, ins_data);
        _time_stamp = ts;
    }
    return ret_t::MIC_RET_SUCCESSED;
}

ret_t MicIns::set_pose(
    const float64_t ts,
    const mic_pose_t& pose)
{
    ret_t ret = ret_t::MIC_RET_FAILED;
    if (ts > _time_stamp)
    {
        _time_stamp = ts;
        _curr_pose = pose;
        ret = ret_t::MIC_RET_SUCCESSED;
    }
    _data_storer.add_data<mic_pose_t>(ts, pose);
    return ret;
}

ret_t MicIns::get_pose(
    const float64_t ts,
    mic_pose_t& pose)
{
    mic_pose_t tmp_pose;
    bool_t res = _data_storer.get_data<mic_pose_t>(ts, tmp_pose);
    if (res)
    {
        pose = tmp_pose;
        return ret_t::MIC_RET_SUCCESSED;
    }
    return ret_t::MIC_RET_FAILED;
};



MIC_NAMESPACE_END