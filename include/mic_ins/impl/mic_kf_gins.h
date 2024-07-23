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
#ifndef MIC_KF_GLOBAL_INTERNAL_NAVIGATION_SYSTEM
#define MIC_KF_GLOBAL_INTERNAL_NAVIGATION_SYSTEM

#include "mic_ins/mic_ins.h"

MIC_NAMESPACE_START

class MicKalmanFilterGlobalIns;
using mic_kf_gins_t = MicKalmanFilterGlobalIns;

class MicKalmanFilterGlobalIns : public MicIns
{
public:
    MicKalmanFilterGlobalIns() = default;
    virtual ~MicKalmanFilterGlobalIns() = default;

protected:

    virtual mic_pose_t update_pose(
        const mic_pose_t& last_pose,
        const mic_ins_data_t& last_ins_data,
        const mic_ins_data_t& curr_ins_data) override;
};

MIC_NAMESPACE_END

#endif

