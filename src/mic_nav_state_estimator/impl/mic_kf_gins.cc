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
#include "mic_nav_state_estimator/impl/mic_kf_gins.h"
#include "kf-gins/insmech.h"
#include "common/rotation.h"

MIC_NAMESPACE_START

using pva_t = PVA;
using imu_t = IMU;

static mic_pva_t pva_to_mic_pva(const pva_t& pva)
{
    mic_pva_t pose;
    pose.attitude = pva.att.qbn.cast<float32_t>();
    pose.position = pva.pos.cast<float32_t>();
    pose.velocity = pva.vel.cast<float32_t>();
    return pose;
}

static pva_t mic_pva_to_pva(const mic_pva_t& pose)
{
    pva_t pva;
    pva.att.qbn = pose.attitude.cast<float64_t>();
    pva.att.euler = Rotation::quaternion2euler(pva.att.qbn);
    pva.att.cbn = Rotation::quaternion2matrix(pva.att.qbn);

    pva.pos = pose.position.cast<float64_t>();
    pva.vel = pose.velocity.cast<float64_t>();
    return pva;
}

// static mic_imu_t imu_to_mic_ins_data(const imu_t& imu)
// {
//     mic_imu_t data;
//     data.time_stamp = imu.time;
//     data.acc = imu.dvel.cast<float32_t>();
//     data.acc_bias = vector_3f_t::Zero();
//     data.gyr = imu.dtheta.cast<float32_t>();
//     data.gyr_bias = vector_3f_t::Zero();
//     return data;
// }

static imu_t mic_imu_to_imu(const mic_imu_t& ins_data)
{
    imu_t imu;
    imu.dt = 0.;
    imu.odovel = 0.;
    imu.time = ins_data.time_stamp;
    imu.dtheta = ins_data.gyr.cast<float64_t>();
    imu.dvel = ins_data.acc.cast<float64_t>();
    return imu;
}

mic_pva_t MicKalmanFilterGlobalIns::update_pose(
    const mic_pva_t& last_pose,
    const mic_imu_t& last_ins_data,
    const mic_imu_t& curr_ins_data)
{
    pva_t pva = mic_pva_to_pva(last_pose);
    pva_t curr_pva = pva;
    imu_t last_imu = mic_imu_to_imu(last_ins_data);
    imu_t curr_imu = mic_imu_to_imu(curr_ins_data);
    INSMech::insMech(pva, curr_pva, last_imu, curr_imu);
    return pva_to_mic_pva(curr_pva);
}

MIC_NAMESPACE_END