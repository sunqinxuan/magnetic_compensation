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
#include "mic_nav_state_estimator/impl/mic_kf_ins_estimator.h"
#include "kf-gins/insmech.h"
#include "common/rotation.h"

MIC_NAMESPACE_START

using pva_t = PVA;
using imu_t = IMU;

static mic_nav_state_t pva_to_mic_nav_state(const pva_t &pva)
{
    mic_nav_state_t pose;
    pose.attitude = pva.att.qbn.cast<float64_t>();
    pose.position = pva.pos.cast<float64_t>();
    pose.velocity = pva.vel.cast<float64_t>();
    return pose;
}

static pva_t mic_nav_state_to_pva(const mic_nav_state_t &pose)
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

static imu_t mic_imu_to_imu(const mic_imu_t &ins_data)
{
    imu_t imu;
    imu.dt = 0.;
    imu.odovel = 0.;
    imu.time = ins_data.time_stamp;
    imu.dtheta = ins_data.gyr.cast<float64_t>();
    imu.dvel = ins_data.acc.cast<float64_t>();
    return imu;
}

ret_t MicKFINSEstimator::propagate(
    const float64_t ts,
    const mic_imu_t &imu_data,
    mic_nav_state_t &nav_state)
{
    ret_t ret = ret_t::MIC_RET_FAILED;

    mic_nav_state_t curr_nav_state;
    mic_imu_t curr_imu;

    bool_t res_nav = _data_storer.get_data<mic_nav_state_t>(_curr_time_stamp, curr_nav_state);
    bool_t res_imu = _data_storer.get_data<mic_imu_t>(_curr_time_stamp, curr_imu);

    if (res_nav && res_imu)
    {
        pva_t pvapre = mic_nav_state_to_pva(curr_nav_state);
        pva_t pvacur = mic_nav_state_to_pva(nav_state);
        imu_t imupre = mic_imu_to_imu(curr_imu);
        imu_t imucur = mic_imu_to_imu(imu_data);

        INSMech::insMech(pvapre, pvacur, imupre, imucur);

        nav_state = pva_to_mic_nav_state(pvacur);

        ret = ret_t::MIC_RET_SUCCESSED;
    }

    return ret;
}

MIC_NAMESPACE_END